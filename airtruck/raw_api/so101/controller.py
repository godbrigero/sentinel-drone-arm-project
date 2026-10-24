"""SO-101 controls: CAD radians in simulation, calibrated radians on hardware."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import TypedDict
import numpy as np
from numpy.typing import NDArray
import mujoco

from airtruck.raw_api.so101.io import (
    ServoTransport,
    JointName,
    PositionCommand,
)

from airtruck.raw_api.so101.base import PositionReader, PositionController
from airtruck.raw_api.so101.calibration import JointCalibration
from airtruck.raw_api.so101.joints import JOINTS
from airtruck.raw_api.so101.position import JointPosition, PositionUnit
from airtruck.raw_api.so101.utils import validate_position_command


class ServoDatasheet(TypedDict):
    encoder_counts: int
    voltage_V: float


class ControllerTuning(TypedDict):
    target_speed_rad_s: float
    target_acceleration_rad_s2: float
    kp_V_per_rad: float
    ki_V_per_rad_s: float
    kd_V_s_per_rad: float


class ServoSpec(TypedDict):
    datasheet: ServoDatasheet
    controller_demo_tuning: ControllerTuning


class SimSO101(PositionController):
    """Control a caller-supplied six-joint SO-101 simulation in CAD radians.

    The caller supplies matching model/data objects and servo parameters.
    No files, paths, environment variables or hardware are accessed.
    Model/data are used in place; servo settings are copied at construction.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        servo_spec: ServoSpec,
        prefix: str = "",
    ) -> None:
        self.model = model
        self.data = data
        self.names = list(JOINTS)
        self._joints = np.array([model.joint(prefix + n).id for n in JOINTS])
        self._actuators = np.array([model.actuator(prefix + n).id for n in JOINTS])
        if any(model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE for j in self._joints):
            raise ValueError("SO101 requires six hinge joints")
        if any(
            model.actuator_trnid[a, 0] != j
            or model.actuator_trntype[a] != mujoco.mjtTrn.mjTRN_JOINT
            for a, j in zip(self._actuators, self._joints, strict=True)
        ):
            raise ValueError("SO101 actuators must drive their corresponding joints")
        self._qpos = model.jnt_qposadr[self._joints].copy()
        self._qvel = model.jnt_dofadr[self._joints].copy()
        self.spec: ServoSpec = {
            "datasheet": servo_spec["datasheet"].copy(),
            "controller_demo_tuning": servo_spec["controller_demo_tuning"].copy(),
        }
        self.tune: ControllerTuning = self.spec["controller_demo_tuning"]
        self.resolution: int = self.spec["datasheet"]["encoder_counts"]
        self.target: NDArray[np.float64] = data.qpos[self._qpos].copy()
        self.profile: NDArray[np.float64] = self.target.copy()
        self.speed: NDArray[np.float64] = np.zeros(6)
        self.integral: NDArray[np.float64] = np.zeros(6)
        self._voltage_commands: dict[JointName, float] = {}
        self._enabled: bool = True
        self._gain: NDArray[np.float64] = model.actuator_gainprm[self._actuators].copy()
        self._bias: NDArray[np.float64] = model.actuator_biasprm[self._actuators].copy()

    def sync_write(self, register: str, values: Mapping[JointName, float]) -> None:
        if register != "Goal_Position":
            raise ValueError(
                "Supported register: Goal_Position (virtual calibrated encoder ticks)"
            )
        for name, ticks in values.items():
            i = self.names.index(name)
            if not np.isfinite(ticks):
                raise ValueError("Target must be finite")
            # Virtual offset places the midpoint of each CAD range at tick 2048.
            mid = np.mean(self.model.jnt_range[self._joints[i]])
            q = (
                np.clip(round(ticks), 0, 4095) - 2048
            ) * 2 * np.pi / self.resolution + mid
            self.target[i] = np.clip(q, *self.model.jnt_range[self._joints[i]])

    def command_radians(self, name: JointName, radians: float) -> None:
        i = self.names.index(name)
        mid = np.mean(self.model.jnt_range[self._joints[i]])
        self.sync_write(
            "Goal_Position",
            {name: float(2048 + (radians - mid) * self.resolution / (2 * np.pi))},
        )

    def observation(self) -> dict[str, float]:
        q = (
            np.round(self.data.qpos[self._qpos] * self.resolution / (2 * np.pi))
            * 2
            * np.pi
            / self.resolution
        )
        return {name + ".pos": float(q[i]) for i, name in enumerate(self.names)}

    def update_control(self) -> None:
        dt = self.model.opt.timestep
        t = self.tune
        remaining = self.target - self.profile
        desired_speed = np.sign(remaining) * np.minimum(
            t["target_speed_rad_s"],
            np.sqrt(2 * t["target_acceleration_rad_s2"] * np.abs(remaining)),
        )
        self.speed += np.clip(
            desired_speed - self.speed,
            -t["target_acceleration_rad_s2"] * dt,
            t["target_acceleration_rad_s2"] * dt,
        )
        delta = self.speed * dt
        cross = np.abs(delta) >= np.abs(remaining)
        self.profile += np.where(cross, remaining, delta)
        self.speed[cross] = 0
        sensed = (
            np.round(self.data.qpos[self._qpos] * self.resolution / (2 * np.pi))
            * 2
            * np.pi
            / self.resolution
        )
        error = self.profile - sensed
        raw = (
            t["kp_V_per_rad"] * error
            + t["ki_V_per_rad_s"] * self.integral
            - t["kd_V_s_per_rad"] * self.data.qvel[self._qvel]
        )
        vmax = self.spec["datasheet"]["voltage_V"]
        # Anti-windup: only integrate when it will not deepen saturation.
        integrate = (np.abs(raw) < vmax) | (np.sign(error) != np.sign(raw))
        self.integral += np.where(integrate, error * dt, 0)
        self.integral = np.clip(
            self.integral, -vmax / t["ki_V_per_rad_s"], vmax / t["ki_V_per_rad_s"]
        )
        for name, voltage in self._voltage_commands.items():
            raw[self.names.index(name)] = voltage
        if self.enabled:
            self.model.actuator_gainprm[self._actuators] = self._gain
            self.model.actuator_biasprm[self._actuators] = self._bias
            self.data.ctrl[self._actuators] = np.clip(raw, -vmax, vmax)
        else:
            self.data.ctrl[self._actuators] = 0
            self.integral[:] = 0
            self.model.actuator_gainprm[self._actuators] = 0
            self.model.actuator_biasprm[self._actuators] = 0

    def set_motor_voltages(self, voltages: Mapping[JointName, float]) -> None:
        vmax = self.spec["datasheet"]["voltage_V"]
        for name, voltage in voltages.items():
            if (
                name not in self.names
                or not math.isfinite(voltage)
                or abs(voltage) > vmax
            ):
                raise ValueError(
                    "Motor voltage must name a joint and be within supply limits"
                )
        self._voltage_commands.update(voltages)

    def use_position_control(self) -> None:
        self._voltage_commands.clear()
        self.target[:] = self.data.qpos[self._qpos]
        self.profile[:] = self.target
        self.speed[:] = 0
        self.integral[:] = 0

    def set_positions(self, positions: PositionCommand) -> None:
        """Set joint targets in CAD radians.

        A mapping of JointPosition values names joints and leaves others unchanged.
        Units are decoded explicitly; values outside the CAD range are clamped.
        Validate the whole command before updating any target.
        """
        validate_position_command(positions, self.names)
        # Resolve all typed commands before touching the simulation targets.
        commands = [(name, position.to_native(self._limits_rad[name]))
                    for name, position in positions.items()]
        for name, position in commands:
            self.command_radians(name, position.value)

    @property
    def positions(self) -> dict[JointName, JointPosition]:
        """Actual unquantized joint positions in CAD radians."""
        return {
            name: JointPosition(float(angle), PositionUnit.RADIANS, self._limits_rad[name])
            for name, angle in zip(self.names, self.data.qpos[self._qpos], strict=True)
        }

    @property
    def targets(self) -> dict[JointName, JointPosition]:
        """Copy of commanded joint positions in CAD radians."""
        return {name: JointPosition(float(angle), PositionUnit.RADIANS, self._limits_rad[name])
                for name, angle in zip(self.names, self.target, strict=True)}

    @property
    def _limits_rad(self) -> dict[JointName, tuple[float, float]]:
        """Joint travel limits in CAD radians."""
        return {
            name: (
                float(self.model.jnt_range[self._joints[i], 0]),
                float(self.model.jnt_range[self._joints[i], 1]),
            )
            for i, name in enumerate(self.names)
        }

    @property
    def enabled(self) -> bool:
        """Whether powered motor control is enabled."""
        return self._enabled

    @enabled.setter
    def enabled(self, enabled: bool) -> None:
        self._enabled = enabled

    def set_enabled(self, enabled: bool) -> None:
        """Enable/disable powered torque on the next simulation step."""
        self.enabled = enabled

    def reset_control(self) -> None:
        """Reset only this controller after the world restores physics state."""
        self._voltage_commands.clear()
        self.target[:] = self.data.qpos[self._qpos]
        self.profile[:] = self.data.qpos[self._qpos]
        self.speed[:] = 0
        self.integral[:] = 0
        self.enabled = True
        self.model.actuator_gainprm[self._actuators] = self._gain
        self.model.actuator_biasprm[self._actuators] = self._bias


class _CalibratedSO101(PositionReader):
    """Shared calibration validation and feedback conversion; construction does no I/O."""

    def __init__(
        self,
        transport: ServoTransport,
        calibration: Mapping[JointName, JointCalibration],
    ) -> None:
        if set(calibration) != set(JOINTS):
            raise ValueError("Provide calibration for all six SO-101 joints")
        ids = [calibration[name].servo_id for name in JOINTS]
        if len(set(ids)) != len(ids):
            raise ValueError("Each joint must use a unique servo ID")
        self._transport = transport
        self._calibration = dict(calibration)

    @property
    def _limits_rad(self) -> dict[JointName, tuple[float, float]]:
        return {name: c.limits_rad for name, c in self._calibration.items()}

    @property
    def positions(self) -> dict[JointName, JointPosition]:
        """Fresh feedback in radians, clamped to each joint's calibrated limits."""
        return {
            name: JointPosition(angle, PositionUnit.RADIANS, self._calibration[name].limits_rad).clamped()
            for name, angle in self._read_positions().items()
        }

    def _read_positions(self) -> dict[JointName, float]:
        """Unclamped feedback for validating the pose before enabling torque."""
        raw = self._transport.read_positions(
            [self._calibration[n].servo_id for n in JOINTS]
        )
        return {
            n: self._calibration[n].to_radians(raw[self._calibration[n].servo_id])
            for n in JOINTS
        }


class LeaderSO101(_CalibratedSO101):
    """Read a human-operated arm in its own calibrated radians.

    Support the arm, then call start() to disable torque before reading. The caller
    owns the transport and must not enable its motors through another adapter.
    This reader exposes no target commands or torque-enable operation.
    """

    def __init__(
        self,
        transport: ServoTransport,
        calibration: Mapping[JointName, JointCalibration],
    ) -> None:
        super().__init__(transport, calibration)
        self._started = False

    def start(self) -> None:
        """Disable all six motors; a failed request leaves reading unavailable.

        Call again after reconnecting. This is a bus request, not torque telemetry.
        """
        self._started = False
        self._transport.set_torque_enabled(
            [self._calibration[n].servo_id for n in JOINTS], False
        )
        self._started = True

    @property
    def positions(self) -> dict[JointName, JointPosition]:
        """Fresh joint feedback, clamped to calibrated travel."""
        if not self._started:
            raise RuntimeError("Call start() to disable leader torque before reading")
        return super().positions


class RealSO101(_CalibratedSO101, PositionController):
    """Hardware arm controller. All calibration entries are required.

    Constructing the adapter performs no I/O. Call set_enabled(True) explicitly;
    this reads the current pose and queues matching targets before enabling torque.
    Hardware transport failures propagate and may have partially applied at the bus.
    """

    def __init__(
        self,
        transport: ServoTransport,
        calibration: Mapping[JointName, JointCalibration],
    ) -> None:
        super().__init__(transport, calibration)
        self._targets: dict[JointName, JointPosition] = {}
        self._enabled = False

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def targets(self) -> dict[JointName, JointPosition]:
        return self._targets.copy()

    def set_positions(self, positions: PositionCommand) -> None:
        if not self.enabled:
            raise RuntimeError(
                "Enable the hardware controller before commanding positions"
            )
        validate_position_command(positions, self._calibration)
        ticks: dict[int, int] = {}
        targets: dict[JointName, JointPosition] = {}
        for name, position in positions.items():
            c = self._calibration[name]
            ticks[c.servo_id] = c.position_to_ticks(position)
            targets[name] = JointPosition(c.to_radians(ticks[c.servo_id]), PositionUnit.RADIANS, c.limits_rad)
        if ticks:
            self._transport.write_positions(ticks)
            self._targets.update(targets)

    def set_enabled(self, enabled: bool) -> None:
        if enabled and self._enabled:
            return
        ids = [self._calibration[n].servo_id for n in JOINTS]
        if enabled:
            raw = self._transport.read_positions(ids)
            positions = {
                name: c.to_radians(raw[c.servo_id])
                for name, c in self._calibration.items()
            }
            # Refuse an out-of-calibration pose rather than command a jump on enable.
            outside = []
            for name, angle in positions.items():
                c = self._calibration[name]
                lo, hi = c.limits_rad
                if not lo <= angle <= hi:
                    endpoints = sorted((c.to_ticks(lo), c.to_ticks(hi)))
                    outside.append(
                        f"{name} (servo {c.servo_id}): tick {raw[c.servo_id]}, "
                        f"allowed ticks {endpoints[0]}..{endpoints[1]}"
                    )
            if outside:
                raise ValueError(
                    "Current hardware pose is outside calibrated travel: "
                    + "; ".join(outside)
                )
            ticks = {
                self._calibration[n].servo_id: self._calibration[n].to_ticks(q)
                for n, q in positions.items()
            }
            self._transport.write_positions(ticks)
            self._transport.set_torque_enabled(ids, True)
            self._targets = {
                name: JointPosition(angle, PositionUnit.RADIANS, self._calibration[name].limits_rad)
                for name, angle in positions.items()
            }
        else:
            self._transport.set_torque_enabled(ids, False)
        self._enabled = enabled
