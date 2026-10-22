"""SO-101 controls: CAD radians in simulation, calibrated radians on hardware."""

from __future__ import annotations

import math
import json
from pathlib import Path
from dataclasses import dataclass, asdict
from collections.abc import Mapping
from typing import TypedDict
import numpy as np
from numpy.typing import NDArray
import mujoco

from airtruck.raw_api.so101.io import (
    ArmController,
    ServoTransport,
    JointName,
    PositionCommand,
)


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


class SimSO101(ArmController):
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

        A mapping names joints and leaves other joints unchanged. A six-element
        vector follows ``JOINTS`` order. Values outside the CAD range are clamped.
        Validate the whole command before updating any target.
        """
        positions = joint_positions(positions)
        for name, radians in positions.items():
            if name not in self.names:
                raise ValueError(f"Unknown SO-101 joint: {name}")
            if not np.isfinite(radians):
                raise ValueError("Targets must be finite")
        for name, radians in positions.items():
            self.command_radians(name, radians)

    @property
    def positions(self) -> dict[JointName, float]:
        """Actual unquantized joint positions in CAD radians."""
        return dict(zip(self.names, map(float, self.data.qpos[self._qpos])))

    @property
    def targets(self) -> dict[JointName, float]:
        """Copy of commanded joint positions in CAD radians."""
        return dict(zip(self.names, map(float, self.target)))

    @property
    def limits(self) -> dict[JointName, tuple[float, float]]:
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


JOINTS: tuple[JointName, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


@dataclass(frozen=True)
class JointCalibration:
    """Measured hardware radians to absolute encoder ticks; zero is calibration-defined."""

    servo_id: int
    zero_tick: int
    direction: int
    limits_rad: tuple[float, float]
    counts_per_revolution: int = 4096

    def __post_init__(self) -> None:
        if any(type(v) is not int for v in (self.servo_id, self.zero_tick, self.direction, self.counts_per_revolution)):
            raise ValueError("Calibration IDs, ticks, direction and resolution must be integers")
        if len(self.limits_rad) != 2 or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in self.limits_rad):
            raise ValueError("limits_rad must contain two numbers")
        if not 0 <= self.servo_id <= 253:
            raise ValueError("servo_id must be between 0 and 253")
        if self.direction not in (-1, 1):
            raise ValueError("direction must be -1 or +1")
        if self.counts_per_revolution <= 0:
            raise ValueError("counts_per_revolution must be positive")
        lo, hi = self.limits_rad
        if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
            raise ValueError("limits_rad must be finite and increasing")
        if not 0 <= self.zero_tick < self.counts_per_revolution:
            raise ValueError("zero_tick is outside the encoder range")
        for angle in (lo, hi):
            tick = (
                self.zero_tick
                + self.direction * angle * self.counts_per_revolution / math.tau
            )
            if not 0 <= tick <= self.counts_per_revolution - 1:
                raise ValueError("Calibrated travel crosses the absolute encoder wrap")

    def to_radians(self, ticks: int) -> float:
        if not 0 <= ticks < self.counts_per_revolution:
            raise ValueError("Encoder feedback is outside its calibrated range")
        return (
            self.direction
            * (ticks - self.zero_tick)
            * math.tau
            / self.counts_per_revolution
        )

    def to_ticks(self, radians: float) -> int:
        if not math.isfinite(radians):
            raise ValueError("Joint targets must be finite")
        lo, hi = self.limits_rad
        angle = min(hi, max(lo, radians))
        # Keep quantization within the calibrated mechanical interval.
        end_ticks = [
            self.zero_tick + self.direction * a * self.counts_per_revolution / math.tau
            for a in (lo, hi)
        ]
        # Endpoints derived from ticks can round a few ulps across an integer.
        end_ticks = [float(round(t)) if math.isclose(t, round(t), rel_tol=0, abs_tol=1e-9) else t
                     for t in end_ticks]
        minimum, maximum = math.ceil(min(end_ticks)), math.floor(max(end_ticks))
        tick = round(
            self.zero_tick
            + self.direction * angle * self.counts_per_revolution / math.tau
        )
        return min(maximum, max(minimum, tick))


def load_joint_calibrations(path: str | Path) -> dict[JointName, JointCalibration]:
    """Load a complete joint-name mapping; reject partial or ambiguous records."""
    def unique(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate calibration key: {key}")
            result[key] = value
        return result

    raw = json.loads(Path(path).read_text(), object_pairs_hook=unique)
    if not isinstance(raw, dict) or set(raw) != set(JOINTS):
        raise ValueError("Calibration must contain exactly all six joint names")
    result: dict[JointName, JointCalibration] = {}
    fields = {"servo_id", "zero_tick", "direction", "limits_rad", "counts_per_revolution"}
    for name in JOINTS:
        record = raw[name]
        if not isinstance(record, dict) or set(record) != fields:
            raise ValueError(f"Invalid calibration fields for {name}")
        limits = record["limits_rad"]
        if not isinstance(limits, list) or len(limits) != 2:
            raise ValueError(f"Invalid limits_rad for {name}")
        result[name] = JointCalibration(
            servo_id=record["servo_id"], zero_tick=record["zero_tick"],
            direction=record["direction"], limits_rad=tuple(limits),
            counts_per_revolution=record["counts_per_revolution"],
        )
    if len({c.servo_id for c in result.values()}) != len(JOINTS):
        raise ValueError("Calibration servo IDs must be unique")
    return result


def joint_calibrations_dict(calibration: Mapping[JointName, JointCalibration]) -> dict[str, object]:
    """JSON-ready mapping; also supports progressive calibration checkpoints."""
    return {name: asdict(value) for name, value in calibration.items()}


def joint_positions(values: PositionCommand) -> Mapping[JointName, float]:
    """Map a command onto ``JOINTS`` order.

    A mapping is left unchanged, so named commands stay available. A vector of
    six radians is assigned, in order, to shoulder_pan, shoulder_lift,
    elbow_flex, wrist_flex, wrist_roll, and gripper.
    """
    if isinstance(values, Mapping):
        return values
    angles = np.asarray(values, dtype=np.float64)
    if angles.ndim != 1 or angles.shape != (len(JOINTS),):
        raise ValueError(
            "Expected six joint targets in JOINTS order: " + ", ".join(JOINTS)
        )
    if not np.isfinite(angles).all():
        raise ValueError("Targets must be finite")
    return {name: float(angle) for name, angle in zip(JOINTS, angles, strict=True)}


class RealSO101(ArmController):
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
        if set(calibration) != set(JOINTS):
            raise ValueError("Provide calibration for all six SO-101 joints")
        ids = [calibration[name].servo_id for name in JOINTS]
        if len(set(ids)) != len(ids):
            raise ValueError("Each joint must use a unique servo ID")
        self._transport = transport
        self._calibration = dict(calibration)
        self._targets: dict[JointName, float] = {}
        self._enabled = False

    @property
    def limits(self) -> dict[JointName, tuple[float, float]]:
        return {name: c.limits_rad for name, c in self._calibration.items()}

    @property
    def enabled(self) -> bool:
        return self._enabled

    @property
    def positions(self) -> dict[JointName, float]:
        raw = self._transport.read_positions(
            [self._calibration[n].servo_id for n in JOINTS]
        )
        return {
            n: self._calibration[n].to_radians(raw[self._calibration[n].servo_id])
            for n in JOINTS
        }

    @property
    def targets(self) -> dict[JointName, float]:
        return self._targets.copy()

    def set_positions(self, positions: PositionCommand) -> None:
        if not self.enabled:
            raise RuntimeError(
                "Enable the hardware controller before commanding positions"
            )
        positions = joint_positions(positions)
        ticks: dict[int, int] = {}
        targets: dict[JointName, float] = {}
        for name, angle in positions.items():
            if name not in self._calibration:
                raise ValueError(f"Unknown SO-101 joint: {name}")
            c = self._calibration[name]
            ticks[c.servo_id] = c.to_ticks(angle)
            targets[name] = c.to_radians(ticks[c.servo_id])
        if ticks:
            self._transport.write_positions(ticks)
            self._targets.update(targets)

    def set_enabled(self, enabled: bool) -> None:
        if enabled and self._enabled:
            return
        ids = [self._calibration[n].servo_id for n in JOINTS]
        if enabled:
            positions = self.positions
            # Refuse an out-of-calibration pose rather than command a jump on enable.
            if any(
                not self.limits[n][0] <= q <= self.limits[n][1]
                for n, q in positions.items()
            ):
                raise ValueError("Current hardware pose is outside calibrated travel")
            ticks = {
                self._calibration[n].servo_id: self._calibration[n].to_ticks(q)
                for n, q in positions.items()
            }
            self._transport.write_positions(ticks)
            self._transport.set_torque_enabled(ids, True)
            self._targets = positions
        else:
            self._transport.set_torque_enabled(ids, False)
        self._enabled = enabled
