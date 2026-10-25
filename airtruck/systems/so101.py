"""One SO-101 API for MuJoCo and the native LeRobot leader/follower classes."""

from __future__ import annotations

from collections import OrderedDict
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from gymnasium import spaces
from lerobot.robots.so_follower import SO101Follower as LeRobotSO101Follower
from lerobot.robots.so_follower import SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader as LeRobotSO101Leader
from lerobot.teleoperators.so_leader import SO101LeaderConfig

from airtruck.systems.base import ROOT, SimSystem, Values, space_values

JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
MODEL = ROOT / "models/so101/so101.xml"


def release(device: Any) -> None:
    """Release every reachable motor and close cameras/serial after partial startup."""
    if not hasattr(device, "bus"):
        return
    errors = []
    try:
        if device.bus.is_connected:
            for motor in device.bus.motors:
                try:
                    device.bus.disable_torque(motor, num_retry=2)
                except Exception as error:
                    errors.append(error)
            try:
                device.bus.disconnect(disable_torque=False)
            except Exception as error:
                errors.append(error)
    finally:
        for camera in getattr(device, "cameras", {}).values():
            if camera.is_connected:
                try:
                    camera.disconnect()
                except Exception as error:
                    errors.append(error)
    if errors:
        raise ExceptionGroup("Hardware cleanup failed", errors)


class SO101Arm(SimSystem):
    """SO-101 joint mapping and measured-pose tracking, shared by both roles."""

    bus: Any
    config: Any
    calibration: dict[str, Any]
    calibration_fpath: Path

    def __init__(
        self,
        name: str,
        *,
        hardware: bool = False,
        model: str | Path = MODEL,
        position: tuple[float, float, float] = (0, 0, 0),
        quaternion: tuple[float, float, float, float] = (1, 0, 0, 0),
        initial_qpos: dict[str, float] | None = None,
    ) -> None:
        SimSystem.__init__(
            self,
            name,
            model=model,
            root_body="base",
            hardware=hardware,
            position=position,
            quaternion=quaternion,
        )
        self.initial_qpos = dict(initial_qpos or {})
        if set(self.initial_qpos) - set(JOINTS):
            raise ValueError("Unknown initial SO-101 joint")
        self.action_space = spaces.Dict(
            OrderedDict(
                (
                    f"{joint}.pos",
                    spaces.Box(
                        np.float32(0 if joint == "gripper" else -100),
                        np.float32(100),
                        shape=(),
                        dtype=np.float32,
                    ),
                )
                for joint in JOINTS
            )
        )
        self.observation_space = self.action_space
        self.controls = {key: 0.0 for key in self.action_space.spaces}
        self._last_sent: Values = {}

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{joint}.pos": float for joint in JOINTS}

    @property
    def observation_features(self) -> dict[str, type]:
        return self.action_features

    @property
    def is_connected(self) -> bool:
        if self.hardware or self._hardware_active:
            return bool(
                getattr(self, "bus", None) is not None and self.bus.is_connected
            )
        return self._model is not None

    @property
    def is_calibrated(self) -> bool:
        if not (self.hardware or self._hardware_active):
            return True  # Simulation uses the prepared model's travel limits.
        bus = getattr(self, "bus", None)
        return bool(bus is not None and bus.is_connected and bus.is_calibrated)

    def make_spec(self) -> mujoco.MjSpec:
        spec = super().make_spec()
        for joint in JOINTS:
            spec.add_equality(
                name=f"hardware_{joint}",
                type=mujoco.mjtEq.mjEQ_JOINT,
                name1=joint,
                active=False,
                data=[0] * 11,
                solref=[0.002, 1],
            )
        return spec

    def bind(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._actuators = np.array(
            [model.actuator(f"{self.name}/{j}").id for j in JOINTS]
        )
        joints = np.array([model.joint(f"{self.name}/{j}").id for j in JOINTS])
        if (
            not np.all(model.jnt_type[joints] == mujoco.mjtJoint.mjJNT_HINGE)
            or not np.all(
                model.actuator_trntype[self._actuators] == mujoco.mjtTrn.mjTRN_JOINT
            )
            or not np.array_equal(model.actuator_trnid[self._actuators, 0], joints)
        ):
            raise ValueError(
                "SO-101 position actuators must drive their named hinge joints"
            )
        self._qpos, self._qvel = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
        self._low, self._high = model.actuator_ctrlrange[self._actuators].T.copy()
        self._input_low, self._input_high = model.actuator_user[
            self._actuators, :2
        ].T.copy()
        self._tracking = np.array(
            [model.eq(f"{self.name}/hardware_{j}").id for j in JOINTS]
        )
        self._gains = model.actuator_gainprm[self._actuators].copy()
        self._biases = model.actuator_biasprm[self._actuators].copy()
        expected = np.array([[0 if j == "gripper" else -100, 100] for j in JOINTS])
        if not np.array_equal(model.actuator_user[self._actuators, :2], expected):
            raise ValueError("SO-101 model must use LeRobot normalized action units")
        super().bind(model, data)

    def _radians(self, action: Values) -> np.ndarray:
        values = np.array([action[f"{j}.pos"] for j in JOINTS])
        return self._low + (values - self._input_low) / (
            self._input_high - self._input_low
        ) * (self._high - self._low)

    def _sim_observation(self) -> Values:
        _, data = self.world
        fraction = np.clip(
            (data.qpos[self._qpos] - self._low) / (self._high - self._low), 0, 1
        )
        values = self._input_low + fraction * (self._input_high - self._input_low)
        return {f"{j}.pos": float(v) for j, v in zip(JOINTS, values, strict=True)}

    def _mirror(self, observation: Values) -> None:
        if self._model is None:
            return
        model, data = self.world
        radians = self._radians(observation)
        data.qpos[self._qpos] = radians
        data.qvel[self._qvel] = 0
        model.eq_data[self._tracking, 0] = radians - model.qpos0[self._qpos]

    def _backend_changed(self) -> None:
        if self._model is None:
            return
        model, data = self.world
        data.eq_active[self._tracking] = self._hardware_active
        model.actuator_gainprm[self._actuators] = (
            0 if self._hardware_active else self._gains
        )
        model.actuator_biasprm[self._actuators] = (
            0 if self._hardware_active else self._biases
        )

    def hold(self) -> None:
        if self.state:
            self.controls.update({key: self.state[key] for key in self.action_features})
            self._last_sent = dict(self.controls)

    def reset(self) -> None:
        self._backend_changed()
        if self._hardware_active:
            self.observe()
        else:
            model, data = self.world
            for joint, value in self.initial_qpos.items():
                index = JOINTS.index(joint)
                if (
                    not np.isfinite(value)
                    or not self._low[index] <= value <= self._high[index]
                ):
                    raise ValueError(f"Initial pose outside limits: {joint}")
                data.qpos[self._qpos[index]] = value
            self.observe()
        self.hold()
        _, data = self.world
        data.ctrl[self._actuators] = self._radians(self.controls)

    def step(self, dt: float) -> None:
        if self._hardware_active:
            if self.controllable and self._command != self._last_sent:
                self._last_sent = self._send_hardware(self._command)
        else:
            _, data = self.world
            data.ctrl[self._actuators] = self._radians(self._command)

    def observe(self) -> Values:
        observation = (
            self._read_hardware() if self._hardware_active else self._sim_observation()
        )
        space_values(self.observation_space, observation)
        self.state = {key: float(observation[key]) for key in self.action_features}
        if self._hardware_active:
            self._mirror(self.state)
        return dict(self.state)

    def get_observation(self) -> Values:
        return self.observe()

    def _read_hardware(self) -> Values:
        raise NotImplementedError

    def _send_hardware(self, action: Values) -> Values:
        raise NotImplementedError

    def _disconnect_hardware(self) -> None:
        release(self)

    def connect(self, calibrate: bool = False) -> None:
        self._calibrate_on_connect = calibrate
        try:
            self.sync_hardware()
        finally:
            self._calibrate_on_connect = False
        if not self.is_connected:
            raise RuntimeError("Simulation must be bound to a driver")

    def disconnect(self) -> None:
        self.close()

    def _connect_hardware(self) -> None:
        self._connect_native(calibrate=getattr(self, "_calibrate_on_connect", False))

    def calibrate(self) -> None:
        if self.hardware:
            raise RuntimeError(
                "Use a hardware-enabled leader or follower for calibration"
            )

    def _connect_native(self, calibrate: bool) -> None:
        raise NotImplementedError

    def _check_connection(self, calibrate: bool) -> None:
        if not self.config.port:
            raise ValueError("A serial port is required for hardware=True")
        if not calibrate and not self.calibration:
            raise ValueError(f"Missing LeRobot calibration: {self.calibration_fpath}")


class SO101Follower(SO101Arm, LeRobotSO101Follower):
    """Native LeRobot follower with the same controls/state in simulation."""

    def __init__(
        self,
        name: str,
        *,
        hardware: bool = False,
        port: str = "",
        id: str | None = None,
        config: SO101FollowerConfig | None = None,
        **kwargs: Any,
    ) -> None:
        native = config or SO101FollowerConfig(
            port=port, id=id or name, use_degrees=False
        )
        if native.use_degrees or native.cameras:
            raise ValueError(
                "This state-based bridge requires use_degrees=False and no cameras"
            )
        LeRobotSO101Follower.__init__(self, native)
        SO101Arm.__init__(self, name, hardware=hardware, **kwargs)

    def configure(self) -> None:
        if not (self.hardware or self._hardware_active):
            return
        if not self.is_calibrated:
            raise RuntimeError("Motor calibration differs from its saved record")
        self.bus.disable_torque()
        positions = self.bus.sync_read("Present_Position", normalize=False)
        for joint, ticks in positions.items():
            calibration = self.calibration[joint]
            if not calibration.range_min <= ticks <= calibration.range_max:
                raise ValueError(f"{joint} is outside calibrated travel")
        self.bus.sync_write("Goal_Position", positions, normalize=False)
        LeRobotSO101Follower.configure(self)

    def _connect_native(self, calibrate: bool) -> None:
        self._check_connection(calibrate)
        LeRobotSO101Follower.connect(self, calibrate=calibrate)

    def _read_hardware(self) -> Values:
        return dict(LeRobotSO101Follower.get_observation(self))

    def _send_hardware(self, action: Values) -> Values:
        return dict(LeRobotSO101Follower.send_action(self, action))

    def send_action(self, action: Values) -> Values:
        """Apply a target immediately, without advancing the shared clock."""
        self.sync_hardware()
        command = self.validate_controls({**self.controls, **action})
        self.controls.update(command)
        if self._hardware_active:
            self._last_sent = self._send_hardware(command)
            return dict(self._last_sent)
        _, data = self.world
        data.ctrl[self._actuators] = self._radians(command)
        return command

    def calibrate(self) -> None:
        if self.hardware:
            LeRobotSO101Follower.calibrate(self)


class SO101Leader(SO101Arm, LeRobotSO101Leader):
    """Input-only leader; simulated controls pose its virtual input device."""

    controllable = False

    def __init__(
        self,
        name: str,
        *,
        hardware: bool = False,
        port: str = "",
        id: str | None = None,
        config: SO101LeaderConfig | None = None,
        **kwargs: Any,
    ) -> None:
        native = config or SO101LeaderConfig(
            port=port, id=id or name, use_degrees=False
        )
        if native.use_degrees:
            raise ValueError("This bridge requires use_degrees=False")
        LeRobotSO101Leader.__init__(self, native)
        SO101Arm.__init__(self, name, hardware=hardware, **kwargs)

    def configure(self) -> None:
        if self.hardware or self._hardware_active:
            LeRobotSO101Leader.configure(self)

    def _connect_native(self, calibrate: bool) -> None:
        self._check_connection(calibrate)
        LeRobotSO101Leader.connect(self, calibrate=calibrate)

    def _read_hardware(self) -> Values:
        return dict(LeRobotSO101Leader.get_action(self))

    def get_action(self) -> Values:
        return self.observe()

    def send_feedback(self, feedback: Values) -> None:
        self.sync_hardware()
        command = self.validate_controls({**self.controls, **feedback})
        if self._hardware_active:
            LeRobotSO101Leader.send_feedback(self, command)
        else:
            self.controls.update(command)
            _, data = self.world
            data.ctrl[self._actuators] = self._radians(command)

    def calibrate(self) -> None:
        if self.hardware:
            LeRobotSO101Leader.calibrate(self)
