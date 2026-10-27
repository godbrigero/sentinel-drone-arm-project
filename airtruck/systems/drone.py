"""One position/yaw flight API for MuJoCo and PX4, in local NED coordinates."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Protocol

import mujoco
import numpy as np
from numpy.typing import NDArray

from airtruck.systems.base import SimSystem, Values, space_values
from airtruck.systems.flight import (
    FlightController,
    NED_TO_WORLD,
    WRENCH_CHANNELS,
    flight_action_space,
    flight_observation_space,
    frame_quaternion,
    rotation,
    yaw_angle,
)
from airtruck.systems.px4 import PX4Backend, PX4Config


class DroneHardware(Protocol):
    """Local NED/FRD telemetry and position/yaw flight commands."""

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def get_observation(self) -> Values: ...
    def send_action(self, action: Values) -> Values: ...
    def engage_offboard_mode(self) -> None: ...
    def arm(self) -> None: ...
    def disarm(self) -> None: ...
    def land(self) -> None: ...


class Drone(SimSystem):
    """Position targets and flight lifecycle shared by simulation and PX4.

    Public positions/velocities use NED; attitudes rotate body FRD into NED.
    The supplied MuJoCo model uses Z-up NWU and body FLU with native wrench
    actuators. A simulation autopilot tracks targets without pose teleports.
    """

    def __init__(
        self,
        name: str,
        *,
        model: str | Path,
        root_body: str = "base",
        backend: DroneHardware | None = None,
        px4: PX4Config | None = None,
        controller: FlightController | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(name, model=model, root_body=root_body, **kwargs)
        self.backend = backend if backend is not None else PX4Backend(name, px4)
        self.controller = controller or FlightController()
        self._backend_open = False
        self._armed, self._offboard, self._landed, self._landing = (
            False,
            False,
            False,
            False,
        )
        self._ground_time = 0.0
        self.action_space = flight_action_space()
        self.observation_space = flight_observation_space()
        self.controls = {"position": np.zeros(3), "yaw": 0.0}

    def attach(self, world: mujoco.MjSpec) -> None:
        super().attach(world)
        # World-referenced welds must be added to the parent after attachment.
        world.add_equality(
            name=f"{self.name}/hardware_pose",
            type=mujoco.mjtEq.mjEQ_WELD,
            objtype=mujoco.mjtObj.mjOBJ_BODY,
            name1=f"{self.name}/{self.root_body}",
            active=False,
            data=[0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1],
            solref=[0.002, 1],
        )

    def bind(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._body = model.body(f"{self.name}/{self.root_body}").id
        joint = int(model.body_jntadr[self._body])
        if (
            model.body_jntnum[self._body] != 1
            or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_FREE
        ):
            raise ValueError("Drone root must have one free joint")
        self._qpos, self._qvel = (
            int(model.jnt_qposadr[joint]),
            int(model.jnt_dofadr[joint]),
        )
        self._actuators = np.array(
            [model.actuator(f"{self.name}/{key}").id for key in WRENCH_CHANNELS]
        )
        if (
            not np.all(
                model.actuator_gaintype[self._actuators] == mujoco.mjtGain.mjGAIN_FIXED
            )
            or not np.allclose(model.actuator_gainprm[self._actuators, 0], 1)
            or not np.all(
                model.actuator_biastype[self._actuators] == mujoco.mjtBias.mjBIAS_NONE
            )
            or not np.all(
                model.actuator_dyntype[self._actuators] == mujoco.mjtDyn.mjDYN_NONE
            )
        ):
            raise ValueError(
                "Drone wrench inputs require stateless unit-gain motor actuators"
            )
        expected = np.array(
            [
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )
        if not np.all(
            model.actuator_trntype[self._actuators] == mujoco.mjtTrn.mjTRN_SITE
        ) or not np.allclose(model.actuator_gear[self._actuators], expected):
            raise ValueError(
                "Drone actuators must apply body-local thrust and torques at a site"
            )
        sites = model.actuator_trnid[self._actuators, 0]
        if len(set(sites)) != 1 or model.site_bodyid[sites[0]] != self._body:
            raise ValueError(
                "Drone wrench actuators need a common site on the root body"
            )
        if not np.allclose(model.site_quat[sites[0]], [1, 0, 0, 0]) or not np.allclose(
            model.site_pos[sites[0]], model.body_ipos[self._body]
        ):
            raise ValueError(
                "Drone wrench site must be at the center of mass and aligned with the body"
            )
        self._tracking = model.eq(f"{self.name}/hardware_pose").id
        self._gains = model.actuator_gainprm[self._actuators].copy()
        self._biases = model.actuator_biasprm[self._actuators].copy()
        self._limits = model.actuator_ctrlrange[self._actuators].copy()
        inertial_rotation = rotation(model.body_iquat[self._body])
        self._inertia = (
            inertial_rotation
            @ np.diag(model.body_inertia[self._body])
            @ inertial_rotation.T
        )
        self._mass = float(model.body_subtreemass[self._body])
        self._geoms = np.array(
            [
                model.body(int(body)).name.startswith(f"{self.name}/")
                for body in model.geom_bodyid
            ]
        )
        if (
            not np.all(model.actuator_ctrllimited[self._actuators])
            or not np.isfinite(model.actuator_ctrlrange[self._actuators]).all()
        ):
            raise ValueError("Drone actuator control ranges must be finite and limited")
        super().bind(model, data)

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

    def _connect_hardware(self) -> None:
        self._backend_open = True
        self.backend.connect()

    def _disconnect_hardware(self) -> None:
        if self._backend_open:
            try:
                self.backend.disconnect()
            finally:
                self._backend_open = False

    def _hold_after_switch(self) -> None:
        # Backend selection must never engage a flight mode or arm a vehicle.
        self._hold_target()

    def _hold_target(self) -> None:
        if self.state:
            self.controls = {
                "position": self.state["position"].copy(),
                "yaw": yaw_angle(self.state["quaternion"]),
            }

    def reset(self) -> None:
        self._backend_changed()
        if not self._hardware_active:
            self._armed = self._offboard = self._landing = False
            self._ground_time = 0.0
        self.observe()
        self._hold_target()
        _, data = self.world
        data.ctrl[self._actuators] = 0
        if self._hardware_active:
            self.backend.send_action(self.validate_controls())

    def engage_offboard_mode(self) -> None:
        self.world  # Require a bound object before any flight command.
        self.sync_hardware()
        command = self.validate_controls()
        if self._hardware_active:
            self.backend.send_action(command)
            self.backend.engage_offboard_mode()
            self.observe()
        else:
            self._offboard = True
            self.observe()
        self._landing = False

    def arm(self) -> None:
        self.world
        self.sync_hardware()
        self.validate_controls()
        if self._hardware_active:
            self.backend.arm()
        else:
            self._armed = True
        self.observe()

    def disarm(self) -> None:
        self.world
        self.sync_hardware()
        if self._hardware_active:
            self.backend.disarm()
        else:
            self._armed = False
        self._landing = False
        self.observe()

    def takeoff(self, height: float = 1.0) -> None:
        """Request a climb by height meters from the measured position; do not step."""
        if not np.isfinite(height) or height <= 0:
            raise ValueError("Takeoff height must be positive and finite")
        self.world
        self.sync_hardware()
        self.observe()
        self._hold_target()
        target = self.controls["position"].copy()
        target[2] -= height
        self.engage_offboard_mode()
        self.arm()
        self.controls["position"] = target

    def hold(self) -> None:
        """Hold measured position/yaw; resume offboard control if already armed."""
        self.world
        self.sync_hardware()
        self.observe()
        self._hold_target()
        if self._armed:
            self.engage_offboard_mode()

    def land(self) -> None:
        """Request native PX4 landing or descend to physical contact in simulation."""
        self.world
        self.sync_hardware()
        self.observe()
        if not self._armed:
            raise RuntimeError("Arm the drone before requesting landing")
        self._hold_target()
        if self._hardware_active:
            self.backend.land()
            self.observe()
        else:
            self._offboard = False
        self._landing, self._ground_time = True, 0.0
        self.observe()

    def _supported(self) -> bool:
        _, data = self.world
        for index in range(data.ncon):
            contact = data.contact[index]
            first, second = self._geoms[contact.geom1], self._geoms[contact.geom2]
            if first != second and contact.dist <= 0.002:
                normal = contact.frame[:3] * (1 if second else -1)
                if normal[2] > 0.5:
                    return True
        return False

    def step(self, dt: float) -> None:
        if self._hardware_active:
            self.backend.send_action(dict[str, Any](self._command))
            return
        model, data = self.world
        if self._landing and self._landed:
            self._ground_time += dt
            if self._ground_time >= 1.0:
                self._armed = self._offboard = self._landing = False
        else:
            self._ground_time = 0.0
        wrench: NDArray[np.float64] = np.zeros(4)
        if self._armed and (self._offboard or self._landing):
            attitude = data.xmat[self._body].reshape(3, 3)
            wrench = self.controller.wrench(
                position_error=NED_TO_WORLD
                * (self._command["position"] - self.state["position"]),
                velocity=NED_TO_WORLD * self.state["linear_velocity_m_s"],
                attitude=attitude,
                angular_velocity=data.qvel[self._qvel + 3 : self._qvel + 6],
                yaw=-float(self._command["yaw"]),
                mass=self._mass,
                inertia=self._inertia,
                gravity=model.opt.gravity,
                landing=self._landing,
            )
        data.ctrl[self._actuators] = np.clip(
            wrench, self._limits[:, 0], self._limits[:, 1]
        )

    def observe(self) -> Values:
        model, data = self.world
        if self._hardware_active:
            observation = space_values(
                self.observation_space, self.backend.get_observation()
            )
            if not np.isclose(np.linalg.norm(observation["quaternion"]), 1):
                raise ValueError("Drone quaternion must have unit length")
            position = NED_TO_WORLD * observation["position"]
            quaternion = frame_quaternion(observation["quaternion"])
            data.qpos[self._qpos : self._qpos + 7] = np.concatenate(
                [position, quaternion]
            )
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, quaternion.astype(np.float64))
            data.qvel[self._qvel : self._qvel + 6] = np.concatenate(
                [
                    NED_TO_WORLD * observation["linear_velocity_m_s"],
                    rotation.reshape(3, 3).T
                    @ (NED_TO_WORLD * observation["angular_velocity_rad_s"]),
                ]
            )
            inverse_position, inverse_quaternion = np.empty(3), np.empty(4)
            mujoco.mju_negPose(
                inverse_position,
                inverse_quaternion,
                position.astype(np.float64),
                quaternion.astype(np.float64),
            )
            model.eq_data[self._tracking, 3:6] = inverse_position
            model.eq_data[self._tracking, 6:10] = inverse_quaternion
        else:
            velocity = np.empty(6)
            mujoco.mj_objectVelocity(
                model, data, mujoco.mjtObj.mjOBJ_XBODY, self._body, velocity, 0
            )
            observation = {
                "position": (NED_TO_WORLD * data.xpos[self._body]).astype(np.float32),
                "quaternion": frame_quaternion(data.xquat[self._body]).astype(
                    np.float32
                ),
                "linear_velocity_m_s": (NED_TO_WORLD * velocity[3:]).astype(np.float32),
                "angular_velocity_rad_s": (NED_TO_WORLD * velocity[:3]).astype(
                    np.float32
                ),
                "armed": self._armed,
                "offboard": self._offboard,
                "landed": self._supported() and np.linalg.norm(velocity[3:]) < 0.15,
            }
        self._armed, self._offboard, self._landed = (
            bool(observation[key]) for key in ("armed", "offboard", "landed")
        )
        self.state = observation
        return dict(self.state)
