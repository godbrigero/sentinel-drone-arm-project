"""Body-wrench controls for a supplied drone model; no replacement Sentinel CAD."""

from __future__ import annotations

from collections import OrderedDict
from pathlib import Path
from typing import Any, Protocol

import mujoco
import numpy as np
from gymnasium import spaces

from airtruck.systems.base import SimSystem, Values, space_values

CONTROLS = ("thrust_n", "roll_torque_nm", "pitch_torque_nm", "yaw_torque_nm")


class DroneHardware(Protocol):
    """A controller backend in SI units; connecting must not arm an aircraft."""

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def get_observation(self) -> Values: ...
    def send_action(self, action: Values) -> Values: ...


class Drone(SimSystem):
    """A free body with four native site actuators, in body-frame SI units.

    The supplied MJCF must contain thrust_n/roll_torque_nm/pitch_torque_nm/
    yaw_torque_nm actuators, driving a common site with the corresponding gears.
    Real aircraft geometry, inertias and a flight-controller backend are supplied
    by the caller, never synthesized from the manufacturer's weight references.
    """

    def __init__(
        self,
        name: str,
        *,
        model: str | Path,
        root_body: str = "base",
        backend: DroneHardware | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(name, model=model, root_body=root_body, **kwargs)
        self.backend = backend
        self._backend_open = False
        self.controls = dict.fromkeys(CONTROLS, 0.0)
        self.observation_space = spaces.Dict(
            OrderedDict(
                [
                    (
                        "position",
                        spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
                    ),
                    ("quaternion", spaces.Box(-1, 1, shape=(4,), dtype=np.float32)),
                    (
                        "linear_velocity_m_s",
                        spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
                    ),
                    (
                        "angular_velocity_rad_s",
                        spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
                    ),
                ]
            )
        )

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
            [model.actuator(f"{self.name}/{key}").id for key in CONTROLS]
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
        self.action_space = spaces.Dict(
            OrderedDict(
                (
                    key,
                    spaces.Box(
                        np.float32(bounds[0]),
                        np.float32(bounds[1]),
                        shape=(),
                        dtype=np.float32,
                    ),
                )
                for key, bounds in zip(
                    CONTROLS, model.actuator_ctrlrange[self._actuators], strict=True
                )
            )
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
        if self.backend is None:
            raise NotImplementedError(
                "Supply a DroneHardware backend before enabling hardware"
            )
        self._backend_open = True
        self.backend.connect()

    def _disconnect_hardware(self) -> None:
        if self.backend is not None and self._backend_open:
            try:
                self.backend.disconnect()
            finally:
                self._backend_open = False

    def reset(self) -> None:
        self._backend_changed()
        if not self._hardware_active:
            self.controls.update(dict.fromkeys(CONTROLS, 0.0))
        self.observe()
        _, data = self.world
        data.ctrl[self._actuators] = 0

    def step(self, dt: float) -> None:
        if self._hardware_active:
            assert self.backend is not None
            self.backend.send_action(dict(self._command))
        else:
            _, data = self.world
            data.ctrl[self._actuators] = [self._command[key] for key in CONTROLS]

    def observe(self) -> Values:
        model, data = self.world
        if self._hardware_active:
            assert self.backend is not None
            observation = space_values(
                self.observation_space, self.backend.get_observation()
            )
            if not np.isclose(np.linalg.norm(observation["quaternion"]), 1):
                raise ValueError("Drone quaternion must have unit length")
            position, quaternion = observation["position"], observation["quaternion"]
            data.qpos[self._qpos : self._qpos + 7] = np.concatenate(
                [position, quaternion]
            )
            rotation = np.empty(9)
            mujoco.mju_quat2Mat(rotation, quaternion.astype(np.float64))
            data.qvel[self._qvel : self._qvel + 6] = np.concatenate(
                [
                    observation["linear_velocity_m_s"],
                    rotation.reshape(3, 3).T @ observation["angular_velocity_rad_s"],
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
                "position": data.xpos[self._body].astype(np.float32).copy(),
                "quaternion": data.xquat[self._body].astype(np.float32).copy(),
                "linear_velocity_m_s": velocity[3:].astype(np.float32),
                "angular_velocity_rad_s": velocity[:3].astype(np.float32),
            }
        self.state = observation
        return dict(self.state)
