"""One position/yaw flight API for MuJoCo and PX4, in local NED coordinates."""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any, Literal, Protocol, TypedDict

import mujoco
import numpy as np
from numpy.typing import NDArray

from airtruck.systems.apriltag import (
    TAGS,
    AprilTagConfig,
    AprilTagDetector,
    TagBoard,
    TagDetection,
)
from airtruck.systems.base import SimSystem, Values, space_values
from airtruck.systems.camera import FLU_TO_FRD, MUJOCO_TO_OPTICAL, Camera
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


class DroneState(TypedDict):
    """`drone.state` and `get_observation()`: the latest measurement.

    Refreshed by every driver reset/step (and by flight commands). Positions and
    velocities use local NED (north, east, down; heights are negative), relative
    to the simulation world origin or the PX4 estimator origin.
    """

    position: NDArray[np.float32]
    """Body origin [north, east, down] in meters."""
    quaternion: NDArray[np.float32]
    """Attitude, wxyz, rotating body FRD vectors into NED; `yaw_angle()` gives heading."""
    linear_velocity_m_s: NDArray[np.float32]
    """Body-origin velocity [north, east, down] in m/s."""
    angular_velocity_rad_s: NDArray[np.float32]
    """Body rotation rate about the NED axes in rad/s (not body FRD)."""
    armed: bool
    """Motors armed."""
    offboard: bool
    """Offboard (position target) flight mode is active."""
    landed: bool
    """On the ground: supporting contact and below 0.15 m/s, or PX4's land detector."""


class DroneControls(TypedDict):
    """`drone.controls`: the target sent at the next driver step.

    Absolute local NED, the same frame as `DroneState`; never relative to the
    current pose. Flight commands (`takeoff()`, `hold()`, `land()`, resets) set it.
    """

    position: NDArray[np.floating[Any]] | list[float]
    """Target body origin [north, east, down] in meters; -1.0 down = 1 m above origin."""
    yaw: float
    """Target heading in radians, [-pi, pi]: 0 north, positive turns toward east."""


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
    def pop_tag_detections(self) -> list[Values]:
        """New tag poses since the last call: camera, position, quaternion, timestamp_s, tag_id."""
        ...


class Drone(SimSystem[DroneState, DroneControls]):
    """Position targets and flight lifecycle shared by simulation and PX4.

    Public positions/velocities use NED; attitudes rotate body FRD into NED.
    `state` keys are listed in `DroneState`; `controls` keys in `DroneControls`.
    The supplied MuJoCo model uses Z-up NWU and body FLU with native wrench
    actuators. A simulation autopilot tracks targets without pose teleports.

    Optional model features are discovered by name: `rotor_*` velocity actuators
    spin propellers from the allocated wrench, and cameras on the root body become
    `cameras`. Tag detections from the PX4 tag topic (hardware) or opt-in image
    detection (`apriltags=`) are published to `tag_board`, the global `TAGS`.
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
        apriltags: AprilTagConfig | None = None,
        tag_board: TagBoard = TAGS,
        camera_rate_hz: float | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(name, model=model, root_body=root_body, **kwargs)
        if camera_rate_hz is not None and not (
            math.isfinite(camera_rate_hz) and camera_rate_hz > 0
        ):
            raise ValueError("camera_rate_hz must be positive and finite")
        self.apriltags, self.tag_board = apriltags, tag_board
        self.camera_rate_hz = camera_rate_hz
        self._detector = (
            AprilTagDetector(apriltags.family, apriltags.tag_size_m)
            if apriltags
            else None
        )
        self.cameras: dict[str, Camera] = {}
        self._renderers: dict[tuple[int, int], mujoco.Renderer] = {}
        self._detected: dict[str, int] = {}
        self._next_frame = self._next_detection = -math.inf
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
        self._bind_rotors(model)
        self._bind_cameras(model)
        super().bind(model, data)

    def _bind_rotors(self, model: mujoco.MjModel) -> None:
        """Map the aggregate wrench to per-rotor thrust for propeller spin."""
        rotors = [
            index
            for index in range(model.nu)
            if model.actuator(index).name.startswith(f"{self.name}/rotor_")
        ]
        self._rotors = np.array(rotors, dtype=int)
        self._rotor_dofs: NDArray[np.intp] = np.zeros(0, dtype=np.intp)
        if not rotors:
            return
        if model.nuser_actuator < 2:
            raise ValueError(
                "Rotor actuators need user='thrust_coefficient torque_coefficient'"
            )
        columns, coefficients, spins, dofs = [], [], [], []
        for actuator in rotors:
            joint = int(model.actuator_trnid[actuator, 0])
            body = int(model.jnt_bodyid[joint])
            gain = model.actuator_gainprm[actuator, 0]
            low, high = model.actuator_ctrlrange[actuator]
            thrust, torque = model.actuator_user[actuator, :2]
            if (
                model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT
                or model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE
                or model.body_parentid[body] != self._body
                or model.actuator_gaintype[actuator] != mujoco.mjtGain.mjGAIN_FIXED
                or model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_AFFINE
                or not np.allclose(model.actuator_biasprm[actuator, :3], [0, 0, -gain])
                or gain <= 0
                or not model.actuator_ctrllimited[actuator]
            ):
                raise ValueError(
                    "Rotors need velocity actuators on hinges of root children"
                )
            if not (thrust > 0 and torque > 0):
                raise ValueError("Rotor thrust/torque coefficients must be positive")
            if low >= 0 < high:
                spin = 1.0  # Counter-clockwise about the axis toward the shaft tip.
            elif high <= 0 > low:
                spin = -1.0
            else:
                raise ValueError(
                    "A rotor control range must allow only one spin direction"
                )
            orientation = rotation(model.body_quat[body])
            axis = orientation @ model.jnt_axis[joint]
            arm = (
                model.body_pos[body]
                + orientation @ model.jnt_pos[joint]
                - model.body_ipos[self._body]
            )
            # Thrust along the axis; propeller drag reacts against the spin.
            columns.append(
                np.concatenate(
                    [[axis[2]], np.cross(arm, axis) - spin * torque / thrust * axis]
                )
            )
            coefficients.append(thrust)
            spins.append(spin)
            dofs.append(int(model.jnt_dofadr[joint]))
        self._allocation = np.linalg.pinv(np.column_stack(columns))
        self._rotor_thrust = np.array(coefficients)
        self._rotor_spin = np.array(spins)
        self._rotor_dofs = np.asarray(dofs, dtype=np.intp)
        self._rotor_limits = model.actuator_ctrlrange[self._rotors].copy()

    def _bind_cameras(self, model: mujoco.MjModel) -> None:
        self.cameras.clear()
        for index in np.flatnonzero(model.cam_bodyid == self._body):
            name = model.camera(int(index)).name.removeprefix(f"{self.name}/")
            width, height = (int(v) for v in model.cam_resolution[index])
            if (width, height) == (1, 1):  # MuJoCo's unspecified resolution.
                width, height = 640, 480
            self.cameras[name] = Camera(
                name,
                width=width,
                height=height,
                fovy_deg=float(model.cam_fovy[index]),
                rotation=FLU_TO_FRD
                @ rotation(model.cam_quat[index])
                @ MUJOCO_TO_OPTICAL,
                position_m=FLU_TO_FRD @ model.cam_pos[index],
            )
        if self.cameras:
            visual = model.vis.global_
            visual.offwidth = max(
                visual.offwidth, *(c.width for c in self.cameras.values())
            )
            visual.offheight = max(
                visual.offheight, *(c.height for c in self.cameras.values())
            )
        missing = set(self.apriltags.cameras if self.apriltags else ()) - set(
            self.cameras
        )
        if missing:
            raise ValueError(
                f"AprilTag cameras are not on {self.name}: {sorted(missing)}"
            )

    @property
    def rotor_speeds_rad_s(self) -> NDArray[np.float64]:
        """Measured signed propeller speeds about each motor axis."""
        _, data = self.world
        return data.qvel[self._rotor_dofs].copy()

    def _spin_rotors(self, wrench: NDArray[np.float64]) -> None:
        if not len(self._rotors):
            return
        _, data = self.world
        thrust = np.clip(self._allocation @ wrench, 0, None)
        speed = self._rotor_spin * np.sqrt(thrust / self._rotor_thrust)
        data.ctrl[self._rotors] = np.clip(
            speed, self._rotor_limits[:, 0], self._rotor_limits[:, 1]
        )

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

    @property
    def is_connected(self) -> bool:
        if self.hardware or self._hardware_active:
            return self._hardware_active and self._backend_open
        return self._model is not None

    def connect(self) -> None:
        """Open the selected backend now; never arms or changes flight mode.

        Optional: `SystemDriver` connects on entering `with`, or at the first
        `reset()`/`step()`. Use this only to connect earlier when the driver is not
        used as a context manager. The drone must already belong to a driver.
        """
        self.sync_hardware()
        if not self.is_connected:
            raise RuntimeError("Simulation must be bound to a driver")

    def disconnect(self) -> None:
        """Stop communication (also done on driver exit); never lands or disarms."""
        self.close()

    def get_observation(self) -> DroneState:
        return self.observe()

    def send_action(self, action: Values) -> Values:
        """Apply a position/yaw target immediately, without advancing the shared clock."""
        self.world
        self.sync_hardware()
        command = self.validate_controls({**self.controls, **action})
        self.controls["position"], self.controls["yaw"] = (
            command["position"],
            command["yaw"],
        )
        if self._hardware_active:
            return dict(self.backend.send_action(command))
        return command

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
        self._next_frame = self._next_detection = -math.inf
        if not self._hardware_active:
            self._armed = self._offboard = self._landing = False
            self._ground_time = 0.0
            self.tag_board.clear(vehicle=self.name)
            for camera in self.cameras.values():
                camera.frame = camera.timestamp_s = None
        self.observe()
        self._hold_target()
        _, data = self.world
        data.ctrl[self._actuators] = 0
        data.ctrl[self._rotors] = 0
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
        target = np.array(self.controls["position"], dtype=np.float64)
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
            # Visual only: armed propellers spin at hover speed on the proxy.
            model, _ = self.world
            hover = self._mass * float(np.linalg.norm(model.opt.gravity))
            self._spin_rotors(np.array([hover if self._armed else 0.0, 0, 0, 0]))
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
        self._spin_rotors(data.ctrl[self._actuators])

    def observe(self) -> DroneState:
        model, data = self.world
        observation: DroneState
        if self._hardware_active:
            measured = space_values(
                self.observation_space, self.backend.get_observation()
            )
            observation = {
                "position": measured["position"],
                "quaternion": measured["quaternion"],
                "linear_velocity_m_s": measured["linear_velocity_m_s"],
                "angular_velocity_rad_s": measured["angular_velocity_rad_s"],
                "armed": bool(measured["armed"]),
                "offboard": bool(measured["offboard"]),
                "landed": bool(measured["landed"]),
            }
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
                "landed": bool(
                    self._supported() and np.linalg.norm(velocity[3:]) < 0.15
                ),
            }
        self._armed = observation["armed"]
        self._offboard = observation["offboard"]
        self._landed = observation["landed"]
        self.state = observation
        if self._hardware_active:
            for record in self.backend.pop_tag_detections():
                self._publish_tag(record, "ros")
        self._process_cameras()
        return self.state.copy()

    def capture(self, camera: str) -> NDArray[np.uint8]:
        """Render a simulated frame now, or return the latest published hardware frame."""
        feed = self.cameras[camera]
        if self._hardware_active:
            if feed.frame is None:
                raise RuntimeError(
                    f"No frame has been published to {self.name}/{camera}"
                )
            return feed.frame
        model, data = self.world
        size = (feed.width, feed.height)
        if size not in self._renderers:
            self._renderers[size] = mujoco.Renderer(model, feed.height, feed.width)
        renderer = self._renderers[size]
        renderer.update_scene(data, camera=f"{self.name}/{camera}")
        feed.publish(renderer.render(), float(data.time))
        assert feed.frame is not None
        return feed.frame

    def latest_tag(
        self, *, camera: str | None = None, tag_id: int | None = None
    ) -> TagDetection | None:
        return self.tag_board.latest(vehicle=self.name, camera=camera, tag_id=tag_id)

    def _process_cameras(self) -> None:
        _, data = self.world
        now = time.monotonic() if self._hardware_active else float(data.time)
        # The tolerance keeps accumulated timestep rounding from skipping a period.
        due = now + 1e-9
        if (
            self.camera_rate_hz
            and not self._hardware_active
            and due >= self._next_frame
        ):
            self._next_frame = now + 1 / self.camera_rate_hz
            for name in self.cameras:
                self.capture(name)
        if (
            self.apriltags is None
            or self._detector is None
            or due < self._next_detection
        ):
            return
        self._next_detection = now + 1 / self.apriltags.rate_hz
        for name in self.apriltags.cameras or tuple(self.cameras):
            feed = self.cameras[name]
            if not self._hardware_active and feed.timestamp_s != data.time:
                self.capture(name)
            if feed.frame is None or self._detected.get(name) == feed.sequence:
                continue
            self._detected[name] = feed.sequence
            for found in self._detector.detect(feed.frame, feed.intrinsics):
                self._publish_tag(
                    {
                        "camera": name,
                        "tag_id": found.tag_id,
                        "position": found.position_m,
                        "quaternion": found.quaternion,
                        "timestamp_s": feed.timestamp_s,
                        "corners_px": found.corners_px,
                    },
                    "camera" if self._hardware_active else "simulation",
                )

    def _publish_tag(
        self, record: Values, source: Literal["simulation", "camera", "ros"]
    ) -> None:
        position = np.asarray(record["position"], dtype=np.float64)
        quaternion = np.asarray(record["quaternion"], dtype=np.float64)
        if (
            position.shape != (3,)
            or quaternion.shape != (4,)
            or not np.isfinite(position).all()
            or not np.isclose(np.linalg.norm(quaternion), 1, atol=1e-3)
        ):
            raise ValueError(
                "Tag poses need a finite position and unit wxyz quaternion"
            )
        feed = self.cameras.get(str(record["camera"]))
        position_ned = quaternion_ned = None
        if feed is not None:
            vehicle = rotation(self.state["quaternion"])
            position_ned = self.state["position"] + vehicle @ feed.body_point(position)
            tag = vehicle @ feed.rotation @ rotation(quaternion)
            quaternion_ned = np.empty(4)
            mujoco.mju_mat2Quat(quaternion_ned, np.ascontiguousarray(tag).reshape(9))
        tag_id = record.get("tag_id")
        self.tag_board.publish(
            TagDetection(
                vehicle=self.name,
                camera=str(record["camera"]),
                tag_id=None if tag_id is None else int(tag_id),
                position_m=position,
                quaternion=quaternion / np.linalg.norm(quaternion),
                timestamp_s=float(record["timestamp_s"]),
                source=source,
                position_ned_m=position_ned,
                quaternion_ned=quaternion_ned,
                corners_px=record.get("corners_px"),
            )
        )

    def close(self) -> None:
        try:
            for renderer in self._renderers.values():
                renderer.close()
        finally:
            self._renderers.clear()
            super().close()
