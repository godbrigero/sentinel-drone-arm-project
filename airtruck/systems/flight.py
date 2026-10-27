"""Shared PX4-style flight schemas and explicit NED/FRD ↔ Z-up/FLU transforms."""

from collections import OrderedDict
from dataclasses import dataclass

import mujoco
import numpy as np
from gymnasium import spaces
from numpy.typing import NDArray

NED_TO_WORLD = np.array([1.0, -1.0, -1.0])
WRENCH_CHANNELS = ("thrust_n", "roll_torque_nm", "pitch_torque_nm", "yaw_torque_nm")


def flight_action_space() -> spaces.Dict:
    return spaces.Dict(
        OrderedDict(
            [
                ("position", spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32)),
                ("yaw", spaces.Box(-np.pi, np.pi, shape=(), dtype=np.float32)),
            ]
        )
    )


def flight_observation_space() -> spaces.Dict:
    return spaces.Dict(
        OrderedDict(
            [
                ("position", spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32)),
                ("quaternion", spaces.Box(-1, 1, shape=(4,), dtype=np.float32)),
                (
                    "linear_velocity_m_s",
                    spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
                ),
                (
                    "angular_velocity_rad_s",
                    spaces.Box(-np.inf, np.inf, shape=(3,), dtype=np.float32),
                ),
                *(
                    (key, spaces.Box(0, 1, shape=(), dtype=np.float32))
                    for key in ("armed", "offboard", "landed")
                ),
            ]
        )
    )


def rotation(quaternion: np.ndarray) -> np.ndarray:
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, np.asarray(quaternion, dtype=np.float64))
    return result.reshape(3, 3)


def frame_quaternion(quaternion: np.ndarray) -> np.ndarray:
    """Convert either direction: NED/FRD ↔ world NWU/body FLU (wxyz)."""
    return np.asarray(quaternion) * [1, 1, -1, -1]


def yaw_angle(quaternion: np.ndarray) -> float:
    matrix = rotation(quaternion)
    return float(np.arctan2(matrix[1, 0], matrix[0, 0]))


@dataclass(frozen=True)
class FlightController:
    """Illustrative simulation tuning, not a reproduction of PX4 firmware gains."""

    position_gain: float = 1.0
    velocity_gain: float = 1.0
    attitude_gain: float = 36.0
    rate_gain: float = 12.0
    max_speed_m_s: float = 1.0
    max_acceleration_m_s2: float = 3.0
    max_tilt_rad: float = 0.5
    landing_speed_m_s: float = 0.3

    def __post_init__(self) -> None:
        if any(not np.isfinite(value) or value <= 0 for value in vars(self).values()):
            raise ValueError("Flight controller settings must be positive and finite")
        if self.max_tilt_rad >= np.pi / 2:
            raise ValueError("max_tilt_rad must be less than pi/2")

    def wrench(
        self,
        *,
        position_error: np.ndarray,
        velocity: np.ndarray,
        attitude: np.ndarray,
        angular_velocity: np.ndarray,
        yaw: float,
        mass: float,
        inertia: np.ndarray,
        gravity: np.ndarray,
        landing: bool,
    ) -> NDArray[np.float64]:
        """Position/velocity → thrust direction → attitude/rate → native wrench."""
        desired_velocity = self.position_gain * position_error
        desired_velocity *= min(
            1.0, self.max_speed_m_s / max(float(np.linalg.norm(desired_velocity)), 1e-9)
        )
        if landing:
            desired_velocity[2] = -self.landing_speed_m_s  # Z-up world.
        acceleration = self.velocity_gain * (desired_velocity - velocity)
        acceleration *= min(
            1.0,
            self.max_acceleration_m_s2 / max(float(np.linalg.norm(acceleration)), 1e-9),
        )
        force = mass * (acceleration - gravity)
        horizontal = np.linalg.norm(force[:2])
        force[:2] *= min(
            1.0, max(force[2], 0) * np.tan(self.max_tilt_rad) / max(horizontal, 1e-9)
        )
        z_axis = force / max(np.linalg.norm(force), 1e-9)
        if np.linalg.norm(z_axis) < 0.5:
            z_axis = np.array([0.0, 0.0, 1.0])
        heading = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        y_axis = np.cross(z_axis, heading)
        y_axis /= np.linalg.norm(y_axis)
        desired = np.column_stack([np.cross(y_axis, z_axis), y_axis, z_axis])
        error_quaternion = np.empty(4, dtype=np.float64)
        relative_rotation: NDArray[np.float64] = np.ascontiguousarray(
            attitude.T @ desired, dtype=np.float64
        )
        mujoco.mju_mat2Quat(error_quaternion, relative_rotation.reshape(9))
        if error_quaternion[0] < 0:
            error_quaternion *= -1
        sine = float(np.linalg.norm(error_quaternion[1:]))
        error = error_quaternion[1:] * (
            2 * np.arctan2(sine, error_quaternion[0]) / max(sine, 1e-9)
        )
        torque = inertia @ (
            self.attitude_gain * error - self.rate_gain * angular_velocity
        )
        torque += np.cross(angular_velocity, inertia @ angular_velocity)
        wrench = np.empty(4, dtype=np.float64)
        wrench[0] = max(0.0, float(force @ attitude[:, 2]))
        wrench[1:] = torque
        return wrench
