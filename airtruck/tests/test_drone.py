"""Flight behavior is exercised through native MuJoCo, not pose assignment."""

from pathlib import Path

import numpy as np
import pytest

from airtruck.simulation.driver import MujocoDriver
from airtruck.systems.drone import Drone
from airtruck.systems.flight import (
    FlightController,
    frame_quaternion,
    rotation,
    yaw_angle,
)


def advance(driver: MujocoDriver, seconds: float) -> None:
    for _ in range(round(seconds / driver.control_dt)):
        driver.step()
    assert not np.any(driver.data.warning.number)


def test_takeoff_position_yaw_hold_and_land_on_native_floor(drone_model: Path) -> None:
    drone = Drone("drone", model=drone_model, position=(0.4, 0, 0.03))
    with MujocoDriver([drone]) as driver:
        driver.reset()
        assert not drone.state["armed"]
        before = driver.data.qpos.copy()
        drone.takeoff(0.5)
        np.testing.assert_array_equal(driver.data.qpos, before)
        assert driver.data.time == 0
        assert drone.state["armed"] and drone.state["offboard"]
        advance(driver, 5)
        np.testing.assert_allclose(drone.state["position"], [0.4, 0, -0.53], atol=0.03)
        drone.controls.update(position=np.array([0.8, 0.2, -0.7]), yaw=0.6)
        advance(driver, 7)
        np.testing.assert_allclose(drone.state["position"], [0.8, 0.2, -0.7], atol=0.03)
        assert yaw_angle(drone.state["quaternion"]) == pytest.approx(0.6, abs=0.03)
        assert driver.data.xpos[drone._body, 1] < 0  # NED east is world -Y.
        drone.hold()
        held = drone.controls["position"].copy()
        advance(driver, 2)
        np.testing.assert_allclose(drone.state["position"], held, atol=0.03)
        drone.land()
        assert not drone.state["offboard"]
        advance(driver, 6)
        assert drone.state["landed"] and not drone.state["armed"]
        assert drone.state["position"][2] == pytest.approx(-0.03, abs=0.003)
        assert driver.info["contacts"]
        assert np.all(driver.data.ctrl[drone._actuators] == 0)


def test_yaw_pi_and_hold_interrupt_landing(drone_model: Path) -> None:
    drone = Drone("drone", model=drone_model, position=(0.4, 0, 0.5))
    with MujocoDriver([drone]) as driver:
        driver.reset()
        drone.engage_offboard_mode()
        drone.arm()
        drone.controls["yaw"] = np.pi
        advance(driver, 4)
        assert abs(yaw_angle(drone.state["quaternion"])) == pytest.approx(
            np.pi, abs=0.03
        )
        drone.land()
        advance(driver, 0.3)
        drone.hold()
        assert drone.state["offboard"]
        target = drone.controls["position"].copy()
        advance(driver, 3)
        np.testing.assert_allclose(drone.state["position"], target, atol=0.03)
        drone.disarm()
        advance(driver, 2)
        assert drone.state["landed"] and not drone.state["armed"]


def test_flight_commands_validate_before_changing_mode(drone_model: Path) -> None:
    drone = Drone("drone", model=drone_model, position=(0.4, 0, 0.5))
    with MujocoDriver([drone]) as driver:
        driver.reset()
        for height in (0, -1, np.nan, np.inf):
            with pytest.raises(ValueError):
                drone.takeoff(height)
        with pytest.raises(RuntimeError, match="Arm"):
            drone.land()
        assert not drone.state["armed"] and not drone.state["offboard"]
        drone.controls["yaw"] = np.nan
        with pytest.raises(ValueError):
            drone.engage_offboard_mode()
        assert driver.data.time == 0


def test_frames_round_trip_with_nonzero_roll_pitch_and_yaw() -> None:
    quaternion = np.array([0.8, 0.2, -0.3, 0.4])
    quaternion /= np.linalg.norm(quaternion)
    converted = frame_quaternion(quaternion)
    transform = np.diag([1, -1, -1])
    np.testing.assert_allclose(
        rotation(converted), transform @ rotation(quaternion) @ transform
    )
    np.testing.assert_allclose(frame_quaternion(converted), quaternion)
    with pytest.raises(ValueError):
        FlightController(max_tilt_rad=np.pi)
