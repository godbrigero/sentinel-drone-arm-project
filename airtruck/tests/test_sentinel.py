"""The prepared Sentinel model: rotors, onboard cameras and simulated AprilTags."""

import mujoco
import numpy as np
import pytest

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.apriltag import (
    TAGS,
    AprilTagConfig,
    TagBoard,
    TagDetection,
    add_apriltag,
)
from airtruck.systems.base import ROOT
from airtruck.systems.sentinel import MODEL, SPAWN_HEIGHT_M, SentinelDrone
from airtruck.systems.flight import rotation, yaw_angle

ARENA = ROOT / "worlds/flight_arena/scene.xml"


def advance(driver: SystemDriver, seconds: float) -> None:
    for _ in range(round(seconds / driver.control_dt)):
        driver.step()
    assert not np.any(driver.data.warning.number)


def test_sentinel_defaults_to_simulation_with_its_built_in_model() -> None:
    drone = SentinelDrone("sentinel")
    assert not drone.hardware and drone.model_path == MODEL
    assert drone.position == (0, 0, SPAWN_HEIGHT_M) == (0, 0, 0.146)
    assert not drone.is_connected
    with SystemDriver([drone], world=ARENA) as driver:
        drone.connect()  # Simulation: bound to the driver, no backend opened.
        assert drone.is_connected and not drone.hardware_active
        driver.reset()
        accepted = drone.send_action({"yaw": 0.5})
        assert accepted["yaw"] == pytest.approx(0.5)
        assert drone.controls["yaw"] == pytest.approx(0.5)
        assert driver.data.time == 0  # Commands never advance the shared clock.
        assert drone.get_observation().keys() == drone.state.keys()
        drone.disconnect()
        assert not drone.is_connected


def test_model_cameras_rotors_and_mass() -> None:
    drone = SentinelDrone("sentinel")
    with SystemDriver([drone], world=ARENA) as driver:
        driver.reset()
        model = driver.model
        body = model.body("sentinel/base").id
        assert model.body_subtreemass[body] == pytest.approx(1.407)
        # Landing feet support the aircraft on the pad (8 mm) after settling.
        advance(driver, 0.5)
        assert drone.state["landed"]
        assert -drone.state["position"][2] == pytest.approx(0.1448 + 0.008, abs=0.002)
        assert set(drone.cameras) == {"front_down", "rear_left"}
        front, rear = drone.cameras["front_down"], drone.cameras["rear_left"]
        # Optical axes in body FRD: forward/down, and straight back.
        np.testing.assert_allclose(
            front.rotation[:, 2], [0.69, -0.007, 0.724], atol=0.01
        )
        np.testing.assert_allclose(rear.rotation[:, 2], [-1, 0.009, -0.024], atol=0.01)
        assert front.position_m[0] > 0.08 and abs(front.position_m[1]) < 0.002
        assert rear.position_m[0] < -0.03 and rear.position_m[1] < -0.03  # FRD left.
        assert (front.width, front.height) == (640, 480)
        assert front.intrinsics[0, 0] == pytest.approx(276.1, abs=0.1)
        image = drone.capture("rear_left")
        assert image.shape == (480, 640, 3) and image.dtype == np.uint8
        assert rear.timestamp_s == driver.data.time
        # CW rotors 1 and 3 may only spin negatively about their motor axis.
        limits = model.actuator_ctrlrange[drone._rotors]
        assert np.all(limits[[0, 2], 1] == 0) and np.all(limits[[1, 3], 0] == 0)


def test_rotors_spin_with_the_allocated_wrench() -> None:
    drone = SentinelDrone("sentinel")
    with SystemDriver([drone], world=ARENA) as driver:
        driver.reset()
        np.testing.assert_array_equal(drone.rotor_speeds_rad_s, 0)
        drone.takeoff(0.8)
        advance(driver, 5)
        assert -drone.state["position"][2] == pytest.approx(0.946 + 0.008, abs=0.03)
        speeds = drone.rotor_speeds_rad_s
        # Hover: equal magnitudes, CW (1, 3) negative and CCW (2, 4) positive.
        hover = np.sqrt(1.407 * 9.81 / 4 / 1.485e-5)
        np.testing.assert_allclose(speeds * [-1, 1, -1, 1], hover, rtol=0.03)
        # Positive NED yaw turns clockwise from above: CCW rotors speed up.
        drone.controls["yaw"] = 0.8
        advance(driver, 0.2)
        assert abs(drone.rotor_speeds_rad_s[1]) > abs(drone.rotor_speeds_rad_s[0])
        advance(driver, 5)
        assert yaw_angle(drone.state["quaternion"]) == pytest.approx(0.8, abs=0.03)
        drone.land()
        advance(driver, 7)
        assert drone.state["landed"] and not drone.state["armed"]
        np.testing.assert_allclose(drone.rotor_speeds_rad_s, 0, atol=1)


def test_simulated_tags_are_optional_and_published_globally() -> None:
    # An upright tag 1.2 m behind the second aircraft's rear-left camera, facing
    # it: face (+Z) along world +X, printed "up" (+Y) along world +Z.
    spec = mujoco.MjSpec.from_file(str(ARENA))
    facing = np.empty(4)
    mujoco.mju_mat2Quat(facing, np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0.0]]).ravel())
    add_apriltag(spec, 6, size_m=0.2, position=(-1.2, 2.04, 0.2), quaternion=facing)
    plain = SentinelDrone("plain")
    seeing = SentinelDrone(
        "seeing",
        position=(0, 2, SPAWN_HEIGHT_M),
        apriltags=AprilTagConfig(tag_size_m=0.2, cameras=("rear_left",)),
    )
    received: list[TagDetection] = []
    unsubscribe = TAGS.subscribe(received.append)
    try:
        with SystemDriver([plain, seeing], world=spec) as driver:
            driver.reset()
            driver.step()
            assert plain.latest_tag() is None  # Detection is opt-in.
            detection = TAGS.latest(vehicle="seeing", tag_id=6)
            assert detection is not None and detection.source == "simulation"
            assert detection is seeing.latest_tag(camera="rear_left")
            assert received and received[-1].vehicle == "seeing"
            # Tag center in NED: x north = world x, y east = -world y, z = -height.
            assert detection.position_ned_m is not None
            np.testing.assert_allclose(
                detection.position_ned_m, [-1.1995, -2.04, -0.2], atol=0.03
            )
            assert detection.distance_m == pytest.approx(1.17, abs=0.03)
            # The tag face (+Z) points back toward the aircraft: north in NED.
            assert detection.quaternion_ned is not None
            assert rotation(detection.quaternion_ned)[:, 2] @ [1, 0, 0] > 0.98
            advance(driver, 0.1)  # The next 10 Hz detection period.
            later = seeing.latest_tag()
            assert later is not None and later.timestamp_s == pytest.approx(0.1)
            driver.reset()
            # Resets clear old tags; the still-visible tag is re-detected at t=0.
            fresh = seeing.latest_tag()
            assert fresh is not None and fresh is not detection
            assert fresh.timestamp_s == 0
    finally:
        unsubscribe()


def test_tag_board_filters_and_unsubscribes() -> None:
    board = TagBoard()
    seen: list[TagDetection] = []
    stop = board.subscribe(seen.append)
    for time_s, camera in ((1.0, "a"), (2.0, "b")):
        board.publish(
            TagDetection(
                vehicle="v",
                camera=camera,
                tag_id=1,
                position_m=np.zeros(3),
                quaternion=np.array([1.0, 0, 0, 0]),
                timestamp_s=time_s,
                source="ros",
            )
        )
    latest = board.latest(vehicle="v")
    assert latest is not None and latest.camera == "b"
    assert len(board.detections(tag_id=1)) == 2 and not board.detections(tag_id=2)
    stop()
    board.clear(vehicle="v")
    assert board.latest() is None and len(seen) == 2
