"""Native LeRobot exercised against fake buses; physical ports are never opened."""

from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import MagicMock

import mujoco
import numpy as np
import pytest
from lerobot.robots.so_follower import SO101Follower as NativeFollower
from lerobot.teleoperators.so_leader import SO101Leader as NativeLeader

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.drone import Drone
from airtruck.systems.so101 import (
    CONNECT_ATTEMPTS,
    JOINTS,
    SO101Arm,
    SO101Follower,
    SO101Leader,
    release,
)


class FakeBus:
    def __init__(self, arm: SO101Arm, reading: dict[str, float]) -> None:
        self.reading = reading
        self.bus = MagicMock()
        self.bus.is_connected = False
        self.bus.is_calibrated = True
        self.bus.motors = arm.bus.motors
        self.normal_commands: list[dict[str, float]] = []
        self.bus.connect.side_effect = self.connect
        self.bus.disconnect.side_effect = self.disconnect
        self.bus.sync_read.side_effect = self.read
        self.bus.sync_write.side_effect = self.write
        arm.bus = self.bus
        arm.calibration = {
            j: SimpleNamespace(range_min=100, range_max=3000) for j in JOINTS
        }

    def connect(self) -> None:
        self.bus.is_connected = True

    def disconnect(self, **kwargs: Any) -> None:
        self.bus.is_connected = False

    def read(self, name: str, **kwargs: Any) -> dict[str, float]:
        if kwargs.get("normalize") is False:
            return dict.fromkeys(JOINTS, 2000)
        return {j: self.reading[f"{j}.pos"] for j in JOINTS}

    def write(self, name: str, values: dict, **kwargs: Any) -> None:
        if name == "Goal_Position" and kwargs.get("normalize") is not False:
            self.normal_commands.append(dict(values))


def test_bridge_is_native_lerobot_and_keeps_native_calibration_namespaces() -> None:
    follower, leader = SO101Follower("follower"), SO101Leader("leader")
    assert isinstance(follower, NativeFollower) and isinstance(leader, NativeLeader)
    assert follower.calibration_fpath.parent.name == "so_follower"
    assert leader.calibration_fpath.parent.name == "so_leader"
    assert follower.calibration_fpath != leader.calibration_fpath
    assert not follower.config.use_degrees and not leader.config.use_degrees
    assert not follower.bus.is_connected and not leader.bus.is_connected


def test_native_lifecycle_is_simulation_aware() -> None:
    arm = SO101Follower("arm")
    with SystemDriver([arm]) as driver:
        driver.reset()
        fake = FakeBus(arm, dict(arm.state))
        assert arm.is_connected and arm.is_calibrated
        arm.connect(calibrate=True)
        arm.configure()
        arm.calibrate()
        fake.bus.connect.assert_not_called()
        fake.bus.sync_read.assert_not_called()
        fake.bus.sync_write.assert_not_called()


def test_flag_switches_backend_and_mirrors_measured_pose_without_homing() -> None:
    arm = SO101Follower("arm", port="/dev/fake")
    with SystemDriver([arm]) as driver:
        driver.reset()
        reading = dict(arm.state)
        reading["shoulder_pan.pos"] = 10
        fake = FakeBus(arm, reading)
        arm.hardware = True
        driver.step()
        assert arm.hardware_active and fake.bus.is_connected
        assert arm.state == reading and arm.controls == reading
        assert (
            fake.normal_commands == []
        )  # Startup holds; it never sends a simulated home.
        assert driver.data.eq_active[arm._tracking].all()
        assert np.all(driver.model.actuator_gainprm[arm._actuators] == 0)
        calls = fake.bus.method_calls
        hold = next(i for i, c in enumerate(calls) if c[0] == "sync_write")
        mode = next(i for i, c in enumerate(calls) if c[0] == "write")
        assert hold < mode
        arm.controls["shoulder_pan.pos"] = 20
        driver.step()
        assert fake.normal_commands[-1]["shoulder_pan"] == 20
        assert arm.state["shoulder_pan.pos"] == 10  # Feedback, not requested target.
        # Native constraints, not repeated target assignment, hold the physical proxy.
        expected = arm._radians(reading)
        mujoco.mj_step(driver.model, driver.data, nstep=40)
        np.testing.assert_allclose(driver.data.qpos[arm._qpos], expected, atol=0.01)
        driver.reset()
        assert arm.state == reading and arm.controls == reading
        arm.hardware = False
        driver.step()
        assert not arm.hardware_active and not fake.bus.is_connected
        assert not driver.data.eq_active[arm._tracking].any()
        arm.controls["shoulder_pan.pos"] = 15
        count = len(fake.normal_commands)
        driver.step()
        assert len(fake.normal_commands) == count
    fake.bus.disconnect.assert_called_once_with(disable_torque=False)


def test_dropped_startup_packet_retries_the_whole_connect(
    caplog: pytest.LogCaptureFixture,
) -> None:
    arm = SO101Follower("arm", port="/dev/fake")
    with SystemDriver([arm]) as driver:
        driver.reset()
        reading = dict(arm.state)
        fake = FakeBus(arm, reading)
        failures = [ConnectionError("no status packet")]

        def flaky_write(*args: Any, **kwargs: Any) -> None:
            if failures:
                raise failures.pop()

        fake.bus.write.side_effect = flaky_write
        arm.hardware = True
        driver.step()
        assert arm.hardware_active and fake.bus.is_connected
        assert fake.bus.connect.call_count == 2
        assert "arm connect failed (no status packet); retrying 2/3" in caplog.text

        arm.hardware = False
        driver.step()
        fake.bus.connect.reset_mock()
        fake.bus.write.side_effect = ConnectionError("no status packet")
        arm.hardware = True
        with pytest.raises(ConnectionError, match="no status packet"):
            driver.step()
        assert fake.bus.connect.call_count == CONNECT_ATTEMPTS
        assert not arm.hardware_active and not fake.bus.is_connected


@pytest.mark.parametrize("arm_type", [SO101Follower, SO101Leader])
def test_failed_interactive_calibration_is_not_retried(
    arm_type: type[SO101Follower] | type[SO101Leader],
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    arm = arm_type("arm", hardware=True, port="/dev/fake")
    fake = FakeBus(arm, dict.fromkeys(arm.action_features, 0.0))
    fake.bus.is_calibrated = False
    calibration = MagicMock(side_effect=ConnectionError("calibration interrupted"))
    monkeypatch.setattr(arm, "calibrate", calibration)

    with pytest.raises(ConnectionError, match="calibration interrupted"):
        arm.connect(calibrate=True)

    calibration.assert_called_once_with()
    fake.bus.connect.assert_called_once_with()
    fake.bus.disconnect.assert_called_once_with(disable_torque=False)
    assert not arm.hardware_active and not fake.bus.is_connected
    assert not fake.normal_commands


def test_native_send_action_returns_clipped_target_without_stepping() -> None:
    arm = SO101Follower("arm", port="/dev/fake")
    arm.config.max_relative_target = 5.0
    with SystemDriver([arm]) as driver:
        driver.reset()
        reading = dict(arm.state)
        fake = FakeBus(arm, reading)
        arm.hardware = True
        driver.step()
        before = driver.data.time
        applied = arm.send_action({"shoulder_pan.pos": 30})
        assert applied["shoulder_pan.pos"] == 5
        assert fake.normal_commands[-1]["shoulder_pan"] == 5
        assert arm.controls["shoulder_pan.pos"] == 30
        assert driver.data.time == before


def test_real_leader_remains_input_only_and_drives_simulated_follower() -> None:
    leader = SO101Leader(
        "leader", port="/dev/fake", position=(-0.25, 0, 0), follower="follower"
    )
    follower = SO101Follower("follower", position=(0.25, 0, 0))
    with SystemDriver([leader, follower]) as driver:
        driver.reset()
        reading = dict(leader.state)
        reading["shoulder_pan.pos"] = 20
        fake = FakeBus(leader, reading)
        leader.hardware = True
        for _ in range(30):
            driver.step()
        assert leader.get_action() == reading
        assert follower.state["shoulder_pan.pos"] > 15
        assert fake.normal_commands == []


def test_invalid_second_system_never_sends_first_hardware_target() -> None:
    arms = [
        SO101Follower("left", port="/dev/fake1", position=(-0.25, 0, 0)),
        SO101Follower("right", port="/dev/fake2", position=(0.25, 0, 0)),
    ]
    with SystemDriver(arms) as driver:
        driver.reset()
        fakes = [FakeBus(arm, dict(arm.state)) for arm in arms]
        for arm in arms:
            arm.hardware = True
        driver.step()
        before = driver.data.time
        arms[0].controls["shoulder_pan.pos"] = 20
        arms[1].controls["shoulder_pan.pos"] = np.nan
        with pytest.raises(ValueError):
            driver.step()
        assert driver.data.time == before
        assert all(not fake.normal_commands for fake in fakes)


def test_partial_connect_failure_releases_and_closes_everything() -> None:
    arm = SO101Follower("arm", port="/dev/fake", hardware=True)
    fake = FakeBus(arm, {})

    def fail() -> None:
        fake.bus.is_connected = True
        raise OSError("partial startup")

    fake.bus.connect.side_effect = fail
    driver = SystemDriver([arm])
    with pytest.raises(OSError, match="partial startup"):
        with driver:
            pytest.fail("Connection should not succeed")
    assert fake.bus.disable_torque.call_count == 6
    fake.bus.disconnect.assert_called_once_with(disable_torque=False)
    assert not arm.hardware_active and arm._model is None


def test_missing_calibration_and_outside_travel_never_enable() -> None:
    arm = SO101Follower("arm", port="/dev/fake", hardware=True)
    fake = FakeBus(arm, {})
    arm.calibration = {}
    with pytest.raises(ValueError, match="Missing"):
        with SystemDriver([arm]):
            pass
    fake.bus.connect.assert_not_called()
    arm = SO101Follower("arm", port="/dev/fake", hardware=True)
    fake = FakeBus(arm, {})
    fake.bus.sync_read.side_effect = lambda *a, **kw: dict.fromkeys(JOINTS, 3100)
    with pytest.raises(ValueError, match="outside"):
        with SystemDriver([arm]):
            pass
    fake.bus.sync_write.assert_not_called()
    fake.bus.configure_motors.assert_not_called()
    assert not fake.bus.is_connected


def test_cleanup_attempts_all_motors_even_after_one_failure() -> None:
    arm = SO101Follower("arm")
    fake = FakeBus(arm, {})
    fake.bus.is_connected = True
    fake.bus.disable_torque.side_effect = [
        OSError("unreachable"),
        None,
        None,
        None,
        None,
        None,
    ]
    with pytest.raises(ExceptionGroup):
        release(arm)
    assert fake.bus.disable_torque.call_count == 6
    fake.bus.disconnect.assert_called_once_with(disable_torque=False)


def test_drone_backend_tracking_and_partial_failure_cleanup(drone_model: Path) -> None:
    backend = MagicMock()
    backend.get_observation.return_value = {
        "position": np.array([0.4, 0, -0.6], dtype=np.float32),
        "quaternion": np.array([2**-0.5, 0, 0, 2**-0.5], dtype=np.float32),
        "linear_velocity_m_s": np.zeros(3, dtype=np.float32),
        "angular_velocity_rad_s": np.zeros(3, dtype=np.float32),
        "armed": False,
        "offboard": False,
        "landed": False,
    }
    drone = Drone("drone", model=drone_model, backend=backend, position=(0.4, 0, 0.5))
    with SystemDriver([drone]) as driver:
        driver.reset()
        drone.hardware = True
        driver.step()
        np.testing.assert_allclose(drone.state["position"], [0.4, 0, -0.6])
        mujoco.mj_step(driver.model, driver.data, nstep=40)
        np.testing.assert_allclose(driver.data.qpos[:3], [0.4, 0, 0.6], atol=0.003)
        np.testing.assert_allclose(
            driver.data.qpos[3:7],
            backend.get_observation.return_value["quaternion"]
            * np.array([1, 1, -1, -1]),
            atol=0.003,
        )
        drone.controls["position"] = np.array([0.4, 0, -0.8])
        driver.step()
        np.testing.assert_allclose(
            backend.send_action.call_args.args[0]["position"], [0.4, 0, -0.8]
        )
        driver.reset()
        np.testing.assert_allclose(drone.controls["position"], [0.4, 0, -0.6])
        drone.hardware = False
        driver.step()
        backend.disconnect.assert_called_once()
    backend = MagicMock()
    backend.connect.side_effect = OSError("partial drone startup")
    drone = Drone("drone", model=drone_model, backend=backend, hardware=True)
    with pytest.raises(OSError):
        with SystemDriver([drone]):
            pass
    backend.disconnect.assert_called_once()


def test_no_silent_hardware_fallback_or_fast_hardware_loop(drone_model: Path) -> None:
    backend = MagicMock()
    backend.connect.side_effect = OSError("PX4 unavailable")
    drone = Drone("drone", model=drone_model, hardware=True, backend=backend)
    with pytest.raises(OSError, match="PX4 unavailable"):
        with SystemDriver([drone]):
            pass
    arm = SO101Follower("arm", port="/dev/fake")
    with SystemDriver([arm]) as driver:
        driver.reset()
        FakeBus(arm, dict(arm.state))
        arm.hardware = True
        with pytest.raises(ValueError, match="real time"):
            driver.run(steps=1, realtime=False)
