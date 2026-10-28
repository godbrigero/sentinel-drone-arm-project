"""Exercise example wiring with native simulation and no GUI or physical I/O."""

from collections.abc import Callable, Sequence
from dataclasses import replace
from types import ModuleType
from typing import Any

import numpy as np
import pytest

from airtruck.examples import (
    drone_1m_in_front_tag,
    drone_takeoff_fly_forward_land,
    drone_takeoff_land,
    one_arm,
)
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.apriltag import TagBoard, TagDetection
from airtruck.systems.base import SimSystem
from airtruck.systems.px4 import PX4Backend
from airtruck.systems.sentinel import SentinelDrone
from airtruck.systems.so101 import SO101Arm


def headless_example(
    monkeypatch: pytest.MonkeyPatch, module: ModuleType
) -> list[SystemDriver]:
    drivers: list[SystemDriver] = []

    def driver_factory(
        systems: Sequence[SimSystem], **kwargs: Any
    ) -> SystemDriver:
        assert all(not system.hardware for system in systems)
        kwargs["render_mode"] = None
        driver = SystemDriver(systems, **kwargs)
        drivers.append(driver)
        return driver

    def reject_hardware(*args: Any, **kwargs: Any) -> None:
        pytest.fail("Example tried to open physical hardware during simulation")

    monkeypatch.setattr(module, "IS_HARDWARE", False)
    monkeypatch.setattr(module, "SystemDriver", driver_factory)
    monkeypatch.setattr(PX4Backend, "connect", reject_hardware)
    monkeypatch.setattr(SO101Arm, "_connect_hardware", reject_hardware)
    return drivers


@pytest.mark.parametrize(
    "module",
    [one_arm, drone_takeoff_land, drone_takeoff_fly_forward_land, drone_1m_in_front_tag],
    ids=["linked-arms", "takeoff-land", "position-flight", "tag-standoff"],
)
def test_example_constructs_steps_and_closes_in_simulation(
    monkeypatch: pytest.MonkeyPatch, module: ModuleType
) -> None:
    drivers = headless_example(monkeypatch, module)
    original_run = SystemDriver.run

    def short_run(
        driver: SystemDriver,
        *,
        update: Callable[[], None] | None = None,
        realtime: bool = True,
        steps: int | None = None,
    ) -> None:
        original_run(driver, update=update, realtime=False, steps=3)

    monkeypatch.setattr(SystemDriver, "run", short_run)
    module.main()

    assert len(drivers) == 1
    driver = drivers[0]
    assert driver.data.time > 0
    assert not np.any(driver.data.warning.number)
    assert np.isfinite(driver.data.qpos).all()
    assert all(not system.hardware_active for system in driver.systems)
    with pytest.raises(RuntimeError, match="closed"):
        driver.step()


@pytest.mark.parametrize("fail_tracking", [False, True], ids=["landing", "exception"])
def test_tag_example_filters_vehicle_and_always_removes_its_callback(
    monkeypatch: pytest.MonkeyPatch, fail_tracking: bool
) -> None:
    drivers = headless_example(monkeypatch, drone_1m_in_front_tag)
    board = TagBoard()

    def drone_factory(*args: Any, **kwargs: Any) -> SentinelDrone:
        kwargs.update(tag_board=board, apriltags=None)
        return SentinelDrone(*args, **kwargs)

    monkeypatch.setattr(drone_1m_in_front_tag, "SentinelDrone", drone_factory)
    detection = TagDetection(
        vehicle="drone",
        camera="front_down",
        tag_id=2,
        position_m=np.array([0.0, 0.0, 2.0]),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
        timestamp_s=1.0,
        source="simulation",
        position_ned_m=np.array([2.0, 1.0, 0.0]),
    )
    calls = 0

    def run_phase(driver: SystemDriver, **kwargs: Any) -> None:
        nonlocal calls
        calls += 1
        drone = driver.systems[0]
        assert isinstance(drone, SentinelDrone)
        before = np.array(drone.controls["position"], copy=True)
        if calls == 2:
            board.publish(replace(detection, vehicle="another-drone"))
            np.testing.assert_array_equal(drone.controls["position"], before)
            board.publish(detection)
            assert not np.array_equal(drone.controls["position"][:2], before[:2])
            assert drone.controls["position"][2] == before[2]
            if fail_tracking:
                raise RuntimeError("tracking interrupted")
        elif calls == 3:
            # Landing must no longer accept horizontal tag-following commands.
            assert not drone.state["offboard"]
            board.publish(replace(detection, position_ned_m=np.array([8.0, 9.0, 0.0])))
            np.testing.assert_array_equal(drone.controls["position"], before)

    monkeypatch.setattr(SystemDriver, "run", run_phase)
    if fail_tracking:
        with pytest.raises(RuntimeError, match="tracking interrupted"):
            drone_1m_in_front_tag.main()
    else:
        drone_1m_in_front_tag.main()

    assert calls == (2 if fail_tracking else 3)
    drone = drivers[0].systems[0]
    before = np.array(drone.controls["position"], copy=True)
    board.publish(replace(detection, position_ned_m=np.array([20.0, 20.0, 0.0])))
    np.testing.assert_array_equal(drone.controls["position"], before)
    with pytest.raises(RuntimeError, match="closed"):
        drivers[0].step()
