"""Contract checks for the public SimSO101 control interface."""
from __future__ import annotations
import runpy
from types import SimpleNamespace
from typing import cast
from collections.abc import Callable
from airtruck.raw_api.so101.joints import PositionCommand
from airtruck.raw_api.so101.position import JointPosition, PositionUnit
import json
import math
from pathlib import Path

import mujoco
import pytest
import numpy as np
from airtruck.raw_api.so101.joints import JOINTS
from airtruck.raw_api.so101.controller import SimSO101, ServoSpec, RealSO101
from airtruck.raw_api.so101.io import ArmController
from airtruck.testing_util import simulation
from airtruck.testing_util import WorldControl
from airtruck.testing_util.mujoco_world import MujocoWorld
from airtruck.testing_util.adapters import SO101Adapter

@pytest.fixture
def arm(world: WorldControl) -> SimSO101:
    # Backend contract tests intentionally inspect the implementation.
    assert isinstance(world, MujocoWorld)
    adapter = world._systems["arm"]
    assert isinstance(adapter, SO101Adapter)
    return adapter.backend

@pytest.fixture
def inputs(arm: SimSO101) -> tuple[mujoco.MjModel, mujoco.MjData, ServoSpec]:
    return arm.model, arm.data, arm.spec

@simulation("worlds/so101_bench")
def test_supplied_data_requires_no_filesystem(inputs: tuple[mujoco.MjModel, mujoco.MjData, ServoSpec], monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    model, data, spec = inputs
    monkeypatch.chdir(tmp_path)

    def forbidden(*args: object, **kwargs: object) -> None:
        raise AssertionError("Controller must use injected data, not load files")

    monkeypatch.setattr("builtins.open", forbidden)
    monkeypatch.setattr(Path, "open", forbidden)
    monkeypatch.setattr(mujoco.MjModel, "from_xml_path", forbidden)
    arm = SimSO101(model, data, servo_spec=spec, prefix="arm/")
    assert arm.model is model
    assert arm.data is data
    original_voltage = arm.spec["datasheet"]["voltage_V"]
    spec["datasheet"]["voltage_V"] = 1.0
    assert arm.spec["datasheet"]["voltage_V"] == original_voltage
    arm.set_positions({"shoulder_pan": JointPosition(math.pi / 36, PositionUnit.RADIANS, arm.positions["shoulder_pan"].limits_rad)})
    arm.update_control()
    assert data.time == 0  # Controllers never advance the world.

@simulation("worlds/so101_bench")
def test_named_command_updates_all_selected_joints(arm: SimSO101) -> None:
    values = [math.pi / 36, 0.0, 0.0, 0.0, -math.pi / 18, 0.0]
    command = {name: JointPosition(value, PositionUnit.RADIANS, arm.positions[name].limits_rad)
               for name, value in zip(JOINTS, values, strict=True)}
    arm.set_positions(command)
    assert list(arm.targets) == list(JOINTS)
    assert arm.targets["shoulder_pan"].value == pytest.approx(values[0], abs=math.pi / 2000)
    assert arm.targets["wrist_roll"].value == pytest.approx(values[4], abs=math.pi / 2000)
    assert arm.targets["elbow_flex"].value == pytest.approx(0, abs=math.pi / 2000)
    before = arm.targets
    with pytest.raises(TypeError, match="mapping"):
        arm.set_positions(cast(PositionCommand, values))
    assert before == arm.targets


@simulation("worlds/so101_bench")
def test_partial_command_and_atomic_validation(arm: SimSO101) -> None:
    arm.set_positions({'shoulder_pan': JointPosition(math.pi / 36, PositionUnit.RADIANS, arm.positions['shoulder_pan'].limits_rad), 'wrist_roll': JointPosition(-math.pi / 18, PositionUnit.RADIANS, arm.positions['wrist_roll'].limits_rad)})
    before = arm.targets
    assert before['shoulder_pan'].value == pytest.approx(math.pi / 36, abs=math.pi / 2000)
    assert before['elbow_flex'].value == 0
    for bad in ({'shoulder_pan': JointPosition(8, PositionUnit.RADIANS, arm.positions['shoulder_pan'].limits_rad), 'missing': JointPosition(0, PositionUnit.RADIANS, (-1, 1))},
                {'shoulder_pan': JointPosition(8, PositionUnit.RADIANS, arm.positions['shoulder_pan'].limits_rad), 'gripper': 0.0}):
        with pytest.raises((ValueError, TypeError)):
            arm.set_positions(cast(PositionCommand, bad))
        assert before == arm.targets

@simulation("worlds/so101_bench")
def test_clamping_and_feedback_copies(arm: SimSO101) -> None:
    arm.set_positions({'shoulder_lift': JointPosition(10000, PositionUnit.RADIANS, arm.positions['shoulder_lift'].limits_rad), 'elbow_flex': JointPosition(-10000, PositionUnit.RADIANS, arm.positions['elbow_flex'].limits_rad)})
    for name in arm.names:
        lo, hi = arm.positions[name].limits_rad
        assert arm.targets[name].value <= hi
        assert arm.targets[name].value >= lo
    copy = arm.positions
    copy['shoulder_pan'] = JointPosition(55, PositionUnit.RADIANS, copy['shoulder_pan'].limits_rad)
    assert arm.positions['shoulder_pan'].value == 0

@simulation("worlds/so101_bench")
def test_stepping_reset_and_torque_disable(arm: SimSO101, world: WorldControl) -> None:
    arm.set_positions({'shoulder_pan': JointPosition(math.pi / 36, PositionUnit.RADIANS, arm.positions['shoulder_pan'].limits_rad)})
    world.step(100)
    assert arm.data.time == pytest.approx(0.1)
    arm.enabled = False
    world.step()
    np.testing.assert_allclose(arm.data.actuator_force, 0)
    world.reset()
    assert arm.data.time == 0
    assert arm.enabled
    np.testing.assert_array_equal(arm.data.qpos, 0)
    assert all((v.value == 0 for v in arm.targets.values()))
    for invalid in (0, -1, 1.2, True):
        with pytest.raises(ValueError):
            world.step(cast(int, invalid))


@simulation("worlds/so101_bench")
def test_shared_abstract_interface(arm: SimSO101) -> None:
    assert issubclass(SimSO101, ArmController)
    assert issubclass(RealSO101, ArmController)
    assert isinstance(arm, ArmController)

    class IncompleteController(ArmController):
        pass

    with pytest.raises(TypeError, match="abstract"):
        ArmController()  # type: ignore[abstract]
    with pytest.raises(TypeError, match="abstract"):
        IncompleteController()  # type: ignore[abstract]

    controller: ArmController = arm
    controller.set_enabled(False)
    assert not controller.enabled
    controller.set_enabled(True)
    controller.set_positions({"shoulder_pan": JointPosition(math.pi / 36, PositionUnit.RADIANS, controller.positions["shoulder_pan"].limits_rad)})
    assert controller.targets["shoulder_pan"].value == pytest.approx(math.pi / 36, abs=math.pi / 2000)


@simulation("worlds/so101_bench")
def test_feedback_and_limits_are_radians(arm: SimSO101) -> None:
    arm.data.qpos[0] = math.pi / 2
    assert arm.positions["shoulder_pan"].value == pytest.approx(math.pi / 2)
    assert arm.observation()["shoulder_pan.pos"] == pytest.approx(math.pi / 2)
    assert arm.positions["shoulder_pan"].limits_rad == pytest.approx(tuple(arm.model.jnt_range[0]))
    arm.command_radians("shoulder_pan", math.pi / 2)
    assert arm.targets["shoulder_pan"].value == pytest.approx(math.pi / 2)


@pytest.mark.sim
def test_six_joint_motion_verification() -> None:

    root = Path(__file__).resolve().parents[4]
    result = runpy.run_path(str(root / "scripts/so101/verify.py"))
    report = result["report"]
    assert report["status"] == "passed"
    assert len(report["independent_motion"]) == 6
    for motion in report["independent_motion"]:
        assert motion["target_rad"] == pytest.approx(math.pi / 36)
        assert motion["error_rad"] < math.pi / 180


@simulation("worlds/so101_bench")
def test_coupled_control_and_recording(world: WorldControl) -> None:
    arm = world.system("arm", ArmController)
    world.advance(3.0)
    arm.set_positions({"shoulder_pan": JointPosition(0.1, PositionUnit.RADIANS, arm.positions["shoulder_pan"].limits_rad)})
    samples = world.record(lambda: arm.positions, 3.0, sample_period_s=0.1)
    assert len(samples) == 31
    assert samples[-1].time_s - samples[0].time_s == pytest.approx(3.0)
    assert samples[-1].value["shoulder_pan"].value == pytest.approx(0.1, abs=0.01)
    assert samples[0].value != samples[-1].value


@simulation("worlds/so101_bench")
def test_fixture_sessions_are_isolated(world_factory: Callable[[], WorldControl]) -> None:
    first, second = world_factory(), world_factory()
    arm = first.system("arm", ArmController)
    arm.set_positions({"shoulder_pan": JointPosition(0.1, PositionUnit.RADIANS, arm.positions["shoulder_pan"].limits_rad)})
    first.step(10)
    assert second.time_s == 0
    assert second.system("arm", ArmController).targets["shoulder_pan"].value == 0
    first.close()
    first.close()
    with pytest.raises(RuntimeError, match="closed"):
        arm.set_enabled(True)
    with pytest.raises(RuntimeError, match="closed"):
        first.step()
    second.step()
    assert second.time_s > 0


@simulation("worlds/so101_bench")
def test_simulation_wait_is_bounded(world: WorldControl) -> None:
    world.wait_until(lambda: world.time_s >= 0.005, timeout_s=0.01)
    start = world.time_s
    with pytest.raises(TimeoutError):
        world.wait_until(lambda: False, timeout_s=0.003)
    assert world.time_s - start == pytest.approx(0.003)
    for invalid in (-1, float("nan"), float("inf")):
        with pytest.raises(ValueError):
            world.advance(invalid)
    with pytest.raises(ValueError):
        world.record(lambda: world.time_s, 0.1, sample_period_s=0)


@simulation("worlds/so101_bench")
def test_session_cleanup_disables_control_when_viewer_close_fails(world: WorldControl, arm: SimSO101, monkeypatch: pytest.MonkeyPatch) -> None:

    def fail_close() -> None:
        raise RuntimeError("viewer close failed")

    monkeypatch.setattr(world, "_viewer", SimpleNamespace(close=fail_close))
    with pytest.raises(ExceptionGroup, match="World cleanup failed"):
        world.close()
    assert not arm.enabled
    world.close()
    with pytest.raises(RuntimeError, match="closed"):
        world.advance(0.01)
