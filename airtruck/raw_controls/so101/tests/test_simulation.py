"""Contract checks for the public SimSO101 control interface."""
import json
import math
from pathlib import Path

import mujoco
import pytest
import numpy as np
from airtruck.raw_controls.so101.controller import JOINTS, SimSO101, ServoSpec, RealSO101
from airtruck.raw_controls.so101.io import ArmController

@pytest.fixture
def inputs() -> tuple[mujoco.MjModel, mujoco.MjData, ServoSpec]:
    model_dir = Path(__file__).resolve().parents[4] / "models" / "so101"
    model = mujoco.MjModel.from_xml_path(str(model_dir / "so101.xml"))
    spec = json.loads((model_dir / "config/servos.json").read_text())
    return model, mujoco.MjData(model), spec

@pytest.fixture
def arm(inputs) -> SimSO101:
    model, data, spec = inputs
    return SimSO101(model, data, servo_spec=spec)

def test_supplied_data_requires_no_filesystem(inputs, monkeypatch, tmp_path) -> None:
    model, data, spec = inputs
    monkeypatch.chdir(tmp_path)

    def forbidden(*args, **kwargs):
        raise AssertionError("Controller must use injected data, not load files")

    monkeypatch.setattr("builtins.open", forbidden)
    monkeypatch.setattr(Path, "open", forbidden)
    monkeypatch.setattr(mujoco.MjModel, "from_xml_path", forbidden)
    arm = SimSO101(model, data, servo_spec=spec)
    assert arm.model is model
    assert arm.data is data
    original_voltage = arm.spec["datasheet"]["voltage_V"]
    spec["datasheet"]["voltage_V"] = 1.0
    assert arm.spec["datasheet"]["voltage_V"] == original_voltage
    arm.set_positions({"shoulder_pan": math.pi / 36})
    arm.step(100)
    assert data.time == pytest.approx(0.1)

def test_vector_command_uses_joint_order(arm) -> None:
    command = [math.pi / 36, 0.0, 0.0, 0.0, -math.pi / 18, 0.0]
    arm.set_positions(command)
    assert list(arm.targets) == list(JOINTS)
    assert arm.targets["shoulder_pan"] == pytest.approx(command[0], abs=math.pi / 2000)
    assert arm.targets["wrist_roll"] == pytest.approx(command[4], abs=math.pi / 2000)
    assert arm.targets["elbow_flex"] == pytest.approx(0, abs=math.pi / 2000)
    before = arm.targets
    with pytest.raises(ValueError):
        arm.set_positions(command[:-1])
    assert before == arm.targets


def test_partial_command_and_atomic_validation(arm) -> None:
    arm.set_positions({'shoulder_pan': math.pi / 36, 'wrist_roll': -math.pi / 18})
    before = arm.targets
    assert before['shoulder_pan'] == pytest.approx(math.pi / 36, abs=math.pi / 2000)
    assert before['elbow_flex'] == 0
    for bad in ({'shoulder_pan': 8, 'missing': 0}, {'shoulder_pan': 8, 'gripper': float('nan')}):
        with pytest.raises(ValueError):
            arm.set_positions(bad)
        assert before == arm.targets

def test_clamping_and_feedback_copies(arm) -> None:
    arm.set_positions({'shoulder_lift': 10000, 'elbow_flex': -10000})
    for name in arm.names:
        lo, hi = arm.limits[name]
        assert arm.targets[name] <= hi
        assert arm.targets[name] >= lo
    copy = arm.positions
    copy['shoulder_pan'] = 55
    assert arm.positions['shoulder_pan'] == 0

def test_stepping_reset_and_torque_disable(arm) -> None:
    arm.set_positions({'shoulder_pan': math.pi / 36})
    arm.step(100)
    assert arm.data.time == pytest.approx(0.1)
    arm.enabled = False
    arm.step()
    np.testing.assert_allclose(arm.data.actuator_force, 0)
    arm.reset()
    assert arm.data.time == 0
    assert arm.enabled
    np.testing.assert_array_equal(arm.data.qpos, 0)
    assert all((v == 0 for v in arm.targets.values()))
    for invalid in (0, -1, 1.2, True):
        with pytest.raises(ValueError):
            arm.step(invalid)


def test_shared_abstract_interface(arm) -> None:
    assert issubclass(SimSO101, ArmController)
    assert issubclass(RealSO101, ArmController)
    assert isinstance(arm, ArmController)

    class IncompleteController(ArmController):
        pass

    with pytest.raises(TypeError, match="abstract"):
        ArmController()
    with pytest.raises(TypeError, match="abstract"):
        IncompleteController()

    controller: ArmController = arm
    controller.set_enabled(False)
    assert not controller.enabled
    controller.set_enabled(True)
    controller.set_positions({"shoulder_pan": math.pi / 36})
    assert controller.targets["shoulder_pan"] == pytest.approx(math.pi / 36, abs=math.pi / 2000)


def test_feedback_and_limits_are_radians(arm) -> None:
    arm.data.qpos[0] = math.pi / 2
    assert arm.positions["shoulder_pan"] == pytest.approx(math.pi / 2)
    assert arm.observation()["shoulder_pan.pos"] == pytest.approx(math.pi / 2)
    assert arm.limits["shoulder_pan"] == pytest.approx(tuple(arm.model.jnt_range[0]))
    arm.command_radians("shoulder_pan", math.pi / 2)
    assert arm.targets["shoulder_pan"] == pytest.approx(math.pi / 2)


def test_six_joint_motion_verification() -> None:
    import runpy

    root = Path(__file__).resolve().parents[4]
    result = runpy.run_path(str(root / "scripts/so101/verify.py"))
    report = result["report"]
    assert report["status"] == "passed"
    assert len(report["independent_motion"]) == 6
    for motion in report["independent_motion"]:
        assert motion["target_rad"] == pytest.approx(math.pi / 36)
        assert motion["error_rad"] < math.pi / 180
