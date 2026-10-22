"""Hardware adapter tests use an in-memory transport, never a serial device."""

import math
import pytest
from collections.abc import Mapping, Sequence
from airtruck.raw_api.so101.io import ArmController
from airtruck.raw_api.so101.controller import JointCalibration, RealSO101 as SO101
from airtruck.raw_api.so101.controller import JOINTS, joint_positions


class FakeTransport:

    def __init__(self) -> None:
        self.positions = {i: 2048 for i in range(1, 7)}
        self.calls: list[str] = []
        self.fail_write = False

    def read_positions(self, servo_ids: Sequence[int]) -> Mapping[int, int]:
        self.calls.append("read")
        return {i: self.positions[i] for i in servo_ids}

    def write_positions(self, positions: Mapping[int, int]) -> None:
        self.calls.append("write")
        if self.fail_write:
            raise OSError("communication failed")
        self.positions.update(positions)

    def set_torque_enabled(self, servo_ids: Sequence[int], enabled: bool) -> None:
        self.calls.append("enable" if enabled else "disable")


@pytest.fixture
def bus() -> FakeTransport:
    return FakeTransport()


@pytest.fixture
def calibration() -> dict[str, JointCalibration]:
    return {
        n: JointCalibration(i + 1, 2048, -1 if i == 1 else 1, (-math.pi / 2, math.pi / 2))
        for i, n in enumerate(JOINTS)
    }


@pytest.fixture
def arm(bus: FakeTransport, calibration: dict[str, JointCalibration]) -> SO101:
    return SO101(bus, calibration)


def test_no_io_until_explicit_enable(arm, bus) -> None:
    assert bus.calls == []
    assert isinstance(arm, ArmController)
    with pytest.raises(RuntimeError):
        arm.set_positions({"shoulder_pan": math.pi / 36})
    arm.set_enabled(True)
    assert bus.calls == ["read", "write", "enable"]
    arm.set_enabled(False)
    assert bus.calls[-1] == "disable"


def test_position_calibration_and_clamping(arm, bus) -> None:
    arm.set_enabled(True)
    arm.set_positions({"shoulder_lift": math.pi / 18, "gripper": 1000})
    assert bus.positions[2] < 2048
    assert arm.positions["shoulder_lift"] == pytest.approx(math.pi / 18, abs=math.pi / 2000)
    assert arm.positions["gripper"] <= math.pi / 2


def test_invalid_batch_and_failed_write_preserve_targets(arm, bus) -> None:
    arm.set_enabled(True)
    targets = arm.targets
    count = len(bus.calls)
    with pytest.raises(ValueError):
        arm.set_positions({"shoulder_pan": math.pi / 36, "missing": 0})
    assert len(bus.calls) == count
    bus.fail_write = True
    with pytest.raises(OSError):
        arm.set_positions({"shoulder_pan": math.pi / 36})
    assert targets == arm.targets


def test_vector_command_uses_joint_order(arm, bus) -> None:
    named = {"shoulder_lift": math.pi / 18}
    assert joint_positions(named) is named
    command = [0.0, math.pi / 18, 0.0, 0.0, 0.0, 0.0]
    assert list(joint_positions(command)) == list(JOINTS)
    arm.set_enabled(True)
    arm.set_positions(command)
    assert bus.positions[2] < 2048
    assert arm.positions["shoulder_lift"] == pytest.approx(math.pi / 18, abs=math.pi / 2000)
    targets = arm.targets
    with pytest.raises(ValueError):
        arm.set_positions([math.pi / 18])
    assert targets == arm.targets


def test_invalid_calibration(bus, calibration) -> None:
    with pytest.raises(ValueError):
        JointCalibration(1, 2048, 0, (-math.pi / 2, math.pi / 2))
    with pytest.raises(ValueError):
        JointCalibration(1, 0, 1, (-math.pi / 2, math.pi / 2))
    calibration["gripper"] = calibration["shoulder_pan"]
    with pytest.raises(ValueError):
        SO101(bus, calibration)


@pytest.mark.parametrize("direction", [-1, 1])
def test_radian_encoder_conversion(direction) -> None:
    calibration = JointCalibration(1, 2048, direction, (-math.pi / 2, math.pi / 2))
    assert calibration.to_ticks(math.pi / 2) == 2048 + direction * 1024
    assert calibration.to_radians(2048 + direction * 1024) == pytest.approx(math.pi / 2)
    assert calibration.to_ticks(-math.pi / 2) == 2048 - direction * 1024
    assert calibration.to_radians(2048) == 0


def test_coupled_fixtures_require_explicit_opt_in(request) -> None:
    with pytest.raises(pytest.fail.Exception, match="require @simulation"):
        request.getfixturevalue("world")
