"""Hardware adapter tests use an in-memory transport, never a serial device."""
from __future__ import annotations
from typing import cast
from airtruck.raw_api.so101.joints import PositionCommand
from airtruck.raw_api.so101.position import JointPosition, PositionUnit

import math
import pytest
from collections.abc import Mapping, Sequence
from airtruck.raw_api.so101.io import ArmController, JointName
from airtruck.raw_api.so101.calibration import JointCalibration
from airtruck.raw_api.so101.controller import RealSO101 as SO101
from airtruck.raw_api.so101.joints import JOINTS
from airtruck.raw_api.so101.controller import LeaderSO101
from airtruck.raw_api.so101.utils import validate_position_command


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
def calibration() -> dict[JointName, JointCalibration]:
    return {
        n: JointCalibration(i + 1, 2048, -1 if i == 1 else 1, (-math.pi / 2, math.pi / 2))
        for i, n in enumerate(JOINTS)
    }


@pytest.fixture
def arm(bus: FakeTransport, calibration: dict[JointName, JointCalibration]) -> SO101:
    return SO101(bus, calibration)


def test_no_io_until_explicit_enable(arm: SO101, bus: FakeTransport) -> None:
    assert bus.calls == []
    assert isinstance(arm, ArmController)
    with pytest.raises(RuntimeError):
        arm.set_positions({"shoulder_pan": JointPosition(math.pi / 36, PositionUnit.RADIANS, arm._limits_rad["shoulder_pan"])})
    arm.set_enabled(True)
    assert bus.calls == ["read", "write", "enable"]
    arm.set_enabled(False)
    assert bus.calls[-1] == "disable"


def test_position_calibration_and_clamping(arm: SO101, bus: FakeTransport) -> None:
    arm.set_enabled(True)
    arm.set_positions({"shoulder_lift": JointPosition(math.pi / 18, PositionUnit.RADIANS, arm._limits_rad["shoulder_lift"]), "gripper": JointPosition(1000, PositionUnit.RADIANS, arm._limits_rad["gripper"])})
    assert bus.positions[2] < 2048
    assert arm.positions["shoulder_lift"].value == pytest.approx(math.pi / 18, abs=math.pi / 2000)
    assert arm.positions["gripper"].value <= math.pi / 2


def test_invalid_batch_and_failed_write_preserve_targets(arm: SO101, bus: FakeTransport) -> None:
    arm.set_enabled(True)
    targets = arm.targets
    count = len(bus.calls)
    with pytest.raises(ValueError):
        arm.set_positions(cast(PositionCommand, {"shoulder_pan": JointPosition(math.pi / 36, PositionUnit.RADIANS, arm._limits_rad["shoulder_pan"]), "missing": JointPosition(0, PositionUnit.RADIANS, (-1.0, 1.0))}))
    assert len(bus.calls) == count
    bus.fail_write = True
    with pytest.raises(OSError):
        arm.set_positions({"shoulder_pan": JointPosition(math.pi / 36, PositionUnit.RADIANS, arm._limits_rad["shoulder_pan"])})
    assert targets == arm.targets


def test_named_command_validation(arm: SO101, bus: FakeTransport) -> None:
    command: PositionCommand = {"shoulder_lift": JointPosition(math.pi / 18, PositionUnit.RADIANS, arm._limits_rad["shoulder_lift"])}
    position = command["shoulder_lift"]
    validate_position_command(command, JOINTS)
    assert command["shoulder_lift"] is position
    arm.set_enabled(True)
    arm.set_positions(command)
    assert bus.positions[2] < 2048
    assert arm.positions["shoulder_lift"].value == pytest.approx(math.pi / 18, abs=math.pi / 2000)
    targets = arm.targets
    with pytest.raises(TypeError, match="mapping"):
        arm.set_positions(cast(PositionCommand, [math.pi / 18]))
    assert targets == arm.targets


def test_invalid_calibration(bus: FakeTransport, calibration: dict[JointName, JointCalibration]) -> None:
    with pytest.raises(ValueError):
        JointCalibration(1, 2048, 0, (-math.pi / 2, math.pi / 2))
    with pytest.raises(ValueError):
        JointCalibration(1, 0, 1, (-math.pi / 2, math.pi / 2))
    calibration["gripper"] = calibration["shoulder_pan"]
    with pytest.raises(ValueError):
        SO101(bus, calibration)


@pytest.mark.parametrize("direction", [-1, 1])
def test_radian_encoder_conversion(direction: int) -> None:
    calibration = JointCalibration(1, 2048, direction, (-math.pi / 2, math.pi / 2))
    assert calibration.to_ticks(math.pi / 2) == 2048 + direction * 1024
    assert calibration.to_radians(2048 + direction * 1024) == pytest.approx(math.pi / 2)
    assert calibration.to_ticks(-math.pi / 2) == 2048 - direction * 1024
    assert calibration.to_radians(2048) == 0


def test_coupled_fixtures_require_explicit_opt_in(request: pytest.FixtureRequest) -> None:
    with pytest.raises(pytest.fail.Exception, match="require @simulation"):
        request.getfixturevalue("world")


def test_leader_disables_torque_and_reads_fresh_calibrated_feedback(
    bus: FakeTransport, calibration: dict[JointName, JointCalibration],
) -> None:
    leader = LeaderSO101(bus, calibration)
    assert bus.calls == []
    assert not isinstance(leader, ArmController)
    assert not hasattr(leader, "set_positions")
    assert not hasattr(leader, "set_enabled")
    with pytest.raises(RuntimeError, match="start"):
        _ = leader.positions
    assert bus.calls == []
    leader.start()
    assert bus.calls == ["disable"]
    bus.positions[2] = 2304
    first = leader.positions
    assert first["shoulder_lift"].value == pytest.approx(-math.pi / 8)
    bus.positions[2] = 2560
    assert leader.positions["shoulder_lift"].value == pytest.approx(-math.pi / 4)
    assert first["shoulder_lift"].value == pytest.approx(-math.pi / 8)
    assert list(first) == list(JOINTS)
    assert leader._limits_rad["gripper"] == (-math.pi / 2, math.pi / 2)
    assert bus.calls == ["disable", "read", "read"]


def test_leader_failed_restart_blocks_reading(
    bus: FakeTransport, calibration: dict[JointName, JointCalibration], monkeypatch: pytest.MonkeyPatch,
) -> None:
    leader = LeaderSO101(bus, calibration)
    leader.start()
    def fail(*args: object) -> None:
        raise OSError("disable failed")
    monkeypatch.setattr(bus, "set_torque_enabled", fail)
    with pytest.raises(OSError, match="disable failed"):
        leader.start()
    with pytest.raises(RuntimeError, match="start"):
        _ = leader.positions
    assert bus.calls == ["disable"]


def test_leader_validates_calibration_and_propagates_missing_feedback(
    bus: FakeTransport, calibration: dict[JointName, JointCalibration],
) -> None:
    with pytest.raises(ValueError, match="six"):
        LeaderSO101(bus, {})
    duplicate = dict(calibration)
    duplicate["gripper"] = duplicate["shoulder_pan"]
    with pytest.raises(ValueError, match="unique"):
        LeaderSO101(bus, duplicate)
    leader = LeaderSO101(bus, calibration)
    leader.start()
    del bus.positions[6]
    with pytest.raises(KeyError):
        _ = leader.positions


@pytest.mark.parametrize("ticks", [0, 1024, 2048, 3072, 4095])
def test_feedback_clamps_both_arms_to_calibrated_limits(
    bus: FakeTransport, calibration: dict[JointName, JointCalibration], ticks: int,
) -> None:
    follower = SO101(bus, calibration)
    leader = LeaderSO101(bus, calibration)
    leader.start()
    bus.positions = {i: ticks for i in range(1, 7)}
    expected = {
        name: min(math.pi / 2, max(-math.pi / 2, c.to_radians(ticks)))
        for name, c in calibration.items()
    }
    assert {n: p.value for n, p in follower.positions.items()} == expected
    assert {n: p.value for n, p in leader.positions.items()} == expected


def test_clamped_feedback_does_not_hide_unsafe_enable_pose(
    bus: FakeTransport, calibration: dict[JointName, JointCalibration],
) -> None:
    arm = SO101(bus, calibration)
    bus.positions[1] = 4095
    assert arm.positions["shoulder_pan"].value == math.pi / 2
    with pytest.raises(ValueError, match="outside calibrated travel"):
        arm.set_enabled(True)
    assert bus.calls == ["read", "read"]
    assert not arm.enabled


def test_enable_reports_all_outside_joints_without_writing(
    bus: FakeTransport, calibration: dict[JointName, JointCalibration],
) -> None:
    arm = SO101(bus, calibration)
    bus.positions[2] = 1023
    bus.positions[6] = 3073
    with pytest.raises(ValueError) as error:
        arm.set_enabled(True)
    assert "shoulder_lift (servo 2): tick 1023, allowed ticks 1024..3072" in str(error.value)
    assert "gripper (servo 6): tick 3073, allowed ticks 1024..3072" in str(error.value)
    assert bus.calls == ["read"]
    assert not arm.enabled


@pytest.mark.parametrize('value', [-1.0, 0.0, 1.0])
def test_shared_normalized_contract_with_different_hardware_ranges(
    bus: FakeTransport, value: float,
) -> None:
    calibration = {n: JointCalibration(i, 1500, -1 if i == 2 else 1, (-0.5, 1.0))
                   for i, n in enumerate(JOINTS, 1)}
    leader = LeaderSO101(bus, calibration)
    leader.start()
    angle = -0.5 + (value + 1) * 0.75
    bus.positions = {c.servo_id: c.to_ticks(angle) for c in calibration.values()}
    assert all(q.value == pytest.approx(value, abs=0.003) for q in leader.normalized_positions.values())
    follower_bus = FakeTransport()
    follower_calibration = {n: JointCalibration(i, 2048, 1, (-1.0, 2.0))
                            for i, n in enumerate(JOINTS, 1)}
    follower = SO101(follower_bus, follower_calibration)
    follower.set_enabled(True)
    follower.set_normalized_positions(leader.normalized_positions)
    assert all(q.value == pytest.approx(-1.0 + (value + 1) * 1.5, abs=0.005)
               for q in follower.positions.values())
    assert all(q.value == pytest.approx(value, abs=0.004) for q in follower.normalized_positions.values())


def test_normalized_commands_clamp_and_validate_before_writes(
    bus: FakeTransport, calibration: dict[JointName, JointCalibration],
) -> None:
    follower = SO101(bus, calibration)
    follower.set_enabled(True)
    follower.set_normalized_positions({'shoulder_pan': JointPosition(-2, PositionUnit.NORMALIZED, follower._limits_rad['shoulder_pan']), 'shoulder_lift': JointPosition(2, PositionUnit.NORMALIZED, follower._limits_rad['shoulder_lift'])})
    assert follower.positions['shoulder_pan'].value == pytest.approx(-math.pi / 2)
    assert follower.positions['shoulder_lift'].value == pytest.approx(math.pi / 2)
    before = list(bus.calls)
    with pytest.raises(ValueError, match='finite'):
        follower.set_normalized_positions({'shoulder_pan': JointPosition(0.0, PositionUnit.NORMALIZED, follower._limits_rad['shoulder_pan']), 'gripper': JointPosition(math.nan, PositionUnit.NORMALIZED, follower._limits_rad['gripper'])})
    assert bus.calls == before
