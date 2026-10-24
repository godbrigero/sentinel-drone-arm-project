"""Explicit unit conversion, range metadata, and controller command boundaries."""
from dataclasses import FrozenInstanceError
import math
from pathlib import Path
from unittest.mock import Mock
from typing import cast, get_args
import numpy as np

import pytest

from airtruck.raw_api.so101.calibration import JointCalibration
from airtruck.raw_api.so101.controller import LeaderSO101, RealSO101
from airtruck.raw_api.so101.io import ArmController, ArmReader
from airtruck.raw_api.so101.joints import JOINTS, JointName, PositionCommand
from airtruck.raw_api.so101.position import JointPosition, PositionUnit
from airtruck.testing_util import load_world


@pytest.mark.parametrize('value', [-0.5, 0.0, 0.75, 2.0])
def test_roundtrip_preserves_native_range_and_asymmetric_zero(value: float) -> None:
    position = JointPosition(value, PositionUnit.RADIANS, (-0.5, 2.0))
    for unit in PositionUnit:
        converted = position.to(unit)
        assert converted.unit == unit
        assert converted.limits_rad == position.limits_rad
        assert converted.to(PositionUnit.RADIANS).value == pytest.approx(value)
    assert position.to(PositionUnit.NORMALIZED).range == (-1, 1)
    assert position.to(PositionUnit.DEGREES).range == pytest.approx((-math.degrees(0.5), math.degrees(2)))
    zero = JointPosition(0, PositionUnit.RADIANS, (-0.5, 2))
    assert zero.to(PositionUnit.NORMALIZED).value == pytest.approx(-0.6)


def test_clamping_and_range_transfer_are_explicit() -> None:
    measured = JointPosition(3, PositionUnit.RADIANS, (-1, 2))
    assert measured.value == 3
    assert measured.clamped().value == 2
    assert measured.for_range((10, 20)).value == 20
    midpoint = JointPosition(0.5, PositionUnit.RADIANS, (-1, 2))
    assert midpoint.for_range((10, 20)).value == 15
    assert midpoint.to(PositionUnit.DEGREES).to(PositionUnit.RADIANS).value == pytest.approx(0.5)
    assert midpoint.limits_rad == (-1, 2)
    with pytest.raises(FrozenInstanceError):
        setattr(midpoint, 'value', 10)


@pytest.mark.parametrize('value,limits', [(math.nan, (-1, 1)), (math.inf, (-1, 1)),
    (0, (0, 0)), (0, (1, -1)), (0, (-1, math.inf))])
def test_invalid_positions_rejected(value: float, limits: tuple[float, float]) -> None:
    with pytest.raises(ValueError):
        JointPosition(value, PositionUnit.RADIANS, limits)


def test_leader_and_real_follower_exchange_typed_normalized_values() -> None:
    calibration = {n: JointCalibration(i, 2048, 1, (-1, 1)) for i, n in enumerate(JOINTS, 1)}
    bus = Mock()
    bus.read_positions.return_value = {i: 2048 for i in range(1, 7)}
    leader = LeaderSO101(bus, calibration)
    leader.start()
    measured = leader.positions['shoulder_pan']
    assert measured == JointPosition(0, PositionUnit.RADIANS, (-1, 1))
    assert leader.normalized_positions['shoulder_pan'] == measured.to(PositionUnit.NORMALIZED)
    follower = RealSO101(bus, calibration)
    follower.set_enabled(True)
    follower.set_positions({'shoulder_pan': JointPosition(30, PositionUnit.DEGREES, (-1, 1))})
    assert follower.targets['shoulder_pan'].value == pytest.approx(math.pi / 6, abs=0.002)
    follower.set_positions({'shoulder_pan': JointPosition(0, PositionUnit.NORMALIZED, (10, 20))})
    assert follower.targets['shoulder_pan'].value == 0
    before = bus.write_positions.call_count
    with pytest.raises(ValueError, match='normalized'):
        follower.set_normalized_positions({'shoulder_pan': measured})
    assert bus.write_positions.call_count == before


def test_simulated_follower_accepts_units_and_retains_feedback_metadata() -> None:
    with load_world(Path(__file__).resolve().parents[4] / 'worlds/so101_bench') as world:
        arm = world.system('arm', ArmController)
        arm.set_positions({'shoulder_pan': JointPosition(30, PositionUnit.DEGREES, (-1, 1))})
        assert arm.targets['shoulder_pan'].value == pytest.approx(math.pi / 6, abs=0.002)
        arm.set_positions({'shoulder_lift': JointPosition(0, PositionUnit.NORMALIZED, (-1, 1))})
        assert arm.targets['shoulder_lift'].value == pytest.approx(sum(arm.positions['shoulder_lift'].limits_rad) / 2, abs=0.002)
        world.advance(6)
        feedback = arm.positions['shoulder_lift']
        assert feedback.unit == PositionUnit.RADIANS
        assert feedback.range == arm.positions['shoulder_lift'].limits_rad
        assert feedback.to(PositionUnit.NORMALIZED).value == pytest.approx(0, abs=0.03)
        world.close()
        with pytest.raises(RuntimeError, match='closed'):
            _ = arm.normalized_positions


@pytest.mark.parametrize('bad', [
    {'shoulder_pan': 0.1}, [0.1] * 6, np.zeros(6),
    {'shoulder_pan': JointPosition(0.1, PositionUnit.RADIANS, (-1, 1)), 'gripper': 0.0},
])
def test_only_typed_mappings_are_accepted_without_partial_commands(bad: object) -> None:
    bus = Mock()
    bus.read_positions.return_value = {i: 2048 for i in range(1, 7)}
    calibration = {n: JointCalibration(i, 2048, 1, (-1, 1)) for i, n in enumerate(JOINTS, 1)}
    hardware = RealSO101(bus, calibration)
    hardware.set_enabled(True)
    with load_world(Path(__file__).resolve().parents[4] / 'worlds/so101_bench') as world:
        for arm in (hardware, world.system('arm', ArmController)):
            before = arm.targets
            writes = bus.write_positions.call_count
            with pytest.raises(TypeError, match='JointPosition'):
                arm.set_positions(cast(PositionCommand, bad))
            assert arm.targets == before
            assert bus.write_positions.call_count == writes


@pytest.mark.parametrize('bad', [{'shoulder_pan': 0.1}, [0.1] * 6, np.zeros(6)])
def test_normalized_commands_reject_untyped_input(bad: object) -> None:
    with load_world(Path(__file__).resolve().parents[4] / 'worlds/so101_bench') as world:
        arm = world.system('arm', ArmController)
        before = arm.targets
        with pytest.raises(TypeError, match='JointPosition'):
            arm.set_normalized_positions(cast(PositionCommand, bad))
        assert arm.targets == before



def test_joint_names_match_runtime_order_and_interfaces_are_abstract() -> None:
    assert tuple(get_args(JointName)) == JOINTS
    assert len(set(JOINTS)) == len(JOINTS)
    assert ArmReader.__abstractmethods__ == {'positions', 'normalized_positions'}
    assert ArmController.__abstractmethods__ == {
        'positions', 'normalized_positions', 'targets', 'enabled',
        'set_enabled', 'set_positions', 'set_normalized_positions',
    }


def test_targets_carry_ranges_and_units_for_both_backends() -> None:
    bus = Mock()
    bus.read_positions.return_value = {i: 2048 for i in range(1, 7)}
    calibration = {n: JointCalibration(i, 2048, 1, (-1, 1)) for i, n in enumerate(JOINTS, 1)}
    hardware = RealSO101(bus, calibration)
    hardware.set_enabled(True)
    with load_world(Path(__file__).resolve().parents[4] / 'worlds/so101_bench') as world:
        for arm in (hardware, world.system('arm', ArmController)):
            assert not hasattr(arm, 'limits')
            targets = arm.targets
            for name, target in targets.items():
                assert isinstance(target, JointPosition)
                assert target.unit == PositionUnit.RADIANS
                assert target.range == arm.positions[name].range
                assert target.to(PositionUnit.NORMALIZED).range == (-1, 1)
            targets.clear()
            assert len(arm.targets) == 6
        simulated = world.system('arm', ArmController)
        world.close()
        for read in (lambda: simulated.targets, lambda: simulated.normalized_positions):
            with pytest.raises(RuntimeError, match='closed'):
                read()
