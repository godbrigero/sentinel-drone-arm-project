"""Leader/follower routing, feedback copying and cleanup without serial hardware."""
import io
from pathlib import Path
from unittest.mock import MagicMock, Mock

import pytest

from airtruck import main as app
from airtruck.raw_api.so101.joints import JOINTS
from airtruck.raw_api.so101.calibration import JointCalibration
from airtruck.raw_api.so101.controller import LeaderSO101
from airtruck.raw_api.so101.io import ArmController
from airtruck.raw_api.so101.position import JointPosition, PositionUnit
from airtruck.testing_util import load_world
from airtruck.util.environment import RunMode


@pytest.fixture
def setup_app(monkeypatch: pytest.MonkeyPatch) -> dict[str, MagicMock]:
    monkeypatch.setattr(app, 'LEADER_PORT', '/dev/leader')
    monkeypatch.setattr(app, 'FOLLOWER_PORT', '/dev/follower')
    monkeypatch.setattr(app, 'LEADER_CALIBRATION', Path('/tmp/leader.json'))
    monkeypatch.setattr(app, 'FOLLOWER_CALIBRATION', Path('/tmp/follower.json'))
    monkeypatch.setattr('builtins.input', lambda _: '')
    mocks = {name: MagicMock() for name in (
        'FeetechTransport', 'LeaderSO101', 'RealSO101', 'load_world',
        'load_joint_calibrations', 'follow',
    )}
    for name, value in mocks.items():
        monkeypatch.setattr(app, name, value)
    return mocks


def test_simulation_uses_physical_leader_and_only_its_calibration(
    monkeypatch: pytest.MonkeyPatch, setup_app: dict[str, MagicMock],
) -> None:
    monkeypatch.setattr(app, 'RUN_MODE', RunMode.SIMULATION)
    mocks = setup_app
    world = mocks['load_world'].return_value.__enter__.return_value
    def tick_once(leader: object, follower: object, tick: Mock) -> None:
        tick()
    mocks['follow'].side_effect = tick_once
    app.main()
    mocks['FeetechTransport'].assert_called_once_with('/dev/leader', baudrate=app.LEADER_BAUDRATE)
    mocks['load_joint_calibrations'].assert_called_once_with(Path('/tmp/leader.json'))
    mocks['LeaderSO101'].return_value.start.assert_called_once()
    mocks['RealSO101'].assert_not_called()
    world.open_window.assert_called_once()
    world.advance.assert_called_once_with(0.02)
    assert mocks['follow'].call_args.args[:2] == (
        mocks['LeaderSO101'].return_value, world.system.return_value,
    )
    mocks['FeetechTransport'].return_value.__exit__.assert_called_once()


@pytest.mark.parametrize('failure', [KeyboardInterrupt, OSError])
def test_real_mode_disables_and_closes_after_follow_failure(
    monkeypatch: pytest.MonkeyPatch, setup_app: dict[str, MagicMock], failure: type[BaseException],
) -> None:
    monkeypatch.setattr(app, 'RUN_MODE', RunMode.REAL)
    mocks = setup_app
    leader_bus, follower_bus = MagicMock(), MagicMock()
    mocks['FeetechTransport'].side_effect = [leader_bus, follower_bus]
    mocks['follow'].side_effect = failure
    with pytest.raises(failure):
        app.main()
    arm = mocks['RealSO101'].return_value
    assert [c.args for c in arm.set_enabled.call_args_list] == [(True,), (False,)]
    assert [c.args for c in mocks['FeetechTransport'].call_args_list] == [('/dev/leader',), ('/dev/follower',)]
    assert [c.args for c in mocks['load_joint_calibrations'].call_args_list] == [
        (Path('/tmp/leader.json'),), (Path('/tmp/follower.json'),),
    ]
    mocks['load_world'].assert_not_called()
    leader_bus.__exit__.assert_called_once()
    follower_bus.__exit__.assert_called_once()


def test_failed_leader_start_never_opens_follower(
    monkeypatch: pytest.MonkeyPatch, setup_app: dict[str, MagicMock],
) -> None:
    monkeypatch.setattr(app, 'RUN_MODE', RunMode.REAL)
    setup_app['LeaderSO101'].return_value.start.side_effect = OSError('no leader')
    with pytest.raises(OSError, match='no leader'):
        app.main()
    setup_app['FeetechTransport'].assert_called_once()
    setup_app['RealSO101'].assert_not_called()
    setup_app['FeetechTransport'].return_value.__exit__.assert_called_once()


@pytest.mark.parametrize('setting,value', [('LEADER_PORT', ''), ('FOLLOWER_PORT', ''),
    ('FOLLOWER_PORT', '/dev/leader'), ('FOLLOWER_CALIBRATION', Path('/tmp/leader.json'))])
def test_invalid_configuration_does_not_open_hardware(
    monkeypatch: pytest.MonkeyPatch, setup_app: dict[str, MagicMock], setting: str, value: str | Path,
) -> None:
    monkeypatch.setattr(app, 'RUN_MODE', RunMode.REAL)
    monkeypatch.setattr(app, setting, value)
    with pytest.raises(ValueError):
        app.main()
    setup_app['FeetechTransport'].assert_not_called()


def test_cancel_does_not_open_hardware(
    monkeypatch: pytest.MonkeyPatch, setup_app: dict[str, MagicMock],
) -> None:
    monkeypatch.setattr(app, 'RUN_MODE', RunMode.SIMULATION)
    monkeypatch.setattr('builtins.input', lambda _: 'quit')
    app.main()
    setup_app['FeetechTransport'].assert_not_called()
    setup_app['load_world'].assert_not_called()


def test_follow_reads_new_pose_each_iteration_and_ticks_after_command(monkeypatch: pytest.MonkeyPatch) -> None:
    leader, follower = Mock(), Mock()
    poses = [{n: JointPosition(angle, PositionUnit.NORMALIZED, (-1, 1))
              for n in JOINTS} for angle in (0.1, 0.2)]
    events: list[object] = []
    # A property mock is unnecessary: replace the pose during each tick.
    leader.normalized_positions = poses[0]
    follower.set_normalized_positions.side_effect = lambda pose: events.append(pose)
    def tick() -> None:
        events.append('tick')
        leader.normalized_positions = poses[1]
    monkeypatch.setattr(app.select, 'select', Mock(side_effect=[([], [], []), ([], [], []), ([1], [], [])]))
    monkeypatch.setattr(app.sys, 'stdin', io.StringIO('quit\n'))
    monkeypatch.setattr(app.time, 'sleep', lambda _: None)
    app.follow(leader, follower, tick)
    assert events == [poses[0], 'tick', poses[1], 'tick']
    leader.set_positions.assert_not_called()


def test_physical_leader_feedback_becomes_simulated_targets(monkeypatch: pytest.MonkeyPatch) -> None:
    bus = Mock()
    bus.read_positions.return_value = {i: 2100 for i in range(1, 7)}
    calibration = {n: JointCalibration(i, 2048, 1, (-1.0, 1.0))
                   for i, n in enumerate(JOINTS, 1)}
    leader = LeaderSO101(bus, calibration)
    leader.start()
    monkeypatch.setattr(app.select, 'select', Mock(side_effect=[([], [], []), ([1], [], [])]))
    monkeypatch.setattr(app.sys, 'stdin', io.StringIO('quit\n'))
    monkeypatch.setattr(app.time, 'sleep', lambda _: None)
    with load_world(Path(__file__).resolve().parents[4] / 'worlds/so101_bench') as world:
        follower = world.system('arm', ArmController)
        app.follow(leader, follower, lambda: world.advance(0.02))
        for name, value in leader.normalized_positions.items():
            low, high = follower.positions[name].limits_rad
            assert follower.targets[name].value == pytest.approx(low + (value.value + 1) * 0.5 * (high - low), abs=0.001)
        assert world.time_s == pytest.approx(0.02)
    bus.write_positions.assert_not_called()


@pytest.mark.parametrize("fraction", [0.0, 0.5, 1.0])
def test_simulation_mapping_reaches_full_joint_ranges(fraction: float) -> None:
    positions = {n: JointPosition(2 * fraction - 1, PositionUnit.NORMALIZED, (-1, 1))
                 for n in JOINTS}
    with load_world(Path(__file__).resolve().parents[4] / 'worlds/so101_bench') as world:
        arm = world.system('arm', ArmController)
        expected = {name: low + fraction * (high - low)
                    for name, position in arm.positions.items()
                    for low, high in [position.limits_rad]}
        arm.set_normalized_positions(positions)
        world.advance(10.0)
        for name in ('shoulder_lift', 'elbow_flex'):
            assert arm.positions[name].value == pytest.approx(expected[name], abs=0.04)
