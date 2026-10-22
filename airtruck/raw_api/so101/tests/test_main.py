"""Application commands and backend lifecycle, without real serial I/O."""
import math
from pathlib import Path
from unittest.mock import Mock

import pytest

from airtruck import main as app
from airtruck.raw_api.so101.io import ArmController
from airtruck.testing_util import load_world
from airtruck.util.environment import RunMode


def test_angle_command_moves_only_selected_joint():
    with load_world(Path(__file__).resolve().parents[4] / 'worlds/so101_bench') as world:
        arm = world.system('arm', ArmController)
        before = arm.targets
        assert app.command(arm, 'shoulder_pan 5')
        assert arm.targets['shoulder_pan'] == pytest.approx(math.radians(5), abs=math.tau / 4096 / 2)
        assert all(arm.targets[n] == q for n, q in before.items() if n != 'shoulder_pan')
        for invalid in ('shoulder_pan nan', 'gripper inf', 'shoulder_pan 9999', 'wrong 3'):
            with pytest.raises(ValueError):
                app.command(arm, invalid)
        assert app.command(arm, 'quit') is False


def test_real_mode_disables_and_closes_after_interruption(monkeypatch):
    monkeypatch.setattr(app, 'RUN_MODE', RunMode.REAL)
    monkeypatch.setattr(app, 'PORT', '/dev/fake')
    monkeypatch.setattr(app, 'CALIBRATION', Path('/tmp/test-calibration.json'))
    monkeypatch.setattr('builtins.input', lambda _: '')
    monkeypatch.setattr(app, 'load_joint_calibrations', Mock(return_value={}))
    transport = Mock()
    transport.__enter__ = Mock(return_value=transport)
    transport.__exit__ = Mock(return_value=False)
    monkeypatch.setattr(app, 'FeetechTransport', Mock(return_value=transport))
    arm = Mock()
    monkeypatch.setattr(app, 'RealSO101', Mock(return_value=arm))
    monkeypatch.setattr(app, 'load_world', Mock(side_effect=AssertionError('Real mode must not open simulation')))
    monkeypatch.setattr(app, 'console', Mock(side_effect=KeyboardInterrupt))
    with pytest.raises(KeyboardInterrupt):
        app.main()
    assert [c.args for c in arm.set_enabled.call_args_list] == [(True,), (False,)]
    transport.__exit__.assert_called_once()
    app.FeetechTransport.assert_called_once_with('/dev/fake')
    app.load_joint_calibrations.assert_called_once_with(Path('/tmp/test-calibration.json'))


def test_simulation_mode_opens_world_without_serial(monkeypatch):
    monkeypatch.setattr(app, 'RUN_MODE', RunMode.SIMULATION)
    world = Mock()
    context = Mock()
    context.__enter__ = Mock(return_value=world)
    context.__exit__ = Mock(return_value=False)
    monkeypatch.setattr(app, 'load_world', Mock(return_value=context))
    monkeypatch.setattr(app, 'FeetechTransport', Mock(side_effect=AssertionError('Simulation must not open serial')))
    monkeypatch.setattr(app, 'console', lambda arm, tick: tick())
    app.main()
    world.open_window.assert_called_once()
    world.advance.assert_called_once_with(0.02)
    context.__exit__.assert_called_once()
