"""Offline endpoint teaching, ID detection and calibration file contract."""
from __future__ import annotations
import json
from contextlib import nullcontext
from collections.abc import Callable, Mapping, Sequence
from pathlib import Path
from typing import Any
import math
from types import SimpleNamespace
import pytest
from airtruck.raw_api.so101.joints import JOINTS
from airtruck.raw_api.so101.calibration import JointCalibration, load_joint_calibrations
from airtruck.raw_api.so101.io import JointName
from scripts.so101.so101_calibrate import Motion, ORDER, make_calibration, save_checkpoint, capture, calibrate, keyboard, main, TerminalUI, select_port


def test_distal_to_proximal_order() -> None:
    assert ORDER == ('gripper', 'wrist_roll', 'wrist_flex', 'elbow_flex', 'shoulder_lift', 'shoulder_pan')


@pytest.mark.parametrize('low,high,zero,direction', [(1000,3000,2000,1),(3000,1000,2000,-1),(0,4095,2048,1)])
def test_signed_endpoints(low: int, high: int, zero: int, direction: int) -> None:
    c = make_calibration(4,low,high,zero)
    assert c.direction == direction
    assert c.to_radians(low) == pytest.approx(c.limits_rad[0])
    assert c.to_radians(high) == pytest.approx(c.limits_rad[1])
    assert c.to_ticks(0) == zero
    assert c.to_ticks(c.limits_rad[0]) == low
    assert c.to_ticks(c.limits_rad[1]) == high


def test_most_motion_tolerates_other_joint_and_backtracking() -> None:
    m=Motion({1:2000,2:2000,3:2000})
    for sample in ({1:2100,2:2010,3:2001},{1:2200,2:2020,3:2000},{1:2000,2:2030,3:2001}):
        m.update(sample)
    assert m.winner(set()) == 1
    assert m.distance[1] == pytest.approx(400*math.tau/4096)
    with pytest.raises(ValueError,match='already assigned'): m.winner({1})


def test_ambiguous_motion_and_wrap_rejected() -> None:
    m=Motion({1:2000,2:2000});m.update({1:2100,2:2100})
    with pytest.raises(ValueError,match='ambiguous'): m.winner(set())
    m=Motion({1:4090,2:2000});m.update({1:30,2:2000})
    assert m.distance[1] < .1
    with pytest.raises(ValueError,match='crossed'): m.winner(set())
    with pytest.raises(ValueError,match='replies'): m.update({1:40})


def test_invalid_zero_and_no_travel() -> None:
    with pytest.raises(ValueError,match='between'): make_calibration(1,1000,3000,3500)
    with pytest.raises(ValueError,match='close'): make_calibration(1,2000,2001,2000)


def test_json_roundtrip_partial_duplicate_and_invalid(tmp_path: Path) -> None:
    path=tmp_path/'calibration.json'
    mapping={n:make_calibration(i+1,1000,3000,2000) for i,n in enumerate(JOINTS)}
    save_checkpoint(path,{'gripper':mapping['gripper']})
    with pytest.raises(ValueError,match='six'): load_joint_calibrations(path)
    save_checkpoint(path,mapping)
    assert load_joint_calibrations(path)==mapping
    assert list(tmp_path.iterdir()) == [path]
    raw=json.loads(path.read_text());raw['gripper']['servo_id']=1
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError,match='unique'): load_joint_calibrations(path)
    raw['gripper']['servo_id']=True;path.write_text(json.dumps(raw))
    with pytest.raises(ValueError,match='integers'): load_joint_calibrations(path)
    path.write_text('{"gripper": {}, "gripper": {}}')
    with pytest.raises(ValueError,match='Duplicate'): load_joint_calibrations(path)


@pytest.mark.parametrize('sweep', [(1000, 3000), (3000, 1000)])
def test_capture_retains_extrema_in_either_order(monkeypatch: pytest.MonkeyPatch, sweep: tuple[int, int]) -> None:
    monkeypatch.setattr('scripts.so101.so101_calibrate.time.sleep', lambda _: None)
    samples = iter([*sweep, 2000, 2100])
    keys = iter(['', '', ' '])
    bus = SimpleNamespace(read_positions=lambda ids: {1: next(samples)})
    motion = Motion({1: 2000})
    capture(bus, [1], motion, lambda: next(keys))
    assert motion.minimum == {1: 1000}
    assert motion.maximum == {1: 3000}
    assert motion.previous == {1: 2100}


@pytest.mark.parametrize('reverse', [False, True])
def test_entire_session_detects_shuffled_ids_and_saves_mapping(tmp_path: Path,monkeypatch: pytest.MonkeyPatch,reverse: bool) -> None:
    monkeypatch.setattr('scripts.so101.so101_calibrate.time.sleep',lambda _:None)
    ids=[1,2,3,4,5,6]; order=[2,6,1,5,3,4]
    snapshots=[]; keys=[]; current={i:2000 for i in ids}
    for servo in order:
        snapshots.append(dict(current)) # Initial baseline
        for value in ([3000, 1000] if reverse else [1000, 3000]):
            current[servo] = value
            snapshots.append(dict(current))
        current[servo] = 2000
        snapshots.extend([dict(current), dict(current)]) # Finish away from either endpoint
        keys.extend(['', '', ' '])
    class Bus:
        def set_torque_enabled(self, ids: Sequence[int], enabled: bool) -> None:
            raise AssertionError("Calibration capture must not change torque")
        def read_positions(self,ids: Sequence[int]) -> Mapping[int, int]: return next(readings)
        def write_positions(self,positions: Mapping[int, int]) -> None: raise AssertionError('Must not drive motors')
    readings=iter(snapshots); presses=iter(keys)
    result=calibrate(Bus(),ids,tmp_path/'calibration.json',lambda:next(presses))
    assert [result[n].servo_id for n in ORDER]==order
    assert all(c.zero_tick==2000 and c.direction==1 for c in result.values())
    assert all(c.to_ticks(c.limits_rad[0]) == 1000 and c.to_ticks(c.limits_rad[1]) == 3000
               for c in result.values())


def test_interruption_preserves_completed_checkpoint(tmp_path: Path,monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr('scripts.so101.so101_calibrate.time.sleep',lambda _:None)
    values=iter([2000,1000,1000,3000,3000,2000,2000,2000,2000])
    class ActualBus:
        def write_positions(self, positions: Mapping[int, int]) -> None:
            raise AssertionError("Must not drive motors")
        def set_torque_enabled(self, ids: Sequence[int], enabled: bool) -> None:
            raise AssertionError("Calibration capture must not change torque")
        def read_positions(self,ids: Sequence[int]) -> Mapping[int, int]:
            v=next(values)
            return {i:v if i==1 else 2000 for i in ids}
    presses=iter(['', '', ' '])
    def key() -> str:
        try: return next(presses)
        except StopIteration: raise KeyboardInterrupt
    path=tmp_path/'partial.json'
    with pytest.raises(KeyboardInterrupt): calibrate(ActualBus(),list(range(1,7)),path,key)
    assert set(json.loads(path.read_text()))=={'gripper'}


def test_terminal_settings_restored_on_interrupt(monkeypatch: pytest.MonkeyPatch) -> None:
    import_target = 'scripts.so101.so101_calibrate'
    calls=[]
    monkeypatch.setattr(import_target+'.sys.stdin',type('Input',(),{'fileno':lambda self:0})())
    monkeypatch.setattr(import_target+'.termios.tcgetattr',lambda fd:['saved'])
    monkeypatch.setattr(import_target+'.tty.setcbreak',lambda fd:calls.append('cbreak'))
    monkeypatch.setattr(import_target+'.termios.tcsetattr',lambda *args:calls.append(args))
    with pytest.raises(KeyboardInterrupt):
        with keyboard(): raise KeyboardInterrupt
    assert calls[0]=='cbreak' and calls[-1][-1]==['saved']


def test_cli_cleanup_never_enables_or_drives(monkeypatch: pytest.MonkeyPatch,tmp_path: Path) -> None:
    calls=[]
    class Transport:
        def __init__(self,*args: object,**kwargs: object) -> None: pass
        def __enter__(self) -> Transport: return self
        def __exit__(self,*args: object) -> None: calls.append('closed')
        def read_positions(self,ids: Sequence[int]) -> Mapping[int, int]: return {i:2000 for i in ids}
        def set_torque_enabled(self,ids: Sequence[int],enabled: bool) -> None:
            assert enabled is False
            calls.append('disable')
        def write_positions(self,values: Mapping[int, int]) -> None: raise AssertionError('Unexpected motion command')
    def interrupt(*args: object) -> None: raise KeyboardInterrupt
    target='scripts.so101.so101_calibrate'
    monkeypatch.setattr(target+'.sys.argv',['calibrate','--port','fake','--output',str(tmp_path/'out.json')])
    monkeypatch.setattr(target+'.sys.stdin',type('Input',(),{'isatty':lambda self:True})())
    monkeypatch.setattr('builtins.input',lambda prompt:'')
    monkeypatch.setattr(target+'.FeetechTransport',Transport)
    monkeypatch.setattr(target+'.keyboard',lambda:nullcontext(lambda:''))
    monkeypatch.setattr(target+'.calibrate',interrupt)
    with pytest.raises(KeyboardInterrupt): main()
    assert calls==['disable','disable','closed']


def test_port_picker_requires_selection_and_retries(monkeypatch: pytest.MonkeyPatch) -> None:
    target = 'scripts.so101.so101_calibrate'
    monkeypatch.setattr(target+'.list_ports.comports', lambda: [
        SimpleNamespace(device='/dev/adapter', description='USB servo adapter')])
    answers = iter(['99', '1'])
    monkeypatch.setattr('builtins.input', lambda _: next(answers))
    assert select_port(None, TerminalUI()) == '/dev/adapter'
    with pytest.raises(ValueError, match='YOUR_PORT'):
        select_port('/dev/tty.usbmodemYOUR_PORT', TerminalUI())
    monkeypatch.setattr('builtins.input', lambda _: 'q')
    with pytest.raises(KeyboardInterrupt):
        select_port(None, TerminalUI())


def test_redirected_ui_has_no_escape_sequences(capsys: pytest.CaptureFixture[str]) -> None:
    ui = TerminalUI()
    ui.heading('Joint 1/6')
    ui.activity('LOW', Motion({1:2000}))
    ui.line('Saved', '32')
    output = capsys.readouterr().out
    assert 'Joint 1/6' in output and 'Saved' in output
    assert '\033' not in output


def test_no_color_and_live_line_cleanup(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    target = 'scripts.so101.so101_calibrate'
    monkeypatch.setattr(target+'.sys.stdout.isatty', lambda: True)
    monkeypatch.setenv('TERM', 'xterm-256color')
    monkeypatch.setenv('NO_COLOR', '1')
    ui = TerminalUI()
    assert ui.paint('Saved', '32') == 'Saved'
    ui.activity('LOW', Motion({1:2000}))
    assert ui.live
    ui.clear()
    assert not ui.live
    assert 'Waiting for movement' in capsys.readouterr().out


@pytest.mark.parametrize('role', ['follower', 'leader', 'both'])
def test_cli_arm_sessions_have_distinct_defaults(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, role: str,
) -> None:
    target = 'scripts.so101.so101_calibrate'
    sessions: list[tuple[Any, ...]] = []
    monkeypatch.setattr(target+'.__file__', str(tmp_path/'scripts/so101/so101_calibrate.py'))
    monkeypatch.setattr(target+'.sys.argv', ['calibrate', '--arm', role])
    monkeypatch.setattr(target+'.sys.stdin', SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr(target+'.calibrate_arm', lambda *args: sessions.append(args))
    main()
    assert [s[0] for s in sessions] == (['follower', 'leader'] if role == 'both' else [role])
    for arm, port, baudrate, ids, output in sessions:
        assert port is None and baudrate == 1_000_000 and ids == list(range(1, 7))
        assert output.name == ('leader-calibration.json' if arm == 'leader' else 'calibration.json')


def test_cli_both_calibrates_independent_buses_and_files(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path,
) -> None:
    target = 'scripts.so101.so101_calibrate'
    calls = []
    follower = tmp_path/'follower.json'
    leader = tmp_path/'leader.json'
    class Transport:
        def __init__(self, port: str, **kwargs: object) -> None: self.port = port
        def __enter__(self) -> "Transport":
            calls.append((self.port, 'open'))
            return self
        def __exit__(self, *args: object) -> None: calls.append((self.port, 'close'))
        def read_positions(self, ids: Sequence[int]) -> Mapping[int, int]: return {i: 2000 for i in ids}
        def set_torque_enabled(self, ids: Sequence[int], enabled: bool) -> None:
            assert enabled is False
            calls.append((self.port, 'disable'))
        def write_positions(self, values: Mapping[int, int]) -> None: raise AssertionError('Unexpected motion command')
    def teach(bus: Transport, ids: list[int], output: Path,
              key: Callable[[], str]) -> dict[JointName, JointCalibration]:
        offset = 0 if bus.port == 'follower' else 100
        mapping = {n: make_calibration(i, 1000+offset, 3000+offset, 2000+offset)
                   for n, i in zip(JOINTS, ids)}
        save_checkpoint(output, mapping)
        return load_joint_calibrations(output)
    monkeypatch.setattr(target+'.sys.argv', ['calibrate', '--arm', 'both',
        '--port', 'follower', '--leader-port', 'leader', '--output', str(follower),
        '--leader-output', str(leader), '--leader-ids', '7', '8', '9', '10', '11', '12'])
    monkeypatch.setattr(target+'.sys.stdin', SimpleNamespace(isatty=lambda: True))
    monkeypatch.setattr('builtins.input', lambda _: '')
    monkeypatch.setattr(target+'.FeetechTransport', Transport)
    monkeypatch.setattr(target+'.keyboard', lambda: nullcontext(lambda: ''))
    monkeypatch.setattr(target+'.calibrate', teach)
    main()
    assert calls == [(role, action) for role in ('follower', 'leader')
                     for action in ('open', 'disable', 'disable', 'close')]
    assert load_joint_calibrations(follower)['gripper'].servo_id == 6
    assert load_joint_calibrations(leader)['gripper'].servo_id == 12
    assert load_joint_calibrations(leader)['gripper'].zero_tick == 2100


@pytest.mark.parametrize('failure', ['same_output', 'existing_leader', 'bad_leader_ids'])
def test_cli_checks_both_sessions_before_connecting(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path, failure: str,
) -> None:
    target = 'scripts.so101.so101_calibrate'
    follower = tmp_path/'follower.json'
    leader = follower if failure == 'same_output' else tmp_path/'leader.json'
    if failure == 'existing_leader': leader.write_text('preserve me')
    argv = ['calibrate', '--arm', 'both', '--output', str(follower), '--leader-output', str(leader)]
    if failure == 'bad_leader_ids': argv += ['--leader-ids', '1', '1', '2', '3', '4', '5']
    monkeypatch.setattr(target+'.sys.argv', argv)
    def unexpected(*args: object) -> None: raise AssertionError('Must validate before connecting')
    monkeypatch.setattr(target+'.calibrate_arm', unexpected)
    with pytest.raises(SystemExit) as exc: main()
    assert exc.value.code == 2
    assert not follower.exists()
    if failure == 'existing_leader': assert leader.read_text() == 'preserve me'
