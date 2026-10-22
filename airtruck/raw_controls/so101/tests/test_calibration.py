"""Offline endpoint teaching, ID detection and calibration file contract."""
import json
from contextlib import nullcontext
import math
from types import SimpleNamespace
import pytest
from airtruck.raw_controls.so101.controller import JOINTS, load_joint_calibrations
from scripts.so101.so101_calibrate import Motion, ORDER, make_calibration, save_checkpoint, capture, calibrate, keyboard, main, TerminalUI, select_port


def test_distal_to_proximal_order():
    assert ORDER == ('gripper', 'wrist_roll', 'wrist_flex', 'elbow_flex', 'shoulder_lift', 'shoulder_pan')


@pytest.mark.parametrize('low,high,zero,direction', [(1000,3000,2000,1),(3000,1000,2000,-1),(0,4095,2048,1)])
def test_signed_endpoints(low, high, zero, direction):
    c = make_calibration(4,low,high,zero)
    assert c.direction == direction
    assert c.to_radians(low) == pytest.approx(c.limits_rad[0])
    assert c.to_radians(high) == pytest.approx(c.limits_rad[1])
    assert c.to_ticks(0) == zero
    assert c.to_ticks(c.limits_rad[0]) == low
    assert c.to_ticks(c.limits_rad[1]) == high


def test_most_motion_tolerates_other_joint_and_backtracking():
    m=Motion({1:2000,2:2000,3:2000})
    for sample in ({1:2100,2:2010,3:2001},{1:2200,2:2020,3:2000},{1:2000,2:2030,3:2001}):
        m.update(sample)
    assert m.winner(set()) == 1
    assert m.distance[1] == pytest.approx(400*math.tau/4096)
    with pytest.raises(ValueError,match='already assigned'): m.winner({1})


def test_ambiguous_motion_and_wrap_rejected():
    m=Motion({1:2000,2:2000});m.update({1:2100,2:2100})
    with pytest.raises(ValueError,match='ambiguous'): m.winner(set())
    m=Motion({1:4090,2:2000});m.update({1:30,2:2000})
    assert m.distance[1] < .1
    with pytest.raises(ValueError,match='crossed'): m.winner(set())
    with pytest.raises(ValueError,match='replies'): m.update({1:40})


def test_invalid_zero_and_no_travel():
    with pytest.raises(ValueError,match='between'): make_calibration(1,1000,3000,3500)
    with pytest.raises(ValueError,match='close'): make_calibration(1,2000,2001,2000)


def test_json_roundtrip_partial_duplicate_and_invalid(tmp_path):
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


def test_capture_allows_high_first_and_fresh_space_read(monkeypatch):
    monkeypatch.setattr('scripts.so101.so101_calibrate.time.sleep',lambda _:None)
    class Bus:
        def read_positions(self,ids): return {1:next(samples)}
    samples=iter([2000,2100,2200]);keys=iter(['h',' '])
    label,values=capture(Bus(),[1],Motion({1:2000}),lambda:next(keys),'LOW',choose_end=True)
    assert label=='HIGH' and values=={1:2200}


def test_entire_session_detects_shuffled_ids_and_saves_mapping(tmp_path,monkeypatch):
    monkeypatch.setattr('scripts.so101.so101_calibrate.time.sleep',lambda _:None)
    ids=[1,2,3,4,5,6]; order=[2,6,1,5,3,4]
    snapshots=[]; keys=[]; current={i:2000 for i in ids}
    for servo in order:
        snapshots.append(dict(current)) # Initial baseline
        current[servo]=1000
        snapshots.extend([dict(current),dict(current)]) # First capture
        current[servo]=3000
        snapshots.extend([dict(current),dict(current)]) # Second capture
        keys.extend([' ',' '])
    class Bus:
        def read_positions(self,ids): return next(readings)
        def write_positions(self,positions): raise AssertionError('Must not drive motors')
    readings=iter(snapshots); presses=iter(keys)
    result=calibrate(Bus(),ids,tmp_path/'calibration.json',lambda:next(presses))
    assert [result[n].servo_id for n in ORDER]==order
    assert all(c.zero_tick==2000 for c in result.values())


def test_interruption_preserves_completed_checkpoint(tmp_path,monkeypatch):
    monkeypatch.setattr('scripts.so101.so101_calibrate.time.sleep',lambda _:None)
    values=iter([2000,1000,1000,3000,3000,2000,2000,2000,2000])
    class ActualBus:
        def read_positions(self,ids):
            v=next(values)
            return {i:v if i==1 else 2000 for i in ids}
    presses=iter([' ',' '])
    def key():
        try: return next(presses)
        except StopIteration: raise KeyboardInterrupt
    path=tmp_path/'partial.json'
    with pytest.raises(KeyboardInterrupt): calibrate(ActualBus(),list(range(1,7)),path,key)
    assert set(json.loads(path.read_text()))=={'gripper'}


def test_terminal_settings_restored_on_interrupt(monkeypatch):
    import_target = 'scripts.so101.so101_calibrate'
    calls=[]
    monkeypatch.setattr(import_target+'.sys.stdin',type('Input',(),{'fileno':lambda self:0})())
    monkeypatch.setattr(import_target+'.termios.tcgetattr',lambda fd:['saved'])
    monkeypatch.setattr(import_target+'.tty.setcbreak',lambda fd:calls.append('cbreak'))
    monkeypatch.setattr(import_target+'.termios.tcsetattr',lambda *args:calls.append(args))
    with pytest.raises(KeyboardInterrupt):
        with keyboard(): raise KeyboardInterrupt
    assert calls[0]=='cbreak' and calls[-1][-1]==['saved']


def test_cli_cleanup_never_enables_or_drives(monkeypatch,tmp_path):
    calls=[]
    class Transport:
        def __init__(self,*args,**kwargs): pass
        def __enter__(self): return self
        def __exit__(self,*args): calls.append('closed')
        def read_positions(self,ids): return {i:2000 for i in ids}
        def set_torque_enabled(self,ids,enabled):
            assert enabled is False
            calls.append('disable')
        def write_positions(self,values): raise AssertionError('Unexpected motion command')
    def interrupt(*args): raise KeyboardInterrupt
    target='scripts.so101.so101_calibrate'
    monkeypatch.setattr(target+'.sys.argv',['calibrate','--port','fake','--output',str(tmp_path/'out.json')])
    monkeypatch.setattr(target+'.sys.stdin',type('Input',(),{'isatty':lambda self:True})())
    monkeypatch.setattr('builtins.input',lambda prompt:'')
    monkeypatch.setattr(target+'.FeetechTransport',Transport)
    monkeypatch.setattr(target+'.keyboard',lambda:nullcontext(lambda:''))
    monkeypatch.setattr(target+'.calibrate',interrupt)
    with pytest.raises(KeyboardInterrupt): main()
    assert calls==['disable','disable','closed']


def test_port_picker_requires_selection_and_retries(monkeypatch):
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


def test_redirected_ui_has_no_escape_sequences(capsys):
    ui = TerminalUI()
    ui.heading('Joint 1/6')
    ui.activity('LOW', Motion({1:2000}))
    ui.line('Saved', '32')
    output = capsys.readouterr().out
    assert 'Joint 1/6' in output and 'Saved' in output
    assert '\033' not in output


def test_no_color_and_live_line_cleanup(monkeypatch, capsys):
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
