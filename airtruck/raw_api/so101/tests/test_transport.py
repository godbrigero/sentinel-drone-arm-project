"""Exercise the SDK boundary without opening a serial port."""
from types import SimpleNamespace
import pytest
import scservo_sdk
from airtruck.raw_api.so101.transport import FeetechTransport, FeetechTransportError
from airtruck.raw_api.so101.controller import RealSO101, JointCalibration, JOINTS


@pytest.fixture
def sdk(monkeypatch):
    state = SimpleNamespace(events=[], opened=False, fail_open=False, fail_id=None,
                            result=-6, error=0, position=2048)

    class Port:
        is_open = False
        def setBaudRate(self, baud):
            self.is_open = True
            state.events.append(('open', baud))
            return not state.fail_open
        def closePort(self):
            self.is_open = False
            state.events.append(('close',))

    class Packet:
        def read2ByteTxRx(self, port, i, address):
            state.events.append(('read', i, address))
            return state.position, state.result if i == state.fail_id else 0, state.error
        def write2ByteTxRx(self, port, i, address, value):
            state.events.append(('write2', i, address, value))
            return state.result if i == state.fail_id else 0, state.error
        def write1ByteTxRx(self, port, i, address, value):
            state.events.append(('write1', i, address, value))
            return state.result if i == state.fail_id else 0, state.error
        def getTxRxResult(self, result): return str(result)
        def getRxPacketError(self, error): return str(error)

    monkeypatch.setattr(scservo_sdk, 'PortHandler', lambda name: Port())
    def packet(version):
        assert version == 0
        return Packet()
    monkeypatch.setattr(scservo_sdk, 'PacketHandler', packet)
    return state


def test_lifecycle_and_registers(sdk):
    bus = FeetechTransport('/test')
    assert sdk.events == []
    with pytest.raises(RuntimeError): bus.read_positions([1])
    with bus:
        bus.connect()  # Idempotent, no second open.
        assert bus.read_positions([1, 2]) == {1: 2048, 2: 2048}
        bus.write_positions({1: 1234})
        bus.set_torque_enabled([1], True)
    bus.close()
    assert sdk.events == [('open', 1_000_000), ('read', 1, 56), ('read', 2, 56),
                          ('write2', 1, 42, 1234), ('write1', 1, 40, 1), ('close',)]
    with pytest.raises(RuntimeError): bus.write_positions({1: 123})


def test_failed_open_is_cleaned_up(sdk):
    sdk.fail_open = True
    bus = FeetechTransport('/test')
    with pytest.raises(FeetechTransportError): bus.connect()
    assert sdk.events[-1] == ('close',)
    with pytest.raises(RuntimeError): bus.read_positions([1])
    sdk.fail_open = False
    with bus: assert bus.read_positions([1]) == {1: 2048}


@pytest.mark.parametrize('values', [{1: 10, 2: 4096}, {254: 0}, {True: 0}, {1: 1.5}, {1: True}])
def test_invalid_batch_has_no_writes(sdk, values):
    with FeetechTransport('/test') as bus:
        before = list(sdk.events)
        with pytest.raises(ValueError): bus.write_positions(values)
        assert sdk.events == before


def test_reply_failure_and_partial_write(sdk):
    with FeetechTransport('/test') as bus:
        sdk.fail_id = 2
        with pytest.raises(FeetechTransportError, match='Servo 2'):
            bus.write_positions({1: 100, 2: 200, 3: 300})
        assert ('write2', 1, 42, 100) in sdk.events
        assert ('write2', 3, 42, 300) not in sdk.events
        with pytest.raises(FeetechTransportError): bus.read_positions([1, 2])


def test_device_error_and_invalid_feedback(sdk):
    with FeetechTransport('/test') as bus:
        sdk.error = 4
        with pytest.raises(FeetechTransportError, match='device=4'): bus.read_positions([1])
        sdk.error = 0
        sdk.position = 0x8001
        with pytest.raises(FeetechTransportError, match='single-turn'): bus.read_positions([1])
        with pytest.raises(ValueError): bus.read_positions([1, 1])


def test_disable_attempts_every_motor(sdk):
    with FeetechTransport('/test') as bus:
        sdk.fail_id = 1
        with pytest.raises(ExceptionGroup): bus.set_torque_enabled([1, 2], False)
        assert ('write1', 2, 40, 0) in sdk.events
        sdk.events.clear()
        with pytest.raises(FeetechTransportError): bus.set_torque_enabled([1, 2], True)
        assert ('write1', 2, 40, 1) not in sdk.events


def test_real_controller_uses_transport(sdk):
    calibration = {n: JointCalibration(i+1, 2048, 1, (-1., 1.)) for i, n in enumerate(JOINTS)}
    with FeetechTransport('/test') as bus:
        arm = RealSO101(bus, calibration)
        arm.set_enabled(True)
        arm.set_positions({'shoulder_pan': .3})
        assert ('write2', 1, 42, 2244) in sdk.events
        assert arm.positions['shoulder_pan'] == 0
        arm.set_enabled(False)
