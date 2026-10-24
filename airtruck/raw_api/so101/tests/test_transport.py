"""Exercise the SDK boundary without opening a serial port."""
from __future__ import annotations
from collections.abc import Mapping
from airtruck.raw_api.so101.position import JointPosition, PositionUnit
from types import SimpleNamespace
from airtruck.raw_api.so101.io import ServoTransport
import pytest
import scservo_sdk  # type: ignore[import-untyped]
from airtruck.raw_api.so101.transport import FeetechTransport, FeetechTransportError
from airtruck.raw_api.so101.controller import RealSO101
from airtruck.raw_api.so101.calibration import JointCalibration
from airtruck.raw_api.so101.joints import JOINTS


@pytest.fixture
def sdk(monkeypatch: pytest.MonkeyPatch) -> SimpleNamespace:
    state = SimpleNamespace(events=[], opened=False, fail_open=False, fail_id=None,
                            result=-6, error=0, position=2048)

    class Port:
        is_open = False
        def setBaudRate(self, baud: int) -> bool:
            self.is_open = True
            state.events.append(('open', baud))
            return not state.fail_open
        def closePort(self) -> None:
            self.is_open = False
            state.events.append(('close',))

    class Packet:
        def read2ByteTxRx(self, port: object, i: int, address: int) -> tuple[int, int, int]:
            state.events.append(('read', i, address))
            return state.position, state.result if i == state.fail_id else 0, state.error
        def write2ByteTxRx(self, port: object, i: int, address: int, value: int) -> tuple[int, int]:
            state.events.append(('write2', i, address, value))
            return state.result if i == state.fail_id else 0, state.error
        def write1ByteTxRx(self, port: object, i: int, address: int, value: int) -> tuple[int, int]:
            state.events.append(('write1', i, address, value))
            return state.result if i == state.fail_id else 0, state.error
        def getTxRxResult(self, result: int) -> str: return str(result)
        def getRxPacketError(self, error: int) -> str: return str(error)

    monkeypatch.setattr(scservo_sdk, 'PortHandler', lambda name: Port())
    def packet(version: int) -> Packet:
        assert version == 0
        return Packet()
    monkeypatch.setattr(scservo_sdk, 'PacketHandler', packet)
    return state


def test_lifecycle_and_registers(sdk: SimpleNamespace) -> None:
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


def test_failed_open_is_cleaned_up(sdk: SimpleNamespace) -> None:
    sdk.fail_open = True
    bus = FeetechTransport('/test')
    with pytest.raises(FeetechTransportError): bus.connect()
    assert sdk.events[-1] == ('close',)
    with pytest.raises(RuntimeError): bus.read_positions([1])
    sdk.fail_open = False
    with bus: assert bus.read_positions([1]) == {1: 2048}


@pytest.mark.parametrize('values', [{1: 10, 2: 4096}, {254: 0}, {True: 0}, {1: 1.5}, {1: True}])
def test_invalid_batch_has_no_writes(sdk: SimpleNamespace, values: Mapping[int, int]) -> None:
    with FeetechTransport('/test') as bus:
        before = list(sdk.events)
        with pytest.raises(ValueError): bus.write_positions(values)
        assert sdk.events == before


def test_reply_failure_and_partial_write(sdk: SimpleNamespace) -> None:
    with FeetechTransport('/test') as bus:
        sdk.fail_id = 2
        with pytest.raises(FeetechTransportError, match='Servo 2'):
            bus.write_positions({1: 100, 2: 200, 3: 300})
        assert ('write2', 1, 42, 100) in sdk.events
        assert ('write2', 3, 42, 300) not in sdk.events
        with pytest.raises(FeetechTransportError): bus.read_positions([1, 2])


def test_device_error_and_invalid_feedback(sdk: SimpleNamespace) -> None:
    with FeetechTransport('/test') as bus:
        sdk.error = 4
        with pytest.raises(FeetechTransportError, match='device=4'): bus.read_positions([1])
        sdk.error = 0
        sdk.position = 0x8001
        with pytest.raises(FeetechTransportError, match='single-turn'): bus.read_positions([1])
        with pytest.raises(ValueError): bus.read_positions([1, 1])


def test_disable_attempts_every_motor(sdk: SimpleNamespace) -> None:
    with FeetechTransport('/test') as bus:
        sdk.fail_id = 1
        with pytest.raises(ExceptionGroup): bus.set_torque_enabled([1, 2], False)
        assert ('write1', 2, 40, 0) in sdk.events
        sdk.events.clear()
        with pytest.raises(FeetechTransportError): bus.set_torque_enabled([1, 2], True)
        assert ('write1', 2, 40, 1) not in sdk.events


def test_real_controller_uses_transport(sdk: SimpleNamespace) -> None:
    calibration = {n: JointCalibration(i+1, 2048, 1, (-1., 1.)) for i, n in enumerate(JOINTS)}
    with FeetechTransport('/test') as bus:
        arm = RealSO101(bus, calibration)
        arm.set_enabled(True)
        arm.set_positions({'shoulder_pan': JointPosition(.3, PositionUnit.RADIANS, arm.positions['shoulder_pan'].limits_rad)})
        assert ('write2', 1, 42, 2244) in sdk.events
        assert arm.positions['shoulder_pan'].value == 0
        arm.set_enabled(False)


def test_voltage_warning_keeps_reads_and_disable_connected(sdk: SimpleNamespace) -> None:
    warnings: list[str] = []
    with FeetechTransport('/test', voltage_warning=warnings.append) as bus:
        sdk.error = 1
        assert bus.read_positions([6]) == {6: 2048}
        sdk.position = 2100
        assert bus.read_positions([6]) == {6: 2100}
        bus.set_torque_enabled([6], False)
        assert len(warnings) == 1 and 'Servo 6' in warnings[0]
        assert ('close',) not in sdk.events
        bus.write_positions({6: 2200})
        bus.set_torque_enabled([6], True)
    with bus:
        bus.read_positions([6])
    assert len(warnings) == 2


@pytest.mark.parametrize('result,error,position', [(-6, 1, 2048), (0, 3, 2048),
    (0, 5, 2048), (0, 4, 2048), (0, 1, 4096)])
def test_warning_policy_preserves_other_failures(
    sdk: SimpleNamespace, result: int, error: int, position: int,
) -> None:
    sdk.fail_id, sdk.result, sdk.error, sdk.position = 6, result, error, position
    with FeetechTransport('/test', voltage_warning=lambda _: None) as bus:
        with pytest.raises(FeetechTransportError):
            bus.read_positions([6])


@pytest.mark.parametrize('operation', ['read', 'write', 'enable', 'disable'])
def test_voltage_status_prints_warning_and_continues_by_default(
    sdk: SimpleNamespace, capsys: pytest.CaptureFixture[str], operation: str,
) -> None:
    sdk.error = 1
    with FeetechTransport('/test') as bus:
        if operation == 'read':
            assert bus.read_positions([6]) == {6: 2048}
        elif operation == 'write':
            bus.write_positions({6: 2200})
        else:
            bus.set_torque_enabled([6], operation == 'enable')
        assert bus.read_positions([6]) == {6: 2048}
        assert ('close',) not in sdk.events
        output = capsys.readouterr().err
        assert output.count('WARNING:') == 1
        assert 'Servo 6' in output and 'device=1' in output



def test_only_configured_gripper_overload_warns_and_continues(
    sdk: SimpleNamespace, capsys: pytest.CaptureFixture[str],
) -> None:
    sdk.error = 32
    with FeetechTransport('/test', gripper_servo_id=4) as bus:
        assert bus.read_positions([4]) == {4: 2048}
        bus.write_positions({4: 2200})
        bus.set_torque_enabled([4], True)
        bus.set_torque_enabled([4], False)
        assert capsys.readouterr().err.count('WARNING:') == 1
        with pytest.raises(FeetechTransportError):
            bus.write_positions({6: 2200})
        sdk.error = 36
        with pytest.raises(FeetechTransportError):
            bus.write_positions({4: 2200})
        sdk.error, sdk.fail_id, sdk.result = 32, 4, -6
        with pytest.raises(FeetechTransportError):
            bus.write_positions({4: 2200})


def test_feetech_explicitly_implements_transport_contract() -> None:
    assert ServoTransport in FeetechTransport.__mro__
    assert not FeetechTransport.__abstractmethods__
    class IncompleteTransport(ServoTransport):
        pass
    with pytest.raises(TypeError, match='abstract'):
        IncompleteTransport()  # type: ignore[abstract]
