"""Acknowledged STS3215 bus I/O using feetech-servo-sdk (protocol 0).

Requires preconfigured position-mode servos with write status replies enabled.
No EEPROM, calibration, mode, speed, acceleration, or torque changes on connect.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from threading import RLock
import sys
from typing import Protocol, cast

from airtruck.raw_api.so101.io import ServoTransport

import scservo_sdk as sdk  # type: ignore[import-untyped]


class _Port(Protocol):
    is_open: bool
    def setBaudRate(self, baudrate: int) -> bool: ...
    def closePort(self) -> None: ...


class _Packet(Protocol):
    def read2ByteTxRx(self, port: _Port, servo_id: int, address: int) -> tuple[int, int, int]: ...
    def write2ByteTxRx(self, port: _Port, servo_id: int, address: int, value: int) -> tuple[int, int]: ...
    def write1ByteTxRx(self, port: _Port, servo_id: int, address: int, value: int) -> tuple[int, int]: ...
    def getTxRxResult(self, result: int) -> str: ...
    def getRxPacketError(self, error: int) -> str: ...


class FeetechTransportError(OSError):
    """A bus/device failure; earlier writes in a batch may already have applied."""


class FeetechTransport(ServoTransport):
    """Raw 0..4095 ticks; calibration and joint names belong to RealSO101.

    Construction is inert. Call connect() or use a context manager. close() only
    closes serial I/O: it does NOT disable torque. Disable through RealSO101 first.
    Batches are sequential acknowledged transactions, not simultaneous motion.
    Voltage-only status replies warn and continue for every operation. A custom
    voltage_warning callback can format the warning; otherwise it prints to stderr.
    An explicitly configured gripper_servo_id also tolerates overload-only replies.
    Other faults remain strict. Warnings are emitted once per servo per connection.
    """

    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56
    BAUDRATES = {1_000_000, 500_000, 250_000, 128_000, 115_200, 57_600, 38_400, 19_200}

    def __init__(
        self, port: str, *, baudrate: int = 1_000_000,
        voltage_warning: Callable[[str], None] | None = None,
        gripper_servo_id: int | None = None,
    ) -> None:
        if not isinstance(port, str) or not port.strip():
            raise ValueError("An explicit serial port is required")
        if type(baudrate) is not int or baudrate not in self.BAUDRATES:
            raise ValueError("Unsupported STS3215 baudrate")
        self.port = port
        if gripper_servo_id is not None:
            self._ids([gripper_servo_id])
        self._gripper_servo_id = gripper_servo_id
        self._warned_gripper_overload = False
        self.baudrate = baudrate
        self._port: _Port | None = None
        self._packet: _Packet | None = None
        self._lock = RLock()
        self._voltage_warning = voltage_warning
        self._warned_voltage_ids: set[int] = set()

    def connect(self) -> None:
        with self._lock:
            if self._port is not None:
                return
            port = cast(_Port, sdk.PortHandler(self.port))
            packet = cast(_Packet, sdk.PacketHandler(0))
            try:
                # SDK setBaudRate opens the serial device at the selected rate.
                if not port.setBaudRate(self.baudrate):
                    raise FeetechTransportError(f"Cannot open {self.port} at {self.baudrate} baud")
            except BaseException:
                if port.is_open:
                    port.closePort()
                raise
            self._port, self._packet = port, packet

    def close(self) -> None:
        with self._lock:
            port = self._port
            self._port = self._packet = None
            self._warned_voltage_ids.clear()
            self._warned_gripper_overload = False
            if port is not None:
                port.closePort()

    def __enter__(self) -> FeetechTransport:
        self.connect()
        return self

    def __exit__(self, *args: object) -> None:
        self.close()

    def _connection(self) -> tuple[_Port, _Packet]:
        if self._port is None or self._packet is None:
            raise RuntimeError("Transport is disconnected; call connect() first")
        return self._port, self._packet

    @staticmethod
    def _ids(servo_ids: Sequence[int]) -> list[int]:
        ids = list(servo_ids)
        if any(type(i) is not int or not 0 <= i <= 253 for i in ids):
            raise ValueError("Servo IDs must be integers 0..253; broadcast is not allowed")
        if len(set(ids)) != len(ids):
            raise ValueError("Duplicate servo IDs")
        return ids

    def _check(self, packet: _Packet, servo_id: int, operation: str,
               result: int, error: int) -> None:
        if result == 0 and error == 32 and servo_id == self._gripper_servo_id:
            if not self._warned_gripper_overload:
                print(
                    f"\nWARNING: Gripper servo {servo_id} {operation}: overload "
                    "(device=32). Continuing. Further gripper overload warnings "
                    "are suppressed until reconnect.",
                    file=sys.stderr, flush=True,
                )
                self._warned_gripper_overload = True
            return
        # Tolerate only valid replies containing exactly the voltage status bit.
        if result == 0 and error == 1:
            if servo_id not in self._warned_voltage_ids:
                message = (
                    f"Servo {servo_id} {operation}: input voltage error (device=1). "
                    "Continuing; voltage may be too low or too high. "
                    "Further voltage-only warnings for this servo are suppressed until reconnect."
                )
                if self._voltage_warning is None:
                    print(f"\nWARNING: {message}", file=sys.stderr, flush=True)
                else:
                    self._voltage_warning(message)
                self._warned_voltage_ids.add(servo_id)
            return
        if result != 0 or error != 0:
            raise FeetechTransportError(
                f"Servo {servo_id} {operation}: communication={result} "
                f"({packet.getTxRxResult(result)}), device={error} "
                f"({packet.getRxPacketError(error)})"
            )

    def read_positions(self, servo_ids: Sequence[int]) -> Mapping[int, int]:
        ids = self._ids(servo_ids)
        with self._lock:
            port, packet = self._connection()
            positions: dict[int, int] = {}
            for i in ids:
                value, result, error = packet.read2ByteTxRx(port, i, self.PRESENT_POSITION)
                self._check(packet, i, "read position", result, error)
                # Reject sign-magnitude/multi-turn feedback; never mask it into a valid angle.
                if not 0 <= value <= 4095:
                    raise FeetechTransportError(f"Servo {i} returned non-single-turn position {value}")
                positions[i] = value
            return positions

    def write_positions(self, positions: Mapping[int, int]) -> None:
        values = dict(positions)
        ids = self._ids(list(values))
        if any(type(v) is not int or not 0 <= v <= 4095 for v in values.values()):
            raise ValueError("Position ticks must be integers 0..4095")
        with self._lock:
            port, packet = self._connection()
            for i in ids:
                result, error = packet.write2ByteTxRx(port, i, self.GOAL_POSITION, values[i])
                self._check(packet, i, "write position", result, error)

    def set_torque_enabled(self, servo_ids: Sequence[int], enabled: bool) -> None:
        ids = self._ids(servo_ids)
        if type(enabled) is not bool:
            raise ValueError("enabled must be a boolean")
        with self._lock:
            port, packet = self._connection()
            failures: list[Exception] = []
            for i in ids:
                try:
                    result, error = packet.write1ByteTxRx(port, i, self.TORQUE_ENABLE, int(enabled))
                    self._check(packet, i, "enable torque" if enabled else "disable torque", result, error)
                except Exception as exc:
                    if enabled:
                        raise  # Do not enable additional motors after a failure.
                    failures.append(exc)  # Still try to disable every requested motor.
            if failures:
                raise ExceptionGroup("Torque disable failed for one or more servos", failures)
