"""Manually teach SO-101 joint IDs, endpoints, direction and zero; never drive motors."""
from __future__ import annotations

import argparse
from collections.abc import Callable, Iterator, Mapping
from contextlib import contextmanager
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import select
import shutil
import sys
import tempfile
import termios
import time
import tty

from serial.tools import list_ports  # type: ignore[import-untyped]

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from airtruck.raw_api.so101.controller import (
    JOINTS, JointCalibration, joint_calibrations_dict, load_joint_calibrations,
)
from airtruck.raw_api.so101.io import JointName, ServoTransport
from airtruck.raw_api.so101.transport import FeetechTransport

# Distal to proximal along the kinematic chain, independent of the arm's pose.
ORDER: tuple[JointName, ...] = tuple(reversed(JOINTS))
DESCRIPTIONS = {
    "gripper": "moving finger / jaw",
    "wrist_roll": "rotation of the entire gripper about the forearm axis",
    "wrist_flex": "hinge immediately behind the gripper",
    "elbow_flex": "middle hinge between forearm and upper arm",
    "shoulder_lift": "hinge lifting the upper arm above the base",
    "shoulder_pan": "base swivel around the upright axis",
}
COUNTS = 4096  # STS3215 encoder resolution, not inferred from partial travel.


class TerminalUI:
    """Small terminal presentation layer; no device access or calibration state."""

    def __init__(self) -> None:
        self.interactive = sys.stdout.isatty() and os.environ.get("TERM") != "dumb"
        self.color = self.interactive and "NO_COLOR" not in os.environ
        self.live = False
        self.updated = 0.0

    def paint(self, text: str, code: str = "36") -> str:
        return f"\033[{code}m{text}\033[0m" if self.color else text

    def clear(self) -> None:
        if self.live:
            print("\r\033[2K", end="", flush=True)
            self.live = False

    def line(self, text: str = "", code: str = "0") -> None:
        self.clear()
        print(self.paint(text, code), flush=True)

    def heading(self, title: str) -> None:
        self.line()
        self.line(title, "1;36")
        self.line("─" * min(68, shutil.get_terminal_size().columns - 1), "2")

    def activity(self, label: str, motion: Motion) -> None:
        if not self.interactive or time.monotonic() - self.updated < 0.15:
            return
        self.updated = time.monotonic()
        leader = max(motion.previous, key=lambda i: motion.distance.get(i, 0))
        travel = motion.distance.get(leader, 0)
        scores = "  ".join(f"{i}:{motion.distance.get(i, 0):.2f}" for i in motion.previous)
        text = (f" {label} | Most motion: ID {leader}, tick {motion.previous[leader]}"
                f" | travel rad [{scores}]" if travel else f" {label} | Waiting for movement…")
        width = max(1, shutil.get_terminal_size().columns - 1)
        print("\r\033[2K" + self.paint(text[:width]), end="", flush=True)
        self.live = True


def select_port(port: str | None, ui: TerminalUI) -> str:
    if port:
        if "YOUR_PORT" in port:
            raise ValueError("Replace YOUR_PORT with your adapter path, or omit --port to choose it")
        return port
    ui.heading("CONNECT  /  Choose your USB servo adapter")
    ports = sorted(list_ports.comports(), key=lambda item: item.device)
    for index, item in enumerate(ports, 1):
        ui.line(f"  {index}. {item.device}  ·  {item.description}")
    if not ports:
        ui.line("No serial adapters found. Connect the adapter and enter its device path.", "33")
    while True:
        choice = input("Port number or device path (Q exits): ").strip()
        if choice.lower() == "q":
            raise KeyboardInterrupt
        if choice.isdigit() and 1 <= int(choice) <= len(ports):
            return str(ports[int(choice) - 1].device)
        if choice.startswith("/dev/") and "YOUR_PORT" not in choice:
            return choice
        ui.line("Choose a listed number or enter a /dev/... path.", "33")


@dataclass
class Motion:
    previous: dict[int, int]
    distance: dict[int, float] = field(default_factory=dict)
    wrapped: set[int] = field(default_factory=set)

    def update(self, values: Mapping[int, int]) -> None:
        if set(values) != set(self.previous):
            raise ValueError("Motor replies changed during recording")
        for servo_id, value in values.items():
            delta = value - self.previous[servo_id]
            if abs(delta) > COUNTS / 2:
                self.wrapped.add(servo_id)
            shortest = (delta + COUNTS // 2) % COUNTS - COUNTS // 2
            # Ignore one-count encoder jitter; score physical angular travel.
            self.distance[servo_id] = self.distance.get(servo_id, 0.0) + (
                abs(shortest) * math.tau / COUNTS if abs(shortest) > 1 else 0.0
            )
        self.previous = dict(values)

    def winner(self, used: set[int]) -> int:
        ranked = sorted(self.previous, key=lambda i: self.distance.get(i, 0), reverse=True)
        best = ranked[0]
        first = self.distance.get(best, 0)
        second = self.distance.get(ranked[1], 0) if len(ranked) > 1 else 0
        if first < 0.05 or first - second < 0.02:
            raise ValueError("Motion is too small or ambiguous; repeat with one joint moving clearly most")
        if best in used:
            raise ValueError(f"Most-moving ID {best} is already assigned; repeat this joint")
        if best in self.wrapped:
            raise ValueError("Selected encoder crossed 0/4095; current controller cannot represent wrapped travel")
        return best


def make_calibration(servo_id: int, low: int, high: int, zero: int) -> JointCalibration:
    if abs(high - low) * math.tau / COUNTS < 0.05:
        raise ValueError("Endpoints are too close; record both ends of the usable range")
    if not min(low, high) <= zero <= max(low, high):
        raise ValueError("Zero must lie between the captured endpoints")
    direction = 1 if high > low else -1
    limits = (direction * (low-zero) * math.tau / COUNTS,
              direction * (high-zero) * math.tau / COUNTS)
    return JointCalibration(servo_id, zero, direction, limits, COUNTS)


@contextmanager
def keyboard() -> Iterator[Callable[[], str]]:
    """Nonblocking space capture with terminal settings restored on every exit."""
    fd = sys.stdin.fileno()
    previous = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        def read_key() -> str:
            if not select.select([fd], [], [], 0)[0]:
                return ""
            key = os.read(fd, 1).decode()
            if key in ("q", "Q", "\x03"):
                raise KeyboardInterrupt
            return key
        yield read_key
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, previous)


def capture(bus: ServoTransport, ids: list[int], motion: Motion,
            key: Callable[[], str], label: str, *, choose_end: bool = False) -> tuple[str, dict[int, int]]:
    ui = TerminalUI()
    ui.line(f"  Record {label} endpoint", "1;33")
    ui.line("  Move gently to the endpoint, hold still, then press [SPACE].")
    if choose_end:
        ui.line("  [L] LOW first   [H] HIGH first   [Q] Quit", "2")
    else:
        ui.line("  [Q] Quit", "2")
    try:
        while True:
            values = dict(bus.read_positions(ids))
            motion.update(values)
            pressed = key()
            if choose_end and pressed.lower() in ("l", "h"):
                label = "LOW" if pressed.lower() == "l" else "HIGH"
                ui.line(f"  Selected {label} first. Hold there and press [SPACE].", "33")
            if pressed == " ":
                # Read again after the key, not a stale pre-key sample.
                values = dict(bus.read_positions(ids))
                motion.update(values)
                ui.line(f"  ✓ {label} captured", "32")
                return label, values
            ui.activity(label, motion)
            time.sleep(0.02)
    finally:
        ui.clear()


def save_checkpoint(path: Path, calibration: Mapping[JointName, JointCalibration]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    content = json.dumps(joint_calibrations_dict(calibration), indent=2) + "\n"
    temporary: str | None = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as output:
            temporary = output.name
            output.write(content)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)
        assert json.loads(path.read_text()) == json.loads(content)
    finally:
        if temporary is not None:
            Path(temporary).unlink(missing_ok=True)


def calibrate(bus: ServoTransport, ids: list[int], output: Path,
              key: Callable[[], str]) -> dict[JointName, JointCalibration]:
    ui = TerminalUI()
    result: dict[JointName, JointCalibration] = {}
    for step, name in enumerate(ORDER, 1):
        while True:
            ui.heading(f"JOINT {step}/6  /  {name.replace('_', ' ').upper()}")
            ui.line(f"  {DESCRIPTIONS[name]}")
            if name == "wrist_roll":
                ui.line("  Use safe usable endpoints; do not assume a physical hard stop.", "33")
            motion = Motion(dict(bus.read_positions(ids)))
            first_label, first = capture(bus, ids, motion, key, "LOW", choose_end=True)
            second_label = "HIGH" if first_label == "LOW" else "LOW"
            _, second = capture(bus, ids, motion, key, second_label)
            try:
                servo_id = motion.winner({c.servo_id for c in result.values()})
                low, high = (first[servo_id], second[servo_id]) if first_label == "LOW" else (second[servo_id], first[servo_id])
                ui.line(f"  Detected servo {servo_id} · {motion.distance[servo_id]:.3f} rad total motion", "36")
                zero = round((low + high) / 2)
                result[name] = make_calibration(servo_id, low, high, zero)
            except ValueError as exc:
                ui.line(f"  Not saved: {exc}", "1;31")
                ui.line("  Repeat both endpoints for this joint. [Q] quits.", "33")
                continue
            save_checkpoint(output, result)
            c = result[name]
            ui.line(f"  ✓ Saved {len(result)}/6  ·  servo {c.servo_id}  ·  zero {c.zero_tick} ticks", "1;32")
            ui.line(f"    Direction {c.direction:+d}  ·  limits {c.limits_rad[0]:+.3f} to {c.limits_rad[1]:+.3f} rad")
            break
    return load_joint_calibrations(output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="USB-to-TTL adapter path; omit for interactive selection")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument("--ids", type=int, nargs="+", default=list(range(1, 7)), help="Candidate IDs; joint assignment is discovered by movement")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[2]/"models/so101/runtime/calibration.json")
    args = parser.parse_args()
    if len(args.ids) != 6 or len(set(args.ids)) != 6 or any(not 0 <= i <= 253 for i in args.ids):
        parser.error("Supply exactly six unique configured motor IDs in 0..253")
    if args.output.exists():
        parser.error("Output already exists; choose a new --output to preserve it")
    if not sys.stdin.isatty():
        parser.error("Run in an interactive macOS/Linux terminal for SPACE capture")
    ui = TerminalUI()
    ui.heading("SO-101  /  HARDWARE CALIBRATION")
    ui.line("  Teach six joints by hand · no commanded motion", "2")
    args.port = select_port(args.port, ui)
    ui.line(f"  Adapter  {args.port}  ·  {args.baudrate:,} baud")
    ui.line(f"  IDs      {', '.join(map(str, args.ids))}")
    ui.line(f"  Save to  {args.output}")
    ui.heading("HOW IT WORKS")
    for step, name in enumerate(ORDER, 1):
        ui.line(f"  {step}. {name.replace('_', ' ')}")
    ui.line()
    ui.line("  LOW = your negative endpoint; HIGH = your positive endpoint.")
    ui.line("  These are angle directions, not height or encoder numbers.")
    ui.line("  Record either end first with [L]/[H], then capture with [SPACE].")
    ui.line("  Zero = midpoint of your endpoints, rounded to one encoder tick.")
    ui.line("  Each completed joint is saved. All six are needed to control the arm.")
    ui.line("  [Q] or [Ctrl-C] exits during recording.", "2")
    ui.heading("BEFORE YOU START")
    ui.line("  All motors will be unpowered. Support the arm before continuing.", "1;33")
    ui.line("  Move gently; never force stops. Wrist roll may have no hard stop.", "33")
    answer = input(ui.paint("  [ENTER] Disable torque and begin   [Q] Exit: ", "1"))
    if answer.strip().lower() == "q":
        raise KeyboardInterrupt
    ui.line("  Connecting and checking all six motor replies…", "36")
    with FeetechTransport(args.port, baudrate=args.baudrate) as bus:
        bus.read_positions(args.ids)  # Require a reply from every candidate before recording.
        try:
            bus.set_torque_enabled(args.ids, False)
            with keyboard() as key:
                calibrate(bus, args.ids, args.output, key)
        finally:
            bus.set_torque_enabled(args.ids, False)
    ui.heading("✓ CALIBRATION COMPLETE  /  6 of 6 saved")
    ui.line(f"  {args.output}", "32")
    ui.line("  Torque remains disabled. Support the arm before disconnecting.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        TerminalUI().line("\nStopped. Any completed joint records remain saved; an incomplete file cannot control the arm.", "33")
        sys.exit(130)
    except (OSError, ValueError, ExceptionGroup) as exc:
        TerminalUI().line(f"\nCalibration failed: {exc}", "1;31")
        if isinstance(exc, ExceptionGroup):
            for error in exc.exceptions:
                TerminalUI().line(f"  {error}", "31")
        sys.exit(1)
