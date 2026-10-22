"""Simple SO-101 angle console; RUN_MODE chooses the local simulator or hardware."""
from __future__ import annotations

from collections.abc import Callable
import math
from pathlib import Path
import select
import sys

from airtruck.constants import CALIBRATION, PORT, RUN_MODE
from airtruck.util.environment import RunMode
from airtruck.raw_api.so101.controller import JOINTS, RealSO101, load_joint_calibrations
from airtruck.raw_api.so101.io import ArmController, JointName
from airtruck.raw_api.so101.transport import FeetechTransport
from airtruck.testing_util import load_world

ROOT = Path(__file__).resolve().parents[1]


def show_angles(arm: ArmController) -> None:
    positions, limits = arm.positions, arm.limits
    print("\nJoint               Angle (deg)      Limits (deg)")
    for name in JOINTS:
        low, high = limits[name]
        print(f"{name:18} {math.degrees(positions[name]):10.1f}"
              f"      {math.degrees(low):7.1f} .. {math.degrees(high):7.1f}")


def command(arm: ArmController, text: str) -> bool:
    """Apply one console command; return False to quit. Input angles are degrees."""
    words = text.strip().lower().split()
    if not words:
        return True
    if words in (["q"], ["quit"], ["exit"]):
        return False
    if words == ["show"]:
        show_angles(arm)
        return True
    if words == ["help"]:
        print("<joint> <degrees>  e.g. shoulder_pan 10 | show | quit")
        return True
    if len(words) != 2 or words[0] not in JOINTS:
        raise ValueError("Use a joint name and degrees, e.g. shoulder_pan 10; or show / quit")
    name: JointName = next(name for name in JOINTS if name == words[0])
    angle = math.radians(float(words[1]))
    low, high = arm.limits[name]
    if not math.isfinite(angle) or not low <= angle <= high:
        raise ValueError(f"{name} must be between {math.degrees(low):.1f} and {math.degrees(high):.1f} degrees")
    arm.set_positions({name: angle})
    print(f"Target: {name} = {math.degrees(arm.targets[name]):.2f} degrees")
    return True


def console(arm: ArmController, tick: Callable[[], None] | None = None) -> None:
    """Keep simulation stepping while waiting for a terminal line, on one thread."""
    show_angles(arm)
    print("\nEnter <joint> <degrees>, show, or quit. Ctrl-C also exits.")
    print("> ", end="", flush=True)
    while True:
        if tick is not None:
            tick()
        if not select.select([sys.stdin], [], [], 0 if tick else 0.05)[0]:
            continue
        text = sys.stdin.readline()
        if not text:  # EOF
            return
        try:
            if not command(arm, text):
                return
        except ValueError as exc:
            print(f"Invalid command: {exc}")
        print("> ", end="", flush=True)


def main() -> None:
    if RUN_MODE == RunMode.SIMULATION:
        print("SIMULATION · angles relative to the saved CAD pose")
        with load_world(ROOT / "worlds/so101_bench") as world:
            world.open_window()
            arm = world.system("arm", ArmController)
            console(arm, lambda: world.advance(0.02))
    elif RUN_MODE == RunMode.REAL:
        if not PORT.strip():
            raise ValueError("Set PORT in airtruck/constants.py to your USB servo adapter path")
        calibration = load_joint_calibrations(CALIBRATION)
        print("REAL HARDWARE · angles relative to your calibrated midpoint")
        print("Support the arm. Enabling holds its current pose; angle commands move it.")
        if input("Press ENTER to enable, or type quit to cancel: ").strip():
            return
        with FeetechTransport(PORT) as transport:
            arm = RealSO101(transport, calibration)
            try:
                arm.set_enabled(True)
                console(arm)
            finally:
                arm.set_enabled(False)
    else:
        raise ValueError(f"Unsupported RUN_MODE: {RUN_MODE}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except (OSError, ValueError, ExceptionGroup) as exc:
        print(f"Control stopped: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
