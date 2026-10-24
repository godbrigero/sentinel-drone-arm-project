"""Copy a physical leader into a simulated or physical SO-101 follower."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import select
import sys
import time
import traceback

from airtruck.constants import (
    FOLLOWER_BAUDRATE, FOLLOWER_CALIBRATION, FOLLOWER_PORT,
    LEADER_BAUDRATE, LEADER_CALIBRATION, LEADER_PORT, RUN_MODE,
)
from airtruck.util.environment import RunMode
from airtruck.raw_api.so101.joints import JOINTS
from airtruck.raw_api.so101.controller import LeaderSO101, RealSO101
from airtruck.raw_api.so101.calibration import load_joint_calibrations
from airtruck.raw_api.so101.io import ArmController
from airtruck.raw_api.so101.position import PositionUnit
from airtruck.raw_api.so101.transport import FeetechTransport
from airtruck.testing_util import load_world

ROOT = Path(__file__).resolve().parents[1]


def show_angles(arm: ArmController) -> None:
    positions = arm.positions
    print("\nJoint               Angle (deg)      Limits (deg)")
    for name in JOINTS:
        degrees = positions[name].to(PositionUnit.DEGREES)
        low, high = degrees.range
        print(
            f"{name:18} {degrees.value:10.1f}"
            f"      {low:7.1f} .. {high:7.1f}"
        )


def follow(leader: LeaderSO101, follower: ArmController,
           tick: Callable[[], None] | None = None) -> None:
    """Exchange normalized [-1, 1] joint positions at up to 50 Hz."""
    print("Following leader. Enter show or quit; Ctrl-C also exits.")
    while True:
        started = time.monotonic()
        if select.select([sys.stdin], [], [], 0)[0]:
            text = sys.stdin.readline()
            if not text or text.strip().lower() in ("q", "quit", "exit"):
                return
            if text.strip().lower() == "show":
                show_angles(follower)
            elif text.strip():
                print("Enter show or quit. Joint targets come from the leader.")
        follower.set_normalized_positions(leader.normalized_positions)
        if tick is not None:
            tick()
        time.sleep(max(0.0, 0.02 - (time.monotonic() - started)))


def main() -> None:
    if RUN_MODE not in (RunMode.SIMULATION, RunMode.REAL):
        raise ValueError(f"Unsupported RUN_MODE: {RUN_MODE}")
    if not LEADER_PORT.strip():
        raise ValueError("Set LEADER_PORT in airtruck/constants.py to the leader USB adapter path")
    if RUN_MODE == RunMode.REAL:
        if not FOLLOWER_PORT.strip():
            raise ValueError("Set FOLLOWER_PORT in airtruck/constants.py to the follower USB adapter path")
        if Path(LEADER_PORT).resolve() == Path(FOLLOWER_PORT).resolve():
            raise ValueError("Leader and follower require separate USB adapter ports")
        if LEADER_CALIBRATION.resolve() == FOLLOWER_CALIBRATION.resolve():
            raise ValueError("Leader and follower require separate calibration files")
    leader_calibration = load_joint_calibrations(LEADER_CALIBRATION)
    follower_calibration = (
        load_joint_calibrations(FOLLOWER_CALIBRATION) if RUN_MODE == RunMode.REAL else None
    )
    print(f"PHYSICAL LEADER → {RUN_MODE.value.upper()} FOLLOWER")
    print("Support the leader: its torque will be disabled.")
    if RUN_MODE == RunMode.REAL:
        print("The follower will enable at its current pose, then move to match the leader.")
    if input("Press ENTER to start following, or type quit to cancel: ").strip():
        return
    with FeetechTransport(LEADER_PORT, baudrate=LEADER_BAUDRATE) as leader_transport:
        leader = LeaderSO101(leader_transport, leader_calibration)
        leader.start()
        # Verify all six leader replies before opening/enabling the follower.
        _ = leader.positions
        if RUN_MODE == RunMode.SIMULATION:
            with load_world(ROOT / "worlds/so101_bench") as world:
                world.open_window()
                follower = world.system("arm", ArmController)
                follow(leader, follower, lambda: world.advance(0.02))
        else:
            assert follower_calibration is not None
            with FeetechTransport(
                FOLLOWER_PORT, baudrate=FOLLOWER_BAUDRATE,
                gripper_servo_id=follower_calibration["gripper"].servo_id,
            ) as transport:
                follower = RealSO101(transport, follower_calibration)
                try:
                    follower.set_enabled(True)
                    follow(leader, follower)
                finally:
                    follower.set_enabled(False)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except (OSError, ValueError, ExceptionGroup) as exc:
        print(f"Control stopped: {exc}", file=sys.stderr)
        # Include grouped servo failures and any error preceding cleanup failure.
        traceback.print_exception(exc)
        raise SystemExit(1) from exc
