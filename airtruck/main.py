"""The same leader/follower objects for simulated and physical teleoperation."""

from __future__ import annotations

import argparse
import math

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.so101 import SO101Arm, SO101Follower, SO101Leader, release


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leader-port", default="")
    parser.add_argument("--follower-port", default="")
    parser.add_argument("--leader-id", default="airtruck_leader")
    parser.add_argument("--follower-id", default="airtruck_follower")
    parser.add_argument("--leader-hardware", action="store_true")
    parser.add_argument("--follower-hardware", action="store_true")
    parser.add_argument("--calibrate", choices=("leader", "follower", "both"))
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("seconds must be positive and finite")
    leader = SO101Leader(
        "leader",
        port=args.leader_port,
        id=args.leader_id,
        hardware=args.leader_hardware,
        position=(-0.25, 0, 0),
        follower="follower",
    )
    follower = SO101Follower(
        "follower",
        port=args.follower_port,
        id=args.follower_id,
        hardware=args.follower_hardware,
        position=(0.25, 0, 0),
    )
    if args.calibrate:
        selected: list[SO101Arm] = {
            "leader": [leader],
            "follower": [follower],
            "both": [leader, follower],
        }[args.calibrate]
        for arm in selected:
            if not arm.config.port:
                parser.error("Calibration requires the corresponding serial port")
            arm.hardware = True
            try:
                arm.bus.connect()
                arm.calibrate()
            finally:
                release(arm)
        return
    if leader.hardware and follower.hardware and args.leader_port == args.follower_port:
        parser.error("Leader and follower need distinct serial ports")
    with SystemDriver(
        [leader, follower], render_mode="human" if args.render else None
    ) as driver:
        driver.reset()
        driver.run(
            steps=math.ceil(args.seconds / driver.control_dt),
            realtime=True,
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Stopped.")
