"""Run an SO-101 beside the Sentinel drone (or another prepared drone MJCF)."""

import argparse
import math
from pathlib import Path

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.drone import Drone
from airtruck.systems.sentinel import SPAWN_HEIGHT_M, SentinelDrone
from airtruck.systems.so101 import SO101Follower


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--drone-model",
        type=Path,
        help="Another prepared drone MJCF (default: Sentinel)",
    )
    parser.add_argument("--drone-body", default="base")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument(
        "--height",
        type=float,
        default=0.5,
        help="Simulated takeoff height above the initial pose in meters",
    )
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    if not all(
        math.isfinite(value) and value > 0 for value in (args.seconds, args.height)
    ):
        parser.error("seconds and height must be positive and finite")
    arm = SO101Follower("arm")
    # Placed clear of the arm, feet on the floor.
    drone = (
        SentinelDrone("drone", position=(0.6, 0, SPAWN_HEIGHT_M))
        if args.drone_model is None
        else Drone(
            "drone",
            model=args.drone_model,
            root_body=args.drone_body,
            position=(0.6, 0, SPAWN_HEIGHT_M),
        )
    )
    with SystemDriver(
        [arm, drone], render_mode="human" if args.render else None
    ) as driver:
        driver.reset()
        arm.controls["shoulder_pan.pos"] = 20
        drone.takeoff(args.height)
        driver.run(
            steps=math.ceil(args.seconds / driver.control_dt), realtime=args.render
        )
        print(drone.state)


if __name__ == "__main__":
    main()
