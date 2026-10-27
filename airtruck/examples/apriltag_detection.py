"""Hover the Sentinel model near a ground AprilTag and report simulated detections."""

from __future__ import annotations

import argparse
import math

import mujoco
import numpy as np

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.apriltag import TAGS, AprilTagConfig, TagDetection, add_apriltag
from airtruck.systems.base import ROOT
from airtruck.systems.sentinel import SentinelDrone


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag-id", type=int, default=0)
    parser.add_argument("--tag-size", type=float, default=0.3, help="Black edge, m")
    parser.add_argument("--distance", type=float, default=1.4, help="Tag north, m")
    parser.add_argument("--height", type=float, default=1.0, help="Climb, m")
    parser.add_argument("--seconds", type=float, default=6)
    parser.add_argument("--render", action="store_true")
    args = parser.parse_args()
    if not all(
        math.isfinite(v) and v > 0
        for v in (args.tag_size, args.distance, args.height, args.seconds)
    ):
        parser.error("Sizes, distances and durations must be positive and finite")
    world = mujoco.MjSpec.from_file(str(ROOT / "worlds/flight_arena/scene.xml"))
    add_apriltag(
        world, args.tag_id, size_m=args.tag_size, position=(args.distance, 0, 0.0011)
    )
    drone = SentinelDrone("drone", apriltags=AprilTagConfig(tag_size_m=args.tag_size))

    def report(detection: TagDetection) -> None:
        if detection.vehicle == drone.name and detection.position_ned_m is not None:
            print(
                f"t={detection.timestamp_s:5.2f}s tag {detection.tag_id} via {detection.camera}: "
                f"camera {np.round(detection.position_m, 3).tolist()} m, "
                f"NED {np.round(detection.position_ned_m, 3).tolist()} m",
                flush=True,
            )

    unsubscribe = TAGS.subscribe(report)
    try:
        with SystemDriver(
            [drone], world=world, render_mode="human" if args.render else None
        ) as driver:
            driver.reset()
            drone.takeoff(args.height)
            driver.run(
                steps=math.ceil(args.seconds / driver.control_dt), realtime=args.render
            )
    finally:
        unsubscribe()
    tag_ned = [args.distance, 0.0, -0.0016]
    latest = drone.latest_tag(tag_id=args.tag_id)
    if latest is None or latest.position_ned_m is None:
        print("The tag was not detected.")
    else:
        error = np.linalg.norm(latest.position_ned_m - tag_ned)
        print(f"Latest NED error against the placed tag: {error * 100:.1f} cm")


if __name__ == "__main__":
    main()
