"""Edit individual arm controls, then advance their shared world once."""

from __future__ import annotations

import argparse
import json
import math
import time
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.base import Values
from airtruck.systems.so101 import SO101Arm, SO101Follower


def two_arms() -> list[SO101Follower]:
    return [
        SO101Follower(
            "left", position=(-0.08, 0, 0), initial_qpos={"elbow_flex": -3.14}
        ),
        SO101Follower(
            "right", position=(0.08, 0, 0), initial_qpos={"elbow_flex": -3.14}
        ),
    ]


def motion(
    arms: Sequence[SO101Arm],
    initial: dict[str, Values],
    time_s: float,
    phase: float = 0,
) -> None:
    """Infrastructure motion, not an expert for a success/reward task."""
    for index, arm in enumerate(arms):
        start = initial[arm.name]
        arm.controls["shoulder_pan.pos"] = float(
            np.clip(
                start["shoulder_pan.pos"]
                + (10 if index == 0 else -25) * math.sin(time_s * 1.2 + phase),
                -100,
                100,
            )
        )
        arm.controls["wrist_roll.pos"] = float(
            np.clip(
                start["wrist_roll.pos"]
                + 8 * math.sin(time_s * 0.8 + index * math.pi / 2 + phase),
                -100,
                100,
            )
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--world", type=Path, help="Base MJCF world without attached systems"
    )
    parser.add_argument("--seconds", type=float, default=8)
    parser.add_argument("--render", choices=("human", "rgb_array"))
    parser.add_argument(
        "--benchmark", action="store_true", help="Skip contact force extraction"
    )
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("seconds must be positive and finite")
    arms = two_arms()
    with SystemDriver(
        arms,
        world=args.world,
        render_mode=args.render,
        include_contacts=not args.benchmark,
    ) as driver:
        driver.reset(seed=42)
        initial = {arm.name: dict(arm.controls) for arm in arms}
        start = time.perf_counter()
        contacts = 0
        for _ in range(math.ceil(args.seconds / driver.control_dt)):
            tick = time.perf_counter()
            motion(arms, initial, float(driver.data.time))
            driver.step()
            contacts += len(driver.info["contacts"])
            if args.render == "rgb_array":
                driver.render()
            if args.render == "human":
                time.sleep(max(0, driver.control_dt - (time.perf_counter() - tick)))
        elapsed = time.perf_counter() - start
        print(
            json.dumps(
                {
                    "systems": [s.name for s in arms],
                    "simulated_seconds": float(driver.data.time),
                    "rollout_wall_seconds": elapsed,
                    "simulation_seconds_per_wall_second": driver.data.time / elapsed,
                    "physics_steps_per_action": driver.substeps,
                    "contact_samples": contacts,
                    "state": {arm.name: arm.state for arm in arms},
                },
                indent=2,
            )
        )


if __name__ == "__main__":
    main()
