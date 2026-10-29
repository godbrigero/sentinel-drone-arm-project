"""Run one simulated leader fixed at its recorded location in the table scan."""

from __future__ import annotations

import argparse
import math

import mujoco

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.base import ROOT
from airtruck.systems.so101 import SO101Leader

WORLD = ROOT / "worlds/table_top/scene.xml"


def table_leader() -> SO101Leader:
    """Read the placement from the world's invisible CAD-root reference site."""
    world = mujoco.MjSpec.from_file(str(WORLD))
    site = world.site("leader_base")
    if site is None:
        raise ValueError("Table world is missing its leader_base placement")
    return SO101Leader(
        "leader",
        position=tuple(float(value) for value in site.pos),
        quaternion=tuple(float(value) for value in site.quat),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=30)
    parser.add_argument("--render", action="store_true", help="Open the MuJoCo viewer")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error("seconds must be positive and finite")
    leader = table_leader()
    with SystemDriver(
        [leader], world=WORLD, render_mode="human" if args.render else None
    ) as driver:
        driver.reset(seed=42)
        driver.run(
            steps=math.ceil(args.seconds / driver.control_dt), realtime=args.render
        )
        mount = tuple(
            float(value) for value in driver.data.site("leader_mount_reference").xpos
        )
        print(f"Leader mount fixed at {mount}; simulated {driver.data.time:.3f} s")


if __name__ == "__main__":
    main()
