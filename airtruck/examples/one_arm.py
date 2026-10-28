"""Teleoperate a follower from its linked leader in the shared world."""

from __future__ import annotations

from lerobot.robots.so_follower.config_so_follower import SO101FollowerConfig
from lerobot.teleoperators.so_leader.config_so_leader import SO101LeaderConfig

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.so101 import SO101Follower, SO101Leader
from airtruck.examples.constants import IS_HARDWARE


def main() -> None:
    follower_arm = SO101Follower(
        "front_follower",
        position=(-0.08, 0, 0),
        initial_qpos={"elbow_flex": -3.14},
        hardware=IS_HARDWARE,
        config=SO101FollowerConfig(
            port="/dev/tty.usbmodem5B7A0002901",
            id="airtruck_follower",
            use_degrees=False,
        ),
    )
    leader_arm = SO101Leader(
        name="front_leader",
        hardware=IS_HARDWARE,
        follower=follower_arm,
        config=SO101LeaderConfig(
            port="/dev/tty.usbmodem5B7A0002911",
            id="airtruck_leader",
            use_degrees=False,
        ),
    )

    with SystemDriver(
        [follower_arm, leader_arm],
        render_mode="human",
        control_dt=0.02,
    ) as driver:
        driver.reset(seed=42)

        driver.run(steps=20000, realtime=True)


if __name__ == "__main__":
    main()
