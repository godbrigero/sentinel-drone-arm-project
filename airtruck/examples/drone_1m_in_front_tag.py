import math

import mujoco
import numpy as np
from numpy.typing import NDArray

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.apriltag import AprilTagConfig, TagDetection
from airtruck.systems.flight import rotation
from airtruck.systems.sentinel import SentinelDrone
from airtruck.systems.base import ROOT
from airtruck.systems.tag_object import AprilTagObject

STANDOFF_M = 1.0
TAG_HOME_M = (1.0, 0.0, 0.1)
TAG_SWAY_M = 0.75
TAG_PERIOD_S = 16.0


def detection_to_position_change(
    detection: TagDetection, drone: SentinelDrone
) -> NDArray[np.float64]:
    """NED correction that puts the camera 1 m horizontally in front of the tag.

    `position_ned_m` is the tag in NED. The north/east move places the camera's
    look direction on that tag at `STANDOFF_M`; down is left unchanged.
    """
    camera = drone.cameras.get(detection.camera)
    tag = detection.position_ned_m
    print(tag)
    if (
        detection.camera != "front_down"
        or camera is None
        or tag is None
        or not drone.state
    ):
        return np.zeros(3)

    body_to_ned = rotation(np.asarray(drone.state["quaternion"], dtype=np.float64))
    camera_ned = (
        np.asarray(drone.state["position"], dtype=np.float64)
        + body_to_ned @ camera.position_m
    )
    offset_xy = np.asarray(tag, dtype=np.float64)[:2] - camera_ned[:2]
    look_xy = (body_to_ned @ camera.rotation @ np.array([0.0, 0.0, 1.0]))[:2]
    look_norm = float(np.linalg.norm(look_xy))
    if look_norm < 1e-6:
        return np.zeros(3)
    delta_xy = offset_xy - look_xy / look_norm * STANDOFF_M
    return np.array([delta_xy[0], delta_xy[1], 0.0])


def tag_position(time_s: float) -> tuple[float, float, float]:
    """East-west sway about the home pose. Starts at home when time is zero."""
    sway = TAG_SWAY_M * math.sin(2 * math.pi * time_s / TAG_PERIOD_S)
    return (TAG_HOME_M[0] + sway, TAG_HOME_M[1] + sway, TAG_HOME_M[2])


def main():
    drone = SentinelDrone(
        "drone", apriltags=AprilTagConfig(tag_size_m=0.16, cameras=("front_down",))
    )
    tag = AprilTagObject("tag", size_m=0.16, position=TAG_HOME_M)
    world = mujoco.MjSpec.from_file(str(ROOT / "worlds/flight_arena/scene.xml"))

    with SystemDriver(
        [drone, tag], world=world, control_dt=0.02, render_mode="human"
    ) as driver:

        def move_tag() -> None:
            tag.set_position(tag_position(float(driver.data.time)))

        driver.reset()

        drone.takeoff(1.5)

        driver.run(steps=250, realtime=True, update=move_tag)

        def tag_position_change(detection: TagDetection) -> None:
            change = detection_to_position_change(detection, drone)
            drone.controls["position"][:2] = (
                np.asarray(drone.state["position"][:2], dtype=np.float64) + change[:2]
            )

        unsubscribe = drone.tag_board.subscribe(tag_position_change)

        driver.run(steps=300, realtime=True, update=move_tag)
        driver.run(steps=250, realtime=True, update=move_tag)
        unsubscribe()


if __name__ == "__main__":
    main()
