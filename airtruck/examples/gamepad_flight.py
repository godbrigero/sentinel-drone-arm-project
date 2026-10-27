"""Xbox flight demo with the shared Drone API and the Sentinel simulation model."""

from __future__ import annotations

import argparse
import math
import os
import time
from pathlib import Path

import mujoco
import numpy as np
import pygame
from pygame._sdl2 import controller

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.apriltag import AprilTagConfig, add_apriltag
from airtruck.systems.base import ROOT
from airtruck.systems.drone import Drone
from airtruck.systems.sentinel import SPAWN_HEIGHT_M, SentinelDrone
from airtruck.systems.flight import yaw_angle

BUTTONS = {
    "takeoff": pygame.CONTROLLER_BUTTON_A,
    "land": pygame.CONTROLLER_BUTTON_B,
    "hold": pygame.CONTROLLER_BUTTON_X,
    "reset": pygame.CONTROLLER_BUTTON_Y,
    "quit": pygame.CONTROLLER_BUTTON_START,
}


def axis(raw: int, deadzone: float = 0.14) -> float:
    value = max(-1.0, min(1.0, raw / 32767))
    return math.copysign(max(0, abs(value) - deadzone) / (1 - deadzone), value)


class XboxPad:
    def __init__(self) -> None:
        # Only SDL input is used. Its dummy video pump avoids another Cocoa window
        # and remains usable in mjpython's simulation thread beside the GLFW viewer.
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        pygame.display.init()
        controller.init()
        self.pad: controller.Controller | None = None
        self.previous: set[str] = set()
        self.name = "Waiting for Xbox controller"

    def poll(self) -> tuple[np.ndarray, set[str]] | None:
        pygame.event.pump()
        pygame.event.clear()
        if self.pad is not None and not self.pad.attached():
            self.pad.quit()
            self.pad = None
            self.previous.clear()
        if self.pad is None:
            for index in range(controller.get_count()):
                if controller.is_controller(index):
                    self.pad = controller.Controller(index)
                    self.name = self.pad.name
                    self.previous = {
                        key
                        for key, button in BUTTONS.items()
                        if self.pad.get_button(button)
                    }
                    print(f"Connected: {self.name}", flush=True)
                    break
        if self.pad is None:
            return None
        current = {
            key for key, button in BUTTONS.items() if self.pad.get_button(button)
        }
        pressed, self.previous = current - self.previous, current
        sticks = np.array(
            [
                axis(self.pad.get_axis(index))
                for index in (
                    pygame.CONTROLLER_AXIS_LEFTX,
                    pygame.CONTROLLER_AXIS_LEFTY,
                    pygame.CONTROLLER_AXIS_RIGHTX,
                    pygame.CONTROLLER_AXIS_RIGHTY,
                )
            ]
        )
        return sticks, pressed

    def close(self) -> None:
        if self.pad is not None:
            self.pad.quit()
        controller.quit()
        pygame.display.quit()


def steer(drone: Drone, sticks: np.ndarray, dt: float) -> None:
    if not drone.state["armed"] or not drone.state["offboard"]:
        return
    yaw, climb, right, forward = sticks * [1, -1, 1, -1]
    target_yaw = float(drone.controls["yaw"]) + yaw * 1.2 * dt
    drone.controls["yaw"] = (target_yaw + np.pi) % (2 * np.pi) - np.pi
    heading = yaw_angle(drone.state["quaternion"])
    velocity = np.array(
        [
            np.cos(heading) * forward - np.sin(heading) * right,
            np.sin(heading) * forward + np.cos(heading) * right,
            -climb * 0.7,
        ]
    )
    target = np.asarray(drone.controls["position"]) + velocity * dt
    error = target - drone.state["position"]
    if np.linalg.norm(error) > 0.6:
        target = drone.state["position"] + error * (0.6 / np.linalg.norm(error))
    drone.controls["position"] = np.clip(target, [-4, -4, -3.5], [4, 4, -0.17])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model", type=Path, help="Another prepared drone MJCF (default: Sentinel)"
    )
    parser.add_argument(
        "--spawn-height",
        type=float,
        default=SPAWN_HEIGHT_M,
        help="Initial body-origin height in meters (default: Sentinel feet on floor)",
    )
    parser.add_argument(
        "--apriltag",
        action="store_true",
        help="Place tag36h11 #0 ahead of the pad and detect it with the drone cameras",
    )
    args = parser.parse_args()
    if not math.isfinite(args.spawn_height) or args.spawn_height < 0:
        parser.error("spawn-height must be finite and nonnegative")
    world = mujoco.MjSpec.from_file(str(ROOT / "worlds/flight_arena/scene.xml"))
    if args.apriltag:
        add_apriltag(world, 0, size_m=0.3, position=(1.5, 0, 0.0011))
    label = "SENTINEL" if args.model is None else args.model.stem.upper()
    pad = XboxPad()
    try:
        apriltags = AprilTagConfig(tag_size_m=0.3) if args.apriltag else None
        position = (0.0, 0.0, args.spawn_height)
        drone = (
            SentinelDrone("drone", position=position, apriltags=apriltags)
            if args.model is None
            else Drone(
                "drone", model=args.model, position=position, apriltags=apriltags
            )
        )
        with SystemDriver([drone], world=world, render_mode="human") as driver:
            driver.reset()
            viewer = driver._viewer
            assert viewer is not None
            with viewer.lock():
                viewer.cam.lookat[:] = [0, 0, 0.7]
                viewer.cam.distance = 4.5
                viewer.cam.azimuth, viewer.cam.elevation = 135, -25
            disconnected = False
            last_log = 0.0
            print(
                "A takeoff | B land | X hold | Y reset | Menu quit | [ ] onboard cameras",
                flush=True,
            )
            while viewer.is_running():
                started = time.monotonic()
                sample = pad.poll()
                sticks, pressed = sample if sample is not None else (np.zeros(4), set())
                if sample is None and not disconnected:
                    drone.hold()
                    print(
                        "Controller disconnected: holding position until reconnection.",
                        flush=True,
                    )
                disconnected = sample is None
                if "quit" in pressed:
                    break
                if "reset" in pressed:
                    driver.reset()
                elif "land" in pressed and drone.state["armed"]:
                    drone.land()
                elif "takeoff" in pressed and not drone.state["armed"]:
                    drone.takeoff(1.0)
                elif "hold" in pressed:
                    drone.hold()
                steer(drone, sticks, driver.control_dt)
                position = drone.state["position"]
                status = "FLYING" if drone.state["armed"] else "DISARMED - press A"
                tag = drone.latest_tag()
                seen = (
                    f"\nTag {tag.tag_id} via {tag.camera}: NED {np.round(tag.position_ned_m, 2).tolist()} at t={tag.timestamp_s:.1f}s"
                    if tag is not None and tag.position_ned_m is not None
                    else "\nNo tag detected"
                    if args.apriltag
                    else ""
                )
                viewer.set_texts(
                    (
                        int(mujoco.mjtFontScale.mjFONTSCALE_150),
                        int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
                        f"{label} | {status}\n{pad.name}\nA Takeoff  B Land  X Hold  Y Reset  Menu Quit\nLeft stick: altitude / yaw\nRight stick: forward / sideways\n[ ] onboard cameras\nHeight: {-position[2]:.2f} m{seen}",
                        None,
                    )
                )
                driver.step()
                if started - last_log > 2.0:
                    print(
                        f"position_ned={position.round(2).tolist()} sticks={sticks.round(2).tolist()} armed={drone.state['armed']}",
                        flush=True,
                    )
                    last_log = started
                time.sleep(max(0, driver.control_dt - (time.monotonic() - started)))
    finally:
        pad.close()


if __name__ == "__main__":
    main()
