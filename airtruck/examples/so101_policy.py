"""Run the fine-tuned SmolVLA yellow-cube policy on the physical SO-101 follower.

Reads the configuration written by `so101_cameras` (follower port, front/wrist
cameras, checkpoint). The follower is the project's `SO101Follower` with
`hardware=True`, ticked by `SystemDriver` at the 30 Hz training rate. Each tick
reads both cameras and the measured joints, runs LeRobot's `predict_action`
(the same call as the simulated evaluation) and writes the targets to
`controls`. LeRobot's `max_relative_target` caps every per-tick joint step.
"""

from __future__ import annotations

import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import mujoco
import numpy as np
import pygame
import torch
from numpy.typing import NDArray
from lerobot.cameras.camera import Camera
from lerobot.common.control_utils import predict_action
from lerobot.policies.common.vla_utils import resize_with_pad
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.utils.device_utils import auto_select_torch_device, get_safe_torch_device
from lerobot.robots.so_follower import SO101FollowerConfig

from airtruck.examples.so101_cameras import (
    CONFIG, HEIGHT, ROLES, VARIANTS, WIDTH, checkpoint, load_config, make_camera,
)
from airtruck.examples.so101_realsense_bridge import RealSenseBridge
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.base import ROOT
from airtruck.systems.so101 import JOINTS, SO101Follower

NAMES = [f"{joint}.pos" for joint in JOINTS]
LOW = np.array([-100, -100, -100, -100, -100, 0], dtype=np.float32)
HIGH = np.full(6, 100, dtype=np.float32)
READY_RAD = np.array([0, 0.4, -0.8, 0.2, 0, -1.0])
"""Training start pose (`so101_pick.REST`), radians in JOINTS order."""
SUBSTEPS = 10
"""Physics substeps of the measured-pose proxy per tick; it is not simulating the arm."""


class ProxyFollower(SO101Follower):
    """Hardware follower whose model timestep divides the policy period exactly."""

    def __init__(self, *args: Any, fps: int, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self.timestep = 1 / (fps * SUBSTEPS)

    def make_spec(self) -> mujoco.MjSpec:
        spec = super().make_spec()
        spec.option.timestep = self.timestep
        return spec


def proxy_world(fps: int) -> mujoco.MjSpec:
    """Bench world with the same timestep as `ProxyFollower`."""
    world = mujoco.MjSpec.from_file(str(ROOT / "worlds/bench/scene.xml"))
    world.option.timestep = 1 / (fps * SUBSTEPS)
    return world


def ready_pose(driver: SystemDriver, arm: SO101Follower) -> dict[str, float]:
    """Training ready pose in normalized units via the model's actuator mapping.

    This is the simulation mapping; it equals the physical pose only if the
    real calibration matches the simulated joint zero and direction.
    """
    pose = {}
    for joint, q in zip(JOINTS, READY_RAD):
        actuator = driver.model.actuator(f"{arm.name}/{joint}")
        (low, high), (in_low, in_high) = actuator.ctrlrange, actuator.user[:2]
        pose[f"{joint}.pos"] = float(in_low + (q - low) / (high - low) * (in_high - in_low))
    return pose


class HomeNotReached(RuntimeError):
    """The measured pose stayed away from the ready pose after homing."""


def home_errors(state: dict[str, float], ready: dict[str, float], tolerance: float) -> dict[str, float]:
    """Joints whose measured position is farther than `tolerance` from the ready pose."""
    return {k: round(state[k] - ready[k], 1) for k in NAMES if abs(state[k] - ready[k]) > tolerance}


def frame(camera: Camera | RealSenseBridge) -> NDArray[np.uint8]:
    image = np.asarray(camera.async_read(timeout_ms=500), dtype=np.uint8)
    if image.shape[:2] != (HEIGHT, WIDTH):
        image = np.asarray(cv2.resize(image, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA), dtype=np.uint8)
    return image


def is_grayscale(image: NDArray[np.uint8], tolerance: float = 2.0) -> bool:
    """True when the colour channels are (nearly) identical, e.g. the D435i's infrared "Depth" interface.

    The policy was trained on colour views; the yellow cube is barely visible in infrared.
    """
    channels = image.astype(np.float32)
    return float(np.mean(np.abs(channels[..., 0] - channels[..., 1]) + np.abs(channels[..., 1] - channels[..., 2]))) < tolerance


def model_view(image: NDArray[np.uint8], pad: tuple[int, int]) -> NDArray[np.uint8]:
    """An RGB frame as SmolVLA's `prepare_images` gives it to the vision encoder.

    `pad` is the config's `resize_imgs_with_padding` (width, height): the image is
    resized without distortion and zero-padded on the left and top.
    """
    tensor = torch.from_numpy(np.ascontiguousarray(image)).permute(2, 0, 1)[None].float() / 255
    padded = resize_with_pad(tensor, pad[1], pad[0], pad_value=0)
    return np.asarray((padded[0].permute(1, 2, 0).clamp(0, 1) * 255).round().to(torch.uint8).numpy())


def padding(policy_config: Any) -> tuple[int, int]:
    """SmolVLA's `resize_imgs_with_padding` as (width, height)."""
    width, height = policy_config.resize_imgs_with_padding
    return int(width), int(height)


class Viewer:
    """Window with both camera frames exactly as captured and passed to the policy
    (WIDTH x HEIGHT RGB, shown 1:1 without scaling); Esc stops the run."""

    BAR = 30

    def __init__(self) -> None:
        pygame.init()
        self.screen = pygame.display.set_mode((len(ROLES) * WIDTH, HEIGHT + self.BAR))
        pygame.display.set_caption("SO-101 policy: camera feeds (Esc to stop)")
        self.font = pygame.font.SysFont("menlo", 15)

    def show(self, images: dict[str, NDArray[np.uint8]], text: str) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                return False
        self.screen.fill((0, 0, 0))
        for column, role in enumerate(ROLES):
            image = images[role]
            self.screen.blit(pygame.surfarray.make_surface(image.swapaxes(0, 1)), (column * WIDTH, self.BAR))
            label = self.font.render(f"{role} {image.shape[1]}x{image.shape[0]}", True, (255, 210, 0), (0, 0, 0))
            self.screen.blit(label, (column * WIDTH + 6, self.BAR + 6))
        self.screen.blit(self.font.render(text, True, (230, 230, 230)), (6, 6))
        pygame.display.flip()
        return True

    def close(self) -> None:
        pygame.quit()


class Recorder:
    """Saves every tick's camera frames (JPEG, as given to the policy) on a background thread and
    the measured state, action and time per tick to `log.npz` in `directory`."""

    def __init__(self, directory: Path) -> None:
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=True)
        self.rows: dict[str, list[Any]] = {"t": [], "state": [], "action": []}
        self.queue: queue.Queue[tuple[Path, NDArray[np.uint8]] | None] = queue.Queue()
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def _write(self) -> None:
        while (item := self.queue.get()) is not None:
            path, image = item
            cv2.imwrite(str(path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 95])

    def add(self, tick: int, t: float, images: dict[str, NDArray[np.uint8]], state: NDArray[np.float32],
            action: NDArray[np.float32]) -> None:
        for role, image in images.items():
            self.queue.put((self.directory / f"{role}_{tick:05d}.jpg", image))
        self.rows["t"].append(t)
        self.rows["state"].append(state)
        self.rows["action"].append(action)

    def close(self, **meta: Any) -> None:
        self.queue.put(None)
        self.thread.join()
        arrays: dict[str, Any] = {"names": np.array(NAMES), **{k: np.asarray(v) for k, v in {**self.rows, **meta}.items()}}
        np.savez(self.directory / "log.npz", **arrays)
        print(f"Recorded {len(self.rows['t'])} ticks to {self.directory}")


def ramp(driver: SystemDriver, arm: SO101Follower, target: dict[str, float], seconds: float, fps: int) -> None:
    """Linear joint-space move from the measured pose to `target`, then one second of settling."""
    start = dict(arm.state)
    ticks = max(1, round(seconds * fps))
    for tick in range(1, ticks + 1):
        blend = tick / ticks
        arm.controls.update({k: start[k] + blend * (target[k] - start[k]) for k in NAMES})
        driver.run(steps=1, realtime=True)
    driver.run(steps=fps, realtime=True)


def confirm(message: str, skip: bool) -> None:
    if skip:
        return
    if not sys.stdin.isatty():
        sys.exit(f"{message}\nNo terminal to confirm; pass --yes to proceed.")
    if input(f"{message}\nPress Enter to continue, or type anything to abort: ").strip():
        sys.exit("Aborted.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--variant", choices=VARIANTS,
                        help="checkpoint to run (default: policy.variant in the config)")
    parser.add_argument("--seconds", type=float, default=20.0, help="policy run time")
    parser.add_argument("--max-step", type=float, default=5.0,
                        help="largest per-tick joint change in normalized units (LeRobot max_relative_target)")
    parser.add_argument("--home-seconds", type=float, default=4.0,
                        help="ramp to the training ready pose first; 0 skips it")
    parser.add_argument("--home-tolerance", type=float, default=10.0,
                        help="largest joint error (normalized units) allowed after homing")
    parser.add_argument("--dry-run", action="store_true",
                        help="run cameras and inference but hold the measured pose")
    parser.add_argument("--view", action=argparse.BooleanOptionalAction, default=True,
                        help="show both camera feeds while running (default; --no-view hides them)")
    parser.add_argument("--yes", action="store_true", help="skip the confirmation prompts")
    parser.add_argument("--record", type=Path,
                        help="save each tick's camera frames, measured state and action to this directory")
    parser.add_argument("--device", help="torch device, e.g. cuda, cuda:1, mps or cpu "
                        "(default: CUDA, then MPS, then CPU)")
    args = parser.parse_args()
    if not (args.seconds > 0 and args.max_step > 0 and args.home_seconds >= 0):
        parser.error("seconds and max-step must be positive; home-seconds nonnegative")

    config = load_config(args.config)
    fps = int(config["fps"])
    variant, path = checkpoint(config, args.variant)
    task = str(config["policy"]["task"])
    device = get_safe_torch_device(args.device) if args.device else auto_select_torch_device()
    print(f"Loading the {variant} checkpoint {path} on {device}...")
    policy = SmolVLAPolicy.from_pretrained(path)
    policy.to(device).eval()
    pre, post = make_pre_post_processors(
        policy.config, pretrained_path=str(path),
        preprocessor_overrides={"device_processor": {"device": device.type}},
    )

    cameras = {role: make_camera(config["cameras"][role]) for role in ROLES}
    arm = ProxyFollower(
        "arm", hardware=True, fps=fps,
        config=SO101FollowerConfig(
            port=config["follower"]["port"], id=config["follower"]["id"],
            use_degrees=False, max_relative_target=args.max_step,
        ),
    )
    viewer = Viewer() if args.view else None
    recorder = Recorder(args.record) if args.record else None
    driver = SystemDriver([arm], world=proxy_world(fps), control_dt=1 / fps,
                          collisions=False, include_contacts=False)
    stopped = False
    try:
        for role, camera in cameras.items():
            camera.connect()
            print(f"{role}: {camera} connected")
            if is_grayscale(frame(camera)):
                sys.exit(
                    f"The {role} camera ({config['cameras'][role]}) gives a grayscale image: this is usually the "
                    "RealSense's infrared interface (macOS 'Depth' camera, or a Linux IR /dev/video node) or a "
                    "camera at a changed index. The policy needs colour views. Re-run so101_cameras and check "
                    "the feeds."
                )

        def observation() -> tuple[dict[str, Any], dict[str, NDArray[np.uint8]]]:
            images = {role: frame(camera) for role, camera in cameras.items()}
            state = np.array([arm.state[name] for name in NAMES], dtype=np.float32)
            return {
                "observation.state": state,
                **{f"observation.images.{role}": image for role, image in images.items()},
            }, images

        def infer(obs: dict[str, Any]) -> NDArray[np.float32]:
            action = predict_action(obs, policy, device, pre, post,
                                    use_amp=False, task=task, robot_type="so101_follower")
            return np.asarray(action.squeeze(0).to("cpu").numpy(), dtype=np.float32)

        with driver:
            driver.reset()  # Connects, verifies calibration and holds the measured pose.
            rest = dict(arm.state)
            print("Measured pose:", {k: round(v, 1) for k, v in rest.items()})
            started = time.perf_counter()
            infer(observation()[0])  # Warm-up; the first MPS/CUDA call is slow.
            print(f"Warm-up inference {time.perf_counter() - started:.1f} s")

            stats = {"tick": 0, "clipped": 0, "late": 0, "slowest": 0.0}
            grasp: dict[str, float] = {}  # measured pose when the gripper first closed (cube still on the table)
            try:
                if args.home_seconds and not args.dry_run:
                    ready = ready_pose(driver, arm)
                    confirm(f"Move to the training ready pose {ready} over {args.home_seconds:g} s?", args.yes)
                    ramp(driver, arm, ready, args.home_seconds, fps)
                    errors = home_errors(arm.state, ready, args.home_tolerance)
                    if errors:
                        raise HomeNotReached(
                            f"Homing stopped short (measured - ready): {errors}. Check the joint "
                            "directions with so101_mirror before running the policy."
                        )

                policy.reset()
                pre.reset()
                post.reset()
                if not args.dry_run:
                    confirm(f"Run the policy on the real arm for {args.seconds:g} s (Ctrl+C stops)?", args.yes)

                def update() -> None:
                    tick_start = time.perf_counter()
                    obs, images = observation()
                    raw = infer(obs)
                    if recorder is not None:
                        recorder.add(int(stats["tick"]), tick_start, images, obs["observation.state"], raw)
                    values = np.clip(raw, LOW, HIGH)
                    stats["clipped"] += int(np.any(values != raw))
                    if not args.dry_run:
                        arm.controls.update({name: float(v) for name, v in zip(NAMES, values)})
                    elapsed = time.perf_counter() - tick_start
                    stats["late"] += int(elapsed > driver.control_dt)
                    stats["slowest"] = max(stats["slowest"], elapsed)
                    text = " ".join(f"{v:6.1f}" for v in values)
                    if stats["tick"] % fps == 0:
                        print(f"t={stats['tick'] / fps:5.1f}s action [{text}] late ticks {stats['late']}")
                    stats["tick"] += 1
                    if not grasp and arm.state["gripper.pos"] < 30:
                        grasp.update(arm.state)
                    if viewer is not None and not viewer.show(images, f"t={stats['tick'] / fps:5.1f}s [{text}]"):
                        driver.stop()

                driver.run(update=update, steps=round(args.seconds * fps), realtime=True)
            except KeyboardInterrupt:
                stopped = True
            except HomeNotReached as error:
                print(error)
                stopped = True
            arm.hold()
            driver.step()  # Send the hold before releasing anything.
            print(
                f"{'Stopped' if stopped else 'Finished'} after {stats['tick']} ticks: "
                f"{stats['late']} over {1000 / fps:.0f} ms (slowest {stats['slowest'] * 1000:.0f} ms), "
                f"{stats['clipped']} actions clipped to range."
            )
            if not args.dry_run and sys.stdin.isatty():
                input("Holding the pose. Support the arm, then press Enter to release torque: ")
            elif not args.dry_run:
                # No terminal: set anything held back down where it was grasped, open, rise, then fold back
                # to the pose measured at start before torque is released.
                print("No terminal: releasing at the grasp pose, then returning to the starting pose.")
                if grasp:
                    ramp(driver, arm, {**grasp, "gripper.pos": arm.state["gripper.pos"]}, 2.5, fps)
                ramp(driver, arm, {**dict(arm.state), "gripper.pos": 60.0}, 1.0, fps)
                ramp(driver, arm, {**ready_pose(driver, arm), "gripper.pos": 60.0}, 2.5, fps)
                ramp(driver, arm, rest, max(args.home_seconds, 3.0), fps)
    finally:
        if recorder is not None:
            recorder.close(variant=variant, checkpoint=str(path), fps=fps, cameras=json.dumps(config["cameras"]))
        for camera in cameras.values():
            if camera.is_connected:
                camera.disconnect()
        if viewer is not None:
            viewer.close()
        driver.close()


if __name__ == "__main__":
    main()
