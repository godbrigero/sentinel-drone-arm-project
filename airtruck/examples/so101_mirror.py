"""Torque-off live check of the physical follower against the simulated training setup.

One window shows, side by side:

- top row: both cameras exactly as the policy receives them (SmolVLA's own
  resize-and-pad), and the simulated arm posed from the measured joints (side);
- bottom row: the training frames whose recorded arm state is nearest the
  measured state, put through the same resize-and-pad, and the simulated arm
  from the front.

Move the real arm by hand: every simulated joint must turn the same physical way
and reach the same end stops. A mismatch means the follower's LeRobot calibration
does not match the simulated normalized mapping that the policy was trained with.

    uv run --locked python -m airtruck.examples.so101_mirror

The follower is opened through LeRobot's leader role, which disables torque and
only reads, using the follower's own calibration file. Support the arm before
starting: it goes limp. Esc or Ctrl+C stops.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import cv2
import mujoco
import numpy as np
import pygame
from numpy.typing import NDArray
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.so_follower import SO101Follower as LeRobotSO101Follower
from lerobot.robots.so_follower import SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101LeaderConfig

from airtruck.examples.so101_cameras import CONFIG, ROLES, checkpoint, load_config, make_camera
from airtruck.examples.so101_policy import NAMES, frame, model_view, padding
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.so101 import SO101Leader

TILE = 320
HEADER = 64
SIDE, FRONT = 0.0, 90.0
"""Offscreen MuJoCo camera azimuths (degrees) for the mirrored arm."""


class Reference:
    """Training frames nearest a joint state, from the checkpoint's own dataset."""

    def __init__(self, checkpoint_path: Path) -> None:
        dataset = json.loads((checkpoint_path / "train_config.json").read_text())["dataset"]
        self.dataset = LeRobotDataset(dataset["repo_id"], root=dataset["root"], video_backend="pyav")
        self.states = np.asarray(self.dataset.hf_dataset["observation.state"], dtype=np.float32)
        self.index = -1
        self.images: dict[str, NDArray[np.uint8]] = {}

    def nearest(self, state: NDArray[np.float32]) -> tuple[int, float]:
        distances = np.linalg.norm(self.states - state, axis=1)
        index = int(np.argmin(distances))
        if index != self.index:
            item: dict[str, Any] = self.dataset[int(index)]  # pyright: ignore[reportAssignmentType]
            self.images = {
                role: (item[f"observation.images.{role}"].permute(1, 2, 0).numpy() * 255).astype(np.uint8)
                for role in ROLES
            }
            self.index = index
        return index, float(distances[index])


def surface(image: NDArray[np.uint8]) -> pygame.Surface:
    """Fit an RGB image into a square tile, keeping its aspect ratio."""
    height, width = image.shape[:2]
    scale = TILE / max(height, width)
    resized = cv2.resize(image, (round(width * scale), round(height * scale)), interpolation=cv2.INTER_AREA)
    tile = np.zeros((TILE, TILE, 3), np.uint8)
    tile[: resized.shape[0], : resized.shape[1]] = resized
    return pygame.surfarray.make_surface(tile.swapaxes(0, 1))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=CONFIG)
    parser.add_argument("--seconds", type=float, help="stop after this long (default: until Esc or Ctrl+C)")
    parser.add_argument("--no-reference", action="store_true", help="skip the nearest training frames")
    args = parser.parse_args()
    config = load_config(args.config)
    _, checkpoint_path = checkpoint(config)
    pad = padding(PreTrainedConfig.from_pretrained(checkpoint_path))
    reference = None if args.no_reference else Reference(checkpoint_path)
    follower = config["follower"]
    saved = LeRobotSO101Follower(SO101FollowerConfig(port="", id=follower["id"], use_degrees=False))
    if not saved.calibration:
        sys.exit(f"No follower calibration at {saved.calibration_fpath}")
    arm = SO101Leader(
        "arm", hardware=True,
        config=SO101LeaderConfig(
            port=follower["port"], id=follower["id"], use_degrees=False,
            calibration_dir=saved.calibration_dir,
        ),
    )
    cameras = {role: make_camera(config["cameras"][role]) for role in ROLES}
    input("Torque will be released on the follower. Support the arm, then press Enter: ")
    driver = SystemDriver([arm], collisions=False, include_contacts=False)
    driver.model.vis.global_.offwidth = max(driver.model.vis.global_.offwidth, TILE)
    driver.model.vis.global_.offheight = max(driver.model.vis.global_.offheight, TILE)
    renderer = mujoco.Renderer(driver.model, TILE, TILE)
    view = mujoco.MjvCamera()
    view.lookat[:] = (0, -0.12, 0.15)
    view.distance, view.elevation = 0.75, -10.0
    blank = np.zeros((TILE, TILE, 3), np.uint8)
    pygame.init()
    screen = pygame.display.set_mode((3 * TILE, 2 * TILE + HEADER))
    pygame.display.set_caption("SO-101 mirror: what the policy sees (Esc stops)")
    font = pygame.font.SysFont("menlo", 14)
    try:
        for camera in cameras.values():
            camera.connect()
        with driver:
            driver.reset()
            if arm.hardware_active and not arm.bus.is_calibrated:
                sys.exit(f"Motor calibration differs from {saved.calibration_fpath}; recalibrate first.")
            print("Move each real joint and compare the window; Esc or Ctrl+C stops.")

            def sim(azimuth: float) -> NDArray[np.uint8]:
                view.azimuth = azimuth
                renderer.update_scene(driver.data, camera=view)
                return renderer.render()

            def update() -> None:
                state = np.array([arm.state[name] for name in NAMES], dtype=np.float32)
                live = {role: model_view(frame(camera), pad) for role, camera in cameras.items()}
                lines = ["joints   pan   lift  elbow  wflex   roll   grip",
                         "real  " + " ".join(f"{v:6.1f}" for v in state)]
                train: dict[str, NDArray[np.uint8]] = {role: blank for role in ROLES}
                if reference is not None:
                    index, distance = reference.nearest(state)
                    train = {role: model_view(reference.images[role], pad) for role in ROLES}
                    lines.append("train " + " ".join(f"{v:6.1f}" for v in reference.states[index])
                                 + f"   (frame {index}, distance {distance:.1f})")
                layout = [
                    ("front: model input, live", live["front"]),
                    ("wrist: model input, live", live["wrist"]),
                    ("sim arm, side", sim(SIDE)),
                    ("front: training, nearest state", train["front"]),
                    ("wrist: training, nearest state", train["wrist"]),
                    ("sim arm, front", sim(FRONT)),
                ]
                screen.fill((20, 20, 20))
                for row, text in enumerate(lines):
                    screen.blit(font.render(text, True, (230, 230, 230)), (8, 4 + 19 * row))
                for index, (label, image) in enumerate(layout):
                    x, y = index % 3 * TILE, HEADER + index // 3 * TILE
                    screen.blit(surface(image), (x, y))
                    screen.blit(font.render(label, True, (255, 210, 0), (0, 0, 0)), (x + 4, y + TILE - 20))
                pygame.display.flip()
                for event in pygame.event.get():
                    if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                        driver.stop()

            steps = None if args.seconds is None else max(1, round(args.seconds / driver.control_dt))
            try:
                driver.run(update=update, steps=steps, realtime=True)
            except KeyboardInterrupt:
                pass
    finally:
        for camera in cameras.values():
            if camera.is_connected:
                camera.disconnect()
        renderer.close()
        pygame.quit()
        driver.close()


if __name__ == "__main__":
    main()
