"""Closed-loop MuJoCo evaluation of a fine-tuned SmolVLA yellow-cube pick policy."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import av
import cv2
import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation  # type: ignore[import-untyped]
from lerobot.common.control_utils import predict_action
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.utils.device_utils import auto_select_torch_device, get_safe_torch_device

from airtruck.examples.so101_pick import (
    BASE,
    CAMERAS,
    DIMS,
    R,
    REST,
    make_driver,
    placements,
)
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.so101 import JOINTS, SO101Follower

TASK = "Pick up the yellow cube."
NAMES = [f"{joint}.pos" for joint in JOINTS]
LOW = np.array([-100, -100, -100, -100, -100, 0], dtype=np.float32)
HIGH = np.full(6, 100, dtype=np.float32)
FPS = 30
DEMO_FRAMES = 375  # Robust demonstrations last 12.5 s.


@dataclass
class Rollout:
    episode: int
    cube_xy_base_m: list[float]
    yaw_rad: float
    final_lift_m: float
    max_lift_m: float
    best_hold_s: float
    held_1s: bool
    success_at_demo_end: bool
    success_at_end: bool
    clipped_steps: int


class Video:
    """H.264 writer for side-by-side front | wrist frames."""

    def __init__(self, path: Path, width: int, height: int) -> None:
        self.container: Any = av.open(str(path), "w")
        self.stream: Any = self.container.add_stream("libx264", rate=FPS)
        self.stream.width, self.stream.height = width, height
        self.stream.pix_fmt = "yuv420p"

    def write(self, frame: NDArray[np.uint8]) -> None:
        for packet in self.stream.encode(av.VideoFrame.from_ndarray(frame, format="rgb24")):
            self.container.mux(packet)

    def close(self) -> None:
        for packet in self.stream.encode():
            self.container.mux(packet)
        self.container.close()


def camera_name(arm: SO101Follower, spec_name: str, body: str) -> str:
    return spec_name if body == "world" else f"{arm.name}/{spec_name}"


def lift(driver: SystemDriver) -> float:
    cube = driver.data.xpos[driver.model.body("cube").id]
    return float((cube - BASE) @ R[:, 2] - DIMS[2] / 2)


def held(driver: SystemDriver) -> bool:
    """Collector success test: lifted 5 cm, both jaw bodies touching, clear of the table."""
    model, data = driver.model, driver.data
    cube = model.geom("cube").id
    bodies = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        if cube in (contact.geom1, contact.geom2):
            other = contact.geom2 if contact.geom1 == cube else contact.geom1
            bodies.add(model.body(model.geom_bodyid[other]).name)
    return lift(driver) >= 0.05 and {"arm/hand", "arm/jaw"} <= bodies and "world" not in bodies


def reset(
    driver: SystemDriver, arm: SO101Follower, xy: NDArray[np.float64], yaw: float,
    rng: np.random.Generator,
) -> None:
    home = REST.copy()
    home[:5] += rng.uniform(-np.deg2rad(2.5), np.deg2rad(2.5), 5)
    arm.initial_qpos = {joint: float(q) for joint, q in zip(JOINTS, home)}
    driver.reset()
    target = BASE + R @ np.r_[xy, DIMS[2] / 2]
    address = driver.model.joint("cube_free").qposadr[0]
    driver.data.qpos[address : address + 3] = target + R[:, 2] * 0.001
    driver.data.qpos[address + 3 : address + 7] = Rotation.from_matrix(
        R @ Rotation.from_euler("z", yaw).as_matrix()
    ).as_quat(scalar_first=True)
    mujoco.mj_forward(driver.model, driver.data)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("checkpoint", type=Path, help="a checkpoints/NNNNNN/pretrained_model folder")
    parser.add_argument("--episodes", type=int, default=20)
    parser.add_argument("--seconds", type=float, default=18.0)
    parser.add_argument("--seed", type=int, default=99)
    parser.add_argument("--videos", type=int, default=3, help="record the first N rollouts")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--train-placements", action="store_true",
        help="use the collector's stratified training placements instead of fresh samples",
    )
    parser.add_argument("--device", help="torch device, e.g. cuda, cuda:1, mps or cpu "
                        "(default: CUDA, then MPS, then CPU)")
    args = parser.parse_args()
    if args.episodes < 1 or args.seconds <= 0:
        parser.error("episodes and seconds must be positive")
    args.out.mkdir(parents=True, exist_ok=True)

    device = get_safe_torch_device(args.device) if args.device else auto_select_torch_device()
    policy = SmolVLAPolicy.from_pretrained(args.checkpoint)
    policy.to(device).eval()
    pre, post = make_pre_post_processors(
        policy.config,
        pretrained_path=str(args.checkpoint),
        preprocessor_overrides={"device_processor": {"device": device.type}},
    )
    driver, arm = make_driver()
    renderers = {spec.name: mujoco.Renderer(driver.model, spec.height, spec.width) for spec in CAMERAS}
    cameras = {spec.name: camera_name(arm, spec.name, spec.body) for spec in CAMERAS}
    width = sum(spec.width for spec in CAMERAS)
    height = max(spec.height for spec in CAMERAS)
    rng = np.random.default_rng(args.seed)
    trained = placements(args.episodes) if args.train_placements else None
    results: list[Rollout] = []
    combined = Video(args.out / "rollouts.mp4", width, height) if args.videos > 0 else None
    try:
        for episode in range(args.episodes):
            if trained is not None:
                xy, yaw = trained[episode, :2].copy(), float(trained[episode, 2])
            else:
                xy = np.array([rng.uniform(-0.07, 0.07), rng.uniform(-0.28, -0.16)])
                yaw = float(rng.uniform(-np.pi / 3, np.pi / 3))
            reset(driver, arm, xy, yaw, rng)
            policy.reset()
            pre.reset()
            post.reset()
            video = Video(args.out / f"rollout_{episode}.mp4", width, height) if episode < args.videos else None
            max_lift, clipped, hold, best_hold, at_demo_end = 0.0, 0, 0, 0, False
            for step in range(int(args.seconds * FPS)):
                state = arm.get_observation()
                images: dict[str, NDArray[np.uint8]] = {}
                for name, renderer in renderers.items():
                    renderer.update_scene(driver.data, camera=cameras[name])
                    images[name] = renderer.render().copy()
                if video is not None:
                    frame = np.hstack([images[spec.name] for spec in CAMERAS])
                    cv2.putText(frame, f"rollout {episode + 1}  t={step / FPS:4.1f}s", (10, 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    video.write(frame)
                    if combined is not None:
                        combined.write(frame)
                observation: dict[str, Any] = {
                    "observation.state": np.array([state[name] for name in NAMES], dtype=np.float32),
                    **{f"observation.images.{name}": image for name, image in images.items()},
                }
                action = predict_action(
                    observation, policy, device, pre, post,
                    use_amp=False, task=TASK, robot_type="so101_follower",
                )
                raw = action.squeeze(0).to("cpu").numpy()
                values = np.clip(raw, LOW, HIGH)
                clipped += int(np.any(values != raw))
                arm.send_action({name: float(v) for name, v in zip(NAMES, values)})
                driver.step()
                max_lift = max(max_lift, lift(driver))
                ok = held(driver)
                hold = hold + 1 if ok else 0
                best_hold = max(best_hold, hold)
                if step == DEMO_FRAMES - 1:
                    at_demo_end = ok
            if video is not None:
                video.close()
            result = Rollout(
                episode=episode,
                cube_xy_base_m=[round(float(v), 4) for v in xy],
                yaw_rad=round(yaw, 3),
                final_lift_m=round(lift(driver), 4),
                max_lift_m=round(max_lift, 4),
                best_hold_s=round(best_hold / FPS, 2),
                held_1s=best_hold >= FPS,
                success_at_demo_end=at_demo_end,
                success_at_end=held(driver),
                clipped_steps=clipped,
            )
            print(json.dumps(asdict(result)), flush=True)
            results.append(result)
    finally:
        if combined is not None:
            combined.close()
        for renderer in renderers.values():
            renderer.close()
        driver.close()
    count = len(results)
    summary = {
        "checkpoint": str(args.checkpoint),
        "episodes": count,
        "held_1s_rate": sum(r.held_1s for r in results) / count,
        "success_at_demo_end_rate": sum(r.success_at_demo_end for r in results) / count,
        "success_at_end_rate": sum(r.success_at_end for r in results) / count,
        "rollouts": [asdict(r) for r in results],
    }
    (args.out / "eval.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(
        f"held>=1s {summary['held_1s_rate']:.0%}  at 12.5 s {summary['success_at_demo_end_rate']:.0%}  "
        f"at end {summary['success_at_end_rate']:.0%}  ({count} rollouts)"
    )


if __name__ == "__main__":
    main()
