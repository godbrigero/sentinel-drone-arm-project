"""Local LeRobot dataset → stock ACT trainer → Gymnasium rollout example."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

import numpy as np
import torch
from gymnasium import spaces
from lerobot.configs.default import DatasetConfig
from lerobot.configs.train import TrainPipelineConfig
from lerobot.configs.types import NormalizationMode
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.configuration_act import ACTConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.scripts.lerobot_train import train as train_policy

from airtruck.examples.two_arms import motion, two_arms
from airtruck.runtime.driver import SystemDriver
from airtruck.runtime.gymnasium import GymnasiumAdapter, feature_names
from airtruck.systems.base import ROOT
from airtruck.systems.so101 import SO101Follower

REPO_ID = "local/airtruck-two-arms"
TASK = "Independent two-arm motion example; no success objective is defined."


def schema(driver: SystemDriver, max_episode_steps: int) -> dict[str, Any]:
    adapter = GymnasiumAdapter(driver)
    if driver.world_path is None:
        raise ValueError("ACT checkpoints record a world file, not an in-memory MjSpec")
    sources = [
        driver.world_path,
        *dict.fromkeys(system.model_path for system in driver.systems),
    ]
    return {
        "world": str(driver.world_path),
        "systems": [
            {
                "name": system.name,
                "model": str(system.model_path),
                "position": list(system.position),
                "quaternion": list(system.quaternion),
                "initial_qpos": system.initial_qpos,
            }
            for system in driver.systems
            if isinstance(system, SO101Follower)
        ],
        "control_dt": driver.control_dt,
        "max_episode_steps": max_episode_steps,
        "action_names": list(feature_names(adapter.action_space)),
        "state_names": list(feature_names(adapter.observation_space)),
        "robot_state_size": spaces.flatdim(driver.systems[0].observation_space),
        "units": "SO-101 body [-100,100], gripper [0,100]; measured joint positions",
        "model_sha256": hashlib.sha256(
            b"".join(path.read_bytes() for path in sources)
        ).hexdigest(),
    }


def record(output: Path, world: Path, episodes: int, seconds: float, seed: int) -> None:
    steps = math.ceil(seconds / 0.02)
    arms = two_arms()
    with SystemDriver(arms, world=world) as driver:
        adapter = GymnasiumAdapter(driver)
        env = driver.as_gymnasium(flatten=True, max_episode_steps=steps)
        metadata = schema(driver, steps)
        features = {
            "observation.state": {
                "dtype": "float32",
                "shape": (metadata["robot_state_size"],),
                "names": metadata["state_names"][: metadata["robot_state_size"]],
            },
            "observation.environment_state": {
                "dtype": "float32",
                "shape": (
                    spaces.flatdim(adapter.observation_space)
                    - metadata["robot_state_size"],
                ),
                "names": metadata["state_names"][metadata["robot_state_size"] :],
            },
            "action": {
                "dtype": "float32",
                "shape": (spaces.flatdim(adapter.action_space),),
                "names": metadata["action_names"],
            },
        }
        dataset = LeRobotDataset.create(
            REPO_ID,
            fps=round(1 / driver.control_dt),
            root=output,
            features=features,
            robot_type="airtruck-so101",
            use_videos=False,
        )
        try:
            for episode in range(episodes):
                observation, _ = env.reset(seed=seed + episode)
                initial = {arm.name: dict(arm.controls) for arm in arms}
                phase = float(driver.rng.uniform(-0.2, 0.2))
                for _ in range(steps):
                    motion(arms, initial, float(driver.data.time), phase)
                    action = np.asarray(
                        spaces.flatten(
                            adapter.action_space,
                            {arm.name: arm.controls for arm in arms},
                        )
                    )
                    dataset.add_frame(
                        {
                            "observation.state": observation[
                                : metadata["robot_state_size"]
                            ],
                            "observation.environment_state": observation[
                                metadata["robot_state_size"] :
                            ],
                            "action": action,
                            "task": TASK,
                        }
                    )
                    observation, _, terminated, truncated, _ = env.step(action)
                    if terminated or truncated:
                        break
                dataset.save_episode()
        finally:
            dataset.finalize()
        (output / "meta/airtruck.json").write_text(
            json.dumps(metadata, indent=2) + "\n"
        )
    verified = LeRobotDataset(REPO_ID, root=output)
    print(
        json.dumps(
            {
                "dataset": str(output),
                "episodes": verified.num_episodes,
                "frames": len(verified),
            }
        )
    )


def act_training_config(
    dataset: Path,
    output: Path,
    *,
    steps: int = 1000,
    device: str = "cpu",
) -> TrainPipelineConfig:
    """Small state-based ACT example; callers can edit the native config objects."""
    return TrainPipelineConfig(
        dataset=DatasetConfig(repo_id=REPO_ID, root=str(dataset.resolve())),
        policy=ACTConfig(
            device=device,
            push_to_hub=False,
            chunk_size=16,
            n_action_steps=1,
            temporal_ensemble_coeff=0.01,
            dim_model=128,
            dim_feedforward=512,
            n_heads=4,
            n_encoder_layers=2,
            n_vae_encoder_layers=2,
            normalization_mapping={
                "STATE": NormalizationMode.MEAN_STD,
                "ENV": NormalizationMode.MEAN_STD,
                "ACTION": NormalizationMode.MEAN_STD,
            },
        ),
        output_dir=output.resolve(),
        steps=steps,
        num_workers=0,
        env_eval_freq=0,
        save_freq=steps,
        log_freq=min(steps, 100),
    )


def train(config: TrainPipelineConfig) -> Path:
    """Run LeRobot in-process and attach this simulation's rollout specification."""
    if (
        config.dataset.root is None
        or config.output_dir is None
        or not config.save_checkpoint
    ):
        raise ValueError(
            "This rollout example needs a local dataset, output directory and saved checkpoint"
        )
    metadata = json.loads(
        (Path(config.dataset.root) / "meta/airtruck.json").read_text()
    )
    # LeRobot owns the complete trainer; no CLI parsing, subprocess, or custom optimization loop.
    train_policy(config)
    checkpoint = config.output_dir / "checkpoints/last/pretrained_model"
    if not (checkpoint / "model.safetensors").is_file():
        raise RuntimeError("LeRobot did not save the expected checkpoint")
    (checkpoint / "airtruck.json").write_text(json.dumps(metadata, indent=2) + "\n")
    if json.loads((checkpoint / "airtruck.json").read_text()) != metadata:
        raise RuntimeError("Saved checkpoint metadata did not verify")
    return checkpoint


def rollout(checkpoint: Path, render: bool, seed: int) -> None:
    metadata = json.loads((checkpoint / "airtruck.json").read_text())
    fields = {"name", "model", "position", "quaternion", "initial_qpos"}
    if any(set(entry) != fields for entry in metadata["systems"]):
        raise ValueError("Checkpoint systems may describe models and poses only")
    arms = [SO101Follower(hardware=False, **entry) for entry in metadata["systems"]]
    with SystemDriver(
        arms,
        world=metadata["world"],
        control_dt=metadata["control_dt"],
        render_mode="human" if render else None,
    ) as driver:
        if schema(driver, metadata["max_episode_steps"]) != metadata:
            raise ValueError(
                "System units, channels, timing or model differ from this checkpoint"
            )
        env = driver.as_gymnasium(
            flatten=True, max_episode_steps=metadata["max_episode_steps"]
        )
        assert isinstance(env.action_space, spaces.Box)
        policy = ACTPolicy.from_pretrained(checkpoint)
        preprocessor, postprocessor = make_pre_post_processors(
            policy.config, pretrained_path=str(checkpoint)
        )
        policy.eval()
        policy.reset()
        observation, info = env.reset(seed=seed)
        clipped = 0
        with torch.inference_mode():
            for _ in range(metadata["max_episode_steps"]):
                started = time.monotonic()
                batch = preprocessor(
                    {
                        "observation.state": torch.from_numpy(
                            observation[: metadata["robot_state_size"]]
                        ),
                        "observation.environment_state": torch.from_numpy(
                            observation[metadata["robot_state_size"] :]
                        ),
                    }
                )
                prediction = (
                    postprocessor(policy.select_action(batch)).squeeze(0).cpu().numpy()
                )
                if not np.isfinite(prediction).all():
                    raise ValueError("ACT produced a non-finite action")
                action = np.clip(
                    prediction, env.action_space.low, env.action_space.high
                ).astype(np.float32)
                clipped += int(np.any(action != prediction))
                observation, _, terminated, truncated, info = env.step(action)
                if render:
                    time.sleep(max(0, driver.control_dt - (time.monotonic() - started)))
                if terminated or truncated:
                    break
        print(
            json.dumps(
                {
                    "simulated_seconds": info["time_s"],
                    "clipped_action_steps": clipped,
                    "finite_state": bool(np.isfinite(observation).all()),
                    "note": "Infrastructure rollout; no task-success metric is defined.",
                }
            )
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    collect = commands.add_parser("record")
    collect.add_argument("--output", type=Path, required=True)
    collect.add_argument("--world", type=Path, default=ROOT / "worlds/bench/scene.xml")
    collect.add_argument("--episodes", type=int, default=4)
    collect.add_argument("--seconds", type=float, default=8)
    collect.add_argument("--seed", type=int, default=42)
    fitting = commands.add_parser("train")
    fitting.add_argument("--dataset", type=Path, required=True)
    fitting.add_argument("--output", type=Path, required=True)
    fitting.add_argument("--steps", type=int, default=1000)
    fitting.add_argument("--device", default="cpu", choices=("cpu", "mps", "cuda"))
    deploy = commands.add_parser("rollout")
    deploy.add_argument("--checkpoint", type=Path, required=True)
    deploy.add_argument("--render", action="store_true")
    deploy.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.command == "record":
        if args.episodes < 1 or not math.isfinite(args.seconds) or args.seconds <= 0:
            parser.error("episodes and seconds must be positive")
        record(
            args.output.resolve(), args.world, args.episodes, args.seconds, args.seed
        )
    elif args.command == "train":
        if args.steps < 1:
            parser.error("steps must be positive")
        config = act_training_config(
            args.dataset, args.output, steps=args.steps, device=args.device
        )
        print(json.dumps({"checkpoint": str(train(config))}))
    else:
        rollout(args.checkpoint.resolve(), args.render, args.seed)


if __name__ == "__main__":
    main()
