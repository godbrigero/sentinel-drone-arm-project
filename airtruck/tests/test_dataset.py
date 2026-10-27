"""A real local LeRobot dataset/train/checkpoint round trip through system objects."""

import json
from pathlib import Path

import numpy as np
import pytest
from gymnasium import spaces
from lerobot.datasets.lerobot_dataset import LeRobotDataset

from airtruck.examples.act import (
    REPO_ID,
    act_training_config,
    record,
    rollout,
    schema,
    train,
)
from airtruck.examples.two_arms import motion, two_arms
from airtruck.runtime.driver import SystemDriver
from airtruck.runtime.gymnasium import GymnasiumAdapter, feature_names
from airtruck.systems.base import ROOT


def test_dataset_preserves_channel_order_and_pre_action_observation(
    tmp_path: Path,
) -> None:
    output = tmp_path / "dataset"
    world = ROOT / "worlds/bench/scene.xml"
    record(output, world, episodes=2, seconds=0.06, seed=42)
    saved = LeRobotDataset(REPO_ID, root=output)
    assert saved.num_episodes == 2 and len(saved) == 6
    metadata = json.loads((output / "meta/airtruck.json").read_text())
    arms = two_arms()
    with SystemDriver(arms, world=world) as driver:
        adapter = GymnasiumAdapter(driver)
        env = driver.as_gymnasium(flatten=True, max_episode_steps=3)
        observation, _ = env.reset(seed=42)
        phase = float(driver.rng.uniform(-0.2, 0.2))
        initial = {arm.name: dict(arm.controls) for arm in arms}
        motion(arms, initial, 0, phase)
        action = np.asarray(
            spaces.flatten(
                adapter.action_space, {arm.name: arm.controls for arm in arms}
            )
        )
        np.testing.assert_allclose(
            np.concatenate(
                [
                    saved[0]["observation.state"].numpy(),
                    saved[0]["observation.environment_state"].numpy(),
                ]
            ),
            observation,
        )
        np.testing.assert_allclose(saved[0]["action"].numpy(), action)
        assert metadata == schema(driver, 3)
        assert saved.meta.features["action"]["names"] == list(
            feature_names(adapter.action_space)
        )


def test_python_config_trains_and_reloads_act_without_cli_parsing(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    dataset = tmp_path / "dataset"
    record(dataset, ROOT / "worlds/bench/scene.xml", episodes=2, seconds=0.2, seed=42)
    checkpoint = train(act_training_config(dataset, tmp_path / "training", steps=1))
    saved = json.loads((checkpoint / "train_config.json").read_text())
    assert saved["steps"] == 1 and saved["policy"]["chunk_size"] == 16
    rollout(checkpoint, render=False, seed=42)
    result = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert result["finite_state"]
    assert result["simulated_seconds"] == pytest.approx(0.2)


def test_checkpoint_metadata_cannot_select_hardware(tmp_path: Path) -> None:
    metadata = {
        "systems": [
            {
                "name": "arm",
                "model": str(ROOT / "models/so101/so101.xml"),
                "position": [0, 0, 0],
                "quaternion": [1, 0, 0, 0],
                "initial_qpos": {},
                "hardware": True,
                "port": "/dev/never-open",
            }
        ]
    }
    (tmp_path / "airtruck.json").write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match="models and poses only"):
        rollout(tmp_path, render=False, seed=42)
