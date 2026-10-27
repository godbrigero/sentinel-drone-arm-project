"""Gymnasium packing and episode boundary; no robot-specific control logic."""

from __future__ import annotations

from collections import OrderedDict
from typing import TYPE_CHECKING, Any

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from airtruck.systems.base import space_values

if TYPE_CHECKING:
    from airtruck.runtime.driver import SystemDriver


class GymnasiumAdapter(gym.Env):
    action_space: spaces.Dict
    observation_space: spaces.Dict
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(self, driver: SystemDriver) -> None:
        self.driver = driver
        self.render_mode = driver.render_mode
        self.metadata = {
            **type(self).metadata,
            "render_fps": round(1 / driver.control_dt),
        }
        self.action_space = spaces.Dict(
            OrderedDict(
                (s.name, s.action_space) for s in driver.systems if s.controllable
            )
        )
        self.observation_space = spaces.Dict(
            OrderedDict((s.name, s.observation_space) for s in driver.systems)
        )

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict, dict]:
        super().reset(seed=seed)
        if options:
            raise ValueError("No reset options are defined")
        return space_values(
            self.observation_space, self.driver.reset(seed=seed)
        ), self.driver.info

    def step(self, action: dict) -> tuple[dict, float, bool, bool, dict]:
        snapshot = space_values(self.action_space, action)

        def apply() -> None:
            for system in self.driver.systems:
                if system.controllable:
                    system.controls.update(snapshot[system.name])

        observation = self.driver.step(update=apply)
        return (
            space_values(self.observation_space, observation),
            0.0,
            False,
            False,
            self.driver.info,
        )

    def render(self) -> np.ndarray | None:
        return self.driver.render()

    def close(self) -> None:
        self.driver.close()


def feature_names(space: spaces.Dict, prefix: str = "") -> tuple[str, ...]:
    """Names in precisely Gymnasium's recursive flattening order."""
    names: list[str] = []
    for name, child in space.spaces.items():
        key = f"{prefix}.{name}" if prefix else name
        if isinstance(child, spaces.Dict):
            names.extend(feature_names(child, key))
        else:
            count = spaces.flatdim(child)
            names.extend([key] if count == 1 else [f"{key}[{i}]" for i in range(count)])
    return tuple(names)
