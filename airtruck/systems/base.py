"""System lifecycle: local controls/state, one shared driver-owned physics clock."""

from __future__ import annotations

import copy
from abc import ABC, abstractmethod
from collections.abc import Mapping
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from gymnasium import spaces

ROOT = Path(__file__).resolve().parents[2]
Values = dict[str, Any]


def space_values(space: spaces.Space, value: Any) -> Any:
    """Validate shape/keys/finiteness and pack numeric values for Gymnasium."""
    if isinstance(space, spaces.Dict):
        if not isinstance(value, Mapping) or set(value) != set(space.spaces):
            raise ValueError(f"Expected exactly these fields: {tuple(space.spaces)}")
        return {
            key: space_values(child, value[key]) for key, child in space.spaces.items()
        }
    if isinstance(space, spaces.Box):
        original = np.asarray(value)
        if (
            original.dtype.kind not in "biuf"
            or original.shape != space.shape
            or not np.isfinite(original).all()
            or np.any(original < space.low)
            or np.any(original > space.high)
        ):
            raise ValueError(f"Value must be finite and match {space}")
        packed = original.astype(space.dtype)
        if not space.contains(packed):
            raise ValueError(f"Value does not match {space}")
        return packed.copy()
    if not space.contains(value):
        raise ValueError(f"Value does not match {space}")
    return copy.deepcopy(value)


class SimSystem(ABC):
    """A prepared physical model with its own command and observation schema."""

    controllable = True
    action_space: spaces.Dict
    observation_space: spaces.Dict

    def __init__(
        self,
        name: str,
        *,
        model: str | Path,
        root_body: str,
        hardware: bool = False,
        position: tuple[float, float, float] = (0, 0, 0),
        quaternion: tuple[float, float, float, float] = (1, 0, 0, 0),
    ) -> None:
        if not name.isidentifier():
            raise ValueError("System names must be identifiers")
        if (
            np.shape(position) != (3,)
            or np.shape(quaternion) != (4,)
            or not np.isfinite(position).all()
            or not np.isclose(np.linalg.norm(quaternion), 1)
        ):
            raise ValueError("Placement needs finite meters and a unit wxyz quaternion")
        self.name, self.model_path, self.root_body = (
            name,
            Path(model).resolve(),
            root_body,
        )
        self.position, self.quaternion = position, quaternion
        self.hardware = hardware
        self._hardware_active = False
        self.rng = np.random.default_rng()
        self.controls: Values = {}
        self.state: Values = {}
        self._command: Values = {}
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None

    @property
    def hardware(self) -> bool:
        return self._hardware

    @hardware.setter
    def hardware(self, enabled: bool) -> None:
        if type(enabled) is not bool:
            raise TypeError("hardware must be a boolean")
        self._hardware = enabled

    @property
    def hardware_active(self) -> bool:
        return self._hardware_active

    @property
    def world(self) -> tuple[mujoco.MjModel, mujoco.MjData]:
        if self._model is None or self._data is None:
            raise RuntimeError(f"{self.name} is not bound to a driver")
        return self._model, self._data

    def make_spec(self) -> mujoco.MjSpec:
        return mujoco.MjSpec.from_file(str(self.model_path))

    def attach(self, world: mujoco.MjSpec) -> None:
        child = self.make_spec()
        body = child.body(self.root_body)
        if body is None:
            raise ValueError(
                f"Missing root body {self.root_body!r} in {self.model_path}"
            )
        frame = world.worldbody.add_frame(pos=self.position, quat=self.quaternion)
        frame.attach_body(body, prefix=f"{self.name}/")

    def bind(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        if self._model is not None:
            raise RuntimeError(f"{self.name} already belongs to a driver")
        self._model, self._data = model, data

    def validate_controls(self, controls: Values | None = None) -> Values:
        command = copy.deepcopy(self.controls if controls is None else controls)
        space_values(self.action_space, command)
        return command

    def prepare_step(self) -> None:
        self._command = self.validate_controls()

    def sync_hardware(self) -> None:
        """Apply a requested backend transition only at a driver boundary."""
        if self.hardware == self._hardware_active:
            return
        if self.hardware:
            try:
                self._connect_hardware()
                self._hardware_active = True
                self._backend_changed()
                self.observe()
                self._hold_after_switch()
            except BaseException:
                try:
                    self._disconnect_hardware()
                finally:
                    self._hardware_active = False
                    self._backend_changed()
                raise
        else:
            try:
                self._disconnect_hardware()
            finally:
                self._hardware_active = False
                self._backend_changed()
            self._hold_after_switch()

    def _hold_after_switch(self) -> None:
        self.hold()

    def _connect_hardware(self) -> None:
        raise NotImplementedError(f"No hardware backend for {type(self).__name__}")

    def _disconnect_hardware(self) -> None:
        pass

    def _backend_changed(self) -> None:
        pass

    def hold(self) -> None:
        """Seed pending controls after reset or backend change, without motion."""

    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def step(self, dt: float) -> None:
        """Apply the prepared command locally; never call mj_step here."""

    @abstractmethod
    def observe(self) -> Values: ...

    def close(self) -> None:
        try:
            self._disconnect_hardware()
        finally:
            self._hardware_active = False
            self._backend_changed()
            self._model, self._data = None, None
