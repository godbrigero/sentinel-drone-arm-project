"""One owner of physics state and time, with independently bound systems."""
from __future__ import annotations

import math
import time
from copy import deepcopy
from collections.abc import Callable, Mapping
from contextlib import nullcontext
from typing import Literal, cast
import numpy as np
import mujoco
from mujoco.viewer import Handle
from airtruck.testing_util.adapters import SystemAdapter
from airtruck.testing_util.interfaces import WorldControl
from airtruck.testing_util.types import Contact, EntityHandle, Pose, Sample

Placement = Literal["fixed", "free", "kinematic"]


class MujocoWorld(WorldControl):
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 systems: Mapping[str, SystemAdapter],
                 entities: Mapping[str, tuple[int, Placement]]) -> None:
        self._model, self._data = model, data
        self._systems = dict(systems)
        self._entities = dict(entities)
        self._owner = object()
        self._closed = False
        self._viewer: Handle | None = None
        claimed: set[int] = set()
        for adapter in self._systems.values():
            ids = adapter.actuator_ids
            if len(set(ids)) != len(ids) or claimed.intersection(ids):
                raise ValueError("Systems cannot share actuator bindings")
            if any(i < 0 or i >= model.nu for i in ids):
                raise ValueError("Invalid actuator binding")
            claimed.update(ids)
        if claimed != set(range(model.nu)):
            raise ValueError("Every actuator must belong to a system")
        for body, mode in self._entities.values():
            if not 0 < body < model.nbody:
                raise ValueError("Entities must refer to real bodies")
            if mode == "free":
                j = int(model.body_jntadr[body])
                if (model.body_jntnum[body] != 1 or j < 0 or
                        model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE):
                    raise ValueError("Free placement requires a free root joint")
            elif mode == "kinematic":
                if model.body_mocapid[body] < 0:
                    raise ValueError("Kinematic placement requires a mocap body")
            elif mode != "fixed":
                raise ValueError("Unknown placement capability")
        self._views = {name: adapter.interface(self._require_open)
                       for name, adapter in self._systems.items()}
        mujoco.mj_forward(model, data)
        self._initial = deepcopy(data)

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("World is closed")

    @property
    def time_s(self) -> float:
        self._require_open()
        return float(self._data.time)

    def system[T](self, name: str, interface: Callable[..., T]) -> T:
        self._require_open()
        if name not in self._views:
            raise KeyError(f"Unknown system {name!r}; available: {tuple(self._views)}")
        view = self._views[name]
        if not isinstance(interface, type):
            raise TypeError("interface must be an interface class")
        if not isinstance(view, interface):
            raise TypeError(f"System {name!r} does not implement {interface.__name__}")
        return cast(T, view)

    def entity(self, name: str) -> EntityHandle:
        self._require_open()
        if name not in self._entities:
            raise KeyError(f"Unknown entity {name!r}")
        return EntityHandle(name, self._owner)

    def _resolve(self, entity: EntityHandle) -> tuple[int, Placement]:
        self._require_open()
        if entity._owner is not self._owner:
            raise ValueError("Entity belongs to a different world")
        return self._entities[entity.name]

    def pose(self, entity: EntityHandle) -> Pose:
        body, _ = self._resolve(entity)
        p, q = self._data.xpos[body], self._data.xquat[body]
        return Pose((float(p[0]), float(p[1]), float(p[2])),
                    (float(q[0]), float(q[1]), float(q[2]), float(q[3])))

    def place(self, entity: EntityHandle, pose: Pose, *, clear_velocity: bool = True) -> None:
        body, mode = self._resolve(entity)
        if mode == "fixed":
            raise ValueError("Fixed bodies cannot be placed at runtime")
        if not isinstance(pose, Pose) or not isinstance(clear_velocity, bool):
            raise TypeError("Expected Pose and boolean clear_velocity")
        with self._viewer.lock() if self._viewer else nullcontext():
            if mode == "kinematic":
                index = self._model.body_mocapid[body]
                self._data.mocap_pos[index] = pose.position_m
                self._data.mocap_quat[index] = pose.quaternion_wxyz
            else:
                joint = self._model.body_jntadr[body]
                q = self._model.jnt_qposadr[joint]
                v = self._model.jnt_dofadr[joint]
                self._data.qpos[q:q+7] = (*pose.position_m, *pose.quaternion_wxyz)
                if clear_velocity:
                    self._data.qvel[v:v+6] = 0
                self._data.qacc_warmstart[:] = 0
            mujoco.mj_forward(self._model, self._data)
        if self._viewer:
            self._viewer.sync()

    def contacts(self) -> tuple[Contact, ...]:
        self._require_open()
        result: list[Contact] = []
        force = np.zeros(6, dtype=np.float64)
        for index in range(self._data.ncon):
            contact = self._data.contact[index]
            mujoco.mj_contactForce(self._model, self._data, index, force)
            a = int(self._model.geom_bodyid[contact.geom1])
            b = int(self._model.geom_bodyid[contact.geom2])
            result.append(Contact(self._model.body(a).name or str(a),
                                  self._model.body(b).name or str(b),
                                  float(force[0]), max(0.0, -float(contact.dist))))
        return tuple(result)

    def open_window(self) -> None:
        self._require_open()
        if self._viewer is not None:
            if self._viewer.is_running():
                return
            self._viewer.close()
        import mujoco.viewer
        self._viewer = mujoco.viewer.launch_passive(
            self._model, self._data, show_right_ui=False)

    def step(self, steps: int = 1) -> None:
        self._require_open()
        if isinstance(steps, bool) or not isinstance(steps, int) or steps < 1:
            raise ValueError("steps must be a positive integer")
        batch_size = max(1, round(1 / (60 * self._model.opt.timestep)))
        remaining = steps
        while remaining:
            batch = min(remaining, batch_size) if self._viewer else remaining
            started = time.monotonic()
            with self._viewer.lock() if self._viewer else nullcontext():
                for _ in range(batch):
                    for adapter in self._systems.values():
                        adapter.update_control()
                    mujoco.mj_step(self._model, self._data)
                # Derived poses must describe the final integrated state.
                mujoco.mj_forward(self._model, self._data)
            if self._viewer:
                self._viewer.sync()
                time.sleep(max(0.0, batch * self._model.opt.timestep - (time.monotonic()-started)))
            remaining -= batch

    def _steps_for(self, duration_s: float) -> int:
        if not math.isfinite(duration_s) or duration_s < 0:
            raise ValueError("duration_s must be finite and nonnegative")
        return math.ceil(duration_s / float(self._model.opt.timestep))

    def advance(self, duration_s: float) -> None:
        self._require_open()
        steps = self._steps_for(duration_s)
        if steps:
            self.step(steps)

    def wait_until(self, predicate: Callable[[], bool], *, timeout_s: float) -> None:
        self._require_open()
        steps = self._steps_for(timeout_s)
        for i in range(steps + 1):
            if predicate():
                return
            if i < steps:
                self.step()
        raise TimeoutError(f"Condition not reached after {timeout_s} simulated seconds")

    def record[T](self, observe: Callable[[], T], duration_s: float,
                  sample_period_s: float = 0.05) -> list[Sample[T]]:
        self._require_open()
        remaining = self._steps_for(duration_s)
        if not math.isfinite(sample_period_s) or sample_period_s <= 0:
            raise ValueError("sample_period_s must be finite and positive")
        interval = self._steps_for(sample_period_s)
        samples = [Sample(self.time_s, deepcopy(observe()))]
        while remaining:
            steps = min(interval, remaining)
            self.step(steps)
            remaining -= steps
            samples.append(Sample(self.time_s, deepcopy(observe())))
        return samples

    def reset(self) -> None:
        self._require_open()
        with self._viewer.lock() if self._viewer else nullcontext():
            mujoco.mj_copyData(self._data, self._model, self._initial)
            for adapter in self._systems.values():
                adapter.reset_control()
            mujoco.mj_forward(self._model, self._data)
        if self._viewer:
            self._viewer.sync()

    def close(self) -> None:
        if self._closed:
            return
        errors: list[Exception] = []
        try:
            if self._viewer:
                self._viewer.close()
        except Exception as error:
            errors.append(error)
        finally:
            for adapter in self._systems.values():
                try:
                    adapter.disable()
                except Exception as error:
                    errors.append(error)
            self._closed = True
        if errors:
            raise ExceptionGroup("World cleanup failed", errors)

    def __enter__(self) -> MujocoWorld:
        self._require_open()
        return self

    def __exit__(self, *args: object) -> None:
        self.close()
