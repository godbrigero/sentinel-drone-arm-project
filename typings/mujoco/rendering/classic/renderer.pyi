# MuJoCo 3.16.0 API declarations; Copyright 2022 DeepMind Technologies Limited.
# Derived under Apache-2.0; see typings/LICENSE.mujoco and typings/README.md.
from __future__ import annotations
import mujoco as mujoco
from mujoco.rendering.classic.gl_context import GLContext
from mujoco.rendering.classic import gl_context
import numpy
import numpy as np
import typing
__all__: list[str] = ['GLContext', 'Renderer', 'gl_context', 'mujoco', 'np']

class Renderer:

    def __del__(self) -> None:
        ...

    def __enter__(self) -> typing.Self:
        ...

    def __exit__(self, exc_type: type[BaseException] | None, exc_value: BaseException | None, traceback: typing.Any) -> None:
        ...

    def __init__(self, model: mujoco._structs.MjModel, height: int=240, width: int=320, max_geom: int=10000, font_scale: mujoco._enums.mjtFontScale=...) -> None:
        ...

    def close(self) -> None:
        ...

    def disable_depth_rendering(self) -> None:
        ...

    def disable_segmentation_rendering(self) -> None:
        ...

    def enable_depth_rendering(self) -> None:
        ...

    def enable_segmentation_rendering(self) -> None:
        ...

    def render(self, *, out: numpy.ndarray | None=None) -> numpy.ndarray:
        ...

    def update_scene(self, data: mujoco._structs.MjData, camera: int | str | mujoco._structs.MjvCamera=-1, scene_option: mujoco._structs.MjvOption | None=None) -> None:
        ...

    @property
    def height(self) -> int:
        ...

    @property
    def model(self) -> mujoco._structs.MjModel:
        ...

    @property
    def scene(self) -> mujoco._structs.MjvScene:
        ...

    @property
    def width(self) -> int:
        ...
