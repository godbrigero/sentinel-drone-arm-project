# MuJoCo 3.16.0 API declarations; Copyright 2022 DeepMind Technologies Limited.
# Derived under Apache-2.0; see typings/LICENSE.mujoco and typings/README.md.
"""Public passive/interactive viewer API for MuJoCo 3.16.0."""
from contextlib import AbstractContextManager
from collections.abc import Callable
from types import TracebackType
from typing import Any, Self, TypeAlias
import numpy as np
from numpy.typing import NDArray
from ._structs import MjModel, MjData, MjvCamera, MjvOption, MjvPerturb, MjvScene, MjrRect, MjvFigure
KeyCallbackType: TypeAlias = Callable[[int], None]
CallbackType: TypeAlias = Callable[[MjModel, MjData], None]
LoaderType: TypeAlias = Callable[[], tuple[MjModel, MjData]]

class Handle:

    @property
    def cam(self) -> MjvCamera:
        ...

    @property
    def opt(self) -> MjvOption:
        ...

    @property
    def pert(self) -> MjvPerturb:
        ...

    @property
    def user_scn(self) -> MjvScene | None:
        ...

    @property
    def m(self) -> MjModel | None:
        ...

    @property
    def d(self) -> MjData | None:
        ...

    def __enter__(self) -> Self:
        ...

    def __exit__(self, exc_type: type[BaseException] | None, exc_val: BaseException | None, exc_tb: TracebackType | None) -> None:
        ...

    def lock(self) -> AbstractContextManager[Any]:
        ...

    def is_running(self) -> bool:
        ...

    def sync(self, state_only: bool=False) -> None:
        ...

    def close(self) -> None:
        ...

    def update_hfield(self, hfieldid: int) -> None:
        ...

    def update_mesh(self, meshid: int) -> None:
        ...

    def update_texture(self, texid: int) -> None:
        ...

    def set_figures(self, viewports_figures: tuple[MjrRect, MjvFigure] | list[tuple[MjrRect, MjvFigure]]) -> None:
        ...

    def set_images(self, viewports_images: tuple[MjrRect, NDArray[np.uint8]] | list[tuple[MjrRect, NDArray[np.uint8]]]) -> None:
        ...

    def set_texts(self, texts: tuple[int | None, int | None, str | None, str | None] | list[tuple[int | None, int | None, str | None, str | None]]) -> None:
        ...

    def clear_figures(self) -> None:
        ...

    def clear_images(self) -> None:
        ...

    def clear_texts(self) -> None:
        ...

def launch(model: MjModel | None=None, data: MjData | None=None, *, loader: LoaderType | None=None, show_left_ui: bool=True, show_right_ui: bool=True) -> None:
    ...

def launch_from_path(path: str) -> None:
    ...

def launch_passive(model: MjModel, data: MjData, *, key_callback: KeyCallbackType | None=None, show_left_ui: bool=True, show_right_ui: bool=True) -> Handle:
    ...
