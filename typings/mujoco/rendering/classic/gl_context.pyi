# MuJoCo 3.16.0 API declarations; Copyright 2022 DeepMind Technologies Limited.
# Derived under Apache-2.0; see typings/LICENSE.mujoco and typings/README.md.
class GLContext:

    def __init__(self, max_width: int, max_height: int) -> None:
        ...

    def make_current(self) -> None:
        ...

    def free(self) -> None:
        ...
