"""Public world contract: control systems through their own typed interfaces."""
from abc import ABC, abstractmethod
from collections.abc import Callable
from airtruck.testing_util.types import Contact, EntityHandle, Pose, Sample


class WorldControl(ABC):
    @property
    @abstractmethod
    def time_s(self) -> float: ...

    @abstractmethod
    def system[T](self, name: str, interface: Callable[..., T]) -> T: ...

    @abstractmethod
    def entity(self, name: str) -> EntityHandle: ...

    @abstractmethod
    def pose(self, entity: EntityHandle) -> Pose: ...

    @abstractmethod
    def place(self, entity: EntityHandle, pose: Pose, *, clear_velocity: bool = True) -> None: ...

    @abstractmethod
    def contacts(self) -> tuple[Contact, ...]: ...

    @abstractmethod
    def step(self, steps: int = 1) -> None: ...

    @abstractmethod
    def advance(self, duration_s: float) -> None: ...

    @abstractmethod
    def wait_until(self, predicate: Callable[[], bool], *, timeout_s: float) -> None: ...

    @abstractmethod
    def record[T](self, observe: Callable[[], T], duration_s: float,
                  sample_period_s: float = 0.05) -> list[Sample[T]]: ...

    @abstractmethod
    def reset(self) -> None: ...

    @abstractmethod
    def close(self) -> None: ...
