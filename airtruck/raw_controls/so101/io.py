"""Common position-control interface for physical and simulated arms.

Angles are radians in the arm's calibrated CAD coordinate system.
"""

from collections.abc import Mapping, Sequence
from abc import ABC, abstractmethod
from typing import Protocol

from numpy.typing import ArrayLike


class ArmController(ABC):
    """Transport-independent interface; time stepping belongs to simulation only."""

    @abstractmethod
    def set_positions(self, positions: Mapping[str, float] | ArrayLike) -> None:
        """Command joint targets in radians, clamping to calibrated limits.

        A mapping selects joints by name. A six-element vector uses the default
        joint order and commands every joint.
        """
        ...

    @property
    @abstractmethod
    def positions(self) -> dict[str, float]:
        """Read joint feedback in radians."""
        ...

    @property
    @abstractmethod
    def targets(self) -> dict[str, float]:
        """Return the last commanded targets in radians."""
        ...

    @property
    @abstractmethod
    def limits(self) -> dict[str, tuple[float, float]]:
        """Return calibrated joint travel in radians."""
        ...

    @property
    @abstractmethod
    def enabled(self) -> bool:
        """Whether powered motor control is enabled."""
        ...

    @abstractmethod
    def set_enabled(self, enabled: bool) -> None:
        """Enable or disable powered motor control."""
        ...


class ServoTransport(Protocol):
    """Implement with a selected serial/Feetech driver; values are raw ticks.

    Implementations must raise on communication failures or missing replies.
    Construction of an SO101 adapter never invokes these methods.
    """

    def read_positions(self, servo_ids: Sequence[int]) -> Mapping[int, int]: ...

    def write_positions(self, positions: Mapping[int, int]) -> None: ...

    def set_torque_enabled(self, servo_ids: Sequence[int], enabled: bool) -> None: ...
