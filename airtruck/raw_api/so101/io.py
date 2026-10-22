"""Common position-control interface for physical and simulated arms.

Angles are radians: the simulation uses CAD zero, while hardware uses its measured calibration zero.
"""

from collections.abc import Mapping, Sequence
from abc import ABC, abstractmethod
from typing import Protocol, Literal, TypeAlias
import numpy as np
from numpy.typing import NDArray

JointName: TypeAlias = Literal[
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"
]
PositionCommand: TypeAlias = Mapping[JointName, float] | Sequence[float] | NDArray[np.float64]


class ArmController(ABC):
    """Transport-independent interface; time stepping belongs to simulation only."""

    @abstractmethod
    def set_positions(self, positions: PositionCommand) -> None:
        """Command joint targets in radians, clamping to calibrated limits.

        A mapping selects joints by name. A six-element vector uses the default
        joint order and commands every joint.
        """
        ...

    @property
    @abstractmethod
    def positions(self) -> dict[JointName, float]:
        """Read joint feedback in radians."""
        ...

    @property
    @abstractmethod
    def targets(self) -> dict[JointName, float]:
        """Return the last commanded targets in radians."""
        ...

    @property
    @abstractmethod
    def limits(self) -> dict[JointName, tuple[float, float]]:
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


class SimulatedArmController(ArmController):
    """Simulation-only motor experiments; voltage is not a hardware command."""

    @abstractmethod
    def set_motor_voltages(self, voltages: Mapping[JointName, float]) -> None:
        """Override selected position loops with motor voltage, within supply limits."""
        ...

    @abstractmethod
    def use_position_control(self) -> None:
        """Clear voltage overrides; hold the currently measured joint positions."""
        ...

    @property
    @abstractmethod
    def motor_voltages(self) -> dict[JointName, float]:
        """Last applied motor commands in volts."""
        ...

    @property
    @abstractmethod
    def motor_torques_nm(self) -> dict[JointName, float]:
        """Current actuator output torque, including the configured limits."""
        ...
