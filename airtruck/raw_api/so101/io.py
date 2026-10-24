"""Common position-control interface for physical and simulated arms.

Feedback values carry units and native ranges. Radian commands use CAD zero in
simulation and measured calibration zero on hardware.
"""

from collections.abc import Mapping, Sequence
from abc import ABC, abstractmethod
from typing import Protocol
from airtruck.raw_api.so101.joints import JointName as JointName, PositionCommand as PositionCommand
from airtruck.raw_api.so101.position import JointPosition


class ArmReader(ABC):
    """Abstract joint feedback; units and ranges belong to each JointPosition."""

    @property
    @abstractmethod
    def positions(self) -> dict[JointName, JointPosition]:
        """Read positions with their units and ranges."""
        ...

    @property
    @abstractmethod
    def normalized_positions(self) -> dict[JointName, JointPosition]:
        """Read positions in the shared normalized representation."""
        ...


class ArmController(ArmReader):
    """Abstract commands and state for a physical or simulated arm."""

    @abstractmethod
    def set_normalized_positions(self, positions: PositionCommand) -> None:
        """Command typed positions in normalized units."""
        ...

    @abstractmethod
    def set_positions(self, positions: PositionCommand) -> None:
        """Command typed positions; unspecified joints retain their targets."""
        ...

    @property
    @abstractmethod
    def targets(self) -> dict[JointName, JointPosition]:
        """Return commanded positions with their units and ranges."""
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

    @abstractmethod
    def read_positions(self, servo_ids: Sequence[int]) -> Mapping[int, int]: ...

    @abstractmethod
    def write_positions(self, positions: Mapping[int, int]) -> None: ...

    @abstractmethod
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
