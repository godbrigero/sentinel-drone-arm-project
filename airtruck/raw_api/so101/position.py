"""Immutable joint positions with explicit units and a native angular range."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


class PositionUnit(Enum):
    RADIANS = "rad"
    DEGREES = "deg"
    NORMALIZED = "normalized"


@dataclass(frozen=True)
class JointPosition:
    """A value and its units, retaining native limits for lossless unit conversion.

    Normalized coordinates use [-1, 1], centered on the range midpoint.
    Construction preserves measurements outside the range; clamping is explicit.
    Unit conversion preserves the same joint coordinates. ``for_range`` explicitly
    transfers the fraction of travel to another joint's native range.
    """

    value: float
    unit: PositionUnit
    limits_rad: tuple[float, float]

    def __post_init__(self) -> None:
        if not isinstance(self.unit, PositionUnit):
            raise ValueError("unit must be a PositionUnit")
        if not math.isfinite(self.value):
            raise ValueError("Position value must be finite")
        if len(self.limits_rad) != 2:
            raise ValueError("Position range requires two endpoints")
        low, high = self.limits_rad
        if not math.isfinite(low) or not math.isfinite(high) or low >= high:
            raise ValueError("Position range must be finite and increasing")
        object.__setattr__(self, "limits_rad", (float(low), float(high)))

    @property
    def range(self) -> tuple[float, float]:
        """Valid range expressed in this value's units."""
        if self.unit == PositionUnit.NORMALIZED:
            return (-1.0, 1.0)
        if self.unit == PositionUnit.DEGREES:
            return (math.degrees(self.limits_rad[0]), math.degrees(self.limits_rad[1]))
        return self.limits_rad

    def clamped(self) -> JointPosition:
        low, high = self.range
        return JointPosition(min(high, max(low, self.value)), self.unit, self.limits_rad)

    def to(self, unit: PositionUnit) -> JointPosition:
        """Convert units, retaining the native range and coordinate convention."""
        if unit == self.unit:
            return self
        low, high = self.limits_rad
        if self.unit == PositionUnit.NORMALIZED:
            radians = low + (self.value + 1) * 0.5 * (high - low)
        elif self.unit == PositionUnit.DEGREES:
            radians = math.radians(self.value)
        else:
            radians = self.value
        if unit == PositionUnit.NORMALIZED:
            value = 2 * (radians - low) / (high - low) - 1
        elif unit == PositionUnit.DEGREES:
            value = math.degrees(radians)
        else:
            value = radians
        return JointPosition(value, unit, self.limits_rad)

    def for_range(self, limits_rad: tuple[float, float]) -> JointPosition:
        """Transfer bounded normalized travel to a destination's radian range."""
        normalized = self.to(PositionUnit.NORMALIZED).clamped()
        return JointPosition(normalized.value, PositionUnit.NORMALIZED, limits_rad).to(PositionUnit.RADIANS)

    def to_native(self, limits_rad: tuple[float, float]) -> JointPosition:
        """Resolve a command at a backend boundary, retaining a typed value.

        Normalized commands use the receiving joint's travel. Angular commands
        preserve their angle and convert units only. Both clamp to receiver limits.
        """
        if self.unit == PositionUnit.NORMALIZED:
            return self.for_range(limits_rad)
        return JointPosition(self.to(PositionUnit.RADIANS).value,
                             PositionUnit.RADIANS, limits_rad).clamped()
