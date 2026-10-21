"""Immutable values at the simulation boundary. SI units, world coordinates."""
from dataclasses import dataclass, field
import math


@dataclass(frozen=True)
class Pose:
    position_m: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion_wxyz: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        position = tuple(float(x) for x in self.position_m)
        quaternion = tuple(float(x) for x in self.quaternion_wxyz)
        if len(position) != 3 or len(quaternion) != 4:
            raise ValueError("Pose needs three coordinates and a wxyz quaternion")
        if not all(math.isfinite(x) for x in (*position, *quaternion)):
            raise ValueError("Pose must be finite")
        if not math.isclose(sum(x*x for x in quaternion), 1.0, abs_tol=1e-6):
            raise ValueError("Pose quaternion must have unit length")
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "quaternion_wxyz", quaternion)


@dataclass(frozen=True)
class EntityHandle:
    name: str
    _owner: object = field(repr=False)


@dataclass(frozen=True)
class Sample[T]:
    time_s: float
    value: T


@dataclass(frozen=True)
class Contact:
    body_a: str
    body_b: str
    normal_force_n: float
    penetration_m: float
