"""Measured encoder calibration records and JSON serialization."""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
from airtruck.raw_api.so101.joints import JOINTS, JointName
from airtruck.raw_api.so101.position import JointPosition

@dataclass(frozen=True)
class JointCalibration:
    """Measured hardware radians to absolute encoder ticks; zero is calibration-defined."""

    servo_id: int
    zero_tick: int
    direction: int
    limits_rad: tuple[float, float]
    counts_per_revolution: int = 4096

    def __post_init__(self) -> None:
        if any(
            type(v) is not int
            for v in (
                self.servo_id,
                self.zero_tick,
                self.direction,
                self.counts_per_revolution,
            )
        ):
            raise ValueError(
                "Calibration IDs, ticks, direction and resolution must be integers"
            )
        if len(self.limits_rad) != 2 or any(
            isinstance(v, bool) or not isinstance(v, (int, float))
            for v in self.limits_rad
        ):
            raise ValueError("limits_rad must contain two numbers")
        if not 0 <= self.servo_id <= 253:
            raise ValueError("servo_id must be between 0 and 253")
        if self.direction not in (-1, 1):
            raise ValueError("direction must be -1 or +1")
        if self.counts_per_revolution <= 0:
            raise ValueError("counts_per_revolution must be positive")
        lo, hi = self.limits_rad
        if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
            raise ValueError("limits_rad must be finite and increasing")
        if not 0 <= self.zero_tick < self.counts_per_revolution:
            raise ValueError("zero_tick is outside the encoder range")
        for angle in (lo, hi):
            tick = (
                self.zero_tick
                + self.direction * angle * self.counts_per_revolution / math.tau
            )
            if not 0 <= tick <= self.counts_per_revolution - 1:
                raise ValueError("Calibrated travel crosses the absolute encoder wrap")

    def to_radians(self, ticks: int) -> float:
        if not 0 <= ticks < self.counts_per_revolution:
            raise ValueError("Encoder feedback is outside its calibrated range")
        return (
            self.direction
            * (ticks - self.zero_tick)
            * math.tau
            / self.counts_per_revolution
        )

    def position_to_ticks(self, position: JointPosition) -> int:
        """Encode a typed command only at the servo boundary."""
        return self.to_ticks(position.to_native(self.limits_rad).value)

    def to_ticks(self, radians: float) -> int:
        if not math.isfinite(radians):
            raise ValueError("Joint targets must be finite")
        lo, hi = self.limits_rad
        angle = min(hi, max(lo, radians))
        # Keep quantization within the calibrated mechanical interval.
        end_ticks = [
            self.zero_tick + self.direction * a * self.counts_per_revolution / math.tau
            for a in (lo, hi)
        ]
        # Endpoints derived from ticks can round a few ulps across an integer.
        end_ticks = [
            float(round(t)) if math.isclose(t, round(t), rel_tol=0, abs_tol=1e-9) else t
            for t in end_ticks
        ]
        minimum, maximum = math.ceil(min(end_ticks)), math.floor(max(end_ticks))
        tick = round(
            self.zero_tick
            + self.direction * angle * self.counts_per_revolution / math.tau
        )
        return min(maximum, max(minimum, tick))


def load_joint_calibrations(path: str | Path) -> dict[JointName, JointCalibration]:
    """Load a complete joint-name mapping; reject partial or ambiguous records."""

    def unique(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate calibration key: {key}")
            result[key] = value
        return result

    raw = json.loads(Path(path).read_text(), object_pairs_hook=unique)
    if not isinstance(raw, dict) or set(raw) != set(JOINTS):
        raise ValueError("Calibration must contain exactly all six joint names")
    result: dict[JointName, JointCalibration] = {}
    fields = {
        "servo_id",
        "zero_tick",
        "direction",
        "limits_rad",
        "counts_per_revolution",
    }
    for name in JOINTS:
        record = raw[name]
        if not isinstance(record, dict) or set(record) != fields:
            raise ValueError(f"Invalid calibration fields for {name}")
        limits = record["limits_rad"]
        if not isinstance(limits, list) or len(limits) != 2:
            raise ValueError(f"Invalid limits_rad for {name}")
        result[name] = JointCalibration(
            servo_id=record["servo_id"],
            zero_tick=record["zero_tick"],
            direction=record["direction"],
            limits_rad=tuple(limits),
            counts_per_revolution=record["counts_per_revolution"],
        )
    if len({c.servo_id for c in result.values()}) != len(JOINTS):
        raise ValueError("Calibration servo IDs must be unique")
    return result


def joint_calibrations_dict(
    calibration: Mapping[JointName, JointCalibration],
) -> dict[str, object]:
    """JSON-ready mapping; also supports progressive calibration checkpoints."""
    return {name: asdict(value) for name, value in calibration.items()}
