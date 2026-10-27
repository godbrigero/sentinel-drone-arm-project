"""Onboard camera geometry and the latest frame, independent of the image source."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

# MuJoCo camera axes (x right, y up, z backward) → optical (x right, y down, z forward).
MUJOCO_TO_OPTICAL = np.diag([1.0, -1.0, -1.0])
# Body FLU ↔ FRD.
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


class Camera:
    """One mounted camera: pinhole intrinsics, body mounting and its latest frame.

    `rotation` maps optical-frame vectors into the body FRD frame; `position_m`
    is the optical center relative to the body origin in FRD meters. Simulation
    renders frames into this object; on hardware, call `publish()` with frames
    from any external source so processing code can use the same interface.
    """

    def __init__(
        self,
        name: str,
        *,
        width: int,
        height: int,
        fovy_deg: float,
        rotation: np.ndarray,
        position_m: np.ndarray,
    ) -> None:
        if width < 1 or height < 1 or not 0 < fovy_deg < 180:
            raise ValueError("Camera needs a positive resolution and 0 < fovy < 180")
        self.name, self.width, self.height, self.fovy_deg = (
            name,
            width,
            height,
            fovy_deg,
        )
        self.rotation = np.asarray(rotation, dtype=np.float64)
        self.position_m = np.asarray(position_m, dtype=np.float64)
        focal = height / 2 / math.tan(math.radians(fovy_deg) / 2)
        # Pixel centers are integer coordinates, as in OpenCV.
        self.intrinsics = np.array(
            [[focal, 0, width / 2 - 0.5], [0, focal, height / 2 - 0.5], [0, 0, 1]]
        )
        self.frame: NDArray[np.uint8] | None = None
        self.timestamp_s: float | None = None
        self.sequence = 0

    def publish(self, image: np.ndarray, timestamp_s: float) -> None:
        """Store an RGB (H, W, 3) or grayscale (H, W) uint8 frame."""
        image = np.asarray(image)
        if image.dtype != np.uint8 or image.shape[:2] != (self.height, self.width):
            raise ValueError(
                f"{self.name} expects uint8 frames of {self.height}x{self.width}"
            )
        if image.ndim not in (2, 3) or (image.ndim == 3 and image.shape[2] != 3):
            raise ValueError("Frames must be grayscale or RGB")
        if not math.isfinite(timestamp_s):
            raise ValueError("Frame timestamps must be finite")
        self.frame, self.timestamp_s = image.copy(), float(timestamp_s)
        self.sequence += 1

    def body_point(
        self, optical_point: np.ndarray | Sequence[float]
    ) -> NDArray[np.float64]:
        """An optical-frame point in body FRD meters."""
        return (
            self.rotation @ np.asarray(optical_point, dtype=np.float64)
            + self.position_m
        )
