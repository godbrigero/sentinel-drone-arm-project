"""Onboard camera geometry and the latest frame, independent of the image source."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import mujoco
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


@dataclass(frozen=True)
class CameraSpec:
    """Pinhole pose in body coordinates: meters, wxyz, -Z forward, +Y up.

    `fovy` alone gives a centred symmetric pinhole. `focal_px` (fx, fy) and
    `principal_px` (cx, cy, OpenCV pixel convention) instead give measured
    intrinsics at `width` x `height`; `fovy` is then ignored by MuJoCo.
    """

    name: str
    body: str
    pos: tuple[float, float, float]
    quat: tuple[float, float, float, float]
    width: int = 640
    height: int = 480
    fovy: float = 80.7
    focal_px: tuple[float, float] | None = None
    principal_px: tuple[float, float] | None = None

    PIXEL_PITCH = 1e-5
    """Nominal sensor pixel size (m); only the focal/pitch ratio matters."""

    def __post_init__(self) -> None:
        if not self.name or not self.body:
            raise ValueError("Camera name and body must be nonempty")
        if self.width < 1 or self.height < 1 or not 0 < self.fovy < 180:
            raise ValueError("Invalid camera resolution or FOV")
        if not np.isfinite((*self.pos, *self.quat)).all():
            raise ValueError("Camera pose must be finite")
        if not np.isclose(np.linalg.norm(self.quat), 1):
            raise ValueError("Camera quaternion must be unit length")
        if (self.focal_px is None) != (self.principal_px is None):
            raise ValueError("Give both focal_px and principal_px, or neither")
        if self.focal_px is not None and not min(self.focal_px) > 0:
            raise ValueError("Focal lengths must be positive")

    def attach(self, spec: mujoco.MjSpec) -> None:
        """Attach before compilation; names are local to the containing spec."""
        body = spec.worldbody if self.body == "world" else spec.body(self.body)
        if body is None:
            raise ValueError(f"Unknown camera body: {self.body}")
        if any(camera.name == self.name for camera in spec.cameras):
            raise ValueError(f"Duplicate camera name: {self.name}")
        camera = body.add_camera(name=self.name, pos=self.pos, quat=self.quat, fovy=self.fovy)
        if self.focal_px is not None and self.principal_px is not None:
            pitch = self.PIXEL_PITCH
            # MuJoCo stores these spec fields as int32/float32 pairs.
            camera.resolution = np.array([self.width, self.height], dtype=np.int32)
            camera.sensor_size = np.array([self.width, self.height], dtype=np.float32) * np.float32(pitch)
            camera.focal_length = np.array(self.focal_px, dtype=np.float32) * np.float32(pitch)
            # MuJoCo's principal offset is measured from the image centre with the opposite sign.
            camera.principal_length = np.array(
                [((self.width - 1) / 2 - self.principal_px[0]) * pitch,
                 ((self.height - 1) / 2 - self.principal_px[1]) * pitch],
                dtype=np.float32,
            )
