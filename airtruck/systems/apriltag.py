"""AprilTag detections shared by simulated cameras and the PX4/ROS tag topic.

`TAGS` is the process-wide board. Every `Drone` publishes into it by default,
whether detections come from its simulated cameras, externally supplied frames,
or the aircraft's ROS `/tag_detections` topic. Import it anywhere:

    from airtruck.systems.apriltag import TAGS
    detection = TAGS.latest(vehicle="drone")
"""

from __future__ import annotations

import threading
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Literal

import cv2
import mujoco
import numpy as np
from numpy.typing import NDArray

FAMILIES = {
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
}
# Data bits per side; the tag adds a one-cell black border on each side.
BITS = {"tag16h5": 4, "tag25h9": 5, "tag36h10": 6, "tag36h11": 6}


@dataclass(frozen=True)
class AprilTagConfig:
    """Opt-in image detection for a drone's cameras.

    `tag_size_m` is the outer edge of the black border. An empty `cameras`
    tuple processes every camera on the drone.
    """

    tag_size_m: float = 0.16
    family: str = "tag36h11"
    cameras: tuple[str, ...] = ()
    rate_hz: float = 10.0

    def __post_init__(self) -> None:
        if self.family not in FAMILIES:
            raise ValueError(f"Unsupported AprilTag family {self.family!r}")
        if not all(np.isfinite(v) and v > 0 for v in (self.tag_size_m, self.rate_hz)):
            raise ValueError("Tag size and detection rate must be positive and finite")


@dataclass(frozen=True)
class TagDetection:
    """One tag pose measurement.

    `position_m`/`quaternion` give the tag pose in the camera's optical frame
    (x right, y down, z forward; wxyz quaternion rotating tag axes into it).
    The tag frame has x right and y up in the printed image, z out of its face.
    NED fields are filled when the camera mounting and vehicle pose are known.
    `tag_id` is None when the source (a ROS PoseStamped) carries no ID.
    """

    vehicle: str
    camera: str
    tag_id: int | None
    position_m: NDArray[np.float64]
    quaternion: NDArray[np.float64]
    timestamp_s: float
    source: Literal["simulation", "camera", "ros"]
    position_ned_m: NDArray[np.float64] | None = None
    quaternion_ned: NDArray[np.float64] | None = None
    corners_px: NDArray[np.float64] | None = field(default=None, repr=False)

    @property
    def distance_m(self) -> float:
        return float(np.linalg.norm(self.position_m))


class TagBoard:
    """Thread-safe latest detection per (vehicle, camera, tag ID)."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: dict[tuple[str, str, int | None], TagDetection] = {}
        self._listeners: list[Callable[[TagDetection], None]] = []

    def publish(self, detection: TagDetection) -> None:
        with self._lock:
            self._latest[(detection.vehicle, detection.camera, detection.tag_id)] = (
                detection
            )
            listeners = list(self._listeners)
        for listener in listeners:
            listener(detection)

    def detections(
        self,
        *,
        vehicle: str | None = None,
        camera: str | None = None,
        tag_id: int | None = None,
    ) -> list[TagDetection]:
        """Matching latest detections, newest first."""
        with self._lock:
            values = list(self._latest.values())
        return sorted(
            (
                d
                for d in values
                if (vehicle is None or d.vehicle == vehicle)
                and (camera is None or d.camera == camera)
                and (tag_id is None or d.tag_id == tag_id)
            ),
            key=lambda d: d.timestamp_s,
            reverse=True,
        )

    def latest(
        self,
        *,
        vehicle: str | None = None,
        camera: str | None = None,
        tag_id: int | None = None,
    ) -> TagDetection | None:
        matches = self.detections(vehicle=vehicle, camera=camera, tag_id=tag_id)
        return matches[0] if matches else None

    def subscribe(self, callback: Callable[[TagDetection], None]) -> Callable[[], None]:
        """Call `callback` for every new detection; returns an unsubscribe function."""
        with self._lock:
            self._listeners.append(callback)

        def unsubscribe() -> None:
            with self._lock:
                if callback in self._listeners:
                    self._listeners.remove(callback)

        return unsubscribe

    def clear(self, *, vehicle: str | None = None) -> None:
        with self._lock:
            for key in [k for k in self._latest if vehicle is None or k[0] == vehicle]:
                del self._latest[key]


TAGS = TagBoard()


@dataclass(frozen=True)
class ImageDetection:
    tag_id: int
    position_m: NDArray[np.float64]
    quaternion: NDArray[np.float64]
    corners_px: NDArray[np.float64]


class AprilTagDetector:
    """OpenCV AprilTag decoding and square-target PnP for a pinhole camera."""

    def __init__(self, family: str = "tag36h11", tag_size_m: float = 0.16) -> None:
        AprilTagConfig(tag_size_m=tag_size_m, family=family)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(FAMILIES[family]), parameters
        )
        half = tag_size_m / 2
        # OpenCV corner order: top-left, top-right, bottom-right, bottom-left.
        self._object = np.array(
            [[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]]
        )

    def detect(
        self,
        image: np.ndarray,
        intrinsics: np.ndarray,
        distortion: np.ndarray | None = None,
    ) -> list[ImageDetection]:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if image.ndim == 3 else image
        corners, ids, _ = self._detector.detectMarkers(gray)
        results = []
        for marker, tag_id in zip(corners, [] if ids is None else ids.ravel()):
            points = np.asarray(marker, dtype=np.float64).reshape(4, 2)
            found, rotation_vector, translation = cv2.solvePnP(
                self._object,
                points,
                intrinsics,
                distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not found:
                continue
            matrix, _ = cv2.Rodrigues(rotation_vector)
            quaternion = np.empty(4)
            mujoco.mju_mat2Quat(
                quaternion, np.ascontiguousarray(matrix, dtype=np.float64).reshape(9)
            )
            results.append(
                ImageDetection(
                    int(tag_id),
                    np.asarray(translation, dtype=np.float64).reshape(3),
                    quaternion,
                    points,
                )
            )
        return results


def apriltag_image(
    tag_id: int, family: str = "tag36h11", pixels_per_cell: int = 16
) -> NDArray[np.uint8]:
    """RGB tag image with a one-cell white quiet zone around the black border."""
    cells = BITS[family] + 2
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(FAMILIES[family]),
        tag_id,
        cells * pixels_per_cell,
        borderBits=1,
    )
    pad = pixels_per_cell
    padded = cv2.copyMakeBorder(
        marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255
    )
    return np.repeat(np.asarray(padded, dtype=np.uint8)[:, :, None], 3, axis=2)


def add_apriltag(
    spec: mujoco.MjSpec,
    tag_id: int,
    *,
    size_m: float = 0.16,
    position: Sequence[float] | np.ndarray = (0, 0, 0),
    quaternion: Sequence[float] | np.ndarray = (1, 0, 0, 0),
    family: str = "tag36h11",
    name: str | None = None,
) -> str:
    """Add a static textured tag to a world specification; returns its geom name.

    The printed face points along the frame's +Z, with the image's up along +Y.
    `size_m` is the black-border edge; the white quiet zone is added around it.
    """
    AprilTagConfig(tag_size_m=size_m, family=family)
    name = name or f"apriltag_{family}_{tag_id}"
    image = apriltag_image(tag_id, family)
    texture = spec.add_texture(
        name=name,
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        width=image.shape[1],
        height=image.shape[0],
        nchannel=3,
    )
    texture.data = image.tobytes()
    material = spec.add_material(name=name)
    material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = name
    half = size_m / 2 * (BITS[family] + 4) / (BITS[family] + 2)
    spec.worldbody.add_geom(
        name=name,
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[half, half, 0.0005],
        pos=list(position),
        quat=list(quaternion),
        material=name,
        contype=0,
        conaffinity=0,
    )
    return name
