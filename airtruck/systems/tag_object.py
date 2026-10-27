"""A movable, simulation-only AprilTag plate textured from a tag SVG.

    tag = AprilTagObject("tag", position=(1, 0, 0.01))
    with SystemDriver([drone, tag], world=ARENA) as driver:
        driver.reset()
        tag.move((0.2, 0, 0))                  # delta, world meters
        tag.set_position((0.5, 0.5, 0.01))     # absolute
        tag.rotate(rpy=(0, 0, 90), degrees=True)  # delta about its own axes
        tag.set_rotation(rpy=(0, 0, 0))        # absolute
        driver.step()
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Literal

import mujoco
import numpy as np
from gymnasium import spaces
from numpy.typing import ArrayLike, NDArray

from airtruck.systems.base import ROOT, SimSystem, Values

DEFAULT_SVG = ROOT / "models/apriltag/tag36h11-2.svg"
PLATE_THICKNESS_M = 0.001
_UNITS_M = {"mm": 0.001, "cm": 0.01, "m": 1.0, "in": 0.0254}
_PATH_TOKEN = re.compile(r"[MmLlHhVvZz]|[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?")


def _subpaths(d: str) -> list[NDArray[np.float64]]:
    """Polygons of an SVG path made of straight segments (M/L/H/V/Z)."""
    tokens = _PATH_TOKEN.findall(d)
    if "".join(tokens).replace(" ", "") != re.sub(r"[\s,]", "", d):
        raise ValueError("Tag SVG paths may only contain M, L, H, V and Z commands")
    polygons: list[list[tuple[float, float]]] = []
    x = y = 0.0
    command, i = "", 0
    while i < len(tokens):
        if tokens[i].isalpha():
            command, i = tokens[i], i + 1
            if command in "Zz":
                if polygons and polygons[-1]:
                    x, y = polygons[-1][0]
                continue
        relative = command.islower()
        kind = command.upper()
        if i + (2 if kind in "ML" else 1) > len(tokens):
            raise ValueError("Tag SVG path ends inside a command")
        if kind in "ML":
            dx, dy = float(tokens[i]), float(tokens[i + 1])
            i += 2
            x, y = (x + dx, y + dy) if relative else (dx, dy)
            if kind == "M":
                polygons.append([])
                command = "l" if relative else "L"  # Implicit lineto after moveto.
        elif kind == "H":
            x = x + float(tokens[i]) if relative else float(tokens[i])
            i += 1
        elif kind == "V":
            y = y + float(tokens[i]) if relative else float(tokens[i])
            i += 1
        else:
            raise ValueError(f"Malformed tag SVG path near {tokens[i]!r}")
        if not polygons:
            raise ValueError("Tag SVG paths must start with a moveto")
        polygons[-1].append((x, y))
    return [np.array(p) for p in polygons if len(p) >= 3]


def _length_m(value: str | None) -> float | None:
    match = re.fullmatch(r"\s*([\d.]+)\s*(mm|cm|m|in)\s*", value or "")
    return float(match[1]) * _UNITS_M[match[2]] if match else None


def load_tag_svg(
    path: str | Path, pixels_per_unit: int = 8
) -> tuple[NDArray[np.uint8], float | None]:
    """Rasterize a flat-filled tag SVG to RGB; returns the image and its width in meters.

    Supports the polygon-path SVGs of the AprilTag image generators. The width is
    None when the SVG does not state a physical unit.
    """
    root = ET.parse(path).getroot()
    namespace = "{http://www.w3.org/2000/svg}"
    view = [float(v) for v in root.get("viewBox", "").replace(",", " ").split()]
    if len(view) != 4 or view[2] <= 0 or view[3] <= 0:
        raise ValueError(f"{path} needs a viewBox")
    left, top, width, height = view
    image = np.full(
        (round(height * pixels_per_unit), round(width * pixels_per_unit), 3),
        255,
        np.uint8,
    )
    for element in root.iter(f"{namespace}path"):
        style = dict(
            (k.strip(), v.strip())
            for k, _, v in (s.partition(":") for s in element.get("style", "").split(";"))
            if k.strip()
        )
        fill = style.get("fill", element.get("fill", "#000000"))
        if fill == "none":
            continue
        if not re.fullmatch(r"#[0-9a-fA-F]{6}", fill):
            raise ValueError(f"Unsupported tag SVG fill {fill!r}")
        polygons = [
            (p - (left, top)) * pixels_per_unit for p in _subpaths(element.get("d", ""))
        ]
        if polygons:
            image[_inside(polygons, image.shape[:2])] = [
                int(fill[k : k + 2], 16) for k in (1, 3, 5)
            ]
    return image, _length_m(root.get("width"))


def _inside(
    polygons: list[NDArray[np.float64]], shape: tuple[int, ...]
) -> NDArray[np.bool_]:
    """Pixels whose centers lie inside the polygons (SVG's nonzero fill rule)."""
    start = np.concatenate(polygons)
    end = np.concatenate([np.roll(p, -1, axis=0) for p in polygons])
    rows = np.arange(shape[0])[:, None] + 0.5
    up = (start[:, 1] <= rows) & (rows < end[:, 1])
    down = (end[:, 1] <= rows) & (rows < start[:, 1])
    with np.errstate(divide="ignore", invalid="ignore"):
        crossing = start[:, 0] + (rows - start[:, 1]) * (end[:, 0] - start[:, 0]) / (
            end[:, 1] - start[:, 1]
        )
    direction = up.astype(int) - down.astype(int)
    centers = np.arange(shape[1]) + 0.5
    # Winding number: signed edge crossings to the right of each pixel center.
    right = crossing[:, None, :] > centers[None, :, None]
    return (np.where(right, direction[:, None, :], 0).sum(axis=2)) != 0


def _quat(value: ArrayLike) -> NDArray[np.float64]:
    q = np.asarray(value, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError("Quaternions must be four finite wxyz numbers")
    return q / np.linalg.norm(q)


def rpy_quaternion(rpy: ArrayLike, *, degrees: bool = False) -> NDArray[np.float64]:
    """wxyz quaternion of roll about X, then pitch about Y, then yaw about Z (fixed axes)."""
    angles = np.asarray(rpy, dtype=np.float64)
    if angles.shape != (3,) or not np.isfinite(angles).all():
        raise ValueError("rpy must be three finite angles")
    if degrees:
        angles = np.radians(angles)
    result = np.array([1.0, 0, 0, 0])
    for axis, angle in zip(np.eye(3), angles):
        step = np.empty(4)
        mujoco.mju_axisAngle2Quat(step, axis, angle)
        mujoco.mju_mulQuat(result, step, result.copy())
    return result


def _rotation(
    quaternion: ArrayLike | None, rpy: ArrayLike | None, degrees: bool
) -> NDArray[np.float64]:
    if quaternion is not None and rpy is None:
        return _quat(quaternion)
    if rpy is not None and quaternion is None:
        return rpy_quaternion(rpy, degrees=degrees)
    raise ValueError("Give exactly one of quaternion or rpy")


class AprilTagObject(SimSystem):
    """A kinematic (mocap) tag plate that can be repositioned while simulating.

    The printed face points along the object's +Z with the image's top along +Y,
    so the default orientation lies flat facing up. `size_m` is the black-border
    edge (the AprilTag convention); the SVG's white quiet zone is added around it.
    By default the size follows the SVG's physical width. Pose changes write
    MuJoCo's mocap pose directly: the plate teleports rather than being driven by
    forces, and `driver.reset()` returns it to its constructor placement.
    The plate does not collide unless `collisions=True`, in which case it acts as
    an immovable surface. There is no hardware counterpart.
    """

    controllable = False
    action_space = spaces.Dict({})

    def __init__(
        self,
        name: str = "apriltag",
        *,
        svg: str | Path = DEFAULT_SVG,
        size_m: float | None = None,
        collisions: bool = False,
        **kwargs: Any,
    ) -> None:
        super().__init__(name, model=svg, root_body="tag", **kwargs)
        self.image, svg_width_m = load_tag_svg(self.model_path)
        match = re.match(r"(tag\w+?)-(\d+)", self.model_path.stem)
        self.family = match[1] if match else None
        self.tag_id = int(match[2]) if match else None
        # Fraction of the image width covered by the black border.
        dark = np.flatnonzero((self.image.min(axis=(0, 2)) < 128))
        if dark.size == 0:
            raise ValueError(f"{self.model_path} contains no dark tag border")
        border_fraction = (dark[-1] - dark[0] + 1) / self.image.shape[1]
        if size_m is None and svg_width_m is None:
            raise ValueError("The SVG has no physical width; pass size_m")
        edge = float(size_m if size_m is not None else svg_width_m * border_fraction)
        if not (math.isfinite(edge) and edge > 0):
            raise ValueError("size_m must be positive and finite")
        self.size_m = edge
        self.plate_size_m = self.size_m / border_fraction
        self.collisions = collisions
        self.observation_space = spaces.Dict(
            {
                "position": spaces.Box(-np.inf, np.inf, (3,), np.float64),
                "quaternion": spaces.Box(-1, 1, (4,), np.float64),
            }
        )
        self._mocap = -1

    def make_spec(self) -> mujoco.MjSpec:
        spec = mujoco.MjSpec()
        height, width = self.image.shape[:2]
        texture = spec.add_texture(
            name="image",
            type=mujoco.mjtTexture.mjTEXTURE_2D,
            width=width,
            height=height,
            nchannel=3,
        )
        # Top row first: a box's +Z face then shows the image upright along +Y
        # (checked by decoding renders; other faces show mirrored copies).
        texture.data = self.image.tobytes()
        material = spec.add_material(name="image")
        material.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "image"
        body = spec.worldbody.add_body(name="tag", mocap=True)
        half = self.plate_size_m / 2
        # A thin printed skin on the body origin, over a blank backing plate that
        # hides the skin's mirrored back face (which would otherwise decode).
        print_thickness = PLATE_THICKNESS_M / 5
        body.add_geom(
            name="face",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[half, half, print_thickness / 2],
            pos=[0, 0, -print_thickness / 2],
            material="image",
            contype=0,
            conaffinity=0,
        )
        body.add_geom(
            name="backing",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[half, half, (PLATE_THICKNESS_M - print_thickness) / 2],
            pos=[0, 0, -(PLATE_THICKNESS_M + print_thickness) / 2],
            rgba=[0.9, 0.9, 0.9, 1],
            contype=int(self.collisions),
            conaffinity=int(self.collisions),
        )
        return spec

    def bind(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        self._mocap = int(model.body_mocapid[model.body(f"{self.name}/tag").id])
        super().bind(model, data)

    @property
    def pose(self) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
        """World position (m) and wxyz quaternion of the printed face's center."""
        if self._data is None:
            return np.array(self.position, float), _quat(self.quaternion)
        return (
            self._data.mocap_pos[self._mocap].copy(),
            self._data.mocap_quat[self._mocap].copy(),
        )

    def set_pose(
        self,
        position: ArrayLike | None = None,
        quaternion: ArrayLike | None = None,
    ) -> None:
        """Absolute world pose; omitted parts are unchanged.

        Before the object joins a driver this changes its initial placement.
        """
        current_position, current_quaternion = self.pose
        if position is not None:
            current_position = np.asarray(position, dtype=np.float64)
            if current_position.shape != (3,) or not np.isfinite(current_position).all():
                raise ValueError("Positions must be three finite meters")
        if quaternion is not None:
            current_quaternion = _quat(quaternion)
        if self._data is None:
            self.position = tuple(current_position.tolist())
            self.quaternion = tuple(current_quaternion.tolist())
        else:
            self._data.mocap_pos[self._mocap] = current_position
            self._data.mocap_quat[self._mocap] = current_quaternion

    def set_position(self, position: ArrayLike) -> None:
        self.set_pose(position=position)

    def move(
        self, delta: ArrayLike, *, frame: Literal["world", "local"] = "world"
    ) -> None:
        """Translate by `delta` meters along world axes or the tag's own axes."""
        offset = np.asarray(delta, dtype=np.float64)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError("Deltas must be three finite meters")
        position, quaternion = self.pose
        if frame == "local":
            rotated = np.empty(3)
            mujoco.mju_rotVecQuat(rotated, offset, quaternion)
            offset = rotated
        elif frame != "world":
            raise ValueError("frame must be 'world' or 'local'")
        self.set_pose(position=position + offset)

    def set_rotation(
        self,
        quaternion: ArrayLike | None = None,
        *,
        rpy: ArrayLike | None = None,
        degrees: bool = False,
    ) -> None:
        """Absolute orientation as a wxyz quaternion or fixed-axis roll/pitch/yaw."""
        self.set_pose(quaternion=_rotation(quaternion, rpy, degrees))

    def rotate(
        self,
        quaternion: ArrayLike | None = None,
        *,
        rpy: ArrayLike | None = None,
        degrees: bool = False,
        frame: Literal["world", "local"] = "local",
    ) -> None:
        """Rotate by a delta about the tag's own axes (default) or world axes.

        The tag rotates about its face center; its position is unchanged.
        """
        delta = _rotation(quaternion, rpy, degrees)
        _, current = self.pose
        result = np.empty(4)
        if frame == "local":
            mujoco.mju_mulQuat(result, current, delta)
        elif frame == "world":
            mujoco.mju_mulQuat(result, delta, current)
        else:
            raise ValueError("frame must be 'world' or 'local'")
        mujoco.mju_normalize4(result)
        self.set_pose(quaternion=result)

    def reset(self) -> None:
        pass

    def step(self, dt: float) -> None:
        pass

    def observe(self) -> Values:
        position, quaternion = self.pose
        self.state = {"position": position, "quaternion": quaternion}
        return self.state
