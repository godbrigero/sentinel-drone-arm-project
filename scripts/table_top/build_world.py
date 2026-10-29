"""Convert the cleaned scan into native MuJoCo assets and table worlds.

Run: uv run --locked python scripts/table_top/build_world.py
Placements and approximate collision dimensions live in the model information.json.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import cv2
import mujoco
import numpy as np
from scipy.ndimage import distance_transform_edt  # type: ignore[import-untyped]

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "models/table-top"
WORLD = ROOT / "worlds/table_top"


def fmt(values: Any) -> str:
    return " ".join(f"{float(x):.12g}" for x in np.ravel(values))


def read_glb(path: Path) -> tuple[dict[str, Any], bytes]:
    data = path.read_bytes()
    magic, version, size = struct.unpack_from("<III", data)
    if (magic, version, size) != (0x46546C67, 2, len(data)):
        raise ValueError("Expected a complete glTF 2 binary")
    length, kind = struct.unpack_from("<II", data, 12)
    if kind != 0x4E4F534A:
        raise ValueError("Missing GLB JSON chunk")
    gltf = json.loads(data[20 : 20 + length])
    binary_length, kind = struct.unpack_from("<II", data, 20 + length)
    if kind != 0x004E4942 or 28 + length + binary_length != len(data):
        raise ValueError("Missing or truncated GLB binary chunk")
    # This builder deliberately accepts the one identity mesh node in this scan.
    nodes = [node for node in gltf["nodes"] if "mesh" in node]
    if (
        len(nodes) != 1
        or nodes[0]["mesh"] != 0
        or any(
            key in nodes[0] for key in ("translation", "rotation", "scale", "matrix")
        )
    ):
        raise ValueError("Scan mesh must have the recorded identity transform")
    return gltf, data[28 + length :]


def accessor(gltf: dict[str, Any], binary: bytes, index: int) -> np.ndarray:
    item = gltf["accessors"][index]
    view = gltf["bufferViews"][item["bufferView"]]
    dtype = np.dtype({5123: "<u2", 5125: "<u4", 5126: "<f4"}[item["componentType"]])
    width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}[item["type"]]
    if view.get("byteStride", width * dtype.itemsize) != width * dtype.itemsize:
        raise ValueError("Unexpected interleaved scan accessor")
    return np.frombuffer(
        binary,
        dtype=dtype,
        count=item["count"] * width,
        offset=view.get("byteOffset", 0) + item.get("byteOffset", 0),
    ).reshape(-1, width)


def write_mesh(
    path: Path, vertices: np.ndarray, uv: np.ndarray, faces: np.ndarray
) -> None:
    """Native MSH keeps all triangles and explicit texture coordinates."""
    if not np.isfinite(vertices).all() or not np.isfinite(uv).all():
        raise ValueError("Nonfinite scan geometry")
    if len(uv) != len(vertices) or faces.max() >= len(vertices):
        raise ValueError("Invalid scan indices")
    path.write_bytes(
        struct.pack("<4i", len(vertices), 0, len(uv), len(faces))
        + vertices.astype("<f4").tobytes()
        + uv.astype("<f4").tobytes()
        + faces.astype("<i4").tobytes()
    )


def bake_colors(
    patches: list[tuple[np.ndarray, np.ndarray, np.ndarray]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Bake continuous planar UV charts, avoiding per-triangle mipmap seams."""
    tile = 512
    columns = math.ceil(math.sqrt(len(patches)))
    rows = math.ceil(len(patches) / columns)
    image = np.zeros((rows * tile, columns * tile, 3), dtype=np.uint8)
    vertices_out, uv_out, faces_out = [], [], []
    vertex_offset = 0
    for index, (vertices, faces, colors) in enumerate(patches):
        # These repaired surfaces are single-valued over their principal plane.
        centered = vertices - vertices.mean(axis=0)
        _, _, axes = np.linalg.svd(centered, full_matrices=False)
        coordinates = centered @ axes[:2].T
        low, high = coordinates.min(axis=0), coordinates.max(axis=0)
        pixels = 4.5 + (coordinates - low) / (high - low) * (tile - 9)
        chart = np.zeros((tile, tile, 3))
        covered = np.zeros((tile, tile), dtype=bool)
        for face in faces:
            a, b, c = pixels[face]
            low_pixel = np.maximum(
                np.floor(np.min(pixels[face], axis=0)).astype(int), 0
            )
            high_pixel = np.minimum(
                np.ceil(np.max(pixels[face], axis=0)).astype(int), tile - 1
            )
            xx, yy = np.meshgrid(
                np.arange(low_pixel[0], high_pixel[0] + 1) + 0.5,
                np.arange(low_pixel[1], high_pixel[1] + 1) + 0.5,
            )
            determinant = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(determinant) < 1e-12:
                raise ValueError("Degenerate repaired-surface UV triangle")
            wa = (
                (b[1] - c[1]) * (xx - c[0]) + (c[0] - b[0]) * (yy - c[1])
            ) / determinant
            wb = (
                (c[1] - a[1]) * (xx - c[0]) + (a[0] - c[0]) * (yy - c[1])
            ) / determinant
            weights = np.stack([wa, wb, 1 - wa - wb], axis=-1)
            mask = np.all(weights >= -1e-7, axis=-1)
            region = (
                slice(low_pixel[1], high_pixel[1] + 1),
                slice(low_pixel[0], high_pixel[0] + 1),
            )
            chart[region][mask] = (weights @ colors[face, :3])[mask]
            covered[region] |= mask
        if not covered.any():
            raise ValueError("Empty repaired-surface texture chart")
        nearest = np.asarray(
            distance_transform_edt(
                ~covered, return_distances=False, return_indices=True
            ),
            dtype=np.int64,
        )
        chart[~covered] = chart[nearest[0][~covered], nearest[1][~covered]]
        srgb = np.where(
            chart <= 0.0031308, chart * 12.92, 1.055 * chart ** (1 / 2.4) - 0.055
        )
        x, y = (index % columns) * tile, (index // columns) * tile
        image[y : y + tile, x : x + tile] = np.rint(np.clip(srgb, 0, 1) * 255).astype(
            np.uint8
        )
        vertices_out.append(vertices)
        uv_out.append((pixels + [x, y]) / [image.shape[1], image.shape[0]])
        faces_out.append(faces + vertex_offset)
        vertex_offset += len(vertices)
    # MuJoCo's image loader and MSH coordinates use this GLB's vertical convention.
    return (
        image,
        np.concatenate(vertices_out),
        np.concatenate(uv_out),
        np.concatenate(faces_out),
    )


def xml(path: Path, root: ET.Element) -> None:
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def build() -> None:
    info = json.loads((MODEL / "information.json").read_text())
    source = MODEL / info["simulation"]["source"]
    if (
        hashlib.sha256(source.read_bytes()).hexdigest()
        != info["table_scan"]["model_sha256"]
    ):
        raise ValueError("Cleaned scan hash differs from its engineering record")
    gltf, binary = read_glb(source)
    assets = MODEL / "assets"
    assets.mkdir(exist_ok=True)
    WORLD.mkdir(parents=True, exist_ok=True)
    transform = np.array(info["table_scan"]["frame"]["gltf_to_z_up_row_major"]).reshape(
        4, 4
    )
    patches: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    triangles = 0
    for primitive in gltf["meshes"][0]["primitives"]:
        if primitive.get("mode", 4) != 4:
            raise ValueError("Only triangle primitives are supported")
        attributes = primitive["attributes"]
        vertices = accessor(gltf, binary, attributes["POSITION"]) @ transform[:3, :3].T
        faces = accessor(gltf, binary, primitive["indices"]).reshape(-1, 3)
        triangles += len(faces)
        if "COLOR_0" in attributes:
            patches.append(
                (vertices, faces, accessor(gltf, binary, attributes["COLOR_0"]))
            )
            continue
        uv = accessor(gltf, binary, attributes["TEXCOORD_0"]).copy()
        write_mesh(assets / "scan.msh", vertices, uv, faces)
        material = gltf["materials"][primitive["material"]]["pbrMetallicRoughness"]
        texture = gltf["textures"][material["baseColorTexture"]["index"]]
        image = gltf["images"][texture["source"]]
        view = gltf["bufferViews"][image["bufferView"]]
        start = view.get("byteOffset", 0)
        pixels = cv2.imdecode(
            np.frombuffer(binary[start : start + view["byteLength"]], np.uint8),
            cv2.IMREAD_COLOR,
        )
        if pixels is None or not cv2.imwrite(
            str(assets / "scan.png"), pixels, [cv2.IMWRITE_PNG_COMPRESSION, 9]
        ):
            raise ValueError("Could not decode/write scan texture")
    if triangles != info["table_scan"]["cleaned_triangle_count"]:
        raise ValueError("Scan triangle count differs from its engineering record")
    pixels, patch, uv, faces = bake_colors(patches)
    write_mesh(assets / "repairs.msh", patch, uv, faces)
    if not cv2.imwrite(
        str(assets / "repairs.png"), cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    ):
        raise ValueError("Could not write repaired-surface texture")

    sim = info["simulation"]
    leader = sim["leader"]
    table = sim["tabletop_collision_proxy"]
    model = ET.Element("mujoco", model="captured_table")
    asset = ET.SubElement(model, "asset")
    body = ET.SubElement(model, "worldbody")
    for name in ("scan", "repairs"):
        ET.SubElement(
            asset,
            "mesh",
            name=f"table_{name}",
            file=f"assets/{name}.msh",
            inertia="shell",
            maxhullvert="64",
        )
        ET.SubElement(
            asset, "texture", name=f"table_{name}", type="2d", file=f"assets/{name}.png"
        )
        ET.SubElement(
            asset,
            "material",
            name=f"table_{name}",
            texture=f"table_{name}",
            specular="0",
            shininess="0",
            emission="0.25",
        )
        ET.SubElement(
            body,
            "geom",
            name=f"table_{name}_visual",
            type="mesh",
            mesh=f"table_{name}",
            material=f"table_{name}",
            contype="0",
            conaffinity="0",
            group="2",
        )
    center = (
        np.array(table["top_center_m"])
        - np.array(table["surface_normal"]) * table["half_size_m"][2]
    )
    ET.SubElement(
        body,
        "geom",
        name="tabletop_collision",
        type="box",
        pos=fmt(center),
        quat=fmt(table["quaternion_wxyz"]),
        size=fmt(table["half_size_m"]),
        group="3",
        rgba="0 0 0 0",
        friction="1 .005 .0001",
    )
    ET.SubElement(
        body,
        "geom",
        name="floor",
        type="plane",
        pos=f"0 0 {sim['floor_collision_proxy']['height_m']}",
        size="2 2 .1",
        group="3",
        rgba="0 0 0 0",
    )
    ET.SubElement(
        body,
        "site",
        name="leader_base",
        pos=fmt(leader["root_position_m"]),
        quat=fmt(leader["quaternion_wxyz"]),
        size=".003",
        rgba="0 0 0 0",
        group="5",
    )
    ET.SubElement(
        body,
        "site",
        name="leader_mount_reference",
        pos=fmt(leader["mount_reference_position_m"]),
        size=".004",
        rgba="0 0 0 0",
        group="5",
    )
    # Paths in included MJCF are relative to the including file's directory.
    for item in asset:
        if "file" in item.attrib:
            item.set("file", "../../models/table-top/" + item.attrib["file"])
    xml(MODEL / "table.xml", model)

    world = ET.Element("mujoco", model="table_world")
    ET.SubElement(world, "include", file="../../models/table-top/table.xml")
    ET.SubElement(world, "compiler", angle="radian", inertiafromgeom="false")
    ET.SubElement(
        world,
        "option",
        timestep="0.0005",
        integrator="implicitfast",
        gravity="0 0 -9.81",
    )
    ET.SubElement(world, "size", memory="32M")
    ET.SubElement(world, "statistic", center="-.05 .1 .72", extent="1.25")
    body = ET.SubElement(world, "worldbody")
    ET.SubElement(
        body,
        "light",
        pos="0 -.5 2",
        dir="0 0 -1",
        diffuse=".6 .6 .6",
        ambient=".25 .25 .25",
    )
    visual = ET.SubElement(world, "visual")
    ET.SubElement(
        visual,
        "global",
        offwidth="1280",
        offheight="960",
        azimuth="120",
        elevation="-25",
    )
    ET.SubElement(
        visual, "headlight", diffuse=".3 .3 .3", ambient=".3 .3 .3", specular="0 0 0"
    )
    xml(WORLD / "scene.xml", world)

    standalone = ET.Element("mujoco", model="table_leader")
    ET.SubElement(standalone, "include", file="scene.xml")
    asset = ET.SubElement(standalone, "asset")
    ET.SubElement(asset, "model", name="so101", file="../../models/so101/so101.xml")
    body = ET.SubElement(standalone, "worldbody")
    frame = ET.SubElement(
        body,
        "frame",
        pos=fmt(leader["root_position_m"]),
        quat=fmt(leader["quaternion_wxyz"]),
    )
    ET.SubElement(frame, "attach", model="so101", body="base", prefix="leader/")
    xml(WORLD / "leader.xml", standalone)
    for path in (WORLD / "scene.xml", WORLD / "leader.xml"):
        compiled = mujoco.MjModel.from_xml_path(str(path))
        print(f"{path.relative_to(ROOT)}: {compiled.nq} DOFs, {compiled.ngeom} geoms")
    print(f"Preserved {triangles:,} scan triangles; baked the repaired vertex colors.")


if __name__ == "__main__":
    build()
