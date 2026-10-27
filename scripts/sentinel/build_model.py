"""Build the Sentinel MJCF from an Onshape glTF export of the Sentinel_Drone assembly.

The export must come from the assembly recorded in `information.json` (Z up,
+X rear, meters). Visual triangles are kept as exported; only grouping by
color and rigid body changes. Collision uses one convex hull per top-level
occurrence, and propellers are visual-only spinning bodies.

    uv run --locked python scripts/sentinel/build_model.py /tmp/sentinel.gltf
"""

import argparse
import base64
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from collections import defaultdict
from collections.abc import Sequence
from pathlib import Path
from shutil import rmtree
from tempfile import mkdtemp

import mujoco
import numpy as np
import trimesh
from scipy.spatial import ConvexHull  # type: ignore[import-untyped]

ROOT = Path(__file__).resolve().parents[2] / "models" / "sentinel-quad"
ASSEMBLY = "Sentinel_Drone"
# Onshape (+X rear, +Y right, +Z up) → body FLU is a 180° yaw about +Z.
ONSHAPE_TO_BODY = np.diag([-1.0, -1.0, 1.0])
SPIN_SIGN = {"CW": -1, "CCW": 1}  # About the motor axis toward the shaft tip.


def fmt(values: np.ndarray | Sequence[float], digits: int = 9) -> str:
    return " ".join(f"{float(value):.{digits}g}" for value in np.ravel(values))


def to_body(points: np.ndarray, origin: np.ndarray) -> np.ndarray:
    return (np.asarray(points) - origin) @ ONSHAPE_TO_BODY.T


def mujoco_camera_quaternion(frame: dict[str, list[float]]) -> np.ndarray:
    """CAD optical frame (x right, y image-down, z outward) → MuJoCo camera."""
    x, y, z = (
        ONSHAPE_TO_BODY @ np.asarray(frame[key])
        for key in ("x_axis", "y_axis", "z_axis")
    )
    # MuJoCo cameras look along -z with +y up in the image.
    matrix = np.column_stack([x, -y, -z])
    if (
        not np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-6)
        or np.linalg.det(matrix) < 0
    ):
        raise ValueError(f"Camera frame is not a right-handed rotation: {frame}")
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(quaternion, matrix.reshape(9))
    return quaternion


def write_obj(path: Path, mesh: trimesh.Trimesh) -> str:
    lines = ["# Onshape Sentinel_Drone export, body frame, meters"]
    lines += [f"v {fmt(vertex, 7)}" for vertex in mesh.vertices]
    lines += [f"f {a + 1} {b + 1} {c + 1}" for a, b, c in mesh.faces]
    content = "\n".join(lines) + "\n"
    path.write_text(content)
    return hashlib.sha256(content.encode()).hexdigest()


def read_gltf(source: Path) -> list[tuple[str, tuple[float, ...], trimesh.Trimesh]]:
    """Every exported triangle as (top-level occurrence, rgba, world mesh).

    A direct reader keeps every exported triangle; Trimesh's scene loader was
    observed to drop a varying number of faces from this export.
    """
    gltf = json.loads(source.read_text())
    buffers = []
    for buffer in gltf["buffers"]:
        header, _, payload = buffer["uri"].partition(",")
        if not header.endswith(";base64"):
            raise ValueError("Expected an embedded base64 glTF buffer")
        buffers.append(base64.b64decode(payload))
    types = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
    widths = {"SCALAR": 1, "VEC3": 3}

    def accessor(index: int) -> np.ndarray:
        record = gltf["accessors"][index]
        view = gltf["bufferViews"][record["bufferView"]]
        dtype, width = np.dtype(types[record["componentType"]]), widths[record["type"]]
        if view.get("byteStride", dtype.itemsize * width) != dtype.itemsize * width:
            raise ValueError("Interleaved glTF buffers are not supported")
        start = view.get("byteOffset", 0) + record.get("byteOffset", 0)
        values = np.frombuffer(
            buffers[view["buffer"]], dtype, record["count"] * width, start
        )
        return values.reshape(-1, width) if width > 1 else values

    def local(node: dict) -> np.ndarray:
        if "matrix" in node:
            return np.asarray(node["matrix"], float).reshape(4, 4).T  # Column-major.
        transform = np.eye(4)
        if "rotation" in node:
            x, y, z, w = node["rotation"]
            matrix = np.empty(9)
            mujoco.mju_quat2Mat(matrix, np.array([w, x, y, z], float))
            transform[:3, :3] = matrix.reshape(3, 3)
        transform[:3, :3] *= node.get("scale", [1, 1, 1])
        transform[:3, 3] = node.get("translation", [0, 0, 0])
        return transform

    root = gltf["nodes"][gltf["scenes"][gltf.get("scene", 0)]["nodes"][0]]
    if root.get("name") != ASSEMBLY or local(root).tolist() != np.eye(4).tolist():
        raise ValueError(
            f"Expected the {ASSEMBLY} root node with an identity transform"
        )
    meshes = []

    def walk(index: int, parent: np.ndarray, group: str) -> None:
        node = gltf["nodes"][index]
        transform = parent @ local(node)
        for primitive in (
            gltf["meshes"][node["mesh"]]["primitives"] if "mesh" in node else []
        ):
            if primitive.get("mode", 4) != 4:
                raise ValueError("Only triangle primitives are supported")
            vertices = accessor(primitive["attributes"]["POSITION"]).astype(float)
            faces = accessor(primitive["indices"]).astype(np.int64).reshape(-1, 3)
            material = (
                gltf["materials"][primitive["material"]]
                if "material" in primitive
                else {}
            )
            rgba = material.get("pbrMetallicRoughness", {}).get(
                "baseColorFactor", [0.8, 0.8, 0.8, 1]
            )
            world = vertices @ transform[:3, :3].T + transform[:3, 3]
            meshes.append(
                (
                    group,
                    tuple(round(float(v), 3) for v in rgba),
                    trimesh.Trimesh(world, faces, process=False),
                )
            )
        for child in node.get("children", []):
            walk(child, transform, group)

    for child in root.get("children", []):
        walk(child, np.eye(4), f"{gltf['nodes'][child].get('name', '')}#{child}")
    return meshes


def build(source: Path) -> None:
    info = json.loads((ROOT / "information.json").read_text())
    sim = info["simulation"]
    origin = np.asarray(sim["body_origin_onshape_mm"]) / 1000
    rotors = info["motors"]["rotors"]
    variants = info["drone"]["propellers"]["CAD_rotation_by_variant"]

    # Groups are top-level assembly occurrences; glTF positions are meters.
    visuals: dict[str, dict[tuple[float, ...], list[trimesh.Trimesh]]] = defaultdict(
        lambda: defaultdict(list)
    )
    hull_sources: dict[str, list[np.ndarray]] = defaultdict(list)
    prop_meshes: dict[str, list[tuple[trimesh.Trimesh, tuple[float, ...]]]] = (
        defaultdict(list)
    )
    triangles = 0
    for group, rgba, mesh in read_gltf(source):
        triangles += len(mesh.faces)
        if "1045" in group:
            prop_meshes[group].append((mesh, rgba))
            continue
        body_mesh = trimesh.Trimesh(
            to_body(mesh.vertices, origin), mesh.faces, process=False
        )
        visuals["base"][rgba].append(body_mesh)
        hull_sources[group].append(body_mesh.vertices)
    if triangles != sim["CAD_export"]["triangles"]:
        raise ValueError(
            f"Export has {triangles} triangles; record expects {sim['CAD_export']['triangles']}"
        )

    # Match each propeller occurrence to the nearest recorded motor shaft.
    rotor_props: dict[int, str] = {}
    for group, parts in prop_meshes.items():
        center = trimesh.util.concatenate([mesh for mesh, _ in parts]).bounds.mean(
            axis=0
        )
        distances = [
            np.linalg.norm(
                center[:2] - np.asarray(r["shaft_shoulder_center_mm"][:2]) / 1000
            )
            for r in rotors
        ]
        rotor = rotors[int(np.argmin(distances))]
        if min(distances) > 0.01 or rotor["id"] in rotor_props:
            raise ValueError(f"Cannot uniquely match propeller {group!r} to a motor")
        variant = re.search(r"\((A|B)\)", group)
        if variant is None or variants[variant.group(1)] != rotor["spin"]:
            raise ValueError(
                f"Propeller {group!r} does not match rotor {rotor['id']} spin {rotor['spin']}"
            )
        rotor_props[rotor["id"]] = group
    if sorted(rotor_props) != [1, 2, 3, 4]:
        raise ValueError(f"Expected four propellers, matched {sorted(rotor_props)}")

    # Stage beside the final files so relative asset paths stay identical.
    staging = Path(mkdtemp(prefix=".build-", dir=ROOT))
    try:
        mass = stage(
            staging,
            info,
            triangles,
            visuals,
            hull_sources,
            prop_meshes,
            rotor_props,
            source,
        )
        old = ROOT / ".assets-old"
        if (ROOT / "assets").exists():
            (ROOT / "assets").rename(old)
        (staging / "assets").rename(ROOT / "assets")
        for name in ("sentinel.xml", "source/cad-manifest.json"):
            (ROOT / name).parent.mkdir(exist_ok=True)
            (staging / name).replace(ROOT / name)
        rmtree(old, ignore_errors=True)
    finally:
        rmtree(staging, ignore_errors=True)
    print(f"Built sentinel.xml: {triangles} triangles, mass {mass:.3f} kg")


def stage(
    staging: Path,
    info: dict,
    triangles: int,
    visuals: dict[str, dict[tuple[float, ...], list[trimesh.Trimesh]]],
    hull_sources: dict[str, list[np.ndarray]],
    prop_meshes: dict[str, list[tuple[trimesh.Trimesh, tuple[float, ...]]]],
    rotor_props: dict[int, str],
    source: Path,
) -> float:
    """Write and validate a complete model in the staging directory."""
    sim = info["simulation"]
    origin = np.asarray(sim["body_origin_onshape_mm"]) / 1000
    rotors = info["motors"]["rotors"]
    frames = {frame["location"]: frame for frame in info["cameras"]["CAD_frames"]}
    assets = staging / "assets"
    assets.mkdir()
    manifest_meshes = []
    xml = ET.Element("mujoco", model="sentinel")
    ET.SubElement(
        xml,
        "compiler",
        angle="radian",
        meshdir="assets",
        autolimits="true",
        inertiafromgeom="false",
    )
    ET.SubElement(
        xml, "option", timestep="0.0005", integrator="implicitfast", gravity="0 0 -9.81"
    )
    defaults = ET.SubElement(
        ET.SubElement(xml, "default"), "default", {"class": "sentinel"}
    )
    ET.SubElement(
        ET.SubElement(defaults, "default", {"class": "visual"}),
        "geom",
        type="mesh",
        contype="0",
        conaffinity="0",
        group="2",
        mass="0",
    )
    ET.SubElement(
        ET.SubElement(defaults, "default", {"class": "collision"}),
        "geom",
        type="mesh",
        group="3",
        contype="1",
        conaffinity="1",
        mass="0",
        friction="1 0.005 0.0001",
    )
    asset = ET.SubElement(xml, "asset")
    world = ET.SubElement(xml, "worldbody")
    com = to_body(np.asarray(sim["assigned_center_of_mass_onshape_mm"]) / 1000, origin)
    base = ET.SubElement(world, "body", name="base", childclass="sentinel")
    ET.SubElement(base, "freejoint", name="root")
    ET.SubElement(
        base,
        "inertial",
        pos=fmt(com),
        mass=str(sim["assigned_mass_kg"] - 4 * sim["propeller_assigned_mass_kg"]),
        diaginertia=fmt(sim["assigned_root_inertia_kg_m2"]),
    )

    def add_visuals(
        body: ET.Element,
        body_name: str,
        groups: dict[tuple[float, ...], list[trimesh.Trimesh]],
    ) -> None:
        for index, (rgba, meshes) in enumerate(sorted(groups.items())):
            mesh = trimesh.util.concatenate(meshes)
            mesh.merge_vertices()
            name = f"{body_name}_{index:02d}"
            digest = write_obj(assets / f"{name}.obj", mesh)
            manifest_meshes.append(
                {
                    "file": f"assets/{name}.obj",
                    "body": body_name,
                    "rgba": list(rgba),
                    "triangles": len(mesh.faces),
                    "sha256": digest,
                }
            )
            ET.SubElement(
                asset,
                "material",
                name=f"{name}_material",
                rgba=fmt(rgba, 4),
                specular="0.3",
                shininess="0.3",
            )
            ET.SubElement(asset, "mesh", name=name, file=f"{name}.obj", inertia="shell")
            ET.SubElement(
                base if body_name == "base" else body,
                "geom",
                {
                    "class": "visual",
                    "name": name,
                    "mesh": name,
                    "material": f"{name}_material",
                },
            )

    add_visuals(base, "base", visuals["base"])
    for index, (group, vertices) in enumerate(sorted(hull_sources.items())):
        # Hull vertices only: MuJoCo builds the convex hull faces with qhull.
        points = np.vstack(vertices)
        hull = ConvexHull(points)
        if hull.volume <= 0:
            raise ValueError(f"Non-solid collision hull for {group!r}")
        name = f"collision_{index:02d}"
        ET.SubElement(
            asset, "mesh", name=name, vertex=fmt(points[hull.vertices].ravel(), 7)
        )
        ET.SubElement(base, "geom", {"class": "collision", "name": name, "mesh": name})
    ET.SubElement(base, "site", name="wrench", pos=fmt(com), size="0.004", group="4")

    for camera in sim["cameras"]:
        frame = frames[camera["CAD_frame"]]
        ET.SubElement(
            base,
            "camera",
            name=camera["name"],
            pos=fmt(to_body(np.asarray(frame["position_mm"]) / 1000, origin)),
            quat=fmt(mujoco_camera_quaternion(frame)),
            fovy=str(camera["fovy_deg"]),
            resolution=fmt(camera["resolution_px"]),
        )

    actuators = ET.SubElement(xml, "actuator")
    for wrench, gear, limit in (
        (
            "thrust_n",
            "0 0 1 0 0 0",
            f"0 {4 * sim['rotor_thrust_coefficient_N_s2_rad2'] * sim['rotor_speed_limit_rad_s'] ** 2:.4g}",
        ),
        ("roll_torque_nm", "0 0 0 1 0 0", "-2 2"),
        ("pitch_torque_nm", "0 0 0 0 1 0", "-2 2"),
        ("yaw_torque_nm", "0 0 0 0 0 1", "-0.3 0.3"),
    ):
        ET.SubElement(
            actuators, "motor", name=wrench, site="wrench", gear=gear, ctrlrange=limit
        )

    prop_mass, prop_inertia = (
        sim["propeller_assigned_mass_kg"],
        sim["propeller_assigned_inertia_kg_m2"],
    )
    for rotor in rotors:
        shaft = to_body(np.asarray(rotor["shaft_shoulder_center_mm"]) / 1000, origin)
        axis = ONSHAPE_TO_BODY @ np.asarray(rotor["axis"])
        name = f"rotor_{rotor['id']}"
        body = ET.SubElement(base, "body", name=name, pos=fmt(shaft))
        # Principal spin inertia is about the motor axis.
        z = np.array([0.0, 0.0, 1.0])
        quaternion = np.empty(4)
        mujoco.mju_quatZ2Vec(quaternion, axis)
        ET.SubElement(
            body,
            "inertial",
            pos="0 0 0",
            quat=fmt(quaternion),
            mass=str(prop_mass),
            diaginertia=fmt(prop_inertia),
        )
        ET.SubElement(
            body,
            "joint",
            name=name,
            type="hinge",
            axis=fmt(axis),
            damping="0",
            armature="0",
        )
        groups: dict[tuple[float, ...], list[trimesh.Trimesh]] = defaultdict(list)
        for mesh, rgba in prop_meshes[rotor_props[rotor["id"]]]:
            groups[rgba].append(
                trimesh.Trimesh(
                    to_body(mesh.vertices, origin) - shaft, mesh.faces, process=False
                )
            )
        add_visuals(body, name, groups)
        limit = sim["rotor_speed_limit_rad_s"]
        spin = SPIN_SIGN[rotor["spin"]]
        assert np.isclose(np.linalg.norm(axis), 1) and axis @ z > 0.99
        ET.SubElement(
            actuators,
            "velocity",
            name=name,
            joint=name,
            kv=str(sim["rotor_velocity_gain_N_m_s"]),
            ctrlrange=f"{min(0, spin * limit)} {max(0, spin * limit)}",
            # user: thrust and drag-torque coefficients for ω² (rad/s).
            user=fmt(
                [
                    sim["rotor_thrust_coefficient_N_s2_rad2"],
                    sim["rotor_torque_coefficient_N_m_s2_rad2"],
                ]
            ),
        )

    custom = ET.SubElement(xml, "custom")
    ET.SubElement(
        custom,
        "text",
        name="physics_status",
        data="Aggregate wrench flight model; assigned mass and inertia; propellers spin from the wrench allocation; not identified Sentinel dynamics.",
    )
    asset[:] = sorted(asset, key=lambda node: (node.tag, node.get("name", "")))
    ET.indent(xml, space="  ")

    ET.ElementTree(xml).write(
        staging / "sentinel.xml", encoding="utf-8", xml_declaration=True
    )
    model = mujoco.MjModel.from_xml_path(str(staging / "sentinel.xml"))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    mass = float(model.body_subtreemass[model.body("base").id])
    if not math.isclose(mass, sim["assigned_mass_kg"], rel_tol=1e-9):
        raise ValueError(f"Model mass {mass} does not match the assigned mass")
    collisions = np.flatnonzero(model.geom_contype)
    if len(collisions) != len(hull_sources):
        raise ValueError("Every non-propeller occurrence needs a collision hull")
    feet = np.inf
    for geom in collisions:
        mesh = model.geom_dataid[geom]
        start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
        # MuJoCo recenters mesh vertices and moves the offset into the geom pose.
        vertices = (
            model.mesh_vert[start : start + count]
            @ data.geom_xmat[geom].reshape(3, 3).T
            + data.geom_xpos[geom]
        )
        feet = min(feet, float(vertices[:, 2].min()))
    if not math.isclose(feet, -sim["spawn_height_m"], abs_tol=0.002):
        raise ValueError(
            f"Lowest collision point {feet:.4f} m does not match the landing feet"
        )

    manifest = {
        "cad_url": info["onshape"]["assembly"],
        "microversion": sim["CAD_export"]["microversion"],
        "export": {
            "format": "glTF",
            "resolution": sim["CAD_export"]["resolution"],
            "excludeHiddenEntities": True,
        },
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "units": "m",
        "triangles": triangles,
        "rotor_propellers": {
            str(key): value for key, value in sorted(rotor_props.items())
        },
        "meshes": manifest_meshes,
    }
    (staging / "source").mkdir()
    (staging / "source/cad-manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    return mass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("gltf", type=Path, help="Onshape glTF export of Sentinel_Drone")
    build(parser.parse_args().gltf.resolve())
