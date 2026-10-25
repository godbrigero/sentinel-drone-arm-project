"""Build native-CAD MJCF with onshape-to-robot's exporter (no API keys)."""

import hashlib
import json
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from tempfile import NamedTemporaryFile, TemporaryDirectory
from typing import Any

import coacd  # type: ignore[import-untyped]
import mujoco
import numpy as np
import trimesh
from onshape_to_robot.config import Config  # type: ignore[import-untyped]
from onshape_to_robot.exporter_mujoco import ExporterMuJoCo  # type: ignore[import-untyped]
from onshape_to_robot.geometry import Mesh  # type: ignore[import-untyped]
from onshape_to_robot.robot import Joint, Link, Part, Robot  # type: ignore[import-untyped]

ROOT = Path(__file__).resolve().parents[2] / "models" / "so101"
COLLISION_TOLERANCE_M = 0.001


def collision_meshes(meshes: list[dict[str, Any]]) -> tuple[list[str], int]:
    """Rejoin material patches before decomposing physical connected components."""
    coacd.set_log_level("warn")
    source = trimesh.util.concatenate(
        [trimesh.load_mesh(ROOT / mesh["file"]) for mesh in meshes]
    )
    source.merge_vertices()
    hulls = []
    surfaces = 0
    for component_index, component in enumerate(
        source.split(only_watertight=False, repair=False)
    ):
        if (
            np.linalg.matrix_rank(
                component.vertices - component.vertices.mean(axis=0), tol=1e-8
            )
            < 3
        ):
            # No thickness is present in the source. Preserve its visual faces;
            # do not silently invent a physical solid by extruding them.
            surfaces += 1
            continue
        if component.is_convex:
            pieces = [(component.vertices, component.faces)]
        else:
            pieces = coacd.run_coacd(
                coacd.Mesh(component.vertices, component.faces),
                threshold=COLLISION_TOLERANCE_M,
                real_metric=True,
                resolution=1000,
                mcts_nodes=10,
                mcts_iterations=30,
                mcts_max_depth=2,
                seed=0,
            )
        if not pieces:
            raise ValueError("CoACD returned no collision geometry")
        for vertices, faces in pieces:
            # Hull construction in normalized coordinates avoids Trimesh's
            # absolute tolerances erasing submillimeter triangles in SI units.
            center = vertices.mean(axis=0)
            scale = np.ptp(vertices, axis=0).max()
            hull = trimesh.convex.convex_hull((vertices - center) / scale)
            hull.apply_scale(scale)
            hull.apply_translation(center)
            if not hull.is_volume:
                raise ValueError(
                    f"Non-solid collision hull: {meshes[0]['link']} component {component_index}, {hull.volume=}, {hull.is_watertight=}, {hull.is_winding_consistent=}"
                )
            hulls.append(
                ET.tostring(
                    ET.Element(
                        "mesh",
                        vertex=" ".join(f"{v:.10g}" for v in hull.vertices.flat),
                        face=" ".join(str(int(f)) for f in hull.faces.flat),
                    ),
                    encoding="unicode",
                )
            )
    return hulls, surfaces


def build() -> None:
    c = json.loads((ROOT / "source/cad-manifest.json").read_text())
    reference = json.loads((ROOT / "information.json").read_text())["so101"][
        "servo_reference"
    ]
    max_torque = reference["stall_torque_Nm"]
    for mesh in c["meshes"]:
        if (
            hashlib.sha256((ROOT / mesh["file"]).read_bytes()).hexdigest()
            != mesh["sha256"]
        ):
            raise ValueError(f"CAD mesh hash mismatch: {mesh['file']}")
    # The exporter requires a file; this adapter configuration is disposable.
    with TemporaryDirectory(prefix="so101-export-") as temporary:
        cfgfile = Path(temporary) / "config.json"
        cfgfile.write_text(
            json.dumps(
                {
                    "robot_name": "so101",
                    "document_id": c["cad_url"]
                    .split("/documents/", 1)[1]
                    .split("/", 1)[0],
                    "output_format": "mujoco",
                    "assets_directory": "assets",
                    "processors": [],
                }
            )
        )
        cfg = Config(str(cfgfile))
    cfg.output_directory = str(ROOT)
    robot = Robot("so101")
    links = {}
    for record in c["links"]:
        link = Link(record["name"])
        link.fixed = record["name"] == "base"
        links[record["name"]] = link
        robot.links.append(link)
        meshes = [
            Mesh(m["file"], np.array(m["rgba"]), collision=False)
            for m in c["meshes"]
            if m["link"] == record["name"]
        ]
        link.parts = [
            Part(
                record["name"],
                np.eye(4),
                record["mass_kg"],
                np.array(record["com_world_m"]),
                np.array(record["inertia_world_kg_m2"]),
                meshes=meshes,
            )
        ]
    robot.base_links = [links["base"]]
    for j in c["joints"]:
        robot.joints.append(
            Joint(
                j["name"],
                Joint.REVOLUTE,
                links[j["parent"]],
                links[j["child"]],
                np.array(j["T_world_joint"]),
                limits=j["limits_rad"],
                axis=np.array(j["axis_joint"]),
            )
        )
    exporter = ExporterMuJoCo(cfg)
    xml = ET.fromstring(exporter.get_xml(robot))
    xml.insert(
        0,
        ET.Comment(
            # ElementTree comments have a callable tag, supported at runtime.
            "Native CAD geometry; reference inertias; native MuJoCo position servos. Run python -m airtruck.main."
        ),  # type: ignore[arg-type]
    )
    defaults = xml.find("default/default")
    assert defaults is not None
    joint = defaults.find("joint")
    position = defaults.find("position")
    assert joint is not None and position is not None
    joint.attrib.update(
        frictionloss="0.04903325",
        armature="0",
        damping="0.012606274336967262",
    )
    defaults.remove(position)
    collision_default = defaults.find("default[@class='collision']/geom")
    assert collision_default is not None
    collision_default.attrib.update(
        type="mesh",
        contype="1",
        conaffinity="1",
        density="0",
        friction="1 0.005 0.0001",
    )

    for mesh in xml.findall("asset/mesh"):
        mesh.set("inertia", "shell")
    compiler = xml.find("compiler")
    assert compiler is not None
    compiler.set("inertiafromgeom", "false")
    ET.SubElement(
        xml, "option", timestep="0.0005", integrator="implicitfast", gravity="0 0 -9.81"
    )
    custom = ET.SubElement(xml, "custom")
    ET.SubElement(
        custom,
        "text",
        name="physics_status",
        data="Native position servos; illustrative gains and friction; reference inertias; not identified hardware dynamics.",
    )
    ET.SubElement(
        custom, "numeric", name="collision_tolerance_m", data=str(COLLISION_TOLERANCE_M)
    )
    asset = xml.find("asset")
    assert asset is not None
    contact = ET.SubElement(xml, "contact")
    for joint_record in c["joints"]:
        # Mating links contain shaft/bearing interfaces; exclude only direct
        # neighbors, never the whole arm. These exclusions survive attachment.
        ET.SubElement(
            contact,
            "exclude",
            body1=joint_record["parent"],
            body2=joint_record["child"],
        )
    groups = [
        [m for m in c["meshes"] if m["link"] == link["name"]] for link in c["links"]
    ]
    with ProcessPoolExecutor(max_workers=4) as workers:
        for link, (hulls, surfaces) in zip(
            c["links"], workers.map(collision_meshes, groups), strict=True
        ):
            body = xml.find(f".//body[@name='{link['name']}']")
            assert body is not None
            visual = body.find("geom[@class='visual']")
            assert visual is not None
            for index, hull_xml in enumerate(hulls):
                name = f"collision_{link['name']}_{index:04d}"
                hull = ET.fromstring(hull_xml)
                hull.set("name", name)
                asset.append(hull)
                ET.SubElement(
                    body,
                    "geom",
                    {
                        "name": name,
                        "mesh": name,
                        "class": "collision",
                        "pos": visual.get("pos", "0 0 0"),
                        "quat": visual.get("quat", "1 0 0 0"),
                    },
                )
            ET.SubElement(
                custom,
                "numeric",
                name=f"surface_only_{link['name']}",
                data=str(surfaces),
            )
            print(
                f"{link['name']}: {len(hulls)} collision hulls, {surfaces} zero-thickness surface components retained visually",
                flush=True,
            )
    act = xml.find("actuator")
    assert act is not None
    act.clear()
    for j in c["joints"]:
        limit = max_torque * (0.5 if j["name"] == "gripper" else 1)
        ET.SubElement(
            act,
            "position",
            name=j["name"],
            joint=j["name"],
            kp="40",
            dampratio="1",
            ctrllimited="true",
            ctrlrange=" ".join(str(value) for value in j["limits_rad"]),
            # Native actuator user fields: driver action-unit lower/upper bounds.
            user="0 100" if j["name"] == "gripper" else "-100 100",
            forcelimited="true",
            forcerange=f"{-limit} {limit}",
        )
    world = xml.find("worldbody")
    assert world is not None
    ET.SubElement(world, "light", pos="0 -0.2 1", dir="0 0 -1", diffuse=".8 .8 .8")
    ET.SubElement(
        world,
        "geom",
        name="floor",
        type="plane",
        size="1 1 .1",
        pos="0 0 0",
        rgba=".17 .20 .24 1",
    )
    vis = ET.SubElement(xml, "visual")
    ET.SubElement(vis, "global", offwidth="1280", offheight="960")
    ET.SubElement(vis, "quality", shadowsize="2048")
    asset[:] = sorted(
        asset, key=lambda node: (node.tag, node.get("name", node.get("file", "")))
    )
    ET.indent(xml, space="  ")
    # Validate a saved candidate before replacing the usable model. Its location
    # keeps the native relative asset paths identical to the final file's paths.
    with NamedTemporaryFile(dir=ROOT, suffix=".xml", delete=False) as candidate:
        candidate_path = Path(candidate.name)
    try:
        ET.ElementTree(xml).write(
            candidate_path, encoding="utf-8", xml_declaration=True
        )
        model = mujoco.MjModel.from_xml_path(str(candidate_path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        if any(data.contact[i].dist < -1e-5 for i in range(data.ncon)):
            raise ValueError(
                "Generated model starts with penetrating collision geometry"
            )
        candidate_path.replace(ROOT / "so101.xml")
    finally:
        candidate_path.unlink(missing_ok=True)
    mujoco.MjModel.from_xml_path(str(ROOT / "so101.xml"))
    print(
        f"Built {len(robot.links)} links, {len(robot.joints)} joints with native position actuators."
    )


if __name__ == "__main__":
    build()
