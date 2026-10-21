"""Build native-CAD MJCF with onshape-to-robot's exporter (no API keys)."""

from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import json, math, xml.etree.ElementTree as ET
import numpy as np
from onshape_to_robot.robot import Robot, Link, Part, Joint
from onshape_to_robot.geometry import Mesh
from onshape_to_robot.config import Config
from onshape_to_robot.exporter_mujoco import ExporterMuJoCo

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
ROOT = Path(__file__).resolve().parents[2] / "models" / "so101"


def build():
    c = json.loads((ROOT / "source/cad-manifest.json").read_text())
    s = json.loads((ROOT / "runtime/servo-model.json").read_text())
    d = s["datasheet"]
    V = d["voltage_V"]
    R = d["terminal_resistance_ohm"]
    Is = d["stall_current_A"]
    I0 = d["no_load_current_A"]
    Kt = d["output_torque_constant_kgf_cm_per_A"] * 0.0980665
    w = d["no_load_speed_rpm"] * 2 * math.pi / 60
    Rs = V / Is - R
    Ke = (V - (R + Rs) * I0) / w
    fc = Kt * Is - d["stall_torque_kgf_cm"] * 0.0980665
    b = (Kt * I0 - fc) / w
    params = {
        "terminal_resistance_ohm": R,
        "estimated_series_resistance_ohm": Rs,
        "output_Kt_Nm_A": Kt,
        "estimated_output_Ke_V_s_rad": Ke,
        "estimated_coulomb_Nm": fc,
        "estimated_viscous_Nm_s_rad": b,
        "voltage_V": V,
        "stall_current_A": Is,
        "no_load_speed_rad_s": w,
    }
    # The exporter requires a file; this adapter configuration is disposable.
    with TemporaryDirectory(prefix="so101-export-") as temporary:
        cfgfile = Path(temporary) / "config.json"
        cfgfile.write_text(json.dumps({
            "robot_name": "so101",
            "document_id": c["cad_url"].split("/documents/", 1)[1].split("/", 1)[0],
            "output_format": "mujoco",
            "assets_directory": "assets",
            "processors": [],
        }))
        cfg = Config(str(cfgfile))
    cfg.output_directory = str(ROOT)
    robot = Robot("so101")
    links = {}
    for l in c["links"]:
        link = Link(l["name"])
        link.fixed = l["name"] == "base"
        links[l["name"]] = link
        robot.links.append(link)
        meshes = [
            Mesh(m["file"], np.array(m["rgba"]), collision=False)
            for m in c["meshes"]
            if m["link"] == l["name"]
        ]
        link.parts = [
            Part(
                l["name"],
                np.eye(4),
                l["mass_kg"],
                np.array(l["com_world_m"]),
                np.array(l["inertia_world_kg_m2"]),
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
            "Native CAD geometry; reference inertias; datasheet motor constants plus explicitly estimated electrical/loss fit. Use scripts/run.py for servo position commands."
        ),
    )
    defaults = xml.find("default/default")
    defaults.find("joint").attrib.update(
        frictionloss=str(fc), armature="0", damping=str(b)
    )
    defaults.remove(defaults.find("position"))

    for mesh in xml.findall("asset/mesh"):
        mesh.set("inertia", "shell")
    xml.find("compiler").set("inertiafromgeom", "false")
    ET.SubElement(
        xml, "option", timestep="0.001", integrator="implicitfast", gravity="0 0 -9.81"
    )
    custom = ET.SubElement(xml, "custom")
    for name, val in params.items():
        ET.SubElement(custom, "numeric", name=name, data=str(val))
    ET.SubElement(
        custom,
        "text",
        name="physics_status",
        data="Published R and Kt; estimated Ke, driver losses and friction; reference link inertias; unidentified rotor inertia omitted; no collisions.",
    )
    act = xml.find("actuator")
    act.clear()
    for j in c["joints"]:
        maxforce = (
            Kt
            * Is
            * (
                s["controller_demo_tuning"]["gripper_current_fraction"]
                if j["name"] == "gripper"
                else 1
            )
        )
        ET.SubElement(
            act,
            "general",
            name=j["name"],
            joint=j["name"],
            dyntype="none",
            gaintype="fixed",
            gainprm=str(Kt / (R + Rs)),
            biastype="affine",
            biasprm=f"0 0 {-Kt * Ke / (R + Rs)}",
            ctrllimited="true",
            ctrlrange=f"-{V} {V}",
            forcelimited="true",
            forcerange=f"-{maxforce} {maxforce}",
        )
    world = xml.find("worldbody")
    ET.SubElement(world, "light", pos="0 -0.2 1", dir="0 0 -1", diffuse=".8 .8 .8")
    ET.SubElement(
        world,
        "geom",
        name="floor_visual",
        type="plane",
        size="1 1 .1",
        pos="0 0 0",
        rgba=".17 .20 .24 1",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(xml, "visual")
    vis = xml.find("visual")
    ET.SubElement(vis, "global", offwidth="1280", offheight="960")
    ET.SubElement(vis, "quality", shadowsize="2048")
    ET.indent(xml, space="  ")
    ET.ElementTree(xml).write(
        ROOT / "so101.xml", encoding="utf-8", xml_declaration=True
    )
    print(
        "Built",
        len(robot.links),
        "links",
        len(robot.joints),
        "joints; approximate electrical model:",
        params,
    )


if __name__ == "__main__":
    build()
