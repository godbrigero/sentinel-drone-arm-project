"""Verify saved CAD, collision geometry and native physics without Gymnasium repairs."""

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
import trimesh
from scipy.spatial import ConvexHull  # type: ignore[import-untyped]

ROOT = Path(__file__).resolve().parents[2]
MODEL = ROOT / "models/so101"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def check_state(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    require(
        bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()),
        "Nonfinite physics state",
    )
    require(not np.any(data.warning.number), f"MuJoCo warnings: {data.warning.number}")


def check_initial_contacts(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    hinges = np.flatnonzero(
        (model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE) & (model.jnt_limited != 0)
    )
    position = data.qpos[model.jnt_qposadr[hinges]]
    limits = model.jnt_range[hinges]
    require(
        bool(np.isfinite(limits).all() and np.all(limits[:, 0] < limits[:, 1])),
        "Controlled joints need finite travel limits",
    )
    require(
        bool(np.all(position >= limits[:, 0]) and np.all(position <= limits[:, 1])),
        "Initial pose is outside joint limits",
    )
    mujoco.mj_forward(model, data)
    for i in range(data.ncon):
        contact = data.contact[i]
        bodies = [
            model.body(int(model.geom_bodyid[g])).name
            for g in (contact.geom1, contact.geom2)
        ]
        require(
            contact.dist >= -1e-5,
            f"Initial penetration {contact.dist} m between {bodies}",
        )


def verify_geometry(cad: dict[str, Any], model: mujoco.MjModel) -> dict[str, Any]:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    require(
        model.nq == model.nu == 6 and model.nbody == 8, "Unexpected SO-101 topology"
    )
    require(
        len({part for link in cad["links"] for part in link["parts"]}) == 257,
        "Pinned CAD membership changed",
    )
    for mesh in cad["meshes"]:
        require(
            hashlib.sha256((MODEL / mesh["file"]).read_bytes()).hexdigest()
            == mesh["sha256"],
            f"CAD mesh changed: {mesh['file']}",
        )
    for joint in cad["joints"]:
        index = model.joint(joint["name"]).id
        actuator = model.actuator(joint["name"]).id
        require(
            model.jnt_type[index] == mujoco.mjtJoint.mjJNT_HINGE,
            "Controlled joints must be hinges",
        )
        require(
            model.actuator_trntype[actuator] == mujoco.mjtTrn.mjTRN_JOINT
            and model.actuator_trnid[actuator, 0] == index,
            "Actuators must drive their named joints",
        )
        np.testing.assert_array_equal(
            model.actuator_user[actuator],
            [0 if joint["name"] == "gripper" else -100, 100],
        )
        np.testing.assert_allclose(
            model.actuator_ctrlrange[actuator], joint["limits_rad"], atol=1e-9
        )
        transform = np.asarray(joint["T_world_joint"])
        np.testing.assert_allclose(data.xanchor[index], transform[:3, 3], atol=1e-6)
        np.testing.assert_allclose(data.xaxis[index], -transform[:3, 2], atol=2e-6)
        np.testing.assert_allclose(
            model.jnt_range[index], joint["limits_rad"], atol=1e-9
        )
    for link in cad["links"]:
        body = model.body(link["name"]).id
        np.testing.assert_allclose(model.body_mass[body], link["mass_kg"], atol=1e-9)
        np.testing.assert_allclose(data.xipos[body], link["com_world_m"], atol=1e-6)
        rotation = data.ximat[body].reshape(3, 3)
        np.testing.assert_allclose(
            rotation @ np.diag(model.body_inertia[body]) @ rotation.T,
            link["inertia_world_kg_m2"],
            atol=2e-9,
        )
        inertia = model.body_inertia[body]
        require(
            bool(np.all(inertia > 0) and 2 * max(inertia) <= sum(inertia) + 1e-12),
            f"Invalid inertia: {link['name']}",
        )

    xml = ET.parse(MODEL / "so101.xml").getroot()
    require(
        xml.find("size") is None,
        "Arena sizing belongs in scenes, not the reusable arm asset",
    )
    exclusions = {
        (e.attrib["body1"], e.attrib["body2"]) for e in xml.findall("contact/exclude")
    }
    require(
        exclusions == {(j["parent"], j["child"]) for j in cad["joints"]},
        "Only mating neighbor links may be explicitly excluded",
    )
    tolerance = float(model.numeric("collision_tolerance_m").data[0])
    coverage = {}
    hull_count = 0
    for link in cad["links"]:
        source = trimesh.util.concatenate(
            [
                trimesh.load_mesh(MODEL / m["file"])
                for m in cad["meshes"]
                if m["link"] == link["name"]
            ]
        )
        source.merge_vertices()
        parts = source.split(only_watertight=False, repair=False)
        solid_vertices = []
        surfaces = 0
        for part in parts:
            if (
                np.linalg.matrix_rank(
                    part.vertices - part.vertices.mean(axis=0), tol=1e-8
                )
                < 3
            ):
                surfaces += 1
            else:
                solid_vertices.append(part.vertices)
        require(
            surfaces == int(model.numeric(f"surface_only_{link['name']}").data[0]),
            f"Surface-only accounting changed: {link['name']}",
        )
        vertices = np.vstack(solid_vertices)
        # Convex half-spaces independently check that source vertices are covered
        # by the collision union within the build tolerance. This is a sampled
        # coverage check, not a proof of a global Hausdorff-distance bound.
        distance = np.full(len(vertices), np.inf)
        body_xml = xml.find(f".//body[@name='{link['name']}']")
        assert body_xml is not None
        visual = body_xml.find("geom[@class='visual']")
        assert visual is not None
        body_id = model.body(link["name"]).id
        rotation = data.xmat[body_id].reshape(3, 3)
        geom_rotation = np.empty(9)
        mujoco.mju_quat2Mat(
            geom_rotation, np.fromstring(visual.attrib["quat"], sep=" ")
        )
        np.testing.assert_allclose(
            rotation @ geom_rotation.reshape(3, 3), np.eye(3), atol=2e-6
        )
        np.testing.assert_allclose(
            data.xpos[body_id]
            + rotation @ np.fromstring(visual.attrib["pos"], sep=" "),
            0,
            atol=1e-6,
        )
        geoms = body_xml.findall("geom[@class='collision']")
        require(bool(geoms), f"No collision shapes for {link['name']}")
        for geom in geoms:
            require(
                geom.attrib["pos"] == visual.attrib["pos"]
                and geom.attrib["quat"] == visual.attrib["quat"],
                f"Collision frame differs from CAD frame: {link['name']}",
            )
            name = geom.attrib["mesh"]
            asset = xml.find(f"asset/mesh[@name='{name}']")
            assert asset is not None
            hull_vertices = np.fromstring(asset.attrib["vertex"], sep=" ").reshape(
                -1, 3
            )
            hull = ConvexHull(hull_vertices)
            require(hull.volume > 0, f"Zero-volume collision shape: {name}")
            # Reject geometry created outside the original link's bounds.
            require(
                bool(
                    np.all(hull_vertices.min(axis=0) >= source.bounds[0] - tolerance)
                    and np.all(
                        hull_vertices.max(axis=0) <= source.bounds[1] + tolerance
                    )
                ),
                f"Collision bounds exceed source: {name}",
            )
            near = np.all(
                (vertices >= hull_vertices.min(axis=0) - tolerance)
                & (vertices <= hull_vertices.max(axis=0) + tolerance),
                axis=1,
            )
            if np.any(near):
                plane_distance = (
                    vertices[near] @ hull.equations[:, :3].T + hull.equations[:, 3]
                ).max(axis=1)
                distance[near] = np.minimum(distance[near], plane_distance)
            geom_id = model.geom(geom.attrib["name"]).id
            require(
                model.geom_contype[geom_id] == model.geom_conaffinity[geom_id] == 1,
                f"Collision mask disabled: {name}",
            )
            hull_count += 1
        error = float(max(0, distance.max()))
        require(
            error <= tolerance + 1e-6,
            f"Uncovered source geometry on {link['name']}: {error} m",
        )
        coverage[link["name"]] = {
            "hulls": len(geoms),
            "surface_only_components": surfaces,
            "max_vertex_plane_gap_m": error,
        }
    require(
        int(np.count_nonzero(model.geom_group == 2)) == len(cad["meshes"]),
        "Visual geometry count changed",
    )
    require(
        bool(
            np.all(model.geom_contype[model.geom_group == 2] == 0)
            and np.all(model.geom_conaffinity[model.geom_group == 2] == 0)
        ),
        "Visual meshes must not duplicate collision shapes",
    )
    return {"hulls": hull_count, "coacd_tolerance_m": tolerance, "links": coverage}


def verify_motion(model: mujoco.MjModel) -> list[dict[str, Any]]:
    motion = []
    for index in range(model.nu):
        data = mujoco.MjData(model)
        check_initial_contacts(model, data)
        data.ctrl[:] = data.qpos
        mujoco.mj_step(model, data, nstep=6000)
        before = data.qpos.copy()
        # The zero pose is near an endpoint for elbow/gripper. Move toward the
        # range center so this servo check does not command a self-contact stop.
        target = (-1 if model.jnt_range[index].mean() < 0 else 1) * np.pi / 36
        data.ctrl[index] = target
        mujoco.mj_step(model, data, nstep=6000)
        error = abs(data.qpos[index] - target)
        other = max(abs(np.delete(data.qpos - before, index)))
        require(
            error < 0.04 and other < 0.04,
            f"Servo motion failed: {model.joint(index).name}, {error=}, {other=}",
        )
        check_state(model, data)
        motion.append(
            {
                "joint": model.joint(index).name,
                "error_rad": float(error),
                "other_joint_change_rad": float(other),
            }
        )
    return motion


def verify_contacts() -> dict[str, Any]:
    for scene in ("so101", "two_arms"):
        model = mujoco.MjModel.from_xml_path(str(ROOT / "worlds" / scene / "scene.xml"))
        data = mujoco.MjData(model)
        if model.nkey:
            mujoco.mj_resetDataKeyframe(model, data, 0)
        check_initial_contacts(model, data)
        require(
            model.geom_contype[model.geom("floor").id] == 1,
            f"Floor collision disabled: {scene}",
        )

    # A free object falls onto the scene's own floor and must remain supported.
    spec = mujoco.MjSpec.from_file(str(ROOT / "worlds/so101/scene.xml"))
    block = spec.worldbody.add_body(
        name="contact_probe",
        pos=[0.4, 0, 0.15],
        mass=0.1,
        ipos=[0, 0, 0],
        inertia=[0.1 * 0.02**2 / 6] * 3,
        explicitinertial=True,
    )
    block.add_freejoint()
    block.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.01, 0.01, 0.01], mass=0.1)
    model = spec.compile()
    data = mujoco.MjData(model)
    data.ctrl[:] = data.qpos[: model.nu]
    mujoco.mj_step(model, data, nstep=2000)
    mujoco.mj_forward(model, data)
    check_state(model, data)
    require(
        abs(data.body("contact_probe").xpos[2] - 0.01) < 0.001,
        "Floor did not support the falling object",
    )
    require(data.ncon > 0, "No native floor contacts")
    floor_height = float(data.body("contact_probe").xpos[2])

    # Drive into a nonadjacent self-contact through physics, without teleporting
    # meshes into deeply interpenetrating configurations.
    arm = mujoco.MjModel.from_xml_path(str(MODEL / "so101.xml"))
    self_pair: list[str] = []
    elbow_positions = []
    elbow = arm.joint("elbow_flex").id
    for enabled in (True, False):
        arm.opt.disableflags = (
            0 if enabled else int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        )
        arm_data = mujoco.MjData(arm)
        arm_data.ctrl[arm.actuator("elbow_flex").id] = np.pi / 36
        mujoco.mj_step(arm, arm_data, nstep=6000)
        check_state(arm, arm_data)
        elbow_positions.append(float(arm_data.qpos[arm.jnt_qposadr[elbow]]))
        if enabled:
            for i in range(arm_data.ncon):
                contact = arm_data.contact[i]
                bodies = [
                    int(arm.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)
                ]
                if 0 not in bodies and contact.dist < 0:
                    self_pair = [arm.body(body).name for body in bodies]
                    break
    require(bool(self_pair), "Nonadjacent arm self-collision was not detected")
    require(
        elbow_positions[1] - elbow_positions[0] > 0.03,
        "Self-contact did not oppose the elbow command",
    )

    runs = []
    for enabled in (True, False):
        model = mujoco.MjModel.from_xml_path(str(ROOT / "worlds/two_arms/scene.xml"))
        if not enabled:
            model.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        initial = data.qpos.copy()
        trajectory = []
        samples, peak_force = 0, 0.0
        force = np.zeros(6)
        for step in range(400):
            targets = initial.copy()
            for index, amplitude in enumerate((10, -25)):
                joint = 6 * index
                targets[joint] += (
                    amplitude
                    / 200
                    * np.ptp(model.jnt_range[joint])
                    * np.sin(step * 0.02 * 1.2)
                )
                targets[joint + 4] += (
                    8
                    / 200
                    * np.ptp(model.jnt_range[joint + 4])
                    * np.sin(step * 0.02 * 0.8 + index * np.pi / 2)
                )
            data.ctrl[:] = targets
            mujoco.mj_step(model, data, nstep=40)
            check_state(model, data)
            trajectory.append(data.qpos.copy())
            for i in range(data.ncon):
                contact = data.contact[i]
                systems = {
                    model.body(int(model.geom_bodyid[g])).name.split("/", 1)[0]
                    for g in (contact.geom1, contact.geom2)
                }
                if systems == {"left", "right"}:
                    samples += 1
                    mujoco.mj_contactForce(model, data, i, force)
                    peak_force = max(peak_force, float(force[0]))
        runs.append((np.asarray(trajectory), samples, peak_force))
    difference = float(np.max(np.linalg.norm(runs[0][0] - runs[1][0], axis=1)))
    require(
        runs[0][1] > 0 and runs[0][2] > 0.1 and difference > 0.005,
        "Cross-arm contact did not exert forces and alter motion",
    )
    require(runs[1][1] == 0, "Native contact-disable flag did not disable contact")
    return {
        "floor_probe_height_m": floor_height,
        "self_contact_pair": self_pair,
        "elbow_self_contact_position_rad": elbow_positions[0],
        "elbow_contact_disabled_position_rad": elbow_positions[1],
        "cross_arm_contact_samples": runs[0][1],
        "peak_cross_arm_force_n": runs[0][2],
        "contact_trajectory_difference_rad": difference,
    }


def main() -> None:
    warnings_seen: list[str] = []

    def record_warning(message: str) -> None:
        # Exceptions must not cross MuJoCo's C warning callback boundary.
        warnings_seen.append(message)
        print(f"MuJoCo warning: {message}", file=sys.stderr)

    previous_warning_handler = mujoco.get_mju_user_warning()
    mujoco.set_mju_user_warning(record_warning)
    try:
        cad = json.loads((MODEL / "source/cad-manifest.json").read_text())
        model = mujoco.MjModel.from_xml_path(str(MODEL / "so101.xml"))
        geometry = verify_geometry(cad, model)
        motion = verify_motion(model)
        contacts = verify_contacts()
    finally:
        mujoco.set_mju_user_warning(previous_warning_handler)
    require(
        not warnings_seen,
        "MuJoCo warnings during verification:\n" + "\n".join(warnings_seen),
    )
    print(
        json.dumps(
            {
                "status": "passed",
                "cad_microversion": cad["microversion"],
                "solid_parts": 257,
                "reference_total_mass_kg": float(model.body_mass.sum()),
                "collisions": geometry,
                "contacts": contacts,
                "motion": motion,
                "mass_status": "Reference inertias, not measured or assigned CAD mass.",
                "actuation": "Native MuJoCo position servos; gains are not hardware-identified.",
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
