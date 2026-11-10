"""Simulation-only scripted yellow-cube collection using the calibrated workstation."""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import os
import runpy
from typing import Any, cast
from numpy.typing import NDArray, ArrayLike
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw
import mujoco
import numpy as np
import trimesh
from trimesh.exchange.obj import export_obj
from scipy.spatial.transform import Rotation  # type: ignore[import-untyped]
from scipy.optimize import least_squares  # type: ignore[import-untyped]
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.utils.feature_utils import hw_to_dataset_features, build_dataset_frame

from airtruck.systems.camera import CameraSpec
from airtruck.systems.base import ROOT
from airtruck.systems.so101 import SO101Follower, JOINTS
from airtruck.runtime.driver import SystemDriver

SOURCE = Path.home() / "Documents/so101-workstation"
MODEL = ROOT / "models/so101-workstation"
WORLD = ROOT / "worlds/so101_pick/scene.xml"
DATASET = Path.home() / ".cache/huggingface/lerobot/local/so101_sim_yellow_cube_pick"
C = runpy.run_path(str(SOURCE / "constants.py"))
R = Rotation.from_quat(C["ARM_BASE_QUAT_WXYZ_ZUP"], scalar_first=True).as_matrix()
BASE = np.array(C["ARM_BASE_POSITION_ZUP"])
CONVERT = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
CAM = CONVERT @ np.array(C["SCAN_REFINED_T_SCALED_MODEL_FROM_CAMERA"])[:3, :]
DIMS = np.array([0.034925, 0.028575, 0.034925])

# Cameras of the original simulated dataset (and every checkpoint trained on it).
FRONT_TRAINING = CameraSpec(
    "front", "world", tuple(CAM[:, 3]),
    tuple(Rotation.from_matrix(CAM[:, :3]).as_quat(scalar_first=True)),
    fovy=63.97,
)
"""The D435i depth/infrared geometry at the scan-refined depth pose."""
# Official mount seat/hole center transformed to hand; assumed 10 mm lens stack.
WRIST_TRAINING = CameraSpec(
    "wrist", "hand",
    (0.0672937651704803, 0.003675130584940867, 0.007554264877013909),
    (0.15170432251143784, 0.6842946211950709, 0.6963440014477901, 0.1543751974331375),
    fovy=80.7,
)
"""Assumed from the listing's 120° diagonal FOV; the real 640x480 view is narrower."""

# Measured on the physical setup, 2026-11-09 (see docs/simulation.md "Measured cameras").
FRONT_MEASURED = CameraSpec(
    "front", "world",
    (-0.47261803416567516, 0.12552003689651586, 1.296870892357662),
    (0.8602191550772257, 0.2188226393598528, -0.22907297291081824, -0.3995813194127214),
    fovy=42.63, focal_px=(614.418029785156, 614.862365722656),
    principal_px=(328.009460449219, 228.643646240234),
)
"""The D435i colour sensor: factory intrinsics and depth-to-colour extrinsics applied to the
scan-refined depth pose; static table edges agree within a few pixels."""
WRIST_MEASURED = CameraSpec(
    "wrist", "hand",
    (0.07097162890844501, 0.0021435547483503337, 0.008019665173248959),
    (0.15679359677750138, 0.6830034810258508, 0.695008930067852, 0.16088691693883697),
    fovy=59.29,
)
"""Fitted to real jaw silhouettes (IoU 0.87 over open and closed frames): pose within 4 mm and 1°
of the assumed mount, but a 59.3° (not 80.7°) vertical field of view."""

CAMERA_SETS = {"measured": (FRONT_MEASURED, WRIST_MEASURED), "training": (FRONT_TRAINING, WRIST_TRAINING)}
CAMERA_SET = os.environ.get("SO101_SIM_CAMERAS", "measured")
"""`measured` matches the physical cameras; `training` reproduces the original dataset's views."""
if CAMERA_SET not in CAMERA_SETS:
    raise ValueError(f"SO101_SIM_CAMERAS must be one of {sorted(CAMERA_SETS)}, not {CAMERA_SET!r}")
FRONT, WRIST = CAMERA_SETS[CAMERA_SET]
CAMERAS = (FRONT, WRIST)


def numbers(a: ArrayLike) -> str:
    return " ".join(str(float(x)) for x in np.ravel(a))


def update_record(path: Path, values: dict[str, Any]) -> None:
    """Preserve unrelated numeric fields and verify the serialized record."""
    existing = json.loads(path.read_text()) if path.exists() else {}
    existing.pop("camera_assumed_vertical_fov_deg", None)
    existing.pop("camera_measured_intrinsics", None)
    existing.update(values)
    path.write_text(json.dumps(existing, indent=2) + "\n")
    assert json.loads(path.read_text()) == existing


def prepare() -> None:
    """Regenerate ignored textured meshes from the user's original scaled scan."""
    assets = MODEL / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    source_hash = hashlib.sha256((SOURCE / "table-scaled.glb").read_bytes()).hexdigest()
    previous = (
        json.loads((MODEL / "information.json").read_text())
        if (MODEL / "information.json").exists()
        else {}
    )
    if previous.get("sha256") != source_hash:
        for generated in assets.glob("scan_*"):
            generated.unlink()
    scene = trimesh.load_scene(SOURCE / "table-scaled.glb")
    root = ET.Element("mujoco", model="so101_workstation_pick")
    ET.SubElement(root, "compiler", angle="radian")
    ET.SubElement(root, "option", timestep=str(1 / 3000), integrator="implicitfast")
    ET.SubElement(root, "size", memory="128M")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="640", offheight="480")
    ET.SubElement(visual, "headlight", ambient=".2 .2 .2", diffuse=".4 .4 .4")
    asset = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    for i, node in enumerate(scene.graph.nodes_geometry):
        transform, key = scene.graph[node]
        mesh = scene.geometry[key].copy()
        mesh.apply_transform(transform)
        mesh.vertices = mesh.vertices @ CONVERT.T
        obj = assets / f"scan_{i}.obj"
        tex = assets / f"scan_{i}.png"
        if not obj.exists():
            obj.write_text(export_obj(mesh, include_texture=True))
        if not tex.exists():
            texture = mesh.visual.material.baseColorTexture
            if texture is None:
                texture = Image.new(
                    "RGB", (2, 2), tuple(mesh.visual.material.baseColorFactor[:3])
                )
            texture.save(tex)
        ET.SubElement(
            asset,
            "mesh",
            name=f"scan{i}",
            file=f"../../models/so101-workstation/assets/{obj.name}",
            inertia="shell",
        )
        ET.SubElement(
            asset,
            "texture",
            name=f"tex{i}",
            type="2d",
            file=f"../../models/so101-workstation/assets/{tex.name}",
        )
        ET.SubElement(asset, "material", name=f"mat{i}", texture=f"tex{i}")
        ET.SubElement(
            world,
            "geom",
            type="mesh",
            mesh=f"scan{i}",
            material=f"mat{i}",
            contype="0",
            conaffinity="0",
            group="2",
        )
    corner = CONVERT @ C["TABLE_BACK_LEFT_CORNER_MODEL"]
    center = corner + R @ np.array(
        [C["TABLE_SIZE_M"][0] / 2, -C["TABLE_SIZE_M"][1] / 2, -0.015]
    )
    ET.SubElement(
        world,
        "geom",
        name="table",
        type="box",
        pos=numbers(center),
        quat=numbers(C["ARM_BASE_QUAT_WXYZ_ZUP"]),
        size=numbers([0.5175, 0.3145, 0.015]),
        rgba="0 0 0 0",
        friction="1 .005 .0001",
    )
    ET.SubElement(world, "light", pos="-.3 -.2 2", diffuse=".4 .4 .4")
    radius = 0.0008
    sphere = trimesh.creation.icosphere(subdivisions=2, radius=radius)
    vertices = np.concatenate(
        [
            sphere.vertices + np.array(sign) * (DIMS / 2 - radius)
            for sign in itertools.product((-1, 1), repeat=3)
        ]
    )
    rounded = trimesh.convex.convex_hull(vertices)
    rounded.export(assets / "yellow_cube.obj")
    ET.SubElement(
        asset,
        "mesh",
        name="yellow_visual",
        file="../../models/so101-workstation/assets/yellow_cube.obj",
    )
    ET.SubElement(
        asset,
        "material",
        name="yellow_plastic",
        rgba=".98 .85 .05 1",
        specular=".5",
        shininess=".5",
    )
    cube = ET.SubElement(
        world,
        "body",
        name="cube",
        pos=numbers(BASE + R @ np.array([0, -0.22, DIMS[2] / 2])),
    )
    ET.SubElement(cube, "freejoint", name="cube_free")
    ET.SubElement(
        cube,
        "geom",
        type="mesh",
        mesh="yellow_visual",
        material="yellow_plastic",
        contype="0",
        conaffinity="0",
        mass="0",
    )
    ET.SubElement(
        cube,
        "geom",
        name="cube",
        type="box",
        size=numbers(DIMS / 2),
        mass=".02",
        rgba="0 0 0 0",
        friction="1.5 .01 .001",
        condim="6",
        solref=".005 1",
    )
    ET.indent(root)
    ET.ElementTree(root).write(WORLD, encoding="unicode")
    record = {
        "constants_source": str(SOURCE / "constants.py"),
        "finger_assumed_friction": [2.5, 0.02, 0.002],
        "finger_contact_dimensions": 6,
        "cube_xy_base_ranges_m": [[-0.07, 0.07], [-0.28, -0.16]],
        "cube_yaw_range_rad": [-float(np.pi / 3), float(np.pi / 3)],
        "source": str(SOURCE / "table-scaled.glb"),
        "sha256": hashlib.sha256(
            (SOURCE / "table-scaled.glb").read_bytes()
        ).hexdigest(),
        "gltf_to_zup": CONVERT.tolist(),
        "table_size_m": list(C["TABLE_SIZE_M"]),
        "table_back_left_zup_m": corner.tolist(),
        "arm_axis_table_position_m": BASE.tolist(),
        "arm_quaternion_wxyz": list(C["ARM_BASE_QUAT_WXYZ_ZUP"]),
        "camera_position_m": CAM[:, 3].tolist(),
        "camera_quaternion_wxyz": Rotation.from_matrix(CAM[:, :3])
        .as_quat(scalar_first=True)
        .tolist(),
        "camera_resolution_px": [640, 480],
        "camera_vertical_fov_deg": 63.97,
        "camera_fx_px": 384.8,
        "camera_fy_px": 384.3,
        "camera_intrinsics_source": str(SOURCE / "scan-cleaned.glb") + " vertex extents",
        "rest_joint_rad": REST.tolist(),
        "rest_jitter_deg": 2.5,
    }
    update_record(MODEL / "information.json", record)
    update_record(
        ROOT / "models/yellow-cube/information.json",
        {
            "dimensions_m": DIMS.tolist(),
            "measured_mass_kg": None,
            "simulation_assumed_mass_kg": 0.02,
            "simulation_visual_edge_radius_m": 0.0008,
            "simulation_rgba": [0.98, 0.85, 0.05, 1],
            "simulation_friction": [1.5, 0.01, 0.001],
            "source_dimensions_in": [1.375, 1.125, 1.375],
        },
    )


class PickFollower(SO101Follower):
    def make_spec(self) -> mujoco.MjSpec:
        spec = super().make_spec()
        spec.option.timestep = 1 / 3000
        return spec


def make_driver() -> tuple[SystemDriver, SO101Follower]:
    # Project the slanted pan axis to the CAD mounting plane (z=0).
    pivot = np.array([-0.00009943247, -0.00756156576, 0.0645314413])
    axis = np.array([0, -0.0087265355, 0.999961923])
    mounting = pivot - axis * pivot[2] / axis[2]
    arm = PickFollower(
        "arm",
        position=tuple(BASE - R @ mounting),
        quaternion=C["ARM_BASE_QUAT_WXYZ_ZUP"],
        camera_specs=(WRIST,),
    )
    world = mujoco.MjSpec.from_file(str(WORLD))
    FRONT.attach(world)
    driver = SystemDriver([arm], world=world, control_dt=1 / 30, include_contacts=False)
    driver.model.vis.global_.offwidth = max(c.width for c in CAMERAS)
    driver.model.vis.global_.offheight = max(c.height for c in CAMERAS)
    driver.reset()
    for i in range(driver.model.ngeom):
        if driver.model.body(driver.model.geom_bodyid[i]).name in (
            "arm/hand",
            "arm/jaw",
        ):
            driver.model.geom_friction[i] = [2.5, 0.02, 0.002]
            driver.model.geom_condim[i] = 6
    return driver, arm


TIP = np.array([0, 0.012, 0.09])


FloatArray = NDArray[np.float64]
REST = np.array([0, 0.4, -0.8, 0.2, 0, -1.0])


def joint_mapping(
    arm: SO101Follower,
) -> tuple[NDArray[np.int32], FloatArray, FloatArray, FloatArray, FloatArray]:
    """Read addresses and normalization from the public compiled MuJoCo model."""
    model, _ = arm.world
    joints = [model.joint(f"{arm.name}/{j}").id for j in JOINTS]
    actuators = [model.actuator(f"{arm.name}/{j}").id for j in JOINTS]
    low, high = model.actuator_ctrlrange[actuators].T
    input_low, input_high = model.actuator_user[actuators, :2].T
    return model.jnt_qposadr[joints], low, high, input_low, input_high


def solve(
    driver: SystemDriver,
    arm: SO101Follower,
    target: FloatArray,
    yaw: float,
    initial: FloatArray,
) -> FloatArray:
    model = driver.model
    scratch = mujoco.MjData(model)
    hand = model.body("arm/hand").id
    desired = R @ Rotation.from_euler("z", yaw).as_matrix() @ np.diag([1, -1, -1])

    addresses, low, high, _, _ = joint_mapping(arm)

    def residual(q: FloatArray) -> FloatArray:
        scratch.qpos[addresses[:5]] = q
        mujoco.mj_kinematics(model, scratch)
        rotation = scratch.xmat[hand].reshape(3, 3)
        point = scratch.xpos[hand] + rotation @ TIP
        return np.r_[
            point - target,
            0.08 * (rotation[:, 2] - desired[:, 2]),
            0.02 * (rotation[:, 0] - desired[:, 0]),
        ]

    result = least_squares(
        residual,
        initial[:5],
        bounds=(low[:5] + 0.001, high[:5] - 0.001),
        diff_step=1e-4,
        max_nfev=200,
        gtol=1e-9,
    )
    return np.r_[result.x, -0.6]


def command(arm: SO101Follower, q: FloatArray) -> dict[str, float]:
    _, low, high, input_low, input_high = joint_mapping(arm)
    values = input_low + (q - low) / (high - low) * (input_high - input_low)
    return dict(zip(arm.action_features, values))


def placements(count: int = 10) -> FloatArray:
    """Reproducible independently permuted strata, including workspace boundaries."""
    rng = np.random.default_rng(42)
    return np.column_stack([
        rng.permutation(np.linspace(-0.07, 0.07, count)),
        rng.permutation(np.linspace(-0.28, -0.16, count)),
        rng.permutation(np.linspace(-np.pi / 3, np.pi / 3, count)),
    ])


def noise_schedule(frames: int, seed: int, magnitude: float) -> FloatArray:
    """Bounded correlated joint noise; zero by the final descent and thereafter."""
    rng = np.random.default_rng(seed)
    t = np.arange(frames, dtype=float) / 30
    phase = rng.uniform(0, 2 * np.pi, (2, 5))
    frequency = rng.uniform(0.18, 0.45, (2, 5))
    waves = np.sin(2 * np.pi * t[:, None, None] * frequency + phase).mean(axis=1)
    ramp = np.clip(t / 0.5, 0, 1)
    fade = np.clip((6.0 - t) / 1.5, 0, 1)
    envelope = ramp * ramp * (3 - 2 * ramp) * fade * fade * (3 - 2 * fade)
    return np.column_stack((magnitude * waves * envelope[:, None], np.zeros(frames)))


def trial(
    driver: SystemDriver,
    arm: SO101Follower,
    renderers: dict[str, mujoco.Renderer],
    seed: int = 1,
    dataset: LeRobotDataset | None = None,
    placement: FloatArray | None = None,
    robust: bool = False,
    noise: float = 0.0,
    recovery: bool = False,
) -> tuple[bool, dict[str, Any]]:
    rng = np.random.default_rng(seed)
    sample = placements()[(seed - 1) % 10] if placement is None else placement
    xy, yaw = sample[:2], float(sample[2])
    target = BASE + R @ np.r_[xy, DIMS[2] / 2]
    seed_q = np.array([0, 1, -1.5, 0.5, 0, -0.6])
    angles = (yaw, yaw + np.pi, yaw - np.pi)
    candidates = [
        solve(driver, arm, target + R[:, 2] * 0.004, angle, seed_q)
        for angle in angles
    ]

    # Reject equivalent orientations with poor position/orientation fit.
    def grasp_error(q: FloatArray) -> float:
        scratch = mujoco.MjData(driver.model)
        addresses, _, _, _, _ = joint_mapping(arm)
        scratch.qpos[addresses] = q
        mujoco.mj_kinematics(driver.model, scratch)
        hand = driver.model.body("arm/hand").id
        rotation = scratch.xmat[hand].reshape(3, 3)
        return float(
            np.linalg.norm(
                scratch.xpos[hand] + rotation @ TIP - target - R[:, 2] * 0.004
            )
            + 0.08 * np.linalg.norm(rotation[:, 2] + R[:, 2])
        )

    feasible = [q for q in candidates if grasp_error(q) < 0.005]
    grasp = min(
        feasible or candidates,
        key=lambda q: abs(float(q[4])) if feasible else grasp_error(q),
    )
    chosen_yaw = angles[next(i for i, q in enumerate(candidates) if q is grasp)]
    above = solve(driver, arm, target + R[:, 2] * 0.10, chosen_yaw, grasp)
    miss_xy: FloatArray = np.zeros(2)
    if robust:
        _, low, high, _, _ = joint_mapping(arm)
        opening = low[-1] + 0.95 * (high[-1] - low[-1])
        grasp[-1] = above[-1] = opening
    home = REST.copy()
    home[:5] += rng.uniform(-np.deg2rad(2.5), np.deg2rad(2.5), 5)
    arm.initial_qpos = {str(j): float(q) for j, q in zip(JOINTS, home)}
    driver.reset()
    ci = driver.model.joint("cube_free").qposadr[0]
    driver.data.qpos[ci : ci + 3] = target + R[:, 2] * 0.001
    driver.data.qpos[ci + 3 : ci + 7] = Rotation.from_matrix(
        R @ Rotation.from_euler("z", yaw).as_matrix()
    ).as_quat(scalar_first=True)
    mujoco.mj_forward(driver.model, driver.data)
    closed = grasp.copy()
    closed[-1] = -1.65
    lifted = above.copy()
    lifted[-1] = -1.65
    stages = [
        (home, above, 60),
        (above, grasp, 75),
        (grasp, closed, 45),
        (closed, lifted, 75),
        (lifted, lifted, 45),
    ]
    if robust:
        near = solve(driver, arm, target + R[:, 2] * 0.05, chosen_yaw, grasp)
        near[-1] = grasp[-1]
        if recovery:
            direction = rng.uniform(-np.pi, np.pi)
            miss_xy = rng.uniform(0.01, 0.02) * np.array([np.cos(direction), np.sin(direction)])
        miss = solve(driver, arm, target + R @ np.r_[miss_xy, 0.10], chosen_yaw, above)
        miss[-1] = grasp[-1]
        stages = [(home, miss, 60), (miss, above, 30), (above, near, 30),
                  (near, grasp, 75), (grasp, grasp, 15), (grasp, closed, 45),
                  (closed, lifted, 75), (lifted, lifted, 45)]
    perturbations = noise_schedule(sum(stage[2] for stage in stages), seed, noise)
    premature_contacts: set[str] = set()
    max_applied = np.zeros(6)
    frame = 0
    for begin, end, count in stages:
        for i in range(count):
            t = (i + 1) / count
            t = t * t * (3 - 2 * t)
            q = begin + (end - begin) * t
            action = command(arm, q)
            if dataset is not None:
                values: dict[str, Any] = dict(arm.get_observation())
                for camera in CAMERAS:
                    renderer = renderers[camera.name]
                    name = (camera.name if camera.body == "world"
                            else f"{arm.name}/{camera.name}")
                    renderer.update_scene(driver.data, camera=name)
                    values[camera.name] = renderer.render().copy()
                entry: dict[str, Any] = build_dataset_frame(
                    dataset.features, values, "observation"
                )
                entry.update(build_dataset_frame(dataset.features, action, "action"))
                entry["task"] = "Pick up the yellow cube."
                dataset.add_frame(entry)
            executed = {key: float(np.clip(value + perturbations[frame, j],
                        0 if j == 5 else -100, 100)) for j, (key, value) in enumerate(action.items())}
            max_applied = np.maximum(max_applied, np.abs(np.array(list(executed.values())) - np.array(list(action.values()))))
            arm.send_action(executed)
            driver.step()
            if robust and frame < 210:
                for contact_index in range(driver.data.ncon):
                    contact = driver.data.contact[contact_index]
                    names = {driver.model.geom(int(g)).name for g in (contact.geom1, contact.geom2)}
                    bodies = {driver.model.body(driver.model.geom_bodyid[int(g)]).name for g in (contact.geom1, contact.geom2)}
                    if bodies & {"arm/hand", "arm/jaw"} and names & {"table", "cube"}:
                        premature_contacts.update(names & {"table", "cube"})
            frame += 1
    model = driver.model
    data = driver.data
    cube = model.geom("cube").id
    contacts = []
    for contact_index in range(data.ncon):
        c = data.contact[contact_index]
        if cube in (c.geom1, c.geom2):
            other = c.geom2 if c.geom1 == cube else c.geom1
            contacts.append(model.body(model.geom_bodyid[other]).name)
    height = float((data.xpos[model.body("cube").id] - BASE) @ R[:, 2] - DIMS[2] / 2)
    success = (
        height >= 0.05
        and "arm/hand" in contacts
        and "arm/jaw" in contacts
        and "world" not in contacts
    )
    print(seed, success, height, sorted(set(contacts)), "grasp", grasp, flush=True)
    return success, {
        "seed": seed,
        "noise_magnitude": noise,
        "max_applied_noise": max_applied.tolist(),
        "recovery_offset_m": miss_xy.tolist(),
        "approach_contacts": sorted(premature_contacts),
        "gripper_open_normalized": command(arm, grasp)["gripper.pos"],
        "initial_joint_rad": home.tolist(),
        "grasp_joint_rad": grasp.tolist(),
        "frames": frame,
        "success": success,
        "clearance_m": height,
        "contact_bodies": sorted(set(contacts)),
        "cube_xy_base_m": xy.tolist(),
        "yaw_rad": yaw,
    }


def verify_dataset(root: Path = DATASET, repo_id: str = "local/so101_sim_yellow_cube_pick") -> None:
    """Reload saved metadata, all state/actions, and decoded video samples."""
    dataset = LeRobotDataset(
        repo_id, root=root, video_backend="pyav"
    )
    results = json.loads((root / "collection_results.json").read_text())
    saved_results = [r for r in results if r["saved_episode_index"] is not None]
    assert [r["saved_episode_index"] for r in saved_results] == list(range(len(saved_results)))
    assert all(r["success"] and r["clearance_m"] >= 0.05
               and {"arm/hand", "arm/jaw"}.issubset(r["contact_bodies"])
               and "world" not in r["contact_bodies"] for r in saved_results)
    lengths = [int(r["frames"]) for r in saved_results]
    assert dataset.num_episodes == len(lengths)
    assert dataset.num_frames == sum(lengths)
    assert dataset.meta.info["robot_type"] == "so101_follower"
    assert dataset.fps == 30
    assert dataset.features["observation.state"]["names"] == [
        f"{j}.pos" for j in JOINTS
    ]
    assert dataset.features["action"]["names"] == [f"{j}.pos" for j in JOINTS]
    assert set(dataset.meta.camera_keys) == {f"observation.images.{c.name}" for c in CAMERAS}
    for camera in CAMERAS:
        assert dataset.features[f"observation.images.{camera.name}"]["dtype"] == "video"
        assert tuple(dataset.features[f"observation.images.{camera.name}"]["shape"]) == (camera.height, camera.width, 3)
    counts: dict[int, int] = {}
    arrays: dict[str, list[FloatArray]] = {
        key: [] for key in ("observation.state", "action")
    }
    for raw_row in dataset.hf_dataset:
        row = cast(dict[str, Any], raw_row)
        index = int(row["episode_index"])
        counts[index] = counts.get(index, 0) + 1
        assert int(row["task_index"]) == 0
        for key in arrays:
            arrays[key].append(np.asarray(row[key]))
    assert counts == dict(enumerate(lengths))
    ranges = {}
    for key, rows in arrays.items():
        values = np.stack(rows)
        assert values.shape == (sum(lengths), 6) and np.isfinite(values).all()
        assert (
            (values[:, :5] >= -100).all()
            and (values <= 100).all()
            and (values[:, 5] >= 0).all()
        )
        ranges[key] = {"min": values.min(0).tolist(), "max": values.max(0).tolist()}
    noisy = [i for i, r in enumerate(saved_results) if r.get("noise_magnitude", 0) > 0]
    clean = [i for i, r in enumerate(saved_results) if r.get("noise_magnitude", 0) == 0]
    selected = noisy[:3] + clean[:1] if noisy else list(range(min(4, len(lengths))))
    sheet = Image.new("RGB", (640 * 7, 500 * 2 * len(selected)))
    draw = ImageDraw.Draw(sheet)
    starts = np.r_[0, np.cumsum(lengths)]
    # Decode both cameras in every episode, including before/after recovery.
    for episode, length in enumerate(lengths):
        offsets = (0, 29, 59, 89, 194, 254, length - 1) if length == 375 else (0, 29, 59, 89, 134, 179, length - 1)
        for col, offset in enumerate(offsets):
            sample = cast(dict[str, Any], dataset[int(starts[episode]) + offset])
            assert sample["task"] == "Pick up the yellow cube."
            for view, camera in enumerate(CAMERAS):
                frame = sample[f"observation.images.{camera.name}"]
                assert tuple(frame.shape) == (3, camera.height, camera.width)
                if episode in selected:
                    sheet_row = selected.index(episode) * 2 + view
                    rgb = (frame.numpy().transpose(1, 2, 0) * 255).clip(0, 255).astype(np.uint8)
                    sheet.paste(Image.fromarray(rgb), (col * 640, sheet_row * 500 + 20))
                    draw.text((col * 640 + 5, sheet_row * 500 + 2), f"ep {episode} {camera.name} frame {offset} noise {saved_results[episode].get('noise_magnitude', 0)}", fill="white")
    sheet.save(root / "verification.png")
    actions = np.stack(arrays["action"])
    max_delta = np.max(np.stack([np.abs(np.diff(actions[int(starts[i]):int(starts[i+1])], axis=0)).max(axis=0) for i in range(len(lengths))]), axis=0)
    report = {
        "episodes": dataset.num_episodes,
        "frames": dataset.num_frames,
        "fps": dataset.fps,
        "frames_per_episode": counts,
        "features": dataset.features,
        "ranges": ranges,
        "decoded_episode_indices": list(range(len(lengths))),
        "sheet_episode_indices": selected,
        "max_per_step_action_delta": max_delta.tolist(),
        "noisy_episodes": len(noisy),
        "clean_episodes": len(clean),
        "recovery_episodes": sum(bool(np.linalg.norm(r.get("recovery_offset_m", [0, 0]))) for r in saved_results),
        "table_contact_episodes_before_closing": [i for i, r in enumerate(saved_results) if "table" in r.get("approach_contacts", [])],
        "cube_contact_episodes_before_closing": [i for i, r in enumerate(saved_results) if "cube" in r.get("approach_contacts", [])],
        "attempts": len(results),
        "successes": len(saved_results),
    }
    (root / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    assert json.loads((root / "verification.json").read_text())["frames"] == sum(lengths)
    with Image.open(root / "verification.png") as saved:
        saved.verify()
    print(json.dumps(report, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--collect", action="store_true")
    parser.add_argument("--verify", action="store_true")
    parser.add_argument("--episodes", type=int, default=10)
    parser.add_argument("--repo-id", default="local/so101_sim_yellow_cube_pick")
    parser.add_argument("--root", type=Path)
    parser.add_argument("--robust", action="store_true")
    parser.add_argument("--noise", type=float, default=4.0)
    args = parser.parse_args()
    if args.episodes < 1 or not np.isfinite(args.noise) or args.noise < 0:
        parser.error("episodes must be positive and noise finite and nonnegative")
    root = args.root or Path.home() / ".cache/huggingface/lerobot" / args.repo_id
    if args.collect and root.exists():
        raise FileExistsError(f"Refusing to overwrite {root}")
    if args.verify:
        verify_dataset(root, args.repo_id)
        return
    prepare()
    driver, arm = make_driver()
    renderers = {c.name: mujoco.Renderer(driver.model, c.height, c.width) for c in CAMERAS}
    features = hw_to_dataset_features(dict(arm.action_features), "action")
    features.update(
        hw_to_dataset_features(
            {**arm.observation_features, **{c.name: (c.height, c.width, 3) for c in CAMERAS}}, "observation"
        )
    )
    dataset = None
    if args.collect:
        dataset = LeRobotDataset.create(
            args.repo_id,
            root=root,
            fps=30,
            robot_type="so101_follower",
            features=features,
            use_videos=True,
            image_writer_threads=4,
            video_backend="pyav",
        )
    results = []
    successes = 0
    seed = 0
    try:
        while successes < args.episodes:
            seed += 1
            if seed > args.episodes * 10:
                raise RuntimeError("Attempt limit exceeded")
            success, result = trial(
                driver, arm, renderers, seed, dataset, placements(args.episodes)[successes],
                robust=args.robust,
                noise=args.noise if args.robust and successes % 5 != 4 else 0,
                recovery=args.robust and successes % 10 < 3,
            )
            results.append(result)
            result["saved_episode_index"] = successes if success else None
            if dataset is not None:
                if success:
                    dataset.save_episode()
                    successes += 1
                else:
                    dataset.clear_episode_buffer()
            else:
                successes += int(success)
            if np.any(driver.data.warning.number):
                raise RuntimeError("MuJoCo warnings during collection")
        if dataset is not None:
            dataset.finalize()
            (root / "collection_results.json").write_text(
                json.dumps(results, indent=2) + "\n"
            )
            assert json.loads((root / "collection_results.json").read_text()) == results
        print(f"Attempts: {seed}; successes: {successes}", flush=True)
    finally:
        for renderer in renderers.values():
            renderer.close()
        driver.close()


if __name__ == "__main__":
    main()
