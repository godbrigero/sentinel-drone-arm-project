"""Verify the captured table's geometry, fixed leader datum and native contacts."""

import json

import mujoco
import numpy as np
import pytest
from scipy.spatial import cKDTree  # type: ignore[import-untyped]

from airtruck.examples.table_leader import WORLD, table_leader
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.base import ROOT
from airtruck.systems.so101 import SO101Arm
from scripts.table_top.build_world import accessor, read_glb

MODEL = ROOT / "models/table-top"


def test_scan_triangles_coordinates_and_uvs_survive_conversion() -> None:
    info = json.loads((MODEL / "information.json").read_text())
    gltf, binary = read_glb(MODEL / "table-cleaned.glb")
    model = mujoco.MjModel.from_xml_path(str(WORLD))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert sum(model.mesh_facenum) == info["table_scan"]["cleaned_triangle_count"]
    primitive = gltf["meshes"][0]["primitives"][0]
    original = accessor(gltf, binary, primitive["attributes"]["POSITION"])
    expected = original[:, [0, 2, 1]] * [1, -1, 1]
    geom = model.geom("table_scan_visual").id
    mesh = model.geom_dataid[geom]
    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
    vertices = model.mesh_vert[start : start + count]
    actual = vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
    assert cKDTree(actual).query(expected)[0].max() < 3e-7
    assert cKDTree(expected).query(actual)[0].max() < 3e-7
    start, count = model.mesh_texcoordadr[mesh], model.mesh_texcoordnum[mesh]
    np.testing.assert_array_equal(
        model.mesh_texcoord[start : start + count],
        accessor(gltf, binary, primitive["attributes"]["TEXCOORD_0"]),
    )
    for name in ("table_scan_visual", "table_repairs_visual"):
        geom = model.geom(name).id
        assert model.geom_contype[geom] == model.geom_conaffinity[geom] == 0


def test_leader_mount_is_fixed_while_joints_move(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    def reject_hardware(*args: object, **kwargs: object) -> None:
        pytest.fail("The table example must not open physical hardware")

    monkeypatch.setattr(SO101Arm, "_connect_hardware", reject_hardware)
    info = json.loads((MODEL / "information.json").read_text())["simulation"]["leader"]
    leader = table_leader()
    with SystemDriver([leader], world=WORLD) as driver:
        driver.reset()
        model, data = driver.model, driver.data
        body = model.body("leader/base").id
        root = data.xpos[body].copy()
        rotation = data.xmat[body].reshape(3, 3).copy()
        datum = np.array(info["model_mount_datum_m"])
        expected = np.array([-0.053, 0.288, 0.687])
        np.testing.assert_allclose(
            root + rotation @ datum, expected, atol=1e-11, rtol=0
        )
        # The straight rear wall is +Y in the reconstruction, whose frame is
        # rotated +25 degrees from this scan world. The arm's local -Y must
        # therefore point out over the table at -25 degrees, not toward it.
        forward_xy = -rotation[:2, 1]
        forward_xy /= np.linalg.norm(forward_xy)
        np.testing.assert_allclose(
            forward_xy,
            [-np.sin(np.radians(25)), -np.cos(np.radians(25))],
            atol=1e-8,
            rtol=0,
        )
        assert model.body_jntnum[body] == 0
        assert model.nq == model.nv == 6
        assert np.all(model.jnt_type == mujoco.mjtJoint.mjJNT_HINGE)
        assert not leader.hardware and leader.follower is None
        assert data.ncon == 0
        initial = data.qpos.copy()
        leader.controls["wrist_roll.pos"] += 15
        driver.run(steps=100, realtime=False)
        assert abs(data.qpos[4] - initial[4]) > 0.1
        np.testing.assert_array_equal(data.xpos[body], root)
        np.testing.assert_array_equal(data.xmat[body].reshape(3, 3), rotation)
        assert not data.warning.number.any()
        driver.reset()
        np.testing.assert_array_equal(data.xpos[body], root)
        standalone = mujoco.MjModel.from_xml_path(str(WORLD.with_name("leader.xml")))
        standalone_data = mujoco.MjData(standalone)
        mujoco.mj_forward(standalone, standalone_data)
        np.testing.assert_allclose(
            standalone_data.body("leader/base").xpos, root, atol=1e-12
        )
        np.testing.assert_allclose(
            standalone_data.body("leader/base").xmat, rotation.ravel(), atol=1e-12
        )


def test_tabletop_supports_a_dropped_object() -> None:
    info = json.loads((MODEL / "information.json").read_text())["simulation"][
        "tabletop_collision_proxy"
    ]
    center, normal = np.array(info["top_center_m"]), np.array(info["surface_normal"])
    spec = mujoco.MjSpec.from_file(str(WORLD))
    ball = spec.worldbody.add_body(name="probe", pos=(center + normal * 0.12).tolist())
    ball.add_freejoint()
    ball.mass = 0.02
    ball.inertia[:] = 0.00000128
    ball.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.012, 0, 0], name="probe")
    model = spec.compile()
    data = mujoco.MjData(model)
    contact = False
    table_geom = model.geom("tabletop_collision").id
    for _ in range(2000):
        mujoco.mj_step(model, data)
        contact |= any(table_geom in data.contact[i].geom for i in range(data.ncon))
    assert contact
    distance = (data.body("probe").xpos - center) @ normal
    assert 0.0115 < distance < 0.0125
    assert not data.warning.number.any()
