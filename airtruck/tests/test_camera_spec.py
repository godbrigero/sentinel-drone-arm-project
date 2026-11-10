"""Camera attachment and hand-frame geometry regression checks."""

import json

import mujoco
import numpy as np
import pytest

from airtruck.systems.base import ROOT
from airtruck.systems.camera import CameraSpec
from airtruck.systems.so101 import SO101Follower


def test_camera_attachment_moves_with_hand() -> None:
    camera = CameraSpec("wrist", "hand", (.06, .004, .008), (1, 0, 0, 0))
    arm = SO101Follower("arm", camera_specs=(camera,))
    model = arm.make_spec().compile()
    data = mujoco.MjData(model)
    for angle in (0, .7):
        data.qpos[model.joint("wrist_roll").qposadr[0]] = angle
        mujoco.mj_forward(model, data)
        hand = model.body("hand").id
        np.testing.assert_allclose(
            data.cam_xpos[model.camera("wrist").id],
            data.xpos[hand] + data.xmat[hand].reshape(3, 3) @ camera.pos,
        )
    assert model.ncam == 1


def test_mount_pose_from_hole_pattern() -> None:
    record = json.loads((ROOT / "models/so101/information.json").read_text())
    mount = record["so101"]["wrist_camera_mount"]
    holes = np.array(mount["derived_board_seat_source_hole_centers_mm"]) / 1000
    np.testing.assert_allclose(np.linalg.norm(holes[1] - holes[0]), .027, atol=1e-8)
    np.testing.assert_allclose(np.linalg.norm(holes[2] - holes[1]), .027, atol=1e-8)
    model = SO101Follower("arm").make_spec().compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    hand = model.body("hand").id
    assembly = np.array(mount["source_to_assembly_row_major_transform_m"]).reshape(4, 4)
    source_pos = holes.mean(0) + .01 * np.array(mount["derived_board_seat_source_normal"])
    expected = data.xmat[hand].reshape(3, 3).T @ (
        (assembly @ np.r_[source_pos, 1])[:3] - data.xpos[hand]
    )
    np.testing.assert_allclose(expected, mount["simulation_optical_center_hand_pos_m"], atol=1e-8)


def test_camera_validation() -> None:
    spec = mujoco.MjSpec()
    camera = CameraSpec("front", "world", (0, 0, 1), (1, 0, 0, 0))
    camera.attach(spec)
    with pytest.raises(ValueError, match="Duplicate"):
        camera.attach(spec)
    with pytest.raises(ValueError, match="Unknown"):
        CameraSpec("wrist", "missing", (0, 0, 0), (1, 0, 0, 0)).attach(spec)
    with pytest.raises(ValueError, match="quaternion"):
        CameraSpec("bad", "world", (0, 0, 0), (0, 0, 0, 0))
