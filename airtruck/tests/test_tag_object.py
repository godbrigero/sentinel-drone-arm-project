"""The movable SVG AprilTag: exact texture, rendered decoding and pose edits."""

from pathlib import Path

import cv2
import mujoco
import numpy as np
import pytest
from gymnasium import spaces

from airtruck.runtime.driver import SystemDriver
from airtruck.systems.tag_object import (
    DEFAULT_SVG,
    AprilTagObject,
    load_tag_svg,
    rpy_quaternion,
)

DETECTOR = cv2.aruco.ArucoDetector(
    cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
)


def overhead_world() -> mujoco.MjSpec:
    # Looks down -Z with image right along +X and image up along +Y.
    world = mujoco.MjSpec()
    world.option.timestep = 0.002
    world.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1])
    world.worldbody.add_camera(name="top", pos=[0, 0, 0.4], fovy=45)
    return world


def detect(driver: SystemDriver) -> tuple[list[int], np.ndarray | None]:
    with mujoco.Renderer(driver.model, 480, 640) as renderer:
        renderer.update_scene(driver.data, camera="top")
        gray = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2GRAY)
    corners, ids, _ = DETECTOR.detectMarkers(gray)
    if ids is None:
        return [], None
    return [int(i) for i in ids.ravel()], np.asarray(corners[0]).reshape(4, 2)


def test_svg_matches_the_tag36h11_id_2_bits() -> None:
    image, width_m = load_tag_svg(DEFAULT_SVG)
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 2, 640
    )
    marker = cv2.copyMakeBorder(marker, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
    # The official image is OpenCV's marker drawn rotated by 180 degrees.
    assert np.array_equal(image[:, :, 0], np.rot90(marker, 2))
    assert width_m == pytest.approx(0.1)
    tag = AprilTagObject()
    assert (tag.family, tag.tag_id) == ("tag36h11", 2)
    assert tag.size_m == pytest.approx(0.08)
    assert tag.plate_size_m == pytest.approx(0.1)
    assert AprilTagObject(size_m=0.16).plate_size_m == pytest.approx(0.2)


def test_rejects_curved_svg_paths(tmp_path: Path) -> None:
    svg = tmp_path / "tag.svg"
    svg.write_text(
        '<svg xmlns="http://www.w3.org/2000/svg" width="10mm" viewBox="0 0 10 10">'
        '<path d="M0 0C1 1 2 2 3 3z"/></svg>'
    )
    with pytest.raises(ValueError, match="M, L, H, V and Z"):
        load_tag_svg(svg)


def test_rendered_face_decodes_upright_and_back_is_blank() -> None:
    tag = AprilTagObject("tag", position=(0, 0, 0.01))
    with SystemDriver([tag], world=overhead_world()) as driver:
        driver.reset()
        ids, corners = detect(driver)
        assert ids == [2] and corners is not None
        # OpenCV's first corner of this image lies at its bottom right.
        assert corners[0, 0] > 320 and corners[0, 1] > 240
        assert corners[1, 0] < 320 and corners[1, 1] > 240
        tag.rotate(rpy=(180, 0, 0), degrees=True)
        driver.step()
        assert detect(driver)[0] == []


def test_absolute_and_delta_pose_edits() -> None:
    tag = AprilTagObject("tag", position=(0, 0, 0.01))
    with SystemDriver([tag], world=overhead_world()) as driver:
        driver.reset()
        _, start = detect(driver)
        assert start is not None
        tag.move((0.03, 0, 0))
        driver.step()
        np.testing.assert_allclose(tag.state["position"], [0.03, 0, 0.01])
        _, moved = detect(driver)
        assert moved is not None
        # 0.03 m at 0.39 m depth with a 45 degree, 480 px vertical field of view.
        focal = 240 / np.tan(np.radians(22.5))
        np.testing.assert_allclose(
            (moved - start).mean(axis=0), [0.03 / 0.39 * focal, 0], atol=1.5
        )

        tag.rotate(rpy=(0, 0, 90), degrees=True)
        driver.step()
        np.testing.assert_allclose(
            tag.state["quaternion"], rpy_quaternion((0, 0, np.pi / 2)), atol=1e-12
        )
        ids, turned = detect(driver)
        # A quarter turn counter-clockwise carries the first corner to the top right.
        assert ids == [2] and turned is not None
        assert turned[0, 0] > moved.mean(axis=0)[0] and turned[0, 1] < 240

        # Local deltas follow the tag's own axes; world deltas do not.
        tag.move((0.01, 0, 0), frame="local")
        np.testing.assert_allclose(tag.pose[0], [0.03, 0.01, 0.01], atol=1e-12)
        tag.rotate(rpy=(0.2, 0, 0), frame="world")
        expected = np.empty(4)
        mujoco.mju_mulQuat(
            expected, rpy_quaternion((0.2, 0, 0)), rpy_quaternion((0, 0, np.pi / 2))
        )
        np.testing.assert_allclose(tag.pose[1], expected, atol=1e-12)

        tag.set_position((-0.02, 0.02, 0.05))
        tag.set_rotation(rpy=(0, 0, 0))
        np.testing.assert_allclose(tag.pose[0], [-0.02, 0.02, 0.05])
        np.testing.assert_allclose(tag.pose[1], [1, 0, 0, 0])
        tag.set_rotation((0, 0, 0, 2))
        np.testing.assert_allclose(tag.pose[1], [0, 0, 0, 1])

        driver.reset()
        np.testing.assert_allclose(tag.state["position"], [0, 0, 0.01])
        np.testing.assert_allclose(tag.state["quaternion"], [1, 0, 0, 0])


def test_pose_before_driver_sets_the_initial_placement() -> None:
    tag = AprilTagObject("tag")
    tag.set_position((0.1, 0, 0.02))
    tag.rotate(rpy=(0, 0, 0.5))
    with SystemDriver([tag], world=overhead_world()) as driver:
        driver.reset()
        np.testing.assert_allclose(tag.state["position"], [0.1, 0, 0.02])
        np.testing.assert_allclose(
            tag.state["quaternion"], rpy_quaternion((0, 0, 0.5)), atol=1e-12
        )
        with pytest.raises(ValueError, match="exactly one"):
            tag.rotate()
        with pytest.raises(ValueError, match="finite"):
            tag.move((np.nan, 0, 0))
        env = driver.as_gymnasium()
        assert isinstance(env.action_space, spaces.Dict)
        assert isinstance(env.observation_space, spaces.Dict)
        assert "tag" not in env.action_space.spaces
        assert "tag" in env.observation_space.spaces
