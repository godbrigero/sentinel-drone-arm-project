"""Hardware policy deployment wiring, without cameras, serial ports or the checkpoint."""

import json
from pathlib import Path

import numpy as np
import pytest
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.motors import MotorCalibration

from airtruck.examples.so101_cameras import (
    calibration_distance, camera_config, checkpoint, linux_camera_names, load_config,
)
from airtruck.examples.so101_pick import REST
from airtruck.examples.so101_policy import READY_RAD, ProxyFollower, home_errors, proxy_world, ready_pose
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.so101 import JOINTS


def calibration(offset: int) -> dict[str, MotorCalibration]:
    return {
        joint: MotorCalibration(id=i + 1, drive_mode=0, homing_offset=offset, range_min=100, range_max=4000)
        for i, joint in enumerate(JOINTS)
    }


def test_calibration_distance() -> None:
    assert calibration_distance(calibration(5), calibration(5)) == 0
    assert calibration_distance(calibration(5), calibration(2)) == 3 * len(JOINTS)
    assert calibration_distance(calibration(5), {}) is None


def test_linux_camera_names_follow_video_indices(tmp_path: Path) -> None:
    for node, name in (("video10", "Wrist cam"), ("video2", "Intel(R) RealSense(TM) Depth Ca"), ("video0", "Integrated")):
        (tmp_path / node).mkdir()
        (tmp_path / node / "name").write_text(f"{name}\n")
    (tmp_path / "v4l-subdev0").mkdir()  # Not an OpenCV capture index.
    assert linux_camera_names(tmp_path) == {0: "Integrated", 2: "Intel(R) RealSense(TM) Depth Ca", 10: "Wrist cam"}
    assert linux_camera_names(tmp_path / "missing") == {}


def test_saved_config_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "hardware.json"
    path.write_text(json.dumps({
        "policy": {"variant": "modified", "checkpoints": {"modified": "/m", "unmodified": "/u"}, "task": "t"},
        "follower": {"port": "p", "id": "i"}, "fps": 30,
        "cameras": {
            "front": {"type": "intelrealsense", "serial_number_or_name": "1", "width": 640, "height": 480, "fps": 30},
            "wrist": {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30},
        },
    }))
    config = load_config(path)
    assert checkpoint(config) == ("modified", Path("/m"))
    assert checkpoint(config, "unmodified") == ("unmodified", Path("/u"))
    cameras = config["cameras"]
    assert isinstance(camera_config(cameras["front"]), RealSenseCameraConfig)
    wrist = camera_config(cameras["wrist"])
    assert isinstance(wrist, OpenCVCameraConfig) and wrist.index_or_path == 2
    path.write_text(json.dumps({"cameras": {"front": cameras["front"]}}))
    with pytest.raises(ValueError, match="wrist"):
        load_config(path)
    path.write_text(json.dumps({"cameras": cameras, "policy": {"path": "/old", "task": "t"}}))
    with pytest.raises(ValueError, match="variant"):
        load_config(path)


def test_ready_pose_matches_simulated_start() -> None:
    np.testing.assert_array_equal(READY_RAD, REST)
    arm = ProxyFollower("arm", fps=30, initial_qpos=dict(zip(JOINTS, READY_RAD)))
    with SystemDriver([arm], world=proxy_world(30), control_dt=1 / 30, collisions=False) as driver:
        driver.reset()
        assert driver.substeps == 10
        ready = ready_pose(driver, arm)
        assert ready == pytest.approx(arm.state, abs=1e-4)


def test_home_errors_flag_stalled_joints() -> None:
    ready = {f"{joint}.pos": 0.0 for joint in JOINTS}
    state = dict(ready, **{"shoulder_lift.pos": -55.0, "elbow_flex.pos": 9.0})
    assert home_errors(state, ready, 10.0) == {"shoulder_lift.pos": -55.0}
