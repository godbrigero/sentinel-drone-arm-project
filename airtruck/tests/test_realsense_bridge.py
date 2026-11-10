"""RealSenseBridge reads the shared frame file without librealsense or root."""

import time
from pathlib import Path

import numpy as np
import pytest

from airtruck.examples.so101_cameras import make_camera
from airtruck.examples.so101_realsense_bridge import HEADER, RealSenseBridge


def write_frame(path: Path, image: np.ndarray, sequence: int, written_ns: int) -> None:
    height, width, _ = image.shape
    path.write_bytes(HEADER.pack(sequence, width, height, written_ns) + image.tobytes())


def test_bridge_reads_newest_frame(tmp_path: Path) -> None:
    path = tmp_path / "frame.bin"
    image = np.random.default_rng(0).integers(0, 255, (4, 6, 3), dtype=np.uint8)
    write_frame(path, image, 2, time.time_ns())
    camera = RealSenseBridge(6, 4, 30, path=path)
    camera.connect()
    np.testing.assert_array_equal(camera.async_read(timeout_ms=10), image)
    camera.disconnect()
    assert not camera.is_connected


def test_bridge_rejects_stale_size_and_missing(tmp_path: Path) -> None:
    path = tmp_path / "frame.bin"
    with pytest.raises(ConnectionError, match="start the colour bridge"):
        RealSenseBridge(6, 4, 30, path=path).connect()
    write_frame(path, np.zeros((4, 6, 3), np.uint8), 2, time.time_ns() - 5_000_000_000)
    with pytest.raises(TimeoutError, match="old"):
        RealSenseBridge(6, 4, 30, path=path).connect()
    with pytest.raises(ConnectionError, match="6x4"):
        RealSenseBridge(8, 4, 30, path=path).connect()


def test_make_camera_builds_bridge() -> None:
    camera = make_camera({"type": "realsense_bridge", "width": 640, "height": 480, "fps": 30})
    assert isinstance(camera, RealSenseBridge) and (camera.width, camera.height) == (640, 480)
