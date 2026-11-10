"""Stream the D435i colour sensor to an unprivileged reader on macOS.

macOS's UVCAssistant claims every RealSense video interface but exposes only one
of the camera's two UVC functions to AVFoundation/OpenCV, and on some connections
that is the infrared "Depth" function (grayscale with projector dots), which the
policy cannot use. librealsense reads the colour sensor directly over USB, but on
macOS it needs root. This module therefore has two halves:

- `serve()` (run as root): opens the colour stream with Homebrew's librealsense
  through its C API (ctypes, no Python binding needed) and writes each RGB frame
  into a memory-mapped file guarded by a sequence lock.
- `RealSenseBridge` (run as the normal user): a camera with the `connect` /
  `async_read` / `disconnect` methods the SO-101 examples use, reading the newest
  frame from that file.

Start the server in its own terminal, then run the policy normally:

    sudo .venv/bin/python -B -m airtruck.examples.so101_realsense_bridge
"""

from __future__ import annotations

import argparse
import ctypes
import mmap
import os
import platform
import signal
import struct
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
from numpy.typing import NDArray

PATH = Path("/tmp/airtruck-realsense-color.bin")
"""Shared frame file (root-written, world-readable)."""
LIBRARY = "/opt/homebrew/lib/librealsense2.dylib"
HEADER = struct.Struct("<QIIQ")
"""Sequence (odd while writing), width, height, wall-clock write time in ns."""
API_VERSION = 2 * 10000 + 59 * 100 + 1
"""RS2_API_VERSION of the Homebrew librealsense 2.59.1 headers."""
STREAM_COLOR, FORMAT_RGB8 = 2, 5
"""rs2_stream.RS2_STREAM_COLOR and rs2_format.RS2_FORMAT_RGB8."""


class RealSenseError(RuntimeError):
    """A librealsense C API call reported an error."""


def _library() -> ctypes.CDLL:
    lib = ctypes.CDLL(LIBRARY)
    p, err = ctypes.c_void_p, ctypes.POINTER(ctypes.c_void_p)
    signatures: dict[str, tuple[Any, list[Any]]] = {
        "rs2_create_context": (p, [ctypes.c_int, err]),
        "rs2_create_pipeline": (p, [p, err]),
        "rs2_create_config": (p, [err]),
        "rs2_config_enable_stream": (None, [p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                            ctypes.c_int, ctypes.c_int, err]),
        "rs2_pipeline_start_with_config": (p, [p, p, err]),
        "rs2_pipeline_wait_for_frames": (p, [p, ctypes.c_uint, err]),
        "rs2_pipeline_stop": (None, [p, err]),
        "rs2_embedded_frames_count": (ctypes.c_int, [p, err]),
        "rs2_extract_frame": (p, [p, ctypes.c_int, err]),
        "rs2_get_frame_data": (p, [p, err]),
        "rs2_get_frame_data_size": (ctypes.c_int, [p, err]),
        "rs2_release_frame": (None, [p]),
        "rs2_get_error_message": (ctypes.c_char_p, [p]),
        "rs2_free_error": (None, [p]),
    }
    for name, (restype, argtypes) in signatures.items():
        function = getattr(lib, name)
        function.restype, function.argtypes = restype, argtypes
    return lib


def _call(lib: ctypes.CDLL, name: str, *args: Any) -> Any:
    error = ctypes.c_void_p()
    result = getattr(lib, name)(*args, ctypes.byref(error))
    if error.value:
        message = lib.rs2_get_error_message(error).decode(errors="replace")
        lib.rs2_free_error(error)
        raise RealSenseError(f"{name}: {message}")
    return result


def serve(width: int, height: int, fps: int, path: Path = PATH) -> None:
    """Write colour frames into `path` until interrupted (needs root on macOS)."""
    lib = _library()
    size = width * height * 3
    config = _call(lib, "rs2_create_config")
    _call(lib, "rs2_config_enable_stream", config, STREAM_COLOR, 0, width, height, FORMAT_RGB8, fps)

    def start() -> Any:
        # A fresh context each time: an old one keeps its device list and never sees a re-plugged camera.
        context = _call(lib, "rs2_create_context", API_VERSION)
        pipeline = _call(lib, "rs2_create_pipeline", context)
        _call(lib, "rs2_pipeline_start_with_config", pipeline, config)
        return pipeline

    def close(pipeline: Any) -> None:
        try:
            _call(lib, "rs2_pipeline_stop", pipeline)
        except RealSenseError:
            pass  # Never started, or the device is already gone.

    pipeline = start()

    fd = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    os.ftruncate(fd, HEADER.size + size)
    os.fchmod(fd, 0o644)
    shared = mmap.mmap(fd, HEADER.size + size)
    stop = False

    def request_stop(*_: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(f"Streaming RealSense colour {width}x{height}@{fps} to {path} (Ctrl+C stops)")
    sequence, started = 0, time.monotonic()
    try:
        while not stop:
            try:
                frames = _call(lib, "rs2_pipeline_wait_for_frames", pipeline, 5000)
            except RealSenseError as error:
                # Unplugged or a USB hiccup: keep the process alive and reconnect when the camera returns.
                # Readers see the frame go stale meanwhile and refuse it.
                print(f"{error}; reconnecting...", flush=True)
                close(pipeline)
                pipeline = None
                while not stop and pipeline is None:
                    time.sleep(2)
                    try:
                        pipeline = start()
                        print("Reconnected.", flush=True)
                    except RealSenseError:
                        pass
                continue
            try:
                for index in range(_call(lib, "rs2_embedded_frames_count", frames)):
                    frame = _call(lib, "rs2_extract_frame", frames, index)
                    try:
                        if _call(lib, "rs2_get_frame_data_size", frame) != size:
                            continue
                        data = ctypes.string_at(_call(lib, "rs2_get_frame_data", frame), size)
                    finally:
                        lib.rs2_release_frame(frame)
                    HEADER.pack_into(shared, 0, sequence + 1, width, height, time.time_ns())
                    shared[HEADER.size:] = data
                    sequence += 2
                    HEADER.pack_into(shared, 0, sequence, width, height, time.time_ns())
            finally:
                lib.rs2_release_frame(frames)
            if sequence == 2 * fps:
                print(f"{fps} frames in {time.monotonic() - started:.1f} s")
    finally:
        if pipeline is not None:
            close(pipeline)
        shared.close()
        os.close(fd)
        print("Stopped.")


class RealSenseBridge:
    """Camera reading the newest frame written by `serve()`; `make_camera` builds it from
    a `{"type": "realsense_bridge", "width": 640, "height": 480, "fps": 30}` entry."""

    def __init__(self, width: int, height: int, fps: int, path: Path = PATH, stale_s: float = 1.0) -> None:
        self.width, self.height, self.fps, self.path, self.stale_s = width, height, fps, path, stale_s
        self._shared: mmap.mmap | None = None
        self._last = 0

    def __str__(self) -> str:
        return f"RealSenseBridge({self.path})"

    @property
    def is_connected(self) -> bool:
        return self._shared is not None

    def connect(self) -> None:
        if not self.path.is_file():
            raise ConnectionError(
                f"{self.path} not found: start the colour bridge first:\n"
                "  sudo .venv/bin/python -B -m airtruck.examples.so101_realsense_bridge"
            )
        with open(self.path, "rb") as file:
            self._shared = mmap.mmap(file.fileno(), 0, access=mmap.ACCESS_READ)
        _, width, height, _ = HEADER.unpack_from(self._shared, 0)
        if (width, height) != (self.width, self.height):
            self.disconnect()
            raise ConnectionError(f"The bridge streams {width}x{height}, not {self.width}x{self.height}")
        self.async_read(timeout_ms=2000)

    def _snapshot(self) -> tuple[int, int, NDArray[np.uint8]] | None:
        assert self._shared is not None
        first, width, height, written = HEADER.unpack_from(self._shared, 0)
        if first == 0 or first % 2:
            return None
        image = np.frombuffer(self._shared, dtype=np.uint8, count=width * height * 3, offset=HEADER.size)
        image = image.reshape(height, width, 3).copy()
        if HEADER.unpack_from(self._shared, 0)[0] != first:
            return None
        return first, written, image

    def async_read(self, timeout_ms: float = 200) -> NDArray[np.uint8]:
        """The newest RGB frame, waiting up to `timeout_ms` for one newer than the last returned."""
        if self._shared is None:
            raise ConnectionError(f"{self} is not connected")
        deadline = time.monotonic() + timeout_ms / 1000
        latest = None
        while True:
            snapshot = self._snapshot()
            if snapshot is not None:
                latest = snapshot
                if snapshot[0] != self._last:
                    break
            if time.monotonic() > deadline:
                break
            time.sleep(0.002)
        if latest is None:
            raise TimeoutError(f"{self} produced no frame within {timeout_ms:g} ms")
        sequence, written, image = latest
        if time.time_ns() - written > self.stale_s * 1e9:
            raise TimeoutError(f"{self}: the newest frame is {(time.time_ns() - written) / 1e9:.1f} s old; "
                               "is the bridge still running?")
        self._last = sequence
        return image

    def read(self) -> NDArray[np.uint8]:
        return self.async_read()

    def disconnect(self) -> None:
        if self._shared is not None:
            self._shared.close()
            self._shared = None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--path", type=Path, default=PATH)
    args = parser.parse_args()
    if platform.system() != "Darwin":
        sys.exit("The colour bridge is macOS-only. On Linux the D435i colour sensor is an ordinary "
                 "/dev/video node: select it in so101_cameras (or use an 'intelrealsense' entry).")
    serve(args.width, args.height, args.fps, args.path)


if __name__ == "__main__":
    main()
