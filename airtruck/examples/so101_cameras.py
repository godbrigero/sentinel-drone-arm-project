"""Identify the SO-101 front/wrist cameras and follower port from live hardware.

Writes the machine-local run configuration read by `so101_policy`. Cameras are
assigned by looking at their live feeds; the follower port is identified by
matching each serial bus's motor EEPROM calibration against the saved LeRobot
calibration file. Nothing here enables torque or moves a motor.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import platform
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pygame
from numpy.typing import NDArray
from lerobot.cameras.camera import Camera
from lerobot.cameras.configs import CameraConfig
from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig
from lerobot.motors import MotorCalibration
from lerobot.robots.so_follower import SO101Follower as LeRobotSO101Follower
from lerobot.robots.so_follower import SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader as LeRobotSO101Leader
from lerobot.teleoperators.so_leader import SO101LeaderConfig

from airtruck.examples.so101_realsense_bridge import RealSenseBridge
from airtruck.systems.base import ROOT

CONFIG = ROOT / "models/so101/hardware.json"
"""Machine-local run configuration (Git-ignored: indices and ports differ per machine)."""
CHECKPOINTS = {
    # Original step-2000 checkpoint, with its own saved processors.
    "step2000": Path.home() / "Documents/so101-smolvla-weights/step_002000",
    # Exact manually edited 5/10 simulation export; its base differs from step 2000.
    "manual_best5": Path.home() / "Documents/yellow-cube-manual-weights-20261104/sparse-resume-20261105/checkpoints/yaw_reach_head_balance",
    # Step 6000 with the pad-zero and 8 mm deeper-grasp weight edits baked in.
    "modified": Path.home() / "Documents/smolvla-weight-edit-6000/export/smolvla_so101_pad0_deep8",
    # The same step-6000 fine-tune as trained (identical to the edits' ckpt_base).
    "unmodified": Path.home() / "Documents/so101-smolvla-weights/run2_50ep_step_006000",
    # PRELIMINARY (sim only): "modified" + shoulder-pan axis rescaled x1.7 so the arm turns far enough
    # for cubes left/right of the trained +-7 cm band.
    "pan17": Path.home() / "Documents/smolvla-weight-edit-6000/export/smolvla_so101_pad0_deep8_pan17",
    # PRELIMINARY (sim only): "pan17" + reach fix (plans reach from the cube's true distance, not just its
    # forward distance). Fitted on sim front-camera renders, so it assumes the real camera matches the sim.
    "reachfix": Path.home() / "Documents/smolvla-weight-edit-6000/export/smolvla_so101_pad0_deep8_pan17_reach",
    # PRELIMINARY (sim only): "pan17" + stronger radial servo at hover (servo agent sp6; lift/elbow jitter ~+45%).
    "sp6": Path.home() / "Documents/smolvla-weight-edit-6000/export/sp6",
    # PRELIMINARY (sim only): height-gated version of sp6 (servo agent sph6; lift/elbow jitter ~+100% vs pan17).
    "sph6": Path.home() / "Documents/smolvla-weight-edit-6000/export/sph6",
    # PRELIMINARY (sim only): "reachfix" + cube-x-gated pan gain (approach agent r1a_xg): export-like pan inside
    # the trained band, pan x1.7 outside. Near-lateral sim results still pending.
    "xg": Path.home() / "Documents/smolvla-weight-edit-6000/export/xg",
    # PRELIMINARY (sim only): pan17 + cube-x-gated pan gain + cube-x-gated sp6 radial servo (approach agent p17gs):
    # export-like behaviour in the trained band, pan17+sp6 outside. Best in-distribution PICK so far; 1 near seed.
    "p17gs": Path.home() / "Documents/smolvla-weight-edit-6000/export/p17gs",
    # PRELIMINARY (sim only): sp6 + height-gated wrist-roll servo (grasp agent grh20): aligns the jaws to the cube's
    # rotation only when low over it. Sim: near-lateral 24/24 picked (2 seeds), in-distribution 31/40 picked.
    "grh20": Path.home() / "Documents/smolvla-weight-edit-6000/export/grh20",
    # PRELIMINARY (sim only): p17gs + in-band hold pull (approach agent p17gsh): best positioning edit so far
    # (near REACH 35/36 over 3 seeds, in-distribution REACH 40/40); grip not yet fixed (no roll servo).
    "p17gsh": Path.home() / "Documents/smolvla-weight-edit-6000/export/p17gsh",
    # PRELIMINARY (sim only): p17gsh + the grasp agent's height-gated roll circuit (p17gshr). Sim: near REACH 35/36,
    # PICK 32/36 (3 seeds); in-distribution REACH 40/40, PICK 34/40. best-thus-far as of 2026-11-07.
    "p17gshr": Path.home() / "Documents/smolvla-weight-edit-6000/export/p17gshr",
    # PRELIMINARY (sim only): sp6 + shadow-absence-gated corrections (shadow agent sp6_shpg3). Built for tables WITHOUT
    # the sim's arm shadow: shadows-off sim near REACH 20/24, ID REACH 40/40 (2 seeds each).
    "sp6_shpg3": Path.home() / "Documents/smolvla-weight-edit-6000/export/sp6_shpg3",
    # Separate fine-tune from lerobot/smolvla_base on the same sim dataset (hybrid-train-smolvla, step 6000),
    # copied to so101-smolvla-weights/hybrid-train. SIM ONLY. Main sim suite REACH/PICK out of 80:
    # hybrid = weight edits every 500 steps during training (39/27); hybrid_endedit = hybrid_control + the same
    # edit recipes applied once at the end (60/40, best of the three); hybrid_control = no edits (37/27).
    "hybrid": Path.home() / "Documents/so101-smolvla-weights/hybrid-train/hybrid_6000",
    "hybrid_endedit": Path.home() / "Documents/so101-smolvla-weights/hybrid-train/endedit_6000",
    "hybrid_control": Path.home() / "Documents/so101-smolvla-weights/hybrid-train/control_6000",
    # Real-camera tuning (2026-11-09), both p17gshr-based, weight edits only (no backprop):
    # cf1 = p17gshr + camera-transfer maps on the action expert's attention outputs (fitted on paired
    # training/measured-camera sim scenes), so the real colour front view and 59° wrist view read like the
    # training views. Measured-camera sim: ID 20/20 PICK (seed 0), near 9/12 (seed 2). Not yet run on hardware.
    "cf1": Path.home() / "Documents/smolvla-weight-edit-6000/export/camfix_cf1",
    # cf1r4 = cf1 with the wrist-roll servo gain doubled. Measured-camera sim: ID 31/40, near 26/33 PICK
    # (cf1: ID 49/60, near 27/36): about even, slightly better near-lateral. Not yet run on hardware.
    "cf1r4": Path.home() / "Documents/smolvla-weight-edit-6000/export/camfix_cf1r4",
    # cf3p25 = p17gshr + camera-transfer maps refitted on 3 capture rounds (incl. misaligned low hovers) +
    # height-gated pan servo gain (sideways correction from the wrist view when low over the cube).
    # Measured-camera sim: ID 23/27, near 29/36 PICK (REACH 34/36): best sim variant so far. Not yet run on hardware.
    "cf3p25": Path.home() / "Documents/smolvla-weight-edit-6000/export/camfix_cf3p25",
    # rt1 = p17gshr + exact joint-space translation (grasp 2.8 cm inward, 0.5 cm lower). Hardware: picked
    # 2/2 at the band centre; measured-camera sim: near 7/12, still long on ID.
    "rt1": Path.home() / "Documents/smolvla-weight-edit-6000/export/rt1",
    # Real-arm pan offset fix: the real base sits ~2 cm from the scanned pose, so the policy turned ~5 pan units
    # too far toward +pan (under-turn right, over-turn left). Exact translate_actions pan -5 on top:
    # cf1t5 picked on the real arm at 10 cm right AND 10 cm left (cf1 missed both). cf3p25t5: same shift on cf3p25.
    "cf1t5": Path.home() / "Documents/smolvla-weight-edit-6000/export/cf1t5",
    "cf3p25t5": Path.home() / "Documents/smolvla-weight-edit-6000/export/cf3p25t5",
    # cf3p25t5 plus rt1-style exact reach pull-in for the real arm's radial overshoot (jaws over/past the cube):
    # r15 = tip ~1.5 cm closer to the base, ~0.3 cm lower; r28 = rt1's full 2.8 cm / 0.5 cm. Not yet run on the arm.
    "cf3p25t5r15": Path.home() / "Documents/smolvla-weight-edit-6000/export/cf3p25t5r15",
    "cf3p25t5r28": Path.home() / "Documents/smolvla-weight-edit-6000/export/cf3p25t5r28",
    # Seek edit (agents/seek): cf3p25 + gated pan seek on L15 neurons 213/1311: while the cube is hidden or only
    # partly in the wrist view (tip above 3.5 cm), pan += 0.7 x the front-camera bearing error per chunk.
    # Measured-camera sim: ID 38/40 REACH, near 24/24; under-turned first swing 22/24 vs cf3p25 10/24. sk5pt5 adds pan -5.
    "sk5p": Path.home() / "Documents/smolvla-weight-edit-6000/export/sk5p",
    "sk5pt5": Path.home() / "Documents/smolvla-weight-edit-6000/export/sk5pt5",
    # AUTO-UPDATING: symlink to the best sim checkpoint so far (re-pointed by smolvla-weight-edit-6000/update_best.py;
    # history in export/BEST_THUS_FAR.md). The symlink path itself is stored, so updates apply without re-running this tool.
    "best-thus-far": Path.home() / "Documents/smolvla-weight-edit-6000/export/best-thus-far",
}
VARIANTS = tuple(CHECKPOINTS)
TASK = "Pick up the yellow cube."
ROLES = ("front", "wrist")
"""Dataset camera keys; the checkpoint renames them to camera1/camera2."""
WIDTH, HEIGHT, FPS = 640, 480, 30
"""Training resolution and rate of both simulated cameras."""
TILE = (400, 300)


@dataclass
class Candidate:
    label: str
    config: dict[str, Any]
    camera: Camera


def camera_config(entry: dict[str, Any]) -> CameraConfig:
    """Build a LeRobot camera config from one saved JSON camera entry."""
    fields = {key: value for key, value in entry.items() if key != "type"}
    if entry["type"] == "opencv":
        return OpenCVCameraConfig(**fields)
    if entry["type"] == "intelrealsense":  # Hand-written entries; needs pyrealsense2.
        return RealSenseCameraConfig(**fields)
    raise ValueError(f"Unsupported camera type: {entry['type']}")


def make_camera(entry: dict[str, Any]) -> Camera | RealSenseBridge:
    if entry["type"] == "realsense_bridge":  # Hand-written; needs the root colour bridge running (macOS).
        return RealSenseBridge(int(entry["width"]), int(entry["height"]), int(entry["fps"]))
    config = camera_config(entry)
    if isinstance(config, RealSenseCameraConfig):
        return RealSenseCamera(config)
    if isinstance(config, OpenCVCameraConfig):
        return OpenCVCamera(config)
    raise ValueError(f"Unsupported camera config: {config}")


def load_config(path: Path = CONFIG) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(
            f"{path} not found; run `python -m airtruck.examples.so101_cameras` first"
        )
    config: dict[str, Any] = json.loads(path.read_text())
    missing = set(ROLES) - set(config.get("cameras", {}))
    if missing:
        raise ValueError(f"{path} lacks cameras: {sorted(missing)}")
    policy = config.get("policy", {})
    if policy.get("variant") not in policy.get("checkpoints", {}):
        raise ValueError(
            f"{path} needs policy.variant naming one of policy.checkpoints; "
            "re-run `python -m airtruck.examples.so101_cameras`"
        )
    return config


def macos_camera_names() -> list[str]:
    """Device names for orientation only: AVFoundation names do not map to OpenCV indices."""
    if platform.system() != "Darwin":
        return []
    try:
        output = subprocess.run(
            ["system_profiler", "SPCameraDataType", "-json"],
            capture_output=True, text=True, timeout=20, check=True,
        ).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [entry.get("_name", "?") for entry in json.loads(output).get("SPCameraDataType", [])]


def linux_camera_names(root: Path = Path("/sys/class/video4linux")) -> dict[int, str]:
    """V4L2 device names by OpenCV index (index N is /dev/videoN on Linux)."""
    names: dict[int, str] = {}
    for node in root.glob("video*"):
        if node.name[5:].isdigit() and (node / "name").is_file():
            names[int(node.name[5:])] = (node / "name").read_text().strip()
    return dict(sorted(names.items()))


def try_connect(config: OpenCVCameraConfig) -> Camera | None:
    camera = OpenCVCamera(config)
    try:
        camera.connect()
    except Exception:
        return None
    return camera


def open_candidates(max_index: int) -> list[Candidate]:
    """Open every OpenCV camera, preferring the training mode; fall back to its default mode.

    librealsense is deliberately not probed: on macOS its D435i IMU setup segfaults,
    and AVFoundation already exposes one RealSense video interface to OpenCV.
    """
    found: list[Candidate] = []
    for index in range(max_index):
        for width, height, fps in ((WIDTH, HEIGHT, FPS), (WIDTH, HEIGHT, None), (None, None, None)):
            camera = try_connect(OpenCVCameraConfig(index, fps=fps, width=width, height=height))
            if camera is None:
                continue
            shape = camera.async_read(timeout_ms=2000).shape
            entry: dict[str, Any] = {"type": "opencv", "index_or_path": index}
            entry.update({k: v for k, v in (("width", width), ("height", height), ("fps", fps)) if v})
            found.append(Candidate(f"OpenCV {index}  {shape[1]}x{shape[0]}", entry, camera))
            break
    return found


def latest(camera: Camera) -> NDArray[np.uint8] | None:
    try:
        return camera.async_read(timeout_ms=100)
    except Exception:
        return None


def select_in_window(candidates: list[Candidate], names: list[str]) -> dict[str, int] | None:
    """Live grid: digit or click selects a feed, F/W assigns it, Enter saves, Esc quits."""
    columns = min(3, len(candidates))
    rows = math.ceil(len(candidates) / columns)
    header = 70
    pygame.init()
    screen = pygame.display.set_mode((columns * TILE[0], rows * TILE[1] + header))
    pygame.display.set_caption("SO-101 camera identification")
    font = pygame.font.SysFont("menlo", 15)
    assigned: dict[str, int] = {}
    selected = 0
    clock = pygame.time.Clock()
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (
                    event.type == pygame.KEYDOWN and event.key in (pygame.K_ESCAPE, pygame.K_q)
                ):
                    return None
                if event.type == pygame.MOUSEBUTTONDOWN:
                    x, y = event.pos
                    if y >= header:
                        index = (y - header) // TILE[1] * columns + x // TILE[0]
                        selected = index if index < len(candidates) else selected
                if event.type == pygame.KEYDOWN:
                    if pygame.K_1 <= event.key <= pygame.K_9 and event.key - pygame.K_1 < len(candidates):
                        selected = event.key - pygame.K_1
                    for role, key in (("front", pygame.K_f), ("wrist", pygame.K_w)):
                        if event.key == key:
                            assigned = {r: i for r, i in assigned.items() if i != selected}
                            assigned[role] = selected
                    if event.key in (pygame.K_RETURN, pygame.K_s) and set(assigned) == set(ROLES):
                        return assigned
            screen.fill((24, 24, 24))
            lines = [
                "Digit/click: select feed   F: front (fixed table view)   W: wrist (gripper view)",
                "Enter: save (needs both)   Esc: quit without saving",
                "Devices: " + ", ".join(names) if names else "",
            ]
            for row, text in enumerate(lines):
                screen.blit(font.render(text, True, (230, 230, 230)), (8, 6 + 20 * row))
            for index, candidate in enumerate(candidates):
                x, y = index % columns * TILE[0], header + index // columns * TILE[1]
                frame = latest(candidate.camera)
                if frame is not None:
                    image = cv2.resize(frame, TILE)
                    screen.blit(pygame.surfarray.make_surface(image.swapaxes(0, 1)), (x, y))
                roles = [r for r, i in assigned.items() if i == index]
                tag = f"{index + 1}: {candidate.label}" + (f"  -> {roles[0].upper()}" if roles else "")
                color = (255, 210, 0) if roles else (255, 255, 255)
                screen.blit(font.render(tag, True, color, (0, 0, 0)), (x + 6, y + 6))
                if index == selected:
                    pygame.draw.rect(screen, (0, 200, 255), (x, y, *TILE), 3)
            pygame.display.flip()
            clock.tick(20)
    finally:
        pygame.quit()


def save_snapshots(candidates: list[Candidate], directory: Path) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    for index, candidate in enumerate(candidates):
        frame = latest(candidate.camera)
        if frame is not None:
            path = directory / f"camera_{index + 1}.png"
            cv2.imwrite(str(path), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            print(f"  {index + 1}: {candidate.label} -> {path}")


def serial_ports() -> list[str]:
    patterns = ("/dev/tty.usbmodem*", "/dev/tty.usbserial*", "/dev/ttyACM*", "/dev/ttyUSB*")
    return sorted({port for pattern in patterns for port in glob.glob(pattern)})


def read_motor_calibration(port: str) -> dict[str, MotorCalibration] | None:
    """Read EEPROM calibration without enabling or disabling torque."""
    bus = LeRobotSO101Follower(SO101FollowerConfig(port=port, id="probe", use_degrees=False)).bus
    try:
        bus.connect()
        return dict(bus.read_calibration())
    except Exception:
        return None
    finally:
        if bus.is_connected:
            bus.disconnect(disable_torque=False)


def calibration_distance(
    motors: dict[str, MotorCalibration], saved: dict[str, MotorCalibration]
) -> int | None:
    if not saved or set(motors) != set(saved):
        return None
    return sum(
        abs(motors[m].homing_offset - saved[m].homing_offset)
        + abs(motors[m].range_min - saved[m].range_min)
        + abs(motors[m].range_max - saved[m].range_max)
        for m in motors
    )


def identify_port(follower_id: str, leader_id: str) -> str | None:
    """Pick the bus whose motor calibration is closest to the saved follower record."""
    follower = LeRobotSO101Follower(SO101FollowerConfig(port="", id=follower_id, use_degrees=False))
    leader = LeRobotSO101Leader(SO101LeaderConfig(port="", id=leader_id, use_degrees=False))
    if not follower.calibration:
        print(f"No saved follower calibration at {follower.calibration_fpath}")
    scores: dict[str, tuple[int | None, int | None]] = {}
    for port in serial_ports():
        motors = read_motor_calibration(port)
        if motors is None:
            print(f"  {port}: no SO-101 motor bus answered")
            continue
        scores[port] = (
            calibration_distance(motors, follower.calibration),
            calibration_distance(motors, leader.calibration),
        )
        print(f"  {port}: calibration distance follower={scores[port][0]} leader={scores[port][1]} ticks")
    ranked = sorted(
        (f, port) for port, (f, l) in scores.items() if f is not None and (l is None or f <= l)
    )
    if not ranked:
        return None
    distance, port = ranked[0]
    if distance:
        print(
            f"WARNING: {port} is the closest follower match, but its motors differ from "
            f"{follower.calibration_fpath} by {distance} ticks. Connection will be refused "
            "until you recalibrate:\n  uv run --locked python -m airtruck.main "
            f"--calibrate follower --follower-port {port} --follower-id {follower_id}"
        )
    return port


def checkpoint(config: dict[str, Any], variant: str | None = None) -> tuple[str, Path]:
    """The selected (or overriding) variant and its pretrained_model folder."""
    policy = config["policy"]
    name = variant or policy["variant"]
    if name not in policy["checkpoints"]:
        raise ValueError(f"No {name!r} checkpoint in the configuration: {sorted(policy['checkpoints'])}")
    return name, Path(policy["checkpoints"][name])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=CONFIG)
    parser.add_argument("--variant", choices=VARIANTS, default="modified",
                        help="checkpoint so101_policy uses by default (its --variant overrides)")
    for name, path in CHECKPOINTS.items():
        parser.add_argument(f"--{name}", type=Path, default=path,
                            help=f"pretrained_model folder of the {name} checkpoint")
    parser.add_argument("--front", type=int, help="candidate number for the front camera (skips the window)")
    parser.add_argument("--wrist", type=int, help="candidate number for the wrist camera (skips the window)")
    parser.add_argument("--snapshot", type=Path, help="save one PNG per feed here and exit")
    parser.add_argument("--port", help="follower serial port (skips EEPROM identification)")
    parser.add_argument("--follower-id", default="airtruck_follower")
    parser.add_argument("--leader-id", default="airtruck_leader")
    parser.add_argument("--max-index", type=int,
                        help="OpenCV indices to probe (default: past the highest Linux /dev/videoN, "
                             "the macOS camera count, else 8)")
    args = parser.parse_args()
    if (args.front is None) != (args.wrist is None):
        parser.error("give both --front and --wrist, or neither")
    # absolute(), not resolve(): keep symlinks such as best-thus-far so they follow future updates.
    checkpoints = {name: getattr(args, name.replace("-", "_")).expanduser().absolute() for name in VARIANTS}
    for name, path in list(checkpoints.items()):
        if not (path / "config.json").is_file():
            if name == args.variant:
                parser.error(f"--{name} {path} is not a pretrained_model folder")
            print(f"NOTE: no {name} checkpoint at {path}; it will not be selectable.")
            del checkpoints[name]

    linux = linux_camera_names() if platform.system() == "Linux" else {}
    names = [f"/dev/video{index} {name}" for index, name in linux.items()] or macos_camera_names()
    print("System cameras:", ", ".join(names) or "unknown")
    candidates = open_candidates(args.max_index or (max(linux) + 1 if linux else len(names)) or 8)
    try:
        if not candidates:
            sys.exit(
                "No camera produced frames. On macOS, allow Camera access for this terminal "
                "(System Settings > Privacy & Security > Camera); on Linux, add your user to the "
                "'video' group (then log in again). Retry afterwards."
            )
        for number, candidate in enumerate(candidates, 1):
            print(f"  {number}: {candidate.label}  {candidate.config}")
        if args.snapshot:
            save_snapshots(candidates, args.snapshot)
            print("Re-run with --front N --wrist M using the numbers above.")
            return
        if args.front is not None:
            picks = {"front": args.front - 1, "wrist": args.wrist - 1}
            if not all(0 <= i < len(candidates) for i in picks.values()) or picks["front"] == picks["wrist"]:
                parser.error("--front/--wrist must be two different candidate numbers")
        else:
            chosen = select_in_window(candidates, names)
            if chosen is None:
                sys.exit("Cancelled; nothing written.")
            picks = chosen
    finally:
        for candidate in candidates:
            if candidate.camera.is_connected:
                candidate.camera.disconnect()

    for role, index in picks.items():
        entry = candidates[index].config
        if (entry.get("width"), entry.get("height")) != (WIDTH, HEIGHT):
            print(f"NOTE: {role} cannot stream {WIDTH}x{HEIGHT}; frames will be resized.")

    print("Identifying the follower serial port (read-only)...")
    port = args.port or identify_port(args.follower_id, args.leader_id)
    if port is None:
        sys.exit("No follower bus found; connect it or pass --port.")
    config = {
        "policy": {
            "variant": args.variant,
            "checkpoints": {name: str(path) for name, path in checkpoints.items()},
            "task": TASK,
        },
        "follower": {"port": port, "id": args.follower_id},
        "fps": FPS,
        "cameras": {role: candidates[index].config for role, index in picks.items()},
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(config, indent=2) + "\n")
    assert json.loads(args.out.read_text()) == config
    print(f"Wrote {args.out}:\n{json.dumps(config, indent=2)}")
    print("Run the policy with:\n  uv run --locked python -m airtruck.examples.so101_policy")


if __name__ == "__main__":
    main()
