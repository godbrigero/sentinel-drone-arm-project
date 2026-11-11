# Airtruck

The runtime is built around system objects with persistent `controls`, observed
`state`, and an optional `hardware` flag. `SystemDriver` runs simulated and
hardware systems on one control clock in a shared native MuJoCo world. A thin Gymnasium adapter supplies episodes and policy inputs;
LeRobot owns SO-101 hardware and calibration.

```python
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.so101 import SO101Follower

left = SO101Follower("left", position=(-0.25, 0, 0))
right = SO101Follower("right", position=(0.25, 0, 0))

with SystemDriver([left, right]) as driver:
    driver.reset()
    left.controls["shoulder_pan.pos"] = 20
    right.controls["shoulder_pan.pos"] = -10
    driver.step()
```

## Run

```sh
uv sync --locked
uv run --locked python -m airtruck.main
uv run --locked mjpython -m airtruck.main --render
uv run --locked python -m airtruck.examples.two_arms
uv run --locked mjpython -m airtruck.examples.table_leader --render
```

`airtruck.main` runs leader/follower teleoperation for 30 seconds by default.
Both objects stay simulated unless their hardware flags are supplied; `--render`
opens the viewer. Calibration and hardware options are described in the
[hardware guide](hardware/README.md). `mjpython` is needed only on macOS, where the
MuJoCo viewer must own the main thread; on Linux use `python` in its place.

`airtruck.examples.two_arms` runs independent motion for eight simulated seconds,
reports contacts and exits without serial connections. Its `--world` option selects
a base MJCF template without attached systems, and `--benchmark` skips contact-force
extraction. Rendering is optional; the headless example does not sleep.

`airtruck.examples.table_leader` places one simulated leader at the recorded
mounting location in the cleaned table scan. Its base is fixed, its joints remain
movable, and it faces directly away from the rear wall over the table. It opens no
hardware connections. See the [table world](simulation.md#leader-on-the-scanned-table)
for placement assumptions, native MJCF files and asset rebuilding.

The edit-and-run `one_arm`, `drone_takeoff_land`,
`drone_takeoff_fly_forward_land` and `drone_1m_in_front_tag` modules share
`airtruck/examples/constants.py`. Its current `IS_HARDWARE = False` selects
simulation; setting it to `True` selects physical hardware. Run these examples with
`uv run --locked mjpython -m airtruck.examples.<module>` on macOS. The arm example
contains device-specific ports and calibration IDs; edit them for your setup.
The drone examples explicitly take off and land. The position example targets
NED `[0, 0.5, -0.5]`; the tag example uses a 0.5 m standoff despite its historical
filename. Neither is a validated autonomous flight mission.

The [yellow-cube collector](simulation.md#scripted-yellow-cube-dataset-simulation-only)
records configurable simulation grasps (ten legacy or fifty with noisy recovery
trajectories) in standard LeRobot format for SmolVLA
fine-tuning, using the scaled workstation scan and front and wrist cameras matched to the
physical ones ([measured cameras](simulation.md#measured-cameras); the original training
views remain selectable).
[SmolVLA fine-tuning](simulation.md#smolvla-fine-tuning) gives the MPS and CUDA
training command and closed-loop simulation results.

## Linux and NVIDIA GPUs

`uv.lock` resolves for Apple-silicon macOS and Linux x86_64 alike; run
`uv sync --locked` on each machine rather than copying `.venv`. On Linux the
locked PyTorch 2.11 wheels bundle the CUDA 13.0 runtime (no system CUDA toolkit is
needed). They require NVIDIA driver 580 or newer (`nvidia-smi` shows it) and a Turing
or newer GPU; an RTX 2080 Ti is Turing, compute capability 7.5. Check the GPU,
including a bfloat16 matrix product (the SmolVLM backbone runs in bfloat16, which
Turing executes without bfloat16 tensor cores):

```sh
uv run --locked python -c "import torch; x = torch.ones(64, 64, device='cuda', dtype=torch.bfloat16); \
print(torch.__version__, torch.version.cuda, torch.cuda.get_device_name(0), torch.cuda.get_device_capability(0), (x @ x)[0, 0].item())"
```

A 2080 Ti should print `(7, 5)` and `64.0`. `so101_pick_eval` and `so101_policy` use
CUDA, then MPS, then CPU; `--device` overrides this. Checkpoints trained on the Mac
save `"device": "mps"`, and LeRobot substitutes the available device when loading them.
They live outside the repository, so copy the `pretrained_model` folders and pass their
paths to `so101_cameras --<variant> PATH` (see
[hardware usage](hardware/README.md#smolvla-yellow-cube-policy-on-the-follower)).

MuJoCo offscreen rendering (dataset collection, simulated evaluation, camera tests)
uses GLFW in a desktop session. Over SSH or without a display, set `MUJOCO_GL=egl`,
which the NVIDIA driver provides. Serial and camera access need membership in the
`dialout` and `video` groups (`sudo usermod -aG dialout,video $USER`, then log in
again). The macOS RealSense bridge is not used on Linux.
CUDA inference and training have not yet been run on a 2080 Ti in this project.

## Code and configuration

- `airtruck/systems/base.py`: `SimSystem` lifecycle, numeric space validation and typed `state`/`controls`.
- `airtruck/systems/so101.py`: shared arm binding plus native LeRobot leader/follower subclasses.
- `airtruck/systems/sentinel.py`: `SentinelDrone`, the Sentinel aircraft for simulation or PX4 hardware (`hardware=`), like `SO101Follower`.
- `airtruck/systems/drone.py`: generic `Drone` with `DroneState`/`DroneControls` keys: shared position/yaw flight commands, propeller spin, cameras and tag ingestion for a prepared free-body MJCF and PX4 hardware.
- `airtruck/systems/flight.py`: NED frame conversion, flight schemas and simulation autopilot.
- `airtruck/systems/px4.py`: typed ROS 2 offboard transport with confirmed commands, telemetry and `/tag_detections`.
- `airtruck/systems/camera.py`: camera intrinsics, body mounting and latest-frame feed; `CameraSpec` attaches MuJoCo pinhole cameras, optionally with measured focal length and principal point.
- `airtruck/systems/apriltag.py`: global `TAGS` detection board, OpenCV AprilTag detector and simulated tag placement.
- `airtruck/systems/tag_object.py`: simulation-only `AprilTagObject` from a tag SVG (`models/apriltag/`), movable by delta or absolute position/rotation.
- `airtruck/runtime/driver.py`: `SystemDriver`: model assembly, backend switching, shared stepping, rendering and cleanup.
- `airtruck/runtime/gymnasium.py`: Gymnasium packing, flattening names and episode boundary.
- `airtruck/examples/two_arms.py`: individual objects controlled before one shared step.
- `airtruck/examples/one_arm.py`: linked leader/follower teleoperation.
- `airtruck/examples/drone_1m_in_front_tag.py`: camera-based tag standoff example.
- `airtruck/examples/so101_pick.py`: simulation-only scripted yellow-cube LeRobot dataset collector and verifier on the scanned workstation; `so101_pick_eval.py` evaluates a SmolVLA checkpoint closed-loop in the same world.
- `airtruck/examples/so101_cameras.py`: identifies the physical front/wrist cameras and follower port and writes `models/so101/hardware.json`.
- `airtruck/examples/so101_mirror.py`: torque-off mirror window of the live camera inputs, nearest training frames and the simulated arm posed from the physical follower, for checking joint directions against the simulated mapping.
- `airtruck/examples/so101_policy.py`: runs a configured SmolVLA pick checkpoint variant on the physical follower from that file, optionally recording each tick; see [hardware usage](hardware/README.md#smolvla-yellow-cube-policy-on-the-follower).
- `airtruck/examples/so101_realsense_bridge.py`: macOS-only, root-run librealsense colour stream for the D435i front camera, read without root by the policy.
- `airtruck/main.py`: teleoperation and calibration through the same objects, using hardware flags.
- `models/sentinel-quad/sentinel.xml`: Sentinel simulation model, generated by `scripts/sentinel/build_model.py` from an Onshape glTF export.
- `models/table-top/scene.xml`: arm-free solid workshop reconstruction with spatially varied visual detail and primitive collision shapes; see the [model guide](../models/table-top/README.md).
- `worlds/bench/scene.xml`: the default base world; each system supplies its own model and placement.
- `worlds/table_top/scene.xml`: scanned table base world; `leader.xml` is the standalone scene with one fixed leader.
- `worlds/so101_pick/scene.xml`: yellow-cube pick world on the scanned SO-101 workstation, generated by `so101_pick` from `models/so101-workstation/source/` (scan glTFs and registration) into the tracked meshes in `models/so101-workstation/assets/`.
- `worlds/so101/scene.xml`, `worlds/two_arms/scene.xml`: standalone native reference scenes for model verification.

The driver contains no SO-101 joint counts, normalization, or drone force mapping.
Native MuJoCo computes actuator forces, dynamics and contacts. SO-101 collision
pieces, exclusions, inertias, servo parameters and action-unit metadata live in
`models/so101/so101.xml`, generated by `scripts/so101/build_model.py` and checked by
`scripts/so101/verify.py`.

The exported SO-101 and Sentinel visual geometry is preserved. The Sentinel model's
mass/inertia are assigned simulation values; see [Sentinel CAD](sentinel/README.md#simulation-model).
The PX4 backend uses ROS 2 position/yaw offboard messages. The controller-contract
tests use a clearly labeled analytic fixture; Sentinel tests use the generated model.
The yellow-cube collection example adds a task-specific grasp success check;
the general runtime has no task-specific reward.

`pyproject.toml` and `uv.lock` own dependencies and the persistent ignored `.venv`.
Engineering records remain model-specific `information.json` files; LeRobot owns
its device calibration cache and saved training configurations. No custom runtime
servo/world JSON, compatibility hierarchy, initializer exports or serial protocol
is required. Use direct module imports.

## Verify

For a reviewed local commit, invoke `$commit` in Codex or `/commit` in Claude Code
from this repository. The [Claude Code adapter](../.claude/skills/commit/SKILL.md)
loads the [shared commit workflow](../.agents/skills/commit/SKILL.md), including
the project preflight and documentation review. Both reuse still-valid checks
from the current conversation and verify the resulting commit without pushing.

```sh
uv run --locked pytest -q
uv run --locked mypy
uv run --locked basedpyright
uv run --locked python scripts/so101/verify.py
```

Tests cover mixed systems, native flight tracking/landing, Sentinel propeller spin
and cameras, simulated and ROS AprilTag detections, PX4 message transport, contact
effects, native LeRobot on fake buses and backend switching. Physical ports are
never opened. Regression tests also exercise the four edit-and-run examples with
hardware disabled and short headless runs, tag callback filtering/cleanup, and
calibration failures without automatic retries. The default pytest command includes
the preflight auditor's six self-tests; they also run directly with
`python3 .agents/skills/airtruck-preflight/scripts/test_preflight.py`.
Camera tests need an offscreen OpenGL context: available on macOS and in a Linux desktop session; otherwise set `MUJOCO_GL=egl` (or `osmesa`).
See [simulation](simulation.md), [hardware](hardware/README.md),
[CAD provenance](so101/README.md), [model fidelity](so101/motor-model.md) and
[Sentinel CAD](sentinel/README.md).
