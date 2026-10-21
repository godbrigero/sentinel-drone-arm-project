# Airtruck — SO-101 native Onshape → MuJoCo

This is a six-servo, fixed-base arm demonstration. It uses the actual 257 Onshape solids, seven native rigid groups, and six revolute mate axes and limits. `onshape-to-robot` 1.8.3 generates MJCF through a native-data adapter; no Onshape API keys are used. MuJoCo 3.16.0 runs the result. The Sentinel is not part of this scene.

## Run

From the project root:

```sh
./scripts/so101/run.command --demo
```

The demo moves each joint about 0.0872665 rad and returns it. The window then remains open for individual control. Click the MuJoCo window; keys **1–6** select shoulder pan, shoulder lift, elbow flex, wrist flex, wrist roll, and gripper. **[ / ]** change the selected target by −0.0872665 rad / +0.0872665 rad. **R** returns all targets to the saved CAD pose. Close the window to stop.

The launcher uses `uv run` with the project dependencies from `pyproject.toml` and `uv.lock`. Run `uv sync --locked` to create or update the project `.venv/` from `uv.lock`. Install `uv` before launching. Python 3.12 was tested on macOS. The renderer requires an available desktop/OpenGL context. Recording also requires `ffmpeg`.

```sh
uv run python scripts/so101/verify.py
uv run python scripts/so101/build_model.py
uv run mjpython scripts/so101/run.py --demo --record /tmp/so101-motion.mp4
```

## Folder contract

Paths are relative to the repository root.

| Path | Purpose |
|---|---|
| `models/so101/config/cad-export.json` | Native CAD revision, rigid memberships, axes, ranges, mesh hashes, reference inertias |
| `models/so101/config/servos.json` | Published specifications, source, explicitly unknown properties and demo tuning |
| `models/so101/config/electrical-fit.json` | Derived, approximate electrical and friction parameters |
| `models/so101/so101.xml` | Generated MJCF, SI units |
| `models/so101/assets/` | Required native CAD visual meshes; all triangles retained, grouped by link/color |
| `scripts/so101/build_model.py` | Rebuild XML with the onshape-to-robot exporter |
| `airtruck/raw_controls/so101/controller.py` | Simulation-only goal-position bus and voltage controller |
| `scripts/so101/run.py` | Live viewer, keyboard controls, small-motion demo and recording |
| `scripts/so101/verify.py` | Fresh model/mesh reads, topology, axes, limits, inertia and six-motion checks |
| `docs/so101/motor-model.md` | Research, equations, sources and fidelity limitations |

Onshape remains the CAD source of truth. The retained meshes and video are required simulation deliverables, not editable CAD replacements. Rebuilding the XML does not re-fetch CAD; a new CAD revision must be exported through the native integration and verified before replacing the pinned geometry manifest.

## Python interface

From the project root, run Python with the `uv run python`:

```python
import json
from pathlib import Path
import mujoco
from airtruck.raw_controls.so101.controller import SimSO101

# Application setup: paths and loading belong to the caller.
model = mujoco.MjModel.from_xml_path("models/so101/so101.xml")
data = mujoco.MjData(model)
spec = json.loads(Path("models/so101/config/servos.json").read_text())
arm = SimSO101(model, data, servo_spec=spec)
arm.set_positions({"shoulder_pan": 0.0872665, "wrist_roll": -0.174533})  # CAD radians
arm.step(3000)  # 3 simulated seconds; no wall-clock sleep
print(arm.positions)  # Actual joint angles in radians
print(arm.targets)  # Quantized, range-clamped targets
print(arm.limits)  # Per-joint minimum/maximum radians
arm.reset()  # Saved pose; clear controller state
```

`SimSO101(model, data, servo_spec=spec)` requires a caller-supplied matching model/data pair and a `ServoSpec` mapping. The controller copies servo settings and uses model/data in place; it never loads files or infers paths. `model`/`data` remain available for MuJoCo rendering. `set_positions` validates a complete command before applying it; unknown joints and non-finite values raise `ValueError`. `enabled=False` disables powered torque on the next step; passive friction remains.

Run API checks with:

```sh
uv run pytest -k simulation
```

## Control semantics

`SimSO101.command_radians(name, radians)` takes a target in **Onshape's saved-pose joint coordinates**, not real motor ticks or LeRobot's calibrated joint zero. `sync_write('Goal_Position', {name: ticks})` emulates 4096-count position targets using virtual offsets. Hardware offsets/directions are unknown; these virtual tick values must not be sent to hardware. Observations use `joint.pos` keys in CAD radians. This mirrors position-goal/position-feedback behavior, not the serial protocol, bus timing, or the manufacturer's undocumented firmware.

The XML actuators receive **voltage**, not position. Run the supplied controller for position operation; opening the XML alone does not install the outer servo loop. The built-in control panel is hidden to avoid presenting voltage sliders as joint targets. Speed/acceleration profiling and encoder quantization are in Python; electrical force generation and joint losses are in MJCF. No real serial device is opened.

The user confirmed the standard **7.4 V STS3215** follower profile; 345:1 gearing follows the official follower specification. Actual supply voltage and motor characteristics have not been measured. Reference link inertias total **0.632006 kg**; this is neither assigned Onshape mass nor a measurement of the full detailed assembly. Collision contacts are disabled for this motion-only demonstration; gravity and articulated dynamics remain active. Cables remain rigid visual geometry.
