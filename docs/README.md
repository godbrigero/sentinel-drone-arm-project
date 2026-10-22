# Airtruck

`airtruck` is a top-level namespace package; `airtruck.testing_util` has an explicit public API. Import controls directly from `airtruck.raw_api.so101.controller` and interfaces from `airtruck.raw_api.so101.io`:

```python
from airtruck.raw_api.so101.io import ArmController  # Shared typed control interface
from airtruck.raw_api.so101.controller import SimSO101  # Explicit simulation backend
from airtruck.raw_api.so101.controller import RealSO101 # Explicit physical backend
```

All paths below are relative to the repository root.

| Folder | Responsibility |
|---|---|
| `airtruck/raw_api/so101/controller.py` | `SimSO101`, `RealSO101`, and hardware joint calibration |
| `airtruck/raw_api/so101/io.py` | `ArmController` abstract base class and `ServoTransport` protocol |
| `airtruck/raw_api/so101/transport.py` | Acknowledged Feetech STS3215 serial transport for `RealSO101` |
| `models/so101/` | MJCF, mesh assets, CAD source manifest, reference data and runtime motor parameters |
| `scripts/so101/` | Model rebuild, verification, viewer commands, and guided hardware calibration |
| `airtruck/testing_util/` | Shared testing utilities: typed worlds, controls, contacts, pytest decorator/fixtures, and utility tests |
| `worlds/so101_bench/` | World manifest and environment scene |
| `airtruck/raw_api/so101/tests/` | Simulation and fake-transport regression tests |
| `docs/` | Usage, hardware and motor-model documentation |
| `typings/` | MuJoCo stubs for static tooling only |
| `global-info-images/` | Existing user-provided reference images |

Device controls live under `airtruck/raw_api/<device>/`; SO-101 simulation and hardware implementations share `controller.py`, with their interfaces alongside in `io.py`. Future drone controls can live under `airtruck/raw_api/drone/`.

The controller receives its dependencies explicitly: `SimSO101(model, data, servo_spec=spec)`. Scripts and test fixtures load the model and configuration; the controller has no filesystem or environment knowledge.

Both backends inherit `ArmController` from `airtruck.raw_api.so101.io`. Its abstract methods and properties must be implemented before a subclass can be instantiated. Both backends expose `set_positions`, `positions`, `targets`, `limits`, `enabled` and `set_enabled`. Hardware calibration defines its own midpoint zero; it does not inherit the simulation CAD zero. Simulation time advances only through the world stepping methods. Physical motors run asynchronously through the supplied driver; the physical adapter deliberately has no simulated `step()` or automatic reset-to-zero.

```python
from airtruck.raw_api.so101.io import ArmController

def move_pan(arm: ArmController) -> None:
    arm.set_positions({"shoulder_pan": 0.0872665})
```

Run `uv sync --locked` to create `.venv/` with the locked dependencies. The [SO-101 instructions](so101/README.md) describe model commands and the [physical adapter instructions](hardware/README.md) explain the hardware boundary.

## Simple arm control

`airtruck/main.py` provides a terminal angle console. Set `RUN_MODE` in
`airtruck/constants.py` to `RunMode.SIMULATION` (default) or `RunMode.REAL`.

Simulation on macOS:

```sh
uv run --locked mjpython -m airtruck.main
```

This opens the MuJoCo viewer and runs the bench world continuously while the
terminal accepts commands. It is a local simulation loop, not a network server.
The main thread owns physics and command handling. Exit with `quit` in the
terminal or Ctrl-C; closing only the viewer does not stop the console.

```text
shoulder_pan 10
wrist_roll -15
show
quit
```

Console angles and displayed limits are **degrees**; the shared controller API
still uses radians. Invalid, non-finite or out-of-limit commands are rejected.
Only the named joint target changes. Simulation zero is the saved CAD pose;
hardware zero is the midpoint in your calibration file.

For hardware, set `RUN_MODE = RunMode.REAL`, `PORT` to your USB adapter path,
and `CALIBRATION` to your calibration JSON path in `airtruck/constants.py`. Then run:

```sh
uv run --locked python -m airtruck.main
```

Real mode requires the complete calibration mapping produced by the
[calibration script](hardware/README.md#guided-hardware-calibration), defaulting to
`models/so101/runtime/calibration.json`. These settings are constants, not command-line options.
Confirm in the terminal to enable torque and hold the measured current pose.
On exit the application attempts torque disable before closing the serial port.
It never opens the simulation viewer in real mode. Hardware operation has not
been physically verified.

## Static typing

The root `pyrightconfig.json` selects `typings/` for Pyright/Pylance. Select `.venv/bin/python` in your editor. The stubs describe MuJoCo 3.16.0 and are not runtime modules.

```sh
uv run mypy
```

`mypy` and `pytest` are in the default development dependency group.
Mypy recursively discovers Python files from the project root (`files = "."`);
new files do not need to be registered individually. This broader scope also
checks scripts and tests previously omitted from the explicit file list.
Existing typing errors in those files currently prevent a clean full-project check.

## Run and verify

Run these commands from the repository root:

```sh
./scripts/so101/run.command --demo
uv run pytest
uv run python scripts/so101/verify.py
```

Verification prints its report to the terminal. Model inputs live under `models/so101/source/` and `models/so101/runtime/`; changing the controller code does not require rebuilding the XML. See the [SO-101 guide](so101/README.md) for rebuilding and recording commands.

## Dependencies

`pyproject.toml` declares dependencies; `uv.lock` pins their resolved versions.
`uv sync --locked` creates `.venv/` and installs runtime and development dependencies.
`uv run` uses this environment automatically; activation is optional:

```sh
source .venv/bin/activate
```

`requirements.txt` is optional and is not needed by uv. Only when a compatibility
workflow requires it, generate it from the lockfile with:

```sh
uv export --locked --no-hashes --output-file requirements.txt
```

Commit `pyproject.toml` and `uv.lock`; keep `.venv/` ignored. If a compatibility
export is maintained, refresh it after dependency changes.
Pytest discovers tests beneath `airtruck/`, including world integration tests.

## What the supporting files do

`models/so101/runtime/servo-model.json` supplies servo parameters to the simulation
controller and model builder. `models/so101/source/cad-manifest.json` supplies
geometry and reference inertias to the builder and verifier; it is not loaded at
simulation startup. `models/so101/information.json` records engineering references.
Derived electrical parameters are embedded in `so101.xml`; exporter configuration
exists only in temporary storage during a build.
`requirements.txt` is a compatibility export; uv itself uses `pyproject.toml` and
`uv.lock`. Pytest and mypy configuration live in `pyproject.toml`.

The [coupled testing guide](so101/README.md#coupled-pytest-simulation) describes the typed `world` fixture, explicit decorator, bounded waits, recordings, and optional GUI. The [world guide](simulation.md) covers multiple independently controlled systems.

## Testing utilities and generated diagnostics

Import `simulation`, `WorldControl`, `load_world`, `Pose`, and `Contact` from
`airtruck.testing_util`. The pytest plugin is
`airtruck.testing_util.pytest_plugin`; its tests and world integration tests live
in `airtruck/testing_util/tests/`. Scripts, the two-arm launcher, and static checks
use this same package. Importing its world API does not import pytest; calling the
decorator does.

`MUJOCO_LOG.TXT` is generated diagnostic output, not a model input. The removed log
contained attachment warnings from the original 1 ms arm timestep versus the
0.5 ms push-world timestep. The loader now assigns the world timestep before
attachment. The log is ignored by Git; ignoring it does not prevent future warnings
or file creation. Investigate any new warnings, then remove stale diagnostic output.

Documentation maintenance and pre-commit review requirements are in `AGENTS.md`.

## Preflight review

Use `$airtruck-preflight` before a commit or to review the project. The project
skill lives at `.agents/skills/airtruck-preflight/`. Its checker runs from
this checkout, verifies tests/types/configuration and folder hygiene, and writes
an HTML review plus evidence to a system temporary directory:

```sh
python3 .agents/skills/airtruck-preflight/scripts/preflight.py --project .
```

The skill also reviews all docs and pending changes, and adds architecture/flow
visuals and current code examples. Automated checks alone do not certify semantic
correctness or hardware physics. This is a manual pre-commit workflow; it does not
install a Git hook, stage files, or create a commit.

Use `$commit` to run the review and create a local commit. Its editable project
instructions live in `.agents/skills/commit/SKILL.md`. It reuses still-valid
checks from the current thread, runs missing checks, reviews documentation, and
verifies the resulting commit. Both skills are stored with this repository.
