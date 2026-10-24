# Airtruck

`airtruck` is a top-level namespace package; `airtruck.testing_util` has an explicit public API. Import controls directly from `airtruck.raw_api.so101.controller` and interfaces from `airtruck.raw_api.so101.io`:

```python
from airtruck.raw_api.so101.io import ArmController  # Shared typed control interface
from airtruck.raw_api.so101.controller import SimSO101  # Explicit simulation backend
from airtruck.raw_api.so101.controller import RealSO101 # Explicit physical backend
from airtruck.raw_api.so101.controller import LeaderSO101 # Human-operated leader feedback
```

All paths below are relative to the repository root.

| Folder | Responsibility |
|---|---|
| `airtruck/raw_api/so101/controller.py` | `SimSO101`, `RealSO101`, and `LeaderSO101` adapters |
| `airtruck/raw_api/so101/calibration.py` | Hardware calibration records and JSON serialization |
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

Both backends inherit `ArmController` from `airtruck.raw_api.so101.io`. Its methods and properties are abstract; shared normalization behavior lives in `base.py`, separately from the contracts in `io.py`. Required members must be implemented before a subclass can be instantiated. Both backends expose `set_positions`, `positions`, `targets`, `normalized_positions`, `enabled` and `set_enabled`. Hardware calibration defines its own midpoint zero; it does not inherit the simulation CAD zero. Simulation time advances only through the world stepping methods. Physical motors run asynchronously through the supplied driver; the physical adapter deliberately has no simulated `step()` or automatic reset-to-zero.

```python
from airtruck.raw_api.so101.io import ArmController
from airtruck.raw_api.so101.position import JointPosition, PositionUnit

def move_pan(arm: ArmController) -> None:
    arm.set_positions({
        "shoulder_pan": JointPosition(0.0872665, PositionUnit.RADIANS, arm.positions["shoulder_pan"].limits_rad)
    })
```

Run `uv sync --locked` to create `.venv/` with the locked dependencies. The [SO-101 instructions](so101/README.md) describe model commands and the [physical adapter instructions](hardware/README.md) explain the hardware boundary.

## Leader/follower control

`airtruck/main.py` reads a human-operated physical leader and continuously copies
its six named joint angles into a follower. `RUN_MODE` in `airtruck/constants.py`
selects the follower backend: `RunMode.SIMULATION` or `RunMode.REAL` (the checked-in default).
The leader is physical in both modes.

Configure `LEADER_PORT`, `LEADER_BAUDRATE` and `LEADER_CALIBRATION` for the leader.
Real mode additionally requires `FOLLOWER_PORT`, `FOLLOWER_BAUDRATE` and
`FOLLOWER_CALIBRATION`. Use separate USB adapters and calibration files for the
two physical arms. Default calibration paths are
`models/so101/runtime/leader-calibration.json` and
`models/so101/runtime/calibration.json`; both baudrates default to 1,000,000.
These settings are constants, not command-line options. Ports must be explicitly
selected; the application does not guess which connected adapter is the leader.
Missing settings, calibration or leader communication stop startup.

Teach both arms with the [calibration script](hardware/README.md#guided-hardware-calibration):

```sh
uv run --locked python scripts/so101/so101_calibrate.py --arm both
```

For a physical leader driving the MuJoCo follower on macOS, first set
`RUN_MODE = RunMode.SIMULATION`:

```sh
uv run --locked mjpython -m airtruck.main
```

Simulation requires only the leader's port and calibration. After your terminal
confirmation, startup disables leader torque and verifies all six replies, then
opens the bench world and viewer. Each loop reads normalized leader feedback in `[-1, 1]` and sends it to
the simulated follower, which converts it to its own CAD joint coordinates and advances physics by 0.02 seconds. Targets are
motor commands, not instantaneous pose changes. The loop runs at up to 50 Hz;
serial and rendering work can reduce that rate.

For a physical follower, set `RUN_MODE = RunMode.REAL`, configure its settings,
and run:

```sh
uv run --locked python -m airtruck.main
```

The start confirmation releases leader torque and authorizes follower motion.
The follower first enables at its current pose, then follows the leader. The
application attempts follower torque disable on exit, communication failure or
partial enable failure, before closing both connections. The leader stays
unpowered. Real mode does not load simulation.

Enter `show` to display follower angles and limits in degrees, or `quit` / Ctrl-C
to stop; EOF also exits. Closing only the viewer does not stop following.
Manual angle commands have been replaced by continuous leader input.

Leader and follower exchange **dimensionless `[-1, 1]` positions** for each named
joint: `-1` is the lower endpoint, `0` the range midpoint, and `+1` the upper
endpoint. `leader.normalized_positions` converts the leader’s own calibrated
radians into this representation. `follower.set_normalized_positions(...)`
converts it into the follower’s own radians, using CAD limits in simulation or
measured calibration limits on hardware. Both modes use the same application
loop; neither arm needs to know the other’s limits.

`positions` and `normalized_positions` return `dict[JointName, JointPosition]`.
Each immutable value carries `.value`, `.unit`, `.limits_rad`, and `.range` in
its current units. Converting units preserves the original range so conversion
back from normalized values remains unambiguous:

```python
from airtruck.raw_api.so101.position import PositionUnit

position = leader.positions["shoulder_lift"]
degrees = position.to(PositionUnit.DEGREES)
normalized = position.to(PositionUnit.NORMALIZED)
print(degrees.value, degrees.unit, degrees.range)
assert normalized.to(PositionUnit.RADIANS).limits_rad == position.limits_rad
```

The class is in `position.py`; joint names/order are in `joints.py`, calibration
records and JSON helpers in `calibration.py`, and command validation in `utils.py`,
all under `airtruck/raw_api/so101/`. Import these from their own modules.

`JointName` is a `Literal` type for static name checking. `JOINTS` is the runtime
ordered tuple used to bind joints/actuators, read servos consistently, and validate
complete calibration records. They intentionally contain the same names; tests
check that the runtime tuple matches the type's names without duplicates.

`targets` returns `JointPosition` objects just like `positions`. There is no
separate public `limits` property: each position or target carries its `.range`
in its own units and `.limits_rad` for explicit backend conversion. Both command methods accept
exactly `Mapping[JointName, JointPosition]`; bare floats, lists, and arrays are
rejected. `set_positions` converts degree values to radians and decodes normalized
values using the receiving arm’s range. `set_normalized_positions` additionally
requires normalized units and rejects angular objects to prevent unit mixing. Entire batches
are validated before commanding motion. Unnamed joints stay unchanged. Commands remain
`JointPosition` objects through validation. `to_native(receiver_limits)` resolves a
typed value at the backend boundary; simulation extracts radians there, while
hardware uses `JointCalibration.position_to_ticks()` to produce encoder ticks.
There is no intermediate dictionary of radian floats.

`position.clamped()` returns a bounded copy. `position.for_range(destination_limits)`
explicitly transfers its normalized fraction into another native radian range;
`.to(unit)` only converts units within the original range. Simulation feedback
preserves actual measured positions even beyond a mechanical limit; hardware
feedback retains its calibrated clamping. Normalized feedback/commands clamp to
`[-1, 1]`. Zero normalized means range midpoint, not necessarily zero radians.
New calibration sweeps define positive as increasing encoder ticks. Verify that
positive corresponds to matching physical directions between arms; normalization
does not infer axis direction. Physical teleoperation remains unverified.

## Static typing

The root `pyrightconfig.json` selects `typings/` for Pyright/Pylance. Select `.venv/bin/python` in your editor. The stubs describe MuJoCo 3.16.0 and are not runtime modules.

```sh
uv run mypy
```

`mypy` and `pytest` are in the default development dependency group.
Mypy recursively discovers Python files from the project root (`files = "."`);
new files do not need to be registered individually. This broader scope also
checks scripts and tests previously omitted from the explicit file list.
The full-project preflight checks scripts, tests, and runtime code together.

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
use this same package. The public package imports pytest through its decorator module; pytest is
provided by the default development dependency group.

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
