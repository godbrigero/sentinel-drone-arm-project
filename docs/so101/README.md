# Airtruck — SO-101 native Onshape → MuJoCo

This is a six-servo, fixed-base arm demonstration. It uses the actual 257 Onshape solids, seven native rigid groups, and six revolute mate axes and limits. `onshape-to-robot` 1.8.3 generates MJCF through a native-data adapter; no Onshape API keys are used. MuJoCo 3.16.0 runs the result. The Sentinel is not part of this scene.

## Wrist camera mount in current Onshape CAD

The live follower assembly contains 258 solids, including the original
[TheRobotStudio hex-nut wrist-camera bracket](https://github.com/TheRobotStudio/SO-ARM100/tree/a758567c3978dfeefe282ede0500085a48fe8f78/Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module).
Its STEP geometry is imported unchanged and attached to `fix_hand` through the
wrist-roll housing's two mounting holes (8.1 mm spacing). It supports the published
32 × 32 mm UVC board format with a 27 × 27 mm camera screw pattern. The bracket
moves with the hand; the six joint definitions and existing part poses are unchanged.
The camera board and mounting screws are not modeled. The supplied photographs
match the board-camera/mount style, but do not establish the exact camera model
or measured dimensions. CAD mass, center of mass and inertia remain unassigned.

The simulation below remains pinned to its earlier 257-part CAD export and does
not include the new bracket or a camera sensor. Rebuilding its XML alone does not
update those meshes. Current CAD geometry and source links are recorded in
`models/so101/information.json`.

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
| `models/so101/source/cad-manifest.json` | Build/verification only: CAD revision, rigid memberships, axes, ranges, mesh hashes, reference inertias |
| `models/so101/runtime/servo-model.json` | Runtime controller inputs and model-build inputs: servo specifications, source, unknowns, demo tuning |
| `models/so101/information.json` | Engineering reference record; not loaded by the controller |
| `models/so101/so101.xml` | Runtime physics model, SI units, including derived electrical parameters |
| `models/so101/assets/` | Required native CAD visual meshes; all triangles retained, grouped by link/color |
| `scripts/so101/build_model.py` | Rebuild XML with the onshape-to-robot exporter |
| `airtruck/raw_api/so101/controller.py` | Follower control, human-operated leader reader, and simulated arm; simulation uses scoped actuator bindings |
| `scripts/so101/run.py` | Live viewer, keyboard controls, small-motion demo and recording |
| `scripts/so101/verify.py` | Fresh model/mesh reads, topology, axes, limits, inertia and six-motion checks |
| `docs/so101/motor-model.md` | Research, equations, sources and fidelity limitations |

The runtime loader reads `so101.xml` and `runtime/servo-model.json`. It passes the
`datasheet` and `controller_demo_tuning` sections to `SimSO101`; the controller
performs no file I/O. The builder also uses the published servo data and gripper
current fraction to generate actuator parameters. Changing build inputs requires
rebuilding `so101.xml`; changing Python PID/profile tuning requires a fresh session.
Current simulation tuning uses Kp = 50 V/rad, Ki = 12 V/(rad·s),
Kd = 2 V·s/rad, target speed 3 rad/s and target acceleration 5 rad/s².
These settings affect simulation only; they are not hardware firmware gains.
The remaining metadata in the servo file documents sources and unmeasured values.

There is no separate electrical-fit JSON: derived values are stored under
`<custom>` in the generated XML and read there by the verifier. The exporter's
required configuration file is created temporarily and removed during each build.

Onshape remains the CAD source of truth. The retained meshes are simulation inputs, not editable CAD replacements; recordings are optional user-requested outputs. Rebuilding the XML does not re-fetch CAD; a new CAD revision must be exported through the native integration and verified before replacing the pinned geometry manifest.

## Python interface

Use a world to own physics and retrieve a typed arm interface:

```python
from airtruck.testing_util import load_world
from airtruck.raw_api.so101.io import ArmController
from airtruck.raw_api.so101.position import JointPosition, PositionUnit

with load_world("worlds/so101_bench") as world:
    arm = world.system("arm", ArmController)
    arm.set_positions({
        "shoulder_pan": JointPosition(0.0872665, PositionUnit.RADIANS, arm.positions["shoulder_pan"].limits_rad),
        "wrist_roll": JointPosition(-0.174533, PositionUnit.RADIANS, arm.positions["wrist_roll"].limits_rad),
    })
    world.advance(3.0)
    print(arm.positions)
    print(arm.targets)
    print({name: position.range for name, position in arm.positions.items()})
    world.reset()
```

`world.system(name, Interface)` returns a typed, runtime-checked control interface.
Arm feedback is copied as immutable `JointPosition` values with explicit units
and ranges; the public interface exposes neither MuJoCo buffers nor
physics stepping. Joint names are statically checked `JointName` literals. Commands
accept only `Mapping[JointName, JointPosition]`; numeric mappings and vectors raise
`TypeError`. For leader/follower exchange,
`normalized_positions` and `set_normalized_positions({...})` use dimensionless
`[-1, 1]` values, with zero at the midpoint of each joint’s own limits. Unknown joints and non-finite
values raise `ValueError` before any command is applied.

Internally, `SimSO101` receives model/data, validated servo settings, and a namespace
prefix. It owns only the six resolved joint/actuator bindings. `update_control()`
writes those actuators without advancing time; `reset_control()` clears only its
controller state. The world updates every controller, then steps physics once.
The same `ArmController` contract is implemented by `RealSO101` for hardware.
`LeaderSO101` provides human-operated hardware feedback after explicitly disabling
torque with `start()`. Calibrate both physical arms with `--arm both`; each has
its own calibration file. Their measured midpoint calibration is independent of CAD coordinates; see the
[guided hardware calibration](../hardware/README.md#guided-hardware-calibration).

## Control semantics

`SimSO101.command_radians(name, radians)` takes a target in **Onshape's saved-pose joint coordinates**, not real motor ticks or LeRobot's calibrated joint zero. `sync_write('Goal_Position', {name: ticks})` emulates 4096-count position targets using virtual offsets. Hardware offsets/directions are unknown; these virtual tick values must not be sent to hardware. Observations use `joint.pos` keys in CAD radians. This mirrors position-goal/position-feedback behavior, not the serial protocol, bus timing, or the manufacturer's undocumented firmware.

The XML actuators receive **voltage**, not position. Run the supplied world and controller for position operation; opening the XML alone does not install the outer servo loop. The built-in control panel is hidden to avoid presenting voltage sliders as joint targets. Speed/acceleration profiling and encoder quantization are in Python; electrical force generation and joint losses are in MJCF. No real serial device is opened.

The user confirmed the standard **7.4 V STS3215** follower profile; 345:1 gearing follows the official follower specification. Actual supply voltage and motor characteristics have not been measured. Reference link inertias total **0.632006 kg**; this is neither assigned Onshape mass nor a measurement of the full detailed assembly. Collision contacts are disabled in the ordinary bench demonstration; the separate two-arm push test enables inter-system contact. In both cases, gravity and articulated dynamics remain active. Cables remain rigid visual geometry.

## Coupled pytest simulation

```python
import pytest
from airtruck.testing_util import simulation, WorldControl
from airtruck.raw_api.so101.io import ArmController
from airtruck.raw_api.so101.position import JointPosition, PositionUnit


@simulation("worlds/so101_bench")
def test_pan_reaches_target(world: WorldControl) -> None:
    arm = world.system("arm", ArmController)
    world.advance(3.0)
    arm.set_positions({
        "shoulder_pan": JointPosition(0.1, PositionUnit.RADIANS, arm.positions["shoulder_pan"].limits_rad)
    })
    history = world.record(lambda: arm.positions, 3.0, sample_period_s=0.1)
    assert history[-1].value["shoulder_pan"].value == pytest.approx(0.1, abs=0.01)
```

The decorator selects a directory containing `world.json` and `scene.xml`; paths
are relative to the pytest root. Loading occurs when `world` or `world_factory` is
requested. These fixtures require explicit `@simulation(...)` opt-in; ordinary
tests do not load worlds. `@simulation()` uses `--sim-world-dir`, which defaults to
`worlds/so101_bench`. `world_factory()` creates independent worlds owned by the test.
Every created world is closed at teardown, even after failed tests or viewer startup.

`world.step(n)` advances physics steps; `world.advance(seconds)` rounds up to whole
steps. `world.record(observe, duration_s, sample_period_s)` copies initial, periodic,
and final observations into timestamped samples. `world.wait_until(predicate,
timeout_s=...)` bounds waiting by simulation time. `world.reset()` restores initial
physics state and controller targets/profiles. Time advances only through these
explicit stepping methods; there is no background worker or communication client.

```sh
uv run --locked pytest
uv run --locked pytest -m sim
uv run --locked pytest -m "not sim"
uv run --locked mjpython -m pytest --sim-gui -k coupled_control
```

The GUI option opens a native window and paces steps to wall-clock time. Headless
operation is the default. World and system handles reject use after closure.
`sim`, `client`, `sim_factory`, and `--sim-model-dir` have been replaced by the world
API. See [world configuration and system adapters](../simulation.md) for multiple
systems, placement, and custom controller types.

## Two-arm physical push test

Run `./scripts/so101/two_arms.command` (or double-click it in Finder). The arms start
extended and parallel, 160 mm apart at their bases. Both shoulder-pan motors drive
left: the weak arm receives -0.2 V, and the pushing arm receives the full configured
-7.4 V for 0.8 simulated seconds. The remaining joints hold extension. The test then
switches back to position control and holds the result for three seconds.

```sh
uv run --locked mjpython -m pytest --sim-gui -s -q -k two_arms_collision_push
```

This test actually uses contact forces. A matching headless run with collisions
disabled verifies that the weak arm's large displacement is caused by the other
arm, rather than its own small drive command. It checks contact force, penetration,
motor limits, and displacement. No external forces, pose teleports, or coordinated
follower commands are applied during the push.

The world is `worlds/two_arms_push/`; the typed test is
`airtruck/testing_util/tests/test_two_arms_push.py`. Left is world -X. The native
window closes at completion. The earlier gesture demo remains available with
`-k two_arms_greeting`; it does not test collisions.

The push world uses convex hulls of the existing volumetric mesh groups, excluding
planar decorative meshes. Self-collisions are disabled to isolate arm-to-arm
contact. Visual CAD geometry and reference inertias are unchanged. Convex hulls
can bridge concave gaps: this establishes force transfer in the configured physics
model, not exact surface contact or measured hardware force. Full voltage does not
mean constant torque; back EMF and the existing 1.96133 Nm actuator limit still apply.
The 0.5 ms timestep and contact solver settings are numerical simulation settings;
friction and stiffness are not calibrated to physical hardware.
