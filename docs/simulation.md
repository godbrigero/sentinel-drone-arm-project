# Testing utilities: simulation worlds and typed systems

All world utilities and pytest integration live in `airtruck/testing_util/`.
Use one public import:

```python
from airtruck.testing_util import simulation, WorldControl, load_world, Pose, Contact
```

`pytest_plugin.py` provides fixtures and CLI options; `decorators.py` supplies the
opt-in marker. `loader.py`, `mujoco_world.py`, `interfaces.py`, `types.py`, and
`adapters.py` implement typed world loading and control. `tests/` contains their
regression tests and the visual demos.

A **world** owns a scene, its physics state, and one simulation clock. A **system**
is a controllable mechanism: an arm, drone, or composite assembly. Retrieve it with
`world.system("name", Interface)`. The result implements that interface and is
checked at runtime. Retrieving an arm as an incompatible interface raises
`TypeError`; an unknown name raises `KeyError`.

A world supports multiple systems, each with its own controller and actuator
bindings. The supplied adapter is SO-101. Other controller types can be registered
explicitly; a drone flight controller/model is not included. Systems can interact
physically while their commands remain independently scoped. Overlapping actuator
ownership and unowned actuators are rejected.

## Directory layout

`worlds/so101_bench/world.json` selects `scene.xml`, then declares its systems:

```json
{
  "schema_version": 1,
  "scene": "scene.xml",
  "systems": [
    {
      "name": "arm",
      "model": "../../models/so101/so101.xml",
      "root_body": "base",
      "controller": "so101",
      "controller_config": "../../models/so101/runtime/servo-model.json",
      "pose": {"position_m": [0, 0, 0], "quaternion_wxyz": [1, 0, 0, 0]},
      "placement": "fixed"
    }
  ],
  "entities": []
}
```

All file paths resolve relative to the world directory. Add a second system entry
with a different name to load another instance. Each instance receives a namespace
such as `arm/`; local joint and actuator names stay unchanged in its control API.
Only the selected root body and its dependencies are attached. Lighting, floor,
solver settings, and timestep belong to the world scene, not each robot instance.
The default bench and gesture worlds leave contact disabled. The push world
explicitly enables inter-system contacts as described below.

Optional `parent_body` attaches the system under an existing scene body or a
previously declared system body (for example `carrier/mount`). Its manifest pose
is relative to that parent. Without a parent, the pose is in world coordinates.
This supports physically mounted systems without merging their controllers. Model
names, including assets and references, are namespaced during attachment.

The loader validates schema version, names, controller types, placement
capabilities, runtime servo fields, and duplicate/unknown manifest fields. It
compiles the assembled world and binds every controller afterward. Contact-enabled
loading also compiles source models to classify volumetric mesh geometry. It does not import Python
modules named in JSON. Scene topology is fixed for the lifetime of the world.

## Placement and observation

An entity handle identifies a scene body. Systems automatically expose their root
body under their system name; additional bodies are declared in `entities`:

```json
{"name": "block", "body": "block_body", "placement": "free"}
```

```python
from airtruck.testing_util import Pose, WorldControl


def arrange(world: WorldControl) -> None:
    block = world.entity("block")
    world.place(block, Pose(position_m=(0.2, 0.0, 0.05)), clear_velocity=True)
    pose = world.pose(block)
    assert pose.position_m == (0.2, 0.0, 0.05)
```

`Pose` is immutable: meters, a unit quaternion in `wxyz` order, and world
coordinates for runtime placement/observation. Returned observations never expose
live MuJoCo array views. Recording deep-copies each observation.

Placement is an immediate test setup operation, not a motor command:

- `fixed`: reject runtime placement; choose initial pose in the manifest.
- `free`: requires a free root joint. Placement optionally clears its six velocity
  components, leaving articulated child joint coordinates unchanged.
- `kinematic`: requires a mocap body; update its prescribed pose.

Arbitrary articulated child links cannot be teleported. Handles are world-specific,
remain valid across reset, and fail after closure. Python private implementation
attributes are not a security boundary.

## Time and lifecycle

Each tick updates all system controls and calls MuJoCo stepping once. Commands and
feedback reads never advance time. Duration methods round up to whole timesteps.
`record()` includes the initial sample and the final integrated state; intervals
also round up. Waiting checks immediately, then after each step until its bound.
Reset restores the initial physics state and clears controller profiles/integrals.
All commands and observations through public handles fail after close.

Use `with load_world(path) as world:` outside pytest. Within pytest use
`@simulation(path)` and annotate the `world` fixture as `WorldControl`; use
`world_factory: Callable[[], WorldControl]` for multiple independent sessions.
Viewer failures and controller cleanup errors are reported after attempting every
cleanup. No hardware transport is opened by the built-in simulation adapter.

## Adding a controller type

Pass `adapters={"controller_name": factory}` to `load_world`. The typed
`AdapterFactory` receives `(model, data, prefix, config)` and returns a
`SystemAdapter` providing:

- `actuator_ids`: the exact actuators owned by the system.
- `interface(require_open)`: a typed public control view that checks world lifetime
  before every command/read and exposes no writable physics state.
- `update_control()`: update owned actuator commands without stepping physics.
- `reset_control()`: clear controller state after the world restores physics.
- `disable()`: disable owned actuators on cleanup, without advancing time.

The factory validates its own configuration and joint/actuator relationships. It
must clean up resources it creates if it fails before returning. Returned adapters
are cleaned up by the world loader if a later binding fails. Adapters execute
sequentially and must not reset shared state or write another system's actuators.
The SO-101 adapter demonstrates this implementation; the mixed-system test adds a
force-controlled linear stage alongside arms.

`world.system()` accepts interface classes without constructing them. Its callable
class-token annotation allows abstract classes with mypy; the runtime requires an
actual class and checks the returned view with `isinstance`. Static examples and
negative checks live in `typings/smoke.py`.

## Contact experiments

Set `inter_system_collisions: true` in a world manifest to enable inter-system
convex-hull contact on volumetric meshes. Collision masks exclude contacts within
each system. Non-volumetric decorative meshes remain visual-only; imported
primitive geoms are not enabled by this mesh-specific option. The world's scene
`<default><geom .../></default>` supplies contact friction, `solref`, and `solimp`.
The world owns the timestep. Ordinary worlds retain their existing collision setup.
[MuJoCo describes the convex-hull approximation here](https://mujoco.readthedocs.io/en/stable/computation/#collision-detection).

A system may specify `initial_joint_positions_rad`, mapping local hinge names to
initial angles. Values must be finite and within configured joint limits. These
positions are applied before controllers are created, and restored on world reset.

`world.contacts()` returns immutable `Contact` snapshots with body names, normal
force in newtons, and penetration in meters, computed from the current physics
state. It does not fabricate contact events from proximity.

For explicitly simulated motor experiments, retrieve
`world.system(name, SimulatedArmController)` from `airtruck.raw_controls.so101.io`.
`set_motor_voltages({...})` overrides the selected position loops, rejecting values
outside the supply range. Other joints keep their position control. Voltage and
output torque feedback are available as `motor_voltages` and `motor_torques_nm`.
`use_position_control()` clears overrides and holds the measured pose; world reset
also clears overrides. Position targets on overridden joints do not take effect
until overrides are cleared. This interface is not implemented by the physical
hardware adapter.
