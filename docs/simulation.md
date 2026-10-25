# System objects and the shared MuJoCo driver

Each `SimSystem` owns its model binding, control schema, observations and optional
hardware connection. `MujocoDriver` owns one MuJoCo model/data pair and physics clock.
It does not interpret arm joints, drone forces, units or controller types.

```python
from airtruck.simulation.driver import MujocoDriver
from airtruck.systems.so101 import SO101Follower

left = SO101Follower("left", position=(-0.25, 0, 0))
right = SO101Follower("right", position=(0.25, 0, 0))

with MujocoDriver([left, right], control_dt=0.02) as driver:
    driver.reset()
    left.controls["shoulder_pan.pos"] = 20
    right.controls["shoulder_pan.pos"] = -10
    driver.step()
    print(left.state, right.state, driver.info)
```

`controls` holds persistent desired targets. Unchanged fields retain their targets;
`state` contains observed values, not desired targets. Editing controls does not
advance physics. `driver.step()` synchronizes pending backend switches, refreshes
inputs, runs an optional update callback, validates **all** control snapshots,
dispatches them through the systems, then advances the shared model once. Every
system moves and exchanges contact forces during the same MuJoCo substeps.

The default world uses a 0.0005 s timestep and a 32 MiB solver arena. A 0.02 s control
period means 40 native physics steps per driver tick. Headless `step()` does not
sleep. Models use meters and wxyz unit quaternions for placements.

## Continuous control

```python
with MujocoDriver([left, right]) as driver:
    driver.reset()

    def update():
        left.controls["shoulder_pan.pos"] = 20
        right.controls["shoulder_pan.pos"] = -10

    driver.run(update=update, realtime=True, steps=100)
```

`run()` is a blocking loop; it does not start hidden worker threads. The callback
runs after fresh input observations and before command validation. Omit `steps` to
run until `driver.stop()` or interruption. Use `realtime=False` for accelerated
simulation. Hardware loops require real-time pacing. Physical devices continue on
their own clocks, and dispatch across serial devices is not an atomic transaction.

## Systems and native worlds

The base contract is in `airtruck/systems/base.py`: `attach`, `bind`, `reset`, `step`
and `observe`. `prepare_step` validates and snapshots a system's pending controls.
A system's `step(dt)` applies its local inputs; it must never call `mujoco.mj_step`.
New system types implement their own action and observation spaces. Scalar and
array controls can coexist; the driver contains no fixed channel counts.

Each object supplies its prepared MJCF model and placement. The driver attaches
its root body under the object's name using native `MjSpec` frames, then compiles
one world. `world=` accepts a base MJCF template without those systems; the default
is `worlds/bench/scene.xml`. Model geometry and collision preparation remain in the
model builder, not the driver.

The standalone `worlds/so101/scene.xml` and `worlds/two_arms/scene.xml` remain native
MuJoCo reference scenes used by `scripts/so101/verify.py`. They already contain
robots and are not base templates for object-based assembly.

`reset()` restores native initial data and asks every system to reset its own
local state. The driver seeds `system.rng` for reproducible object-owned reset
randomization. SO-101 `initial_qpos` optionally supplies initial joint positions in
radians. Hardware-enabled arms read and hold their measured pose; reset does not
command real hardware to a simulated initial pose. Context exit closes every
system and rendering resource, including after partial connection failures.

## SO-101 and LeRobot

`SO101Arm` contains the arm mapping. `SO101Follower` and `SO101Leader` inherit it
and the corresponding LeRobot classes. The SDK still owns motor communication,
calibration and hardware configuration. The bridge currently supports joint-state
observations, `use_degrees=False`, and no cameras.

Local keys are LeRobot keys such as `shoulder_pan.pos` and `gripper.pos`. Five body
channels use [-100,100]; the gripper uses [0,100]. The model's actuator `user` fields
store those same bounds. They map to the native actuator control ranges in radians.
Equal normalized travel does not establish matching real joint directions; physical
calibration must be checked before following.

A follower exposes `send_action()` and `get_observation()`. A leader exposes
`get_action()` and `send_feedback()`. `send_action()` applies a target immediately
and returns the target accepted by LeRobot, including native relative-target
clipping. It never advances the simulation clock. Use `controls` plus `driver.step()`
for coordinated ticks. A real leader remains an input device: ordinary driver
steps read it and never command its motors. Simulated leader controls pose its
virtual input device. Leaders are observations, not Gymnasium action channels.

See [hardware usage](hardware/README.md) for flags, calibration and teleoperation.

## Drone control

`airtruck/systems/drone.py` supplies a separate free-body controller. It requires a
caller-supplied, prepared model; it does not create or simplify Sentinel geometry.
There is currently no local Sentinel MJCF or flight-controller implementation.

```python
from airtruck.systems.drone import Drone

arm = SO101Follower("arm")
drone = Drone("drone", model="/path/to/prepared-drone.xml", position=(0.4, 0, 0.5))

with MujocoDriver([arm, drone]) as driver:
    driver.reset()
    arm.controls["shoulder_pan.pos"] = 20
    drone.controls["thrust_n"] = 4
    drone.controls["yaw_torque_nm"] = 0.02
    driver.step()
    print(drone.state["position"])
```

The drone root body needs one free joint, assigned inertia, and four native site
actuators named `thrust_n`, `roll_torque_nm`, `pitch_torque_nm`, `yaw_torque_nm`.
They must be stateless, unit-gain, bias-free motor actuators. Their gears are respectively `0 0 1 0 0 0`, `0 0 0 1 0 0`, `0 0 0 0 1 0`,
`0 0 0 0 0 1`. They share a body-aligned site at the root's center of mass and
finite native `ctrlrange` limits. MuJoCo rotates these local wrenches as the body
moves; no Python force integration or collision solver is implemented.

State contains world position of the root body frame, wxyz quaternion, world
linear velocity of that same origin in m/s, and world angular velocity in rad/s.
Center-of-mass offsets do not change the reported velocity reference point. A `DroneHardware` implementation supplies
`connect`, `disconnect`, `get_observation`, and `send_action` in this SI schema.
Connecting the backend must not arm an aircraft. Reset preserves existing hardware
wrench targets; a simulated reset clears them to zero. Enabling hardware without a
backend raises an error. Reference aircraft weights are not assigned CAD inertia.

The analytic free-body fixture in `airtruck/tests/conftest.py` tests thrust, torque,
rotated forces, floor contact and backend tracking. It is not a Sentinel model.
`airtruck.examples.mixed_systems` runs an arm with a supplied drone model.

## Gymnasium and ACT

```python
with MujocoDriver([left, right]) as driver:
    env = driver.as_gymnasium(max_episode_steps=1000, flatten=True)
    observation, info = env.reset(seed=42)
    observation, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

The adapter packs each system's spaces. By default actions/observations are nested
by system name; `flatten=True` uses standard Gymnasium flattening wrappers.
`TimeLimit` owns episode truncation. There is no manipulation objective: the base
adapter returns zero reward and no task termination. Task-specific rewards and
success conditions belong in a separate Gymnasium wrapper.

Use Gymnasium `SyncVectorEnv` or `AsyncVectorEnv` with factories that create fresh
systems and drivers for each worker. Each worker owns an independent whole world;
the interacting objects within one world are not separate workers. Async factories
need the usual `if __name__ == "__main__":` entry point. One system instance cannot
belong to two drivers.

```sh
uv run --locked python -m airtruck.examples.act record --output /tmp/airtruck-dataset --episodes 4 --seconds 8
uv run --locked python -m airtruck.examples.act train --dataset /tmp/airtruck-dataset --output /tmp/airtruck-act --steps 1000
uv run --locked python -m airtruck.examples.act rollout --checkpoint /tmp/airtruck-act/checkpoints/last/pretrained_model
```

The example records a standard LeRobot v3 dataset, calls LeRobot's Python trainer,
and reloads native ACT checkpoints/preprocessors. The first arm supplies
`observation.state`; the other arm supplies `observation.environment_state`.
Both are measured positions available through the same object API in simulation
or hardware. Stock ACT requires an image or environment-state input. This example
uses numerical state, without synthetic hardware velocity measurements or vision
weight downloads. Demonstrations pair observations at t with commands at t before
physics advances. They demonstrate infrastructure, not task expertise.

`act_training_config(dataset, output, steps=1000, device="cpu")` returns native
LeRobot configuration dataclasses. `train(config)` returns the saved checkpoint.
The trainer handles normalization, action chunks, optimization and checkpointing.
Model sources, object placements, channels and timing are checked on rollout.
Old checkpoint metadata is not supported. Nothing is uploaded to the Hub.

The example uses a small 128-dimensional ACT, 16-action chunks and per-step
inference with temporal ensembling. Three updates only check plumbing, not a
learned skill. Use fresh output directories. Available accelerators can be chosen
with `--device mps` or `--device cuda`.

## Rendering and contact fidelity

`render_mode="human"` opens a passive MuJoCo viewer; on macOS use `mjpython`.
`render_mode="rgb_array"` lets `driver.render()` return RGB uint8 frames.
The driver only renders when requested. Native `driver.model` and `driver.data`
remain available for diagnostics; direct writes bypass the object API.

The builder emits collision geometry and mating-link exclusions into
`models/so101/so101.xml`. Original CAD visual triangles and reference inertias stay
unchanged. CoACD uses a 1 mm concavity target, not a certified global surface-error
bound. Zero-thickness source surfaces remain visual-only. Nonadjacent self-contact,
cross-system contact and floor contact are native MuJoCo physics.
`collisions=False` uses MuJoCo's global contact-disable flag for diagnostics.
`include_contacts=False` skips force extraction. Reported contact forces are
simulation results, not physical force measurements.

On macOS, LeRobot's video/OpenCV dependencies can emit duplicate FFmpeg Objective-C
class warnings. They are not suppressed. Camera/video workflows are not validated
by this joint-state example.
