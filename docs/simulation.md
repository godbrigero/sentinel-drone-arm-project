# System objects and the shared system driver

Each `SimSystem` owns its model binding, control schema, observations and optional
hardware connection. `SystemDriver` (`airtruck/runtime/driver.py`) runs every system,
simulated or hardware, on one control clock. It owns one MuJoCo model/data pair:
simulated systems are physics there, and hardware systems appear as proxies that
track their measured state. It does not interpret arm joints, drone forces, units
or controller types.

```python
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.so101 import SO101Follower

left = SO101Follower("left", position=(-0.25, 0, 0))
right = SO101Follower("right", position=(0.25, 0, 0))

with SystemDriver([left, right], control_dt=0.02) as driver:
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
with SystemDriver([left, right]) as driver:
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
their own clocks, and dispatch across systems is not an atomic transaction.

## Systems and native worlds

The base contract is in `airtruck/systems/base.py`: `attach`, `bind`, `reset`, `step`
and `observe`. `prepare_step` validates and snapshots a system's pending controls.
A system's `step(dt)` applies its local inputs; it must never call `mujoco.mj_step`.
New system types implement their own action and observation spaces. Scalar and
array controls can coexist; the driver contains no fixed channel counts.

Each object supplies its prepared MJCF model and placement. The driver attaches
its root body under the object's name using native `MjSpec` frames, then compiles
one world. `world=` accepts a base MJCF template without those systems, or an
in-memory `mujoco.MjSpec` (copied, so the caller's specification is unchanged; used
for generated AprilTag textures). The default is `worlds/bench/scene.xml`. Model geometry and collision preparation remain in the
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

## Drone position control and flight commands

`SentinelDrone` is the aircraft object, used like `SO101Follower`: one class whose
`hardware` flag selects the backend, with identical controls, state, flight
commands, cameras and tag detections in both. `hardware=False` (the default) flies
the built-in MuJoCo model `models/sentinel-quad/sentinel.xml`, built from the
Onshape assembly (see [Sentinel CAD](sentinel/README.md#simulation-model));
`hardware=True` flies the real aircraft through the PX4 ROS 2 backend. The flag
can also be changed later and takes effect at the next driver tick. Connecting or
switching backends never arms the aircraft or selects offboard mode. The default
placement `position=(0, 0, SPAWN_HEIGHT_M)` (0.146 m) stands it on its feet on a
floor at z = 0.

```python
from airtruck.systems.sentinel import SPAWN_HEIGHT_M, SentinelDrone

drone = SentinelDrone("drone")                  # Simulation.
drone = SentinelDrone("drone", hardware=True)   # The real aircraft via PX4.
```

`Drone` in `airtruck/systems/drone.py` is the generic class underneath, for any
prepared drone MJCF (`Drone("quad", model=path, position=...)`). Both provide
`connect()`, `disconnect()`, `is_connected`, `get_observation()` and
`send_action(action)`, with the same meaning as on the arms: `connect()` opens the
selected backend at once (otherwise the driver opens it on entry), and
`send_action()` applies a position/yaw target without advancing the clock.

```python
arm = SO101Follower("arm")
drone = SentinelDrone("drone", position=(0.6, 0, SPAWN_HEIGHT_M))

with SystemDriver([arm, drone]) as driver:
    driver.reset()
    drone.takeoff(0.5)
    driver.run(steps=250, realtime=drone.hardware)
    drone.controls["position"] = [0.6, 0.2, -0.8]
    drone.controls["yaw"] = 0.3
    arm.controls["shoulder_pan.pos"] = 20
    driver.run(steps=250, realtime=drone.hardware)
    drone.land()
    driver.run(steps=300, realtime=drone.hardware)
```

Controls are `position` (three absolute local NED meters) and `yaw` (radians in
[-pi, pi]). NED means north, east, down: negative Z requests height above the
estimator's local origin. Positive yaw turns from north toward east. Positions
are not automatically relative to the current pose or terrain. The constructor's
`position=` and `quaternion=` still place a model in the MuJoCo Z-up world.
For drone conversion, world axes are north/west/up and the model body is
forward/left/up: NED `[x, y, z]` maps to world `[x, -y, -z]`. Arm units are unchanged.

The flight methods do not advance physics:

- `engage_offboard_mode()` selects position control, holding the pending target.
- `arm()` requests arming; `disarm()` requests disarming without force-override flags.
- `takeoff(height)` holds measured XY/yaw, selects offboard, arms, and requests a
  climb by that many meters from the measured position. It returns after setting
  the target, not after reaching the requested height.
- `hold()` captures measured position/yaw and resumes offboard if already armed.
- `land()` requests PX4's actual landing mode. Simulation descends at a bounded
  speed, detects upward supporting native contacts and low speed, then disarms
  after one second of sustained ground contact. Landing is asynchronous: inspect
  `state["landed"]` and `state["armed"]` while continuing to step the driver.

State contains NED position, a wxyz body-FRD-to-NED quaternion, NED linear and
angular velocities, and measured `armed`, `offboard`, `landed` flags. Simulation
reports body-origin velocity even with an offset COM. Backend changes seed a hold
at the observed pose. A simulated reset disarms and restores the initial model;
a hardware reset holds measured position without arming, homing, or changing its
flight mode. A hardware-tracked proxy converts the measured pose into the native
Z-up world and uses MuJoCo constraints between observations. Virtual contact forces
are not sent back to real hardware.

### Simulation controller and model contract

`FlightController` in `airtruck/systems/flight.py` implements bounded position/velocity
and attitude/rate feedback with gravity compensation. It converts targets to the
model's four native wrench actuators once per driver tick; MuJoCo integrates the
result and computes gravity, collisions and contact forces over the physics
substeps. Simulation flight never assigns qpos to reach a commanded target.
Unarmed drones receive zero motor inputs and fall under gravity.

The model root needs one free joint, assigned inertia, and stateless, unit-gain,
bias-free site motor actuators named `thrust_n`, `roll_torque_nm`,
`pitch_torque_nm`, `yaw_torque_nm`. Their gears are respectively `0 0 1 0 0 0`,
`0 0 0 1 0 0`, `0 0 0 0 1 0`, `0 0 0 0 0 1`, with finite limited control ranges.
They share a body-aligned site at the root COM. These wrench channels are internal
controller outputs, not policy actions. Use an aggregate rigid-body mass/inertia
appropriate to the supplied aircraft and enough actuator authority to support it.
Manufacturer reference aircraft weight does not establish assigned CAD inertia.

Two optional features are discovered by name. Velocity actuators named `rotor_*`
on hinge joints of root-child bodies spin propellers: each tick, the applied wrench
is allocated to per-rotor thrust from each rotor's position, axis and spin, and the
propeller target speed is `sqrt(thrust / k_T)`. The actuator `user` fields hold
`k_T k_Q` (N/(rad/s)² and N·m/(rad/s)²); a control range of `[-max, 0]` or
`[0, max]` fixes the spin direction (positive is counter-clockwise about the motor
axis toward the shaft tip). `drone.rotor_speeds_rad_s` reports measured signed
speeds. Propeller spin is visual: propellers carry their mass but negligible spin
inertia, so it does not feed back into the aggregate wrench flight model. Hardware
proxies spin armed propellers at hover speed. Cameras on the root body become
`drone.cameras`; see [onboard cameras and AprilTags](#onboard-cameras-and-apriltags).

The controller gains and response limits are editable through `controller=FlightController(...)`.
They are illustrative simulation settings: matching the command API does not make
this controller dynamically identical to PX4 firmware or the physical aircraft.
The default controller is tested at a 0.02 s control period with the analytic
free-body fixture and the Sentinel model (takeoff, hover, yaw and landing). No
automatic tag alignment or landing mission is implemented.
`airtruck.examples.mixed_systems --height 0.5` runs the same flight API with the
Sentinel model beside an arm, without connecting hardware; `--drone-model` selects
another prepared model.

### Onboard cameras and AprilTags

The Sentinel model has two cameras from the CAD optical frames: `front_down`
(tracking camera, front center, about 46° below forward) and `rear_left` (rear
stereo left camera, looking straight back). Both are 640×480 pinhole cameras;
their focal lengths and the reference lens data are in the model's
`information.json`. `drone.cameras[name]` holds `intrinsics` (OpenCV K),
`rotation`/`position_m` (optical frame x right, y down, z forward, into body FRD),
and the latest `frame` with `timestamp_s`.

```python
image = drone.capture("front_down")  # Simulation: renders the current state now.
```

In simulation `capture()` renders with MuJoCo's offscreen renderer;
`camera_rate_hz=` additionally refreshes every camera at that simulated rate. On
hardware nothing is rendered: publish frames from any source with
`drone.cameras["front_down"].publish(image, timestamp_s)`, and `capture()` returns
the latest published frame. Processing code therefore reads the same objects in
both modes.

Tag detections go to `airtruck.systems.apriltag.TAGS`, a process-wide, thread-safe
board, and are available from any module:

```python
from airtruck.systems.apriltag import TAGS, AprilTagConfig, add_apriltag

world = mujoco.MjSpec.from_file(str(ROOT / "worlds/flight_arena/scene.xml"))
add_apriltag(world, 0, size_m=0.3, position=(1.4, 0, 0.0011))  # Flat, facing up.
drone = SentinelDrone("drone", apriltags=AprilTagConfig(tag_size_m=0.3))
with SystemDriver([drone], world=world) as driver:
    driver.reset()
    drone.takeoff(1.0)
    driver.run(steps=300, realtime=False)
    tag = TAGS.latest(vehicle="drone", tag_id=0)  # Or drone.latest_tag(tag_id=0).
    print(tag.position_m, tag.position_ned_m)
```

Image detection is opt-in (`apriltags=None` by default). When enabled it runs at
`rate_hz` (default 10 Hz) on the configured cameras (all by default), using
OpenCV's AprilTag dictionaries (`tag36h11` default; `tag16h5`, `tag25h9`,
`tag36h10`) and square-target PnP. `tag_size_m` is the black-border edge.
On hardware, PX4 tag messages are always ingested (see the
[PX4 backend](#px4-ros-2-backend)), and enabled image detection runs on published
frames. Each `TagDetection` has the vehicle, camera, tag ID (None for ROS poses),
the tag pose in the camera optical frame (`position_m`, wxyz `quaternion`), the
timestamp, its source (`simulation`, `camera` or `ros`), and NED tag position and
orientation computed from the camera mounting and the vehicle state at that tick.
The tag frame has x right and y up in the printed image, with z out of its face.
`TAGS.subscribe(callback)` delivers every new detection; simulated resets clear
that vehicle's detections.

`add_apriltag()` adds a static textured tag to a world specification; its face
points along the frame's +Z and the image's top along +Y. In the test scenes the
simulated estimate is within about 1 cm at 1.2 m and 3–4 cm at 1.8 m oblique range.
Simulated cameras use ideal pinhole projection without lens distortion, noise,
motion blur or exposure effects, and the 166° fisheye tracking lens is represented
only by its central focal length. `airtruck.examples.apriltag_detection` hovers
near a tag and prints detections; the gamepad demo accepts `--apriltag`.

#### Movable SVG tag object

`AprilTagObject` (`airtruck/systems/tag_object.py`) is a simulation-only system
textured from a tag SVG; the default is `models/apriltag/tag36h11-2.svg`
(tag36h11 ID 2). Pass it to the driver like any other system and move it while
simulating:

```python
from airtruck.systems.tag_object import AprilTagObject

tag = AprilTagObject("tag", position=(1.4, 0, 0.0011))  # Flat, facing up.
with SystemDriver([drone, tag], world=ROOT / "worlds/flight_arena/scene.xml") as driver:
    driver.reset()
    tag.move((0.2, 0, 0))                       # Delta, world meters.
    tag.move((0, 0.1, 0), frame="local")        # Delta along the tag's own axes.
    tag.set_position((0.5, 0.5, 0.0011))        # Absolute.
    tag.rotate(rpy=(0, 0, 30), degrees=True)    # Delta about its own axes.
    tag.rotate(rpy=(0.2, 0, 0), frame="world")  # Delta about world axes.
    tag.set_rotation(rpy=(0, 0, 0))             # Absolute; or a wxyz quaternion.
    tag.set_pose((1, 0, 0.5), (1, 0, 0, 0))
    driver.step()
    print(tag.state["position"], tag.state["quaternion"], tag.pose)
```

Positions are MuJoCo world meters (Z up); rotations take either a wxyz quaternion
or fixed-axis roll/pitch/yaw (`rpy`, radians unless `degrees=True`). Rotations turn
about the face center. The face points along +Z with the printed image's top
along +Y. Edits write MuJoCo's mocap pose immediately, so the tag teleports
rather than being pushed by forces; before joining a driver they change its
initial placement, and `driver.reset()` restores that placement. `size_m` is the
black-border edge and defaults to the SVG's physical size (80 mm border on a
100 mm plate); the quiet zone is kept around it. The plate does not collide unless
`collisions=True`, which makes it an immovable surface. A blank backing hides the
printed skin's mirrored rear face, so the tag cannot be decoded from behind.
The object has observations (its pose) but no Gymnasium actions or hardware mode.

The SVG loader supports flat-filled, straight-segment paths (M/L/H/V/Z) as
produced by AprilTag image generators. This official image is OpenCV's
`DICT_APRILTAG_36h11` marker 2 rotated 180°: OpenCV decodes it as ID 2 with the
correct position, but its tag x/y axes point opposite the printed image's
right/up, so orientations from the image detector differ by 180° about the face
normal from this object's pose.

### PX4 ROS 2 backend

See [ROS installation and hardware configuration](hardware/README.md#px4-drone).
`PX4Backend` publishes `OffboardControlMode`, `TrajectorySetpoint` and
`VehicleCommand`; it subscribes to odometry, vehicle status, land detection and
command acknowledgements. It streams a measured hold for at least 1.1 seconds
before requesting offboard, and waits for both command acceptance and the measured
mode/arming change. Mode/arming flags never mean only "request sent".
Position-only messages set unused velocity/acceleration feed-forward fields to NaN.

The backend also subscribes to the onboard tag detector, as the earlier offboard
landing node did: `geometry_msgs/PoseStamped` on `/tag_detections` with sensor-data
QoS. Each pose is the tag in the camera optical frame (x right, y down, z forward),
attributed to `PX4Config.tag_camera` (default `front_down`); the topic and camera
are configurable. Every drone observation drains new poses into `TAGS` with NED
estimates, so detections update whenever the driver steps. The message carries no
tag ID. Which physical camera feeds the onboard detector is not verified.

Each hardware backend owns a ROS context and executor. Driver reads and bounded
flight-command waits pump that executor; there is no background flight thread and
no shutdown of another application's ROS context. Run the hardware driver in real
time at the default 50 Hz rate. Its ROS timer sends setpoints/heartbeats at 20 Hz.
Fresh telemetry is required; stale data stops the heartbeat, and estimator-origin
resets require reconnecting rather than reusing an old absolute target. Closing
only stops communication; it does not silently land or disarm a flying aircraft.
PX4's configured offboard-loss behavior applies if communication stops.

Gymnasium packs four drone action numbers and sixteen observation numbers using
the same schemas in either backend. For simulation training, a task wrapper's
`reset()` should explicitly request takeoff/arming after resetting the base
environment; base resets never auto-arm. Hardware episode resets must be managed
explicitly rather than running accelerated vector workers against a real aircraft.
Absolute position/velocity spaces are unbounded, while the API rejects nonfinite
values. Gymnasium's checker therefore emits range/normalization advice. A task
wrapper can bound its workspace and normalize policy inputs/actions for training;
LeRobot's dataset statistics still own policy normalization.

## Gymnasium and ACT

The interactive controller example is available with:

```sh
uv run --locked mjpython -m airtruck.examples.gamepad_flight
uv run --locked mjpython -m airtruck.examples.gamepad_flight --apriltag
uv run --locked mjpython -m airtruck.examples.gamepad_flight --model models/demo_quad/quad.xml --spawn-height 0.11
```

It uses the Sentinel model and `worlds/flight_arena/scene.xml`. `--apriltag` places
tag36h11 #0 1.5 m north of the pad, enables detection on both cameras and shows the
latest detection; `[` and `]` cycle the viewer through the onboard cameras. SDL's standard game-controller mapping reads
the Xbox pad without creating another visible window. Left stick controls climb
and yaw, and right stick moves forward/sideways relative to the current heading.
A takes off, B lands, X holds, Y resets, and Menu closes the demo. Releasing the
sticks leaves a held position target; disconnecting the controller captures a
measured hold. Targets are bounded to the demonstration arena with a small
tracking leash. `--model` can select another model using the prepared drone
contract, such as the analytic `models/demo_quad/quad.xml`. The demo always flies
in simulation.

Both models' mass/inertia are assigned simulation settings recorded in their own
`information.json`; they are not CAD or measured Sentinel properties.

```python
with SystemDriver([left, right]) as driver:
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
The example requires a world file; it rejects an in-memory `MjSpec` world.

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
