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
inputs, lets linked systems write their targets (an `SO101Leader` with
`follower=` sets that follower's controls), runs an optional update callback, validates **all** control snapshots,
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
runs after fresh input observations and leader-to-follower links, and before
command validation. Omit `steps` to
run until `driver.stop()` or interruption. Use `realtime=False` for accelerated
simulation. Hardware loops require real-time pacing. Physical devices continue on
their own clocks, and dispatch across systems is not an atomic transaction.

## Systems and native worlds

The base contract is in `airtruck/systems/base.py`: `attach`, `bind`, `reset`, `step`
and `observe`. Optional `link` resolves named references to other systems in the
same driver, and `drive` writes those systems' controls each tick before the
callback. `prepare_step` validates and snapshots a system's pending controls.
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
observations, `use_degrees=False`, and no LeRobot robot cameras. Simulated pinhole
cameras can be attached with `camera_specs` and rendered separately
(see the [yellow-cube dataset](#scripted-yellow-cube-dataset-simulation-only)).

Local keys are LeRobot keys such as `shoulder_pan.pos` and `gripper.pos`. Five body
channels use [-100,100]; the gripper uses [0,100]. The model's actuator `user` fields
store those same bounds. They map to the native actuator control ranges in radians.
Equal normalized travel does not establish matching real joint directions; physical
calibration must be checked before following.

A follower exposes `send_action()` and `get_observation()`. A leader exposes
`get_action()` and `send_feedback()`. `send_action()` applies a target immediately
and returns the target accepted by LeRobot, including native relative-target
clipping. It never advances the simulation clock. Use `controls` plus `driver.step()`
for coordinated ticks. `SO101Leader(..., follower=follower)`, given the follower
object or its system name, makes every tick copy the leader's pose into that
follower; see [hardware usage](hardware/README.md). A real leader remains an input
device: ordinary driver steps read it and never command its motors. Simulated leader controls pose its
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

`drone.controls` and `drone.state` are dictionaries typed by `DroneControls` and
`DroneState` (`airtruck/systems/drone.py`), so editors complete their keys and
type checkers reject unknown keys. Both use local NED: north, east, down, so
negative Z is height above the simulation world origin or PX4 estimator origin.

| Key | Meaning |
| --- | --- |
| `controls["position"]` | Target body origin `[north, east, down]` in meters (array or list) |
| `controls["yaw"]` | Target heading in radians, [-pi, pi]: 0 north, positive toward east |
| `state["position"]` | Measured body origin `[north, east, down]` in meters (`float32`) |
| `state["quaternion"]` | wxyz attitude rotating body FRD into NED; `yaw_angle()` gives heading |
| `state["linear_velocity_m_s"]` | Body-origin velocity `[north, east, down]` in m/s |
| `state["angular_velocity_rad_s"]` | Rotation rate about the NED axes (not body FRD) in rad/s |
| `state["armed"]`, `["offboard"]`, `["landed"]` | Measured flight flags, plain `bool` |

Targets are absolute, not relative to the current pose or terrain: to move 1 m
north, add `[1, 0, 0]` to the current target. The constructor's `position=` and
`quaternion=` still place a model in the MuJoCo Z-up world.
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

Simulation reports body-origin velocity even with an offset COM. Simulated
`landed` means supporting contact below 0.15 m/s; hardware derives it from PX4's
arming state and takeoff time (see
[flight command semantics](hardware/README.md#flight-command-semantics)).
Backend changes seed a hold at the observed pose. A simulated reset disarms and restores the initial model; a hardware reset holds measured position
without arming, homing, or changing its flight mode. A hardware-tracked proxy
converts the measured pose into the native Z-up world and uses MuJoCo constraints
between observations. Virtual contact forces are not sent back to real hardware.

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
free-body fixture and the Sentinel model (takeoff, hover, yaw and landing).
The default gains are underdamped; the hover tests allow ten simulated seconds
for takeoff settling and check both position error and low velocity. The
`drone_1m_in_front_tag` example adjusts XY targets from detections at a configured
0.5 m standoff, then requests landing. This is an illustrative example, not a
validated alignment or landing mission. The mixed arm/drone snippet above uses
the same public API without connecting hardware.

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
frames. Each `TagDetection` has the vehicle, camera, tag ID (None for a ROS pose
whose `frame_id` is not a decimal ID),
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
only by its central focal length. The tag snippet above demonstrates simulated
detection without a hardware connection.

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
`VehicleCommand`; it subscribes to odometry and vehicle status (the Sentinel's
PX4 1.14 bridge publishes no command ACK or land detector). It streams a measured
hold for at least 1.1 seconds before requesting offboard, and re-sends each command
until the measured mode/arming change confirms it. Mode/arming flags never mean
only "request sent".
Position-only messages set unused velocity/acceleration feed-forward fields to NaN.

The backend also subscribes to the onboard tag detector, as the earlier offboard
landing node did: `geometry_msgs/PoseStamped` on `/tag_detections` with sensor-data
QoS. Each pose is the tag in the camera optical frame (x right, y down, z forward),
attributed to `PX4Config.tag_camera` (default `front_down`); the topic and camera
are configurable. Every drone observation drains new poses into `TAGS` with NED
estimates, so detections update whenever the driver steps. VOXL's
`voxl_mpa_to_ros2` puts the decimal tag ID in `header.frame_id`; other strings give
`tag_id=None`. Which physical camera feeds the onboard detector is not verified.

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

## Gymnasium

The adapter remains available for policy integrations. The former ACT
record/train/rollout and gamepad example modules have been removed; this checkout
now includes the simulation-only yellow-cube dataset collector below, its
closed-loop simulation evaluation and a [hardware policy runner](hardware/README.md#smolvla-yellow-cube-policy-on-the-follower).
Training uses LeRobot's own CLI. LeRobot still owns native
arm communication, calibration and policy-training APIs.

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

## Captured table scene

`models/table-top/table-cleaned.glb` is the textured room/table scan with the
captured arm removed. The supplied `10_28_2026.zip` remains unchanged as a local,
Git-ignored source; its member hash is recorded, and the builder needs only the
cleaned GLB. Small
occluded sections of the white sheet, wall and underlying support surface are
reconstructed from neighboring scan geometry and colors; those repairs are
estimates. The rest of the scene retains the captured geometry and textures.
The scan retains its open boundaries and imperfections. Its geometry supplies the
visuals for `worlds/table_top/scene.xml`; contact uses a separate fitted tabletop
box and a floor plane. The wall, room clutter, table legs and other scanned
surfaces are visual only. Collision proxies and friction are approximate, not
measurements of the furniture.

The GLB preserves the original Y-up frame and nominal meter units. Its empty
`removed_arm_base_reference_approximate` node records the estimated base position
at `(-0.053, 0.687, -0.288)` m. Converting coordinates with
`(x, y, z) -> (x, -z, y)` gives `(-0.053, 0.288, 0.687)` m in Z-up coordinates.
This datum is the median of a manually selected pedestal surface region projected
onto the adjacent fitted tabletop, not a calibrated robot/CAD origin. Marker
axes belong to the scan frame; they do not establish the arm's orientation.
The original scan origin is retained, so this position is not relative to the
center of the table. Physical scale, calibrated base orientation, joint pose and
placement error bounds are unverified. Numeric values, source hash, selection
bounds and simulation assumptions are recorded in `models/table-top/information.json`.

### Leader on the scanned table

```sh
uv run --locked python -m airtruck.examples.table_leader --seconds 2
uv run --locked mjpython -m airtruck.examples.table_leader --render --seconds 300
```

The example creates only an `SO101Leader`, in simulation, with no serial connection
or linked follower. Its mounting datum is fixed at the recorded Z-up position;
the six arm joints remain movable through `leader.controls`. Base drift is
prevented by the model's fixed root body, not by servo gains or repeated pose
overwrites. The arm faces directly away from the straight rear wall, out over the
table (−25° yaw in the scan world), as specified by the user. The wall's outward
normal is taken from the solid reconstruction and transformed into the scan
frame; the base's up axis follows the fitted tabletop normal. Rotating the CAD
root preserves the recorded mounting datum at `(-0.053, 0.288, 0.687)` m.
This is the requested simulation orientation, not a calibration of the physical
mount or the captured shoulder-pan joint. Initial
joint positions are the existing model's zero configuration, not a recovered
capture pose. Both SO-101 roles currently share the existing follower CAD and
servo model; this is not a separately identified leader mechanism.

The world's invisible `leader_base` site stores the **CAD root** placement. The
`leader_mount_reference` site stores the recorded scan location. They differ
because the model datum is the shoulder-pan axis projected onto the base's
bottom support plane. `table_leader()` reads the root site directly from MJCF;
the engineering JSON is used only by the builder.

```python
from airtruck.examples.table_leader import WORLD, table_leader
from airtruck.runtime.driver import SystemDriver

leader = table_leader()
with SystemDriver([leader], world=WORLD) as driver:
    driver.reset()
    leader.controls["wrist_roll.pos"] += 15
    driver.run(steps=100, realtime=False)
```

`worlds/table_top/scene.xml` is the base world for `SystemDriver`.
`worlds/table_top/leader.xml` is a standalone MJCF with the same fixed leader
already attached, for direct MuJoCo loading; do not pass it to a driver that
also attaches a leader. Regenerate both worlds and their assets with:

```sh
uv run --locked python scripts/table_top/build_world.py
uv run --locked pytest -q airtruck/tests/test_table_world.py
```

The builder checks the cleaned GLB hash, converts Y-up to Z-up without rescaling,
and retains all 109,868 triangles in native MSH files. The original base-color
texture is decoded losslessly to PNG, and repaired vertex colors are baked into
continuous texture charts. Normal/occlusion/PBR maps are not imported into the
MuJoCo visualization. The required meshes and textures live under
`models/table-top/assets/`; `table.xml` contains the environment geometry and
placement sites. Changing a recorded placement or collision assumption requires
rebuilding. Tests verify mesh coordinates/UVs, agreement between standalone and
driver placements, unchanged base pose while joints move, and tabletop contact
with a dropped object.

### Solid Blender reconstruction

`models/table-top/scene.xml` provides the separate arm-free reconstruction made
from solid Blender objects. It retains full detail around the plywood platform
and work sheet, reduces surrounding furniture detail, and supplies textured OBJ
visuals plus 103 analytic collision shapes. The shelf openings remain clear.
`models/table-top/tabletop.blend` is the editable source; usage, contact limits
and export details are in the [model guide](../models/table-top/README.md).

The reconstruction uses a Z-up frame rotated +25° around Z relative to the scan's
Z-up frame. Its `removed_arm_mount` site is at (-0.160, 0.267, 0.689) m; this is the
modeled mounting-foot underside. The work sheet directly below it is recorded
separately at z = 0.69016313 m. These modeled datums and the original scan estimate
are kept separately in `information.json`. Physical scale and arm calibration
are unverified. Pass this scene as `world=` to use it as a static environment;
no robot is included.

`models/table-top/leader.xml` adds the articulated SO-101 to this reconstructed
scene. Its mounting datum sits on the work sheet at (-0.160, 0.267, 0.69016313) m,
with the CAD-root offset accounted for. Its forward axis points directly away
from the wall (world -Y in horizontal projection), while its base normal follows
the work sheet. The base is fixed and all six native joints remain movable.
This user-selected orientation is separate from the captured arm's estimated
yaw. Load `leader.xml` directly with MuJoCo; it already contains the arm.

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

## Scripted yellow-cube dataset (simulation only)

```sh
uv run --locked python -m airtruck.examples.so101_pick --collect
uv run --locked python -m airtruck.examples.so101_pick --verify

# Robust dataset (separate output; original remains untouched):
uv run --locked python -m airtruck.examples.so101_pick --collect --robust --episodes 50 --noise 4 --repo-id local/so101_sim_yellow_cube_pick_50
uv run --locked python -m airtruck.examples.so101_pick --verify --repo-id local/so101_sim_yellow_cube_pick_50
```

The collector uses `SO101Follower` and `SystemDriver`; it has no hardware option.
The workstation is self-contained in the repository. `models/so101-workstation/source/`
holds the scaled scan `table-scaled.glb`, the camera-frame scan `scan-cleaned.glb`
(the source of the training front-camera intrinsics) and `registration.json`: the
scan-refined camera pose, fitted table corners and arm base pose in the scan's glTF
frame, plus their Z-up conversions. The collector regenerates
`worlds/so101_pick/scene.xml`, the tracked textured meshes under
`models/so101-workstation/assets/` and the numeric records from these files; with
unchanged inputs the output is byte-identical, so a fresh clone needs no regeneration.
The scan is visual-only; the fitted table rectangle has a 30 mm thick analytic
collision box. Source hash, SI dimensions and poses are recorded in
`models/so101-workstation/information.json`. The scan capture and registration tools
that produced these inputs are not part of the project.

The transformation from glTF to MuJoCo is `(x, y, z) → (x, -z, y)`. The training
front camera (`FRONT_TRAINING`) uses the exact transformed
`SCAN_REFINED_T_SCALED_MODEL_FROM_CAMERA` from `registration.json`, with 640×480 RGB and a 63.97° vertical
FOV (about 79.5° horizontal). This is the RealSense depth origin, with color aligned
to depth. Camera-frame vertex extents in `scan-cleaned.glb` give fx ≈384.8 px and
fy ≈384.3 px; MuJoCo uses square pixels and the specified vertical FOV
(f ≈384.35 px). The default front view is the colour sensor instead; see
[measured cameras](#measured-cameras).
The pan axis is projected from its CAD pivot to the CAD mounting plane before
placing that intersection at `ARM_BASE_POSITION_ZUP`; subtracting the entire
64.5 mm pivot height would bury the base. The corrected framing includes the
sheet, plywood edges, wall and arm near the top-center, with the grasp region visible.

The expert solves bounded numerical inverse kinematics in a separate scratch
`MjData`, then sends smooth cubic joint targets through the follower. Recording
never writes simulated joint/object positions: pose assignment happens only at
reset. Physics runs at 1/3000 s, with 100 substeps per 30 Hz frame. Existing arm
geometry, position gains and force limits are unchanged. Hand and jaw contact
friction is assumed `[2.5, .02, .002]` with six-dimensional contacts.

The default legacy trajectory lasts 10 s (300 frames): reach for 2 s while opening the gripper,
descend for 2.5 s, close for 1.5 s, lift for 2.5 s, and hold for 1.5 s.
Every episode starts from the cube-independent ready pose
`[0, 0.4, -0.8, 0.2, 0, -1.0]` radians in JOINTS order (pan, lift, elbow,
wrist flex, wrist roll, gripper), with seeded ±2.5° jitter on the first five
joints. The nominal gripper tip is 28.6 cm above the mounting plane.
Ten deterministically permuted strata (placement seed 42) span X ±70 mm,
Y −280 to −160 mm forward of the base, and yaw ±60°, including the boundaries.
Equivalent 180° grasp orientations are solved first; among solutions with less
than 5 mm combined position/orientation residual, the smallest wrist roll is
selected. Failures retry the same placement with a new ready-pose jitter seed;
collection aborts after ten times the requested episode count in attempts.

`--episodes N` selects the number of deterministic, independently permuted strata
per axis (seed 42, including the same boundaries). `--repo-id` selects the dataset
identity and cache path; `--root PATH` overrides that path. Existing roots are
refused before preparation. Without `--collect`, the requested trials run physics
without recording. Defaults preserve the original ten placements and trajectory.

`--robust` uses 375 frames (12.5 s): 2 s reach to a pre-grasp, 1 s
horizontal correction, 1 s coarse descent, 2.5 s final 46 mm descent, 0.5 s
pause, 1.5 s close, 2.5 s lift and 1.5 s hold. The approach gripper target is
95/100 (0.07854 rad), versus approximately 60/100 in the legacy trajectory.
For three episodes in each ten, the initial pre-grasp misses horizontally by
a seeded 10–20 mm and then corrects at 100 mm above the cube center. Those
waypoints are expert targets and remain in the action labels.

Four episodes in each five receive DART-style disturbances on joints 0–4:
the mean of two seeded sinusoids per joint at 0.18–0.45 Hz, bounded by
`--noise` normalized units (default 4). The envelope rises smoothly over 0.5 s
and fades over 4.5–6 s, leaving the final 0.5 s descent and pause unperturbed.
Commands are clipped to actuator limits. Recorded actions are the clean expert
targets; observations and both cameras reflect the perturbed executed state.
Gripper commands receive no noise. Thus the 50-episode recipe has 40 noisy,
10 clean and 15 offset/correction episodes. This supplies corrective labels;
closed-loop learned-policy robustness still requires a new training evaluation.

The yellow object's box collision approximation measures 34.925×28.575×34.925 mm; its assumed
mass is 0.02 kg, with measured mass unknown. The visual mesh has assumed 0.8 mm
rounded edges and glossy plastic material; collision remains a box.
The training wrist view (`WRIST_TRAINING`) simulates the user-confirmed Seeed Studio
ET-S231 V1 at 640×480. The listing's 120° diagonal FOV over 1920×1080 gives
f ≈635.9 px; assuming a 1440×1080 center crop resized to 640×480 gives fy ≈282.6 px.
The specified rounded vertical FOV is 80.7° (MuJoCo fy ≈282.5 px). The real
640×480 view is narrower: 59.29° vertical ([measured cameras](#measured-cameras)).
Both wrist views are ideal pinholes without distortion, manual-focus effects or
sensor noise.

The official mount STL has four 2 mm holes on a 27×27 mm pattern.
Their seat-side centers average to (-47.573754, -34.308910, 17.5) mm
in the source frame, with outward normal (sin 25°, cos 25°, 0).
Applying the recorded source-to-assembly transform and inverse compiled
zero-pose hand transform gives the hand-frame seat. The optical center is
10 mm outward (assumed board/lens stack), at
(0.06729377, 0.00367513, 0.00755426) m; quaternion wxyz is
(0.15170432, 0.68429462, 0.69634400, 0.15437520).
Camera +Y follows the board's in-plane (-cos 25°, sin 25°, 0) direction:
the jaw tips project below the optical axis. The silhouette fit under
[measured cameras](#measured-cameras) agrees with this pose and roll within
4 mm and 1°. The pinned arm still omits
the bracket/board geometry; the sensor adds no mass, inertia or contacts.
Rest looks away from the table; the cube enters view during approach.

Reusable `CameraSpec(name, body, pos, quat, width, height, fovy, focal_px,
principal_px)` lives in `airtruck.systems.camera`. Poses use meters and MuJoCo wxyz
rotation (+X right, +Y up, -Z viewing direction). `fovy` alone gives a centred
pinhole with square pixels. Measured intrinsics go in `focal_px` (fx, fy) and
`principal_px` (cx, cy, OpenCV pixel convention), both or neither. MuJoCo then
ignores `fovy` and uses `sensor_size`, `focal_length` and `principal_length` at a
nominal 10 µm pixel pitch. MuJoCo measures the principal offset from the image
centre with the opposite sign, so `attach()` stores ((width−1)/2 − cx,
(height−1)/2 − cy) times the pitch. Pass `camera_specs=(spec,)` to an
SO-101 arm; its `make_spec()` attaches the cameras before driver namespacing
(e.g. `arm/wrist`). `spec.attach(world_spec)` also supports a world camera.
The collector's `CAMERAS` tuple, chosen from `CAMERA_SETS` by `SO101_SIM_CAMERAS`,
drives image features and one renderer per camera resolution. Edit `CAMERA_SETS`
to swap a model/pose/intrinsics; add arm-mounted specs to `camera_specs` too.
The collector itself never opens
physical cameras; [hardware policy runs](hardware/README.md#smolvla-yellow-cube-policy-on-the-follower)
read the physical front and wrist feeds through LeRobot cameras or the RealSense bridge.

Only episodes ending with the cube at least 50 mm clear of the table, contacting
both fixed and moving jaws and without table contact are saved. Failed buffers
are discarded. By default ten successes are finalized in
`~/.cache/huggingface/lerobot/local/so101_sim_yellow_cube_pick`; existing output is
never overwritten. `collection_results.json` records all attempted seeds and
placements, initial/grasp joints, frame counts, saved episode indices and
end-state evidence, noise bounds, actual applied noise peaks, recovery offsets,
and any hand/jaw contacts with cube/table before closing. No Hub upload or training is performed.

LeRobot's standard feature/frame helpers produce `observation.state` and `action`
(six normalized `.pos` channels: five in −100…100, gripper in 0…100), plus
`observation.images.front` and `observation.images.wrist` videos. Every frame has task “Pick up the yellow cube.”
The installed LeRobot API accepts the task as the frame's `task` key. These are
standard fine-tuning inputs for `lerobot/smolvla_base`; configure both image
mappings when training. Simulation success does not establish hardware transfer.
The verifier reloads all state/action rows, checks counts against saved collection
metadata, normalization ranges, fps and camera features, and decodes seven samples
from both cameras of every episode. It reports within-episode maximum per-step
action changes. The sheet alternates front/wrist rows for three noisy episodes
and one clean episode at start, mid-reach, missed pre-grasp, corrected
pre-grasp, final descent, closed grasp and lifted hold
(or up to four legacy episodes). It writes
`verification.json` and `verification.png` inside the dataset directory. Collection
creates no debug PNGs or temporary preview files.

The 50-episode robust collection completed in 50 attempts (18,750 frames).
All final cube clearances were 77.3–103.8 mm, with both jaw bodies contacting
and no final table contact. No hand/jaw table contact was detected before
closing; seven episodes had cube contact during descent. A detailed first-episode
check found fixed-jaw contact at 5.17 s and moving-jaw contact at 8.03 s, during
closing. Contact auditing samples at the 30 Hz control boundary.
The maximum within-episode action step was 3.001 normalized units (gripper);
the maximum across joints 0–4 was 2.514. Both camera streams decoded at seven
sample times in every episode. The saved sheet shows three noisy recovery episodes
and one clean episode.

### Measured cameras

`so101_pick` defines two camera sets; `so101_pick_eval` imports the same `CAMERAS`.
The default `measured` set (`FRONT_MEASURED`, `WRIST_MEASURED`) matches the physical
cameras. `SO101_SIM_CAMERAS=training` restores `FRONT_TRAINING` and `WRIST_TRAINING`,
the views of both datasets above and of every checkpoint trained on them. Any other
value raises `ValueError`.

```sh
SO101_SIM_CAMERAS=training uv run --locked python -m airtruck.examples.so101_pick_eval \
  path/to/pretrained_model --episodes 20 --seconds 18 --out /tmp/so101_eval
```

The physical front feed is the D435i colour sensor (through the
[RealSense bridge](hardware/README.md#smolvla-yellow-cube-policy-on-the-follower)), not
the depth origin of the training view. Its factory calibration, read with
`rs-enumerate-devices -c`, gives fx, fy = 614.418, 614.862 px and principal point
cx, cy = 328.009, 228.644 px at 640×480: a 55.02° × 42.63° FOV. `FRONT_MEASURED`
uses these intrinsics through `focal_px`/`principal_px`. Its pose is the
scan-refined depth pose with the factory depth-to-colour extrinsic applied:
translation (0.014605, 0.000223, 0.000328) m and a rotation of about 0.76°. A final
static-edge refinement then moved it 5.6 mm along the optical axis and about 0.1°: it
minimized the distance from the rendered table and board edges (arm masked out) to real
colour-frame edges. Hence the stored pose is 15.7 mm and 0.81° from the depth pose rather
than 14.6 mm. Static table and board edges in a render agree with a real colour frame
within a few pixels.

The wrist pose and vertical FOV were fitted to the real jaw silhouettes in wrist
frames of a recorded hardware run, with the gripper open and closed (IoU 0.865).
`WRIST_MEASURED` is at (0.070972, 0.002144, 0.008020) m in the hand frame, with
quaternion wxyz (0.156794, 0.683003, 0.695009, 0.160887) and a 59.29° vertical FOV.
The pose is within 4 mm and 1° of the mount-derived training pose; the assumed
80.7° FOV was the real error. A distortion-free pinhole matched the silhouettes, so
distortion is not modelled. Both values are also in the numeric records
(`models/so101-workstation/information.json`, `models/so101/information.json`).

All existing checkpoints were trained on the training views and see shifted,
zoomed-in images under the measured set: the front view is about 21° narrower
vertically and offset to the colour sensor; the wrist view is about 21° narrower.
The closed-loop results below used the training views; re-run with
`SO101_SIM_CAMERAS=training` to reproduce them. `--collect` now records measured
views unless the variable is set, so new datasets differ from the existing ones.

### SmolVLA fine-tuning

Fine-tune `lerobot/smolvla_base` (the `smolvla` extra is in `pyproject.toml`)
from the repository root, with `DEVICE=mps` on Apple silicon or `DEVICE=cuda` on
Linux with an NVIDIA GPU ([setup](README.md#linux-and-nvidia-gpus)). The MPS
fallback variable has no effect on CUDA. The base checkpoint expects
`camera1`–`camera3`, so map the dataset cameras; the unused third view is
skipped:

```sh
DEVICE=mps  # or cuda
PYTORCH_ENABLE_MPS_FALLBACK=1 uv run --locked lerobot-train \
  --policy.path=lerobot/smolvla_base --policy.device=$DEVICE --policy.push_to_hub=false \
  --dataset.repo_id=local/so101_sim_yellow_cube_pick_50 \
  --dataset.root=$HOME/.cache/huggingface/lerobot/local/so101_sim_yellow_cube_pick_50 \
  --dataset.video_backend=pyav --dataset.image_transforms.enable=true \
  --rename_map='{"observation.images.front":"observation.images.camera1","observation.images.wrist":"observation.images.camera2"}' \
  --batch_size=32 --steps=6000 --save_freq=1000 --wandb.enable=false \
  --output_dir=$HOME/.cache/huggingface/lerobot/train/smolvla_so101_sim_yellow_cube_50
```

`pyav` is required because torchcodec cannot load without a system FFmpeg.
On an M3 Max this ran at about 4.3 s/step (7.2 h; final loss 0.017). CUDA training
has not been profiled here. If batch 32 does not fit in an 11 GB RTX 2080 Ti, halve
`--batch_size` until it does; a smaller batch changes the optimisation, so results are
not directly comparable with this run. Each
checkpoint is about 0.9 GB of weights plus 0.4 GB of optimizer state.
The saved preprocessor applies the camera renaming, so inference passes
`observation.images.front`/`wrist` unchanged. Clip predicted actions to
−100…100 (gripper 0…100): the policy occasionally exceeds the joint range.

Evaluate a saved `pretrained_model` folder closed-loop in the same world:

```sh
PYTORCH_ENABLE_MPS_FALLBACK=1 uv run --locked python -m airtruck.examples.so101_pick_eval \
  path/to/pretrained_model --episodes 20 --seconds 18 --videos 3 --out /tmp/so101_eval
```

`--out` must be writable and is created if needed; choose a location outside the repository.
`--device` selects the torch device (default CUDA, then MPS, then CPU). Headless Linux
rendering needs `MUJOCO_GL=egl`.
It writes `eval.json` and H.264 front|wrist videos (`rollout_N.mp4`,
`rollouts.mp4`). `--train-placements` reuses the collector's stratified placements.
Each rollout reports holding for at least 1 s, holding at 12.5 s, and holding at the end.

Closed-loop evaluation used the training camera views, fresh uniformly sampled
placements in the same region, the same rest-pose jitter and success test, and 18 s
rollouts.
Step 6000 picked and held the cube at least 1 s in 10/20 rollouts, also
10/20 at 12.5 s (the demonstration length); step 5000 achieved 8/20.
After 12.5 s, states are outside the demonstrations and held cubes are often released.
The 10-episode dataset fitted its demonstrations offline yet failed closed-loop
(0/5, including its own placements). Replaying demonstrations with σ = 2
normalized-unit joint offsets succeeded only 6/10, so recovery data and the
wider grasp were added. The main remaining failure is closing about 8–14 mm
before full descent. These are simulation results only.
