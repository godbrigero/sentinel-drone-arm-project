# Hardware control: SO-101 and PX4

Use the same `SO101Follower` or `SO101Leader` object for both backends. These classes
inherit the corresponding native LeRobot class and share `SO101Arm` simulation
bindings. No separate hardware controller hierarchy or serial protocol is added.

```python
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.so101 import SO101Follower, SO101Leader

leader = SO101Leader("leader", hardware=True, port="/dev/YOUR_LEADER",
                     id="airtruck_leader", position=(-0.25, 0, 0),
                     follower="follower")
follower = SO101Follower("follower", hardware=False, port="/dev/YOUR_FOLLOWER",
                         id="airtruck_follower", position=(0.25, 0, 0))

with SystemDriver([leader, follower]) as driver:
    driver.reset()
    driver.run(realtime=True)
```

`follower=` is the `SO101Follower` that the leader teleoperates: the object itself
(`follower=follower`) or its system name (`follower="follower"`; not a port or
LeRobot ID). The driver resolves it when it is constructed and raises `ValueError`
if that follower is not in the same driver. Each tick it
copies the leader's freshly measured pose into the follower's `controls` before the
`update` callback, so a callback can still override individual targets. Omit
`follower=` to use the leader only as an input. On start, a hardware follower
moves directly to the leader's pose, so pose the leader near the follower first.

Set `follower.hardware=True` to use the physical follower. Setting the flag after
creation applies the transition at the next driver tick; `hardware_active` reports
the connected backend. Keep a port configured if you intend to switch later.
Hardware failures raise errors; there is no silent fallback to simulation.

Both backends use local LeRobot keys such as `shoulder_pan.pos`, five body channels
in [-100,100], and gripper in [0,100]. `controls` holds desired targets; `state`
holds measured feedback. `send_action()` retains immediate target dispatch and
returns LeRobot's accepted targets if relative-target limiting clips the request.
It does not advance the simulation clock. `get_observation()` and leader
`get_action()` work through the current backend. A real leader stays input-only in
the driver loop. The bridge currently requires `use_degrees=False` and no LeRobot
robot cameras; `so101_policy` opens its cameras separately.
Native config objects may be passed with `config=` for SDK options such as PID
coefficients and `max_relative_target`.

## Connection, switching and reset

Constructing an object creates no serial connection. The driver context connects
hardware-enabled systems. Explicit `connect()` also supports hardware-only use
without a simulation driver. SDK compatibility does not make a simulator advance
automatically: a simulation still needs driver ticks or a running driver loop.

Normal connection requires a saved LeRobot calibration that matches the motors.
The follower disables torque, reads raw positions, rejects positions outside the
calibrated range, and queues the current raw pose before native SDK configuration
enables position control. Normal connection never starts interactive calibration.
LeRobot's startup writes have no retries, so a serial `ConnectionError` during
connection (for example "There is no status packet") releases the arm and reruns
the whole sequence, up to `CONNECT_ATTEMPTS` (3) times, logging a warning on each
retry. Runtime position reads use LeRobot's own `num_read_retries`. Calibration is
never retried. Repeated failures usually mean a loose servo cable or unstable
supply. Virtual arms report
`is_calibrated=True` from their prepared model limits; `configure()` and
`calibrate()` perform no hardware I/O in simulation. Explicit hardware
`connect(calibrate=True)` may run LeRobot's interactive calibration.

On activation, the bridge reads and holds the measured pose. `reset()` re-reads
hardware and clears pending arm targets to that measured pose; it does not home
physical hardware. On deactivation it releases/closes hardware and resumes
simulation from the last measured pose. Cleanup attempts all motors, cameras and
systems even when startup or another cleanup operation fails.

Measured hardware pose is mirrored into the MuJoCo body. Native tracking
constraints hold the proxy between samples, and its simulated motor gains are
disabled during hardware tracking. Simulation contacts can affect virtual objects;
those calculated forces are not sent to physical motors. Proxy velocities are not
physical velocity measurements. A real arm is not physically constrained by
virtual obstacles, and physical devices run on wall time rather than simulated time.

## Calibration and CLI

LeRobot owns calibration and stores separate device IDs under its native cache:

- `~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/<id>.json`
- `~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json`

LeRobot environment variables can override these cache locations. Calibration
uses LeRobot's interactive procedure and writes homing/range settings. Supply
separate ports and stable IDs. Standard motor IDs 1–6 and compatible STS3215 servos
are required; see the [SO-101 guide](https://huggingface.co/docs/lerobot/v0.6.1/en/so101).

```sh
uv run --locked python -m airtruck.main --calibrate both --leader-port /dev/YOUR_LEADER --follower-port /dev/YOUR_FOLLOWER
uv run --locked python -m airtruck.main --leader-hardware --leader-port /dev/YOUR_LEADER --seconds 30
uv run --locked python -m airtruck.main --leader-hardware --follower-hardware --leader-port /dev/YOUR_LEADER --follower-port /dev/YOUR_FOLLOWER --seconds 30
```

The second command drives a simulated follower; the third drives a physical one.
Hardware flags explicitly request connection. Add `--render` with `mjpython` on
macOS. Without hardware flags the objects remain simulated. There is no `--real`
branch or alternate simulation-following API.

Tests use fake buses while exercising native LeRobot connect/configure/read/write
methods. They do not open physical ports. Physical calibration, installed voltage,
real timing, dynamics and motion remain unverified. Cleanup cannot guarantee an
unreachable device is unpowered; errors remain visible.

## SmolVLA yellow-cube policy on the follower

Two examples run a fine-tuned SmolVLA `pretrained_model` folder (see
[fine-tuning](../simulation.md#smolvla-fine-tuning)) on the
physical follower. Checkpoint variants are configured in `so101_cameras.CHECKPOINTS`
and the saved hardware configuration. `modified`
(default, `~/Documents/smolvla-weight-edit-6000/export/smolvla_so101_pad0_deep8`)
is step 6000 with baked-in weight edits: zeroed padding action inputs and a grasp
about 8 mm deeper, calibrated in simulation. `unmodified`
(`~/Documents/so101-smolvla-weights/run2_50ep_step_006000`) is the same fine-tune
as trained. Both use identical normalization statistics. Run them from a terminal that macOS allows to use the camera
(System Settings › Privacy & Security › Camera). On Linux, your user needs the `video`
and `dialout` groups instead
([Linux setup](../README.md#linux-and-nvidia-gpus)). Checkpoint folders are not in the
repository; copy them to the Linux machine and pass `--<variant> PATH` to `so101_cameras`.

```sh
uv run --locked python -m airtruck.examples.so101_cameras            # --variant unmodified to default to it
uv run --locked python -m airtruck.examples.so101_mirror      # torque off: check joint directions
uv run --locked python -m airtruck.examples.so101_policy --dry-run
uv run --locked python -m airtruck.examples.so101_policy
uv run --locked python -m airtruck.examples.so101_policy --variant unmodified
```

Two additional variants select the requested comparison directly:

```sh
uv run --locked python -m airtruck.examples.so101_policy --variant step2000
uv run --locked python -m airtruck.examples.so101_policy --variant manual_best5
```

`step2000` selects the unmodified `~/Documents/so101-smolvla-weights/step_002000`.
`manual_best5` selects the exact `yaw_reach_head_balance` export under
`~/Documents/yellow-cube-manual-weights-20261104/sparse-resume-20261105/checkpoints/`.
The latter passed 5/10 simulation cases and changes 88,332 values from its original
checkpoint. Its original is different from step 2000, so this is not a same-base
before/after comparison. Each variant loads its own saved processors. Physical
performance of `manual_best5` has not been verified. Existing variants and the
saved default remain available unchanged.

Three variants come from a separate fine-tune of `lerobot/smolvla_base` on the same
simulation dataset and normalization (`~/Documents/hybrid-train-smolvla`, step 6000).
Their copies are kept under `~/Documents/so101-smolvla-weights/hybrid-train/`:

```sh
uv run --locked python -m airtruck.examples.so101_policy --variant hybrid
uv run --locked python -m airtruck.examples.so101_policy --variant hybrid_endedit
uv run --locked python -m airtruck.examples.so101_policy --variant hybrid_control
```

- `hybrid` was hand-edited every 500 training steps.
- `hybrid_control` is the same run without edits.
- `hybrid_endedit` is `hybrid_control` with the same edit recipes applied once at the end.

In the simulated main suite they reached 39, 37 and 60 of 80 cubes respectively, and
picked 27, 27 and 40. None has been run on the physical arm.

These variants are tuned for the real cameras, the colour front sensor and the 59° wrist
view (see [measured cameras](../simulation.md#measured-cameras)), or for the real arm.
All derive from p17gshr with weight edits only:

```sh
uv run --locked python -m airtruck.examples.so101_policy --variant cf1
uv run --locked python -m airtruck.examples.so101_policy --variant cf1r4
uv run --locked python -m airtruck.examples.so101_policy --variant cf3p25
uv run --locked python -m airtruck.examples.so101_policy --variant cf1t5
uv run --locked python -m airtruck.examples.so101_policy --variant cf3p25t5
uv run --locked python -m airtruck.examples.so101_policy --variant cf3p25t5r15
uv run --locked python -m airtruck.examples.so101_policy --variant cf3p25t5r28
uv run --locked python -m airtruck.examples.so101_policy --variant sk5pt5
uv run --locked python -m airtruck.examples.so101_policy --variant rt1
```

- `cf1` (`export/camfix_cf1`) adds camera-transfer maps on the action expert's attention
  outputs, so the real views read like the training views. It has not yet run on the arm.
- `cf1r4` (`export/camfix_cf1r4`) is `cf1` with the wrist-roll servo gain doubled. In simulation it scores about the same as `cf1`, slightly better beside the band. It has not yet run on the arm.
- `cf3p25` (`export/camfix_cf3p25`) refits the camera-transfer maps on more scenes, including misaligned low hovers. It adds a height-gated pan servo that corrects sideways from the wrist view when low over the cube. It is the best variant in simulation, picking 29/36 beside the band and 23/27 inside it. It has not yet run on the arm.
- `cf1t5` and `cf3p25t5` are `cf1` and `cf3p25` with an exact pan translation of −5 units. On the real arm the base sits about 2 cm from the scanned pose, so the policy turned about 5 units too far toward +pan. `cf1t5` picked the cube 10 cm left and 10 cm right of centre, where `cf1` missed both.
- `cf3p25t5r15` and `cf3p25t5r28` are `cf3p25t5` with the same kind of reach pull-in as `rt1`, for the real arm's radial overshoot (jaws stopping on top of or past the cube). `r15` moves the grasp about 1.5 cm closer to the base and 0.3 cm lower; `r28` uses `rt1`'s full 2.8 cm and 0.5 cm. Neither has run on the arm yet.
- `sk5p` is `cf3p25` with a "seek" circuit on two layer-15 expert neurons. While the cube is hidden or only partly in the wrist view and the jaws are above 3.5 cm, it turns the base toward where the front camera sees the cube, by 0.7 of the remaining angle per action chunk; it is idle once the cube is in view. In the measured-camera simulation it keeps `cf3p25`'s scores and recovers an under-turned first swing (22/24 reached vs 10/24). It does not extend the reach, so cubes far outside the trained area can still be missed short. `sk5pt5` adds the real-arm pan −5. Not yet run on the arm.
- `rt1` adds an exact joint-space translation that grasps 2.8 cm closer to the base and 0.5 cm lower. It picked the cube 2/2 on the arm at the band centre.

`so101_cameras.CHECKPOINTS` also lists preliminary simulation-only weight edits of
step 6000 (`pan17` to `sp6_shpg3`), each commented with its simulation results.
`best-thus-far` is a symlink to the best of them, re-pointed by
`~/Documents/smolvla-weight-edit-6000/update_best.py` (history in that export's
`BEST_THUS_FAR.md`). `so101_cameras` saves the symlink path itself, so a re-pointed
symlink applies without re-running it.

The RealSense notes below are macOS-specific. On Linux, the D435i registers several
`/dev/video` nodes (depth, infrared and colour, each with a metadata node); OpenCV index N
is `/dev/videoN`. `so101_cameras` prints each node's name and probes past the highest
index, so choose the colour feed in its window. The colour bridge refuses to start on
Linux.

macOS's AVFoundation exposes only one of the D435i's video interfaces to OpenCV. It
appears in `system_profiler SPCameraDataType` as "Depth Camera 435i RGB" (colour)
or "… Depth". The Depth interface is the IR imager: grayscale with the projector's
dots, unusable for this policy. On this machine the exposed interface changed between
sessions for an unidentified reason, so check the device list `so101_cameras` prints.
macOS's UVCAssistant claims all of the camera's video interfaces but presents only that one
stream (also through CoreMediaIO), so no unprivileged setting selects colour. The camera has
been seen at 480 Mb/s (USB 2) even on a direct USB-C port.

The working colour path is `airtruck/examples/so101_realsense_bridge.py`. It uses Homebrew's
librealsense 2.59.1 C API through ctypes, as root, and writes 640×480 RGB frames at 30 fps
to `/tmp/airtruck-realsense-color.bin`. A front camera entry
`{"type": "realsense_bridge", "width": 640, "height": 480, "fps": 30}` in
`models/so101/hardware.json` reads it without root. Start the bridge in its own terminal
before `so101_policy`. While it runs, the RealSense disappears from OpenCV, so check the wrist
camera index with `so101_cameras --snapshot`:

```sh
sudo .venv/bin/python -B -m airtruck.examples.so101_realsense_bridge   # Ctrl+C stops
```
`so101_policy` refuses to start when a configured camera gives a grayscale frame (for example the Depth interface).
A Continuity Camera iPhone also adds an OpenCV index, so re-run `so101_cameras` whenever the device list changes. The Python binding
`pyrealsense2-macosx` 2.56.5 segfaults in `pipeline.start()` while it creates the
D435i IMU HID device (`hid_open_path` → `IOHIDDeviceOpen` on a null device), with
or without root; hence the bridge above uses Homebrew's library, colour stream only.

The training front view has the depth camera's geometry (f ≈ 384 px at 640×480,
63.97° vertical FOV). The colour sensor the policy now sees is narrower (42.63°
vertical, factory calibration) and offset about 15 mm, and the wrist camera's real
view is narrower than assumed. The simulator now defaults to these
[measured cameras](../simulation.md#measured-cameras); every checkpoint was trained
on the old views. With the colour front view, a real `best-thus-far` run approaches
the cube and closes beside it; before, it wandered.

`so101_cameras` opens every OpenCV index (on macOS, as many as `system_profiler`
lists; on Linux, up to the highest `/dev/videoN`), preferring the 640×480 at 30 fps training mode. It does not probe
librealsense. Hand-written `intelrealsense` camera entries are accepted where
`pyrealsense2` works, for example on Linux.
It shows the live feeds in a window. Select a feed with its digit or a click, then
press F for the fixed table view or W for the gripper view, and Enter to save.
`--snapshot DIR` saves one PNG per feed instead; `--front N --wrist M` then
assigns them without a window. macOS device names are listed for orientation only,
because they do not identify OpenCV indices; Linux names do. The follower port is found by
reading each bus's EEPROM calibration (read-only: torque is not changed) and
comparing it with the saved follower and leader records; `--port` skips this. A
nonzero difference is reported, because normal connection will then be refused
until the follower is recalibrated. The result is `models/so101/hardware.json`
(Git-ignored, machine-local): `policy.variant`, the `policy.checkpoints` folders
(`--<variant> PATH`, for example `--modified`, replaces a default; a missing
unselected folder is omitted), task, port, LeRobot ID, rate and camera configs under the dataset keys
`front` and `wrist`. `so101_policy --variant` overrides the saved variant for one run.

`so101_mirror` opens the follower through LeRobot's leader role, which disables
torque and only reads, using the follower's calibration file. Its window shows
both live camera feeds as the policy receives them, the nearest training frames
(`--no-reference` skips them) and an offscreen render of the simulated arm posed
from the measured joints, with the normalized values; no MuJoCo viewer is opened.
Move the real arm by hand: every joint must turn the same physical way and reach
the same end stops as the simulated arm. Matching normalized ranges does not
establish matching directions, and the policy's targets assume the simulated
mapping. Support the arm before starting; it goes limp.

`so101_policy` loads that file and uses `SO101Follower(hardware=True)` in a
`SystemDriver` at 30 Hz. Its MuJoCo world only mirrors the measured pose (ten
proxy substeps, contacts disabled). After connecting it runs one warm-up
inference (on CUDA, then MPS, then CPU, unless `--device` is given), ramps over `--home-seconds` (default 4; 0 skips) to the
simulated start pose `[0, 0.4, -0.8, 0.2, 0, -1.0]` rad. If any joint is then more than
`--home-tolerance` (default 10) normalized units away, it holds and stops instead of
starting the policy: a joint driven the wrong way, or blocked, stalls there because
`max_relative_target` limits each command to `--max-step` beyond the measured position.
Otherwise it runs the policy
for `--seconds` (default 20). Every tick reads both cameras and the measured
joints and calls LeRobot's `predict_action`, as `so101_pick_eval` does in
simulation. Predicted targets are clipped to the joint ranges, and
`--max-step` (default 5 normalized units per tick) is LeRobot's
`max_relative_target`. Training actions changed by at most about 3 units per tick.
Each motion phase asks for confirmation unless `--yes` is given; without a terminal
the run exits at the first confirmation instead. `--dry-run` runs cameras and
inference but holds the measured pose. A window, shown by default, has both camera
frames exactly as given to the policy (640×480 each, unscaled; `--no-view` hides it);
Esc stops. On completion, Ctrl+C or Esc the arm holds its pose. From a terminal it
then waits for Enter, and exit releases torque. Without a terminal (only with `--yes`) it
first returns to the pose where the gripper first closed, opens the gripper, ramps
through the ready pose back to the pose measured at start (over `--home-seconds`, at
least 3 s), then releases torque.

`--record DIR` saves every policy tick's frames as given to the policy
(`front_NNNNN.jpg`, `wrist_NNNNN.jpg`, JPEG quality 95) and `log.npz` with per-tick
`t` (`time.perf_counter()` seconds at tick start), measured `state` and unclipped predicted `action`, plus
`names`, `variant`, `checkpoint`, `fps` and the `cameras` configuration as JSON.
Homing and warm-up are not recorded. The log is written however the run ends.

Frames of another resolution are resized to 640×480. The ready pose and every
policy target use the simulated normalized mapping. They match the physical arm only if its
calibration matches the simulated joint zero and direction. The simulated
images have no distortion or sensor noise. SmolVLA predicts 50-step chunks: in
the MPS test each refill took about 0.35 s. The previous target is held during
that tick, so ticks overrun once per chunk.

## PX4 drone

Use the same `SentinelDrone` object for both backends, as with the arms:

```python
from airtruck.systems.sentinel import SentinelDrone

drone = SentinelDrone("drone", hardware=True)

with SystemDriver([drone]) as driver:   # Connects; never arms.
    driver.reset()
    drone.takeoff(1.0)
    driver.run(steps=250, realtime=True)
    drone.land()
```

`hardware=True` uses the built-in PX4 ROS 2 backend. It accepts the same NED
`position`/`yaw` controls and `arm`, `takeoff`, `hold`, `land`, `disarm`,
`send_action`, `get_observation`, `connect` and `disconnect` methods as simulation.
The built-in model is the hardware-tracked proxy in the shared world; the generic
`Drone(..., model=path, hardware=True)` accepts another prepared drone MJCF.
`PX4Config(namespace="px4_1", system_id=2)` selects a namespaced vehicle;
pass it as `SentinelDrone(..., px4=config)`. A custom backend may implement `DroneHardware`,
including `pop_tag_detections()` (return `[]` when it has no tag source).

Tag poses from the aircraft's detector arrive on `/tag_detections`
(`geometry_msgs/PoseStamped`, camera optical frame, tag ID in `header.frame_id`)
and are published to the global `TAGS` board on every observation; `PX4Config(tag_topic=..., tag_camera=...)` changes
the topic and the camera they are attributed to. Camera frames are not received
over ROS: publish them with `drone.cameras[name].publish(image, timestamp_s)` to use
`capture()` and opt-in image detection. See
[onboard cameras and AprilTags](../simulation.md#onboard-cameras-and-apriltags).

ROS imports are direct and required when importing the drone modules. On this
macOS workspace, native ROS 2 Humble/rclpy 3.3.21 with `rmw_fastrtps_cpp` 6.2.10
(Fast DDS 2.6.11), geometry_msgs 4.9.1 and the Sentinel's PX4 1.14 messages are
installed in Pixi's `airtruck-ros` environment. An ignored
`.venv/lib/python3.12/site-packages/airtruck_ros.pth` exposes those bindings and the
ROS prefix to the persistent uv Python 3.12 environment and defaults
`RMW_IMPLEMENTATION` to `rmw_fastrtps_cpp`; RoboStack's Humble build otherwise
selects Cyclone DDS. The SDK is a real native installation, not the type overlay or
a runtime mock.

Humble is used because the aircraft's Foxy cannot be installed here: RoboStack
publishes Foxy only for linux-64 with Python 3.8, while this project needs Python
3.12. Humble's ROS graph messages match Foxy's (see
[Sentinel aircraft link](#sentinel-aircraft-link)).

To reproduce the native installation, install `pixi` and the platform C/C++ build
tools, then run from the project root:

```sh
uv sync --locked
uv run --locked python scripts/drone/install_ros.py
```

Python packages remain locked by `uv.lock`. Native ROS distribution/build inputs
are pinned under `[tool.airtruck.ros]` in `pyproject.toml`; Pixi owns its native
environment and package manifest outside the checkout. Because `pixi global install`
merges specs into an existing environment, the installer first removes any existing
`airtruck-ros` environment and recreates it from exactly the pinned packages. It
then builds PX4 messages at `px4_msgs` revision
`6954ed341da55e5b514780d6301dd2e4647c2e49` without their linter tests, removes any
previously installed `px4_msgs` revision, sets the native library install and
link paths, registers the SDK in `.venv`, prints the selected RMW, and removes
temporary source and build artifacts.
Re-run it after replacing `.venv` or changing `px4_revision`. ROS packages are not
normal PyPI dependencies; `uv sync` alone does not install native ROS or generate
messages. The Pixi environment provides no `ros2` command-line tool; use `rclpy`
from `uv run --locked python` for inspection.

### Sentinel aircraft link

The Sentinel runs ModalAI VOXL SDK 1.6.2 with `voxl-px4 1.14.0-2.0.133`, ROS 2 Foxy
and `voxl-microdds-agent` (eProsima `MicroXRCEAgent`) on the VOXL 2. Its installed
`px4_msgs` definitions (`/opt/ros/foxy/mpa_to_ros2/install/px4_msgs/share/px4_msgs/msg`,
183 messages) are byte-identical to the pinned revision above. PX4 1.14 messages
carry no `MESSAGE_VERSION`, so topics are unversioned (`/fmu/out/vehicle_status`).
Its bridge publishes no `vehicle_command_ack` or `vehicle_land_detected`; the
backend does not subscribe to either.

The Mac and aircraft must share a network and `ROS_DOMAIN_ID` (unset on the
aircraft, which means `0`). In the factory SoftAP mode, join the `VOXL-…` Wi-Fi
network; the aircraft is `192.168.8.1` (`ssh root@192.168.8.1`). `voxl-wifi` on the
aircraft switches to station mode on an existing router. On the aircraft,
`voxl-inspect-services` must show `voxl-px4` and `voxl-microdds-agent` running, and
`offboard_mode` in `/etc/modalai/voxl-vision-hub.conf` must be `"off"` so the
vision hub does not send competing offboard setpoints (`voxl-configure-microdds`
can reset it). To list the topics visible from the Mac:

```sh
ROS_DOMAIN_ID=0 uv run --locked python -c "
import time, rclpy
rclpy.init(); n = rclpy.create_node('topic_list'); end = time.time() + 3
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.2)
for name, types in sorted(n.get_topic_names_and_types()): print(name, types)
"
```

The backend needs `/fmu/out/vehicle_odometry`, `/fmu/out/vehicle_status`,
`/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint` and
`/fmu/in/vehicle_command`. Match message definitions, namespace and system IDs;
the ROS installation does not flash or configure the aircraft. The aircraft bridge
offers best-effort, transient-local `/fmu/out` writers and best-effort, volatile
`/fmu/in` readers; the backend's sensor-data subscriptions and best-effort
publishers are compatible with both.

DDS discovery sends multi-kilobyte UDP datagrams. The aircraft's `wlan0` reported
an MTU of 2312 while macOS Wi-Fi allows at most 1500, so the aircraft's IP fragments
larger than 1500 bytes never reached the Mac: participants were found but no
topics appeared, while TCP (ssh, web portal) and small UDP (MAVLink) worked. The
aircraft and Mac Wi-Fi MTUs must match (check the Mac with
`networksetup -getMTU Wi-Fi` and the aircraft with `ip link show wlan0`).

Before each flight session, with the Mac on the aircraft's SoftAP network, run:

```sh
uv run --locked python scripts/drone/provision.py
```

It logs in as `root@192.168.8.1` with the VOXL factory password, runs
`voxl-bind-spektrum -s` to put the Spektrum receiver into bind mode, sets
`wlan0`'s MTU to 1500 and confirms it with `ip link show wlan0`. Any non-zero exit
status or a different MTU stops it with an error. The MTU change is not persistent,
so re-run the script after every aircraft boot. Edit the constants at the top of
the script for a different address, password or MTU.

The aircraft's tag topic comes from ModalAI's `voxl_mpa_to_ros2` node (ROS 2 Foxy,
`ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node`). It publishes `tag_detections`
while a subscriber is matched, puts the decimal tag ID in `header.frame_id` (which
the backend parses), and repeatedly prints harmless `awk` errors from its periodic
`voxl-list-pipes --mode-types` call. Every ROS 2 process also publishes graph
bookkeeping on the DDS topic `ros_discovery_info`, whose `Gid` is 24 bytes up to
Humble but 16 bytes from Iron onward (including Jazzy). With the earlier Jazzy
install, Foxy nodes misread the Mac's samples and could abort with
`std::bad_alloc`. Humble's graph messages and GID construction are identical to
Foxy's upstream sources, and the installed SDK's messages decode exactly with
Foxy's layout, so no workaround is used. Foxy/Humble interoperability on the
aircraft has not yet been confirmed.

### Flight command semantics

The adapter uses position/yaw setpoints and a 20 Hz heartbeat. Without command
ACKs, each mode/arming/land command is re-sent every 0.5 s until measured
`vehicle_status` confirms it, or raises `TimeoutError` after `PX4Config.timeout_s`;
a rejected command is reported only as that timeout. Connection never arms;
explicit takeoff first holds the measured pose, warms up offboard, confirms
mode/arming and then changes the position target. `land()` sends
`VEHICLE_CMD_NAV_LAND` and confirms landing mode, not touchdown.

Outgoing messages carry `timestamp = 0`, which the PX4 1.14 bridge replaces with
its arrival time. A host-clock timestamp would be shifted by the aircraft clock's
offset and could make offboard heartbeats look stale (`COM_OF_LOSS_T`).

`state["landed"]` is derived from `vehicle_status`: true while disarmed or while
armed before PX4 detects takeoff (`takeoff_time == 0`). PX4 sets `takeoff_time`
on detected takeoff and clears it only on disarm, so after a flight `landed`
becomes true when PX4 auto-disarms after touchdown (`COM_DISARM_LAND` seconds), or
on an explicit disarm if auto-disarm is disabled. Keep stepping in real time and
inspect measured land/arming status. Shutdown does not auto-disarm; communication
loss invokes the aircraft's configured failsafe.
See [flight semantics and coordinate frames](../simulation.md#drone-position-control-and-flight-commands).

Native ROS message serialization and node/executor/type support have been checked
locally on an isolated DDS domain, and the Sentinel's PX4 topics were listed from
the Mac over its SoftAP Wi-Fi. The automated PX4 transport tests replace the ROS
graph with an in-memory executor; they test warmup, setpoints, status confirmation,
command re-sending, zero timestamps, timeouts, derived landing, frame conversion, tag-pose ingestion
and cleanup without connecting to a flight controller. Receiving Sentinel
telemetry through the backend, ROS 2 Foxy/Humble interoperability for commands,
PX4 SITL, physical flight, Wi-Fi timing and actual flight-controller gains remain
unverified. Simulation tracks the same API through an illustrative controller; it
does not run PX4 firmware.
