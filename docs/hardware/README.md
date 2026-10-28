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
the driver loop. The bridge currently requires `use_degrees=False` and no cameras.
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
