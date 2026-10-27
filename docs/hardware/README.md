# Hardware control: SO-101 and PX4

Use the same `SO101Follower` or `SO101Leader` object for both backends. These classes
inherit the corresponding native LeRobot class and share `SO101Arm` simulation
bindings. No separate hardware controller hierarchy or serial protocol is added.

```python
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.so101 import SO101Follower, SO101Leader

leader = SO101Leader("leader", hardware=True, port="/dev/YOUR_LEADER",
                     id="airtruck_leader", position=(-0.25, 0, 0))
follower = SO101Follower("follower", hardware=False, port="/dev/YOUR_FOLLOWER",
                         id="airtruck_follower", position=(0.25, 0, 0))

with SystemDriver([leader, follower]) as driver:
    driver.reset()

    def update():
        follower.controls.update(leader.get_action())

    driver.run(update=update, realtime=True)
```

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
enables position control. Normal connection never starts interactive calibration. Virtual arms report
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
(`geometry_msgs/PoseStamped`, camera optical frame) and are published to the global
`TAGS` board on every observation; `PX4Config(tag_topic=..., tag_camera=...)` changes
the topic and the camera they are attributed to. Camera frames are not received
over ROS: publish them with `drone.cameras[name].publish(image, timestamp_s)` to use
`capture()` and opt-in image detection. See
[onboard cameras and AprilTags](../simulation.md#onboard-cameras-and-apriltags).

ROS imports are direct and required when importing the drone modules. On this
macOS workspace, native ROS 2 Jazzy/rclpy 7.1.12, geometry_msgs 5.3.8 and PX4
release/1.16 messages are installed in Pixi's `airtruck-ros` environment. An ignored
`.venv/lib/python3.12/site-packages/airtruck_ros.pth` exposes those bindings and the
ROS prefix to the persistent uv Python 3.12 environment. The SDK is a real native
installation, not the type overlay or a runtime mock.

To reproduce the native installation, install `pixi` and the platform C/C++ build
tools, then run from the project root:

```sh
uv sync --locked
uv run --locked python scripts/drone/install_ros.py
```

Python packages remain locked by `uv.lock`. Native ROS distribution/build inputs
are pinned under `[tool.airtruck.ros]` in `pyproject.toml`; Pixi owns its native
environment and package manifest outside the checkout. The installer builds PX4
messages at revision `392e831c1f659429ca83902e66820d7094591410`, sets the native
library install paths, registers the SDK in `.venv`, and removes temporary source
and build artifacts. Re-run it after replacing `.venv`. ROS packages are not normal
PyPI dependencies; `uv sync` alone does not install native ROS or generate messages.

The aircraft must run a compatible PX4 firmware/message version and the XRCE-DDS
bridge/agent must already be configured. The backend uses the message class's
`MESSAGE_VERSION` to select versioned output topics (for example
`/fmu/out/vehicle_status_v1`). Match firmware, message definitions, namespace and
system IDs; this installation does not flash or configure the aircraft.

The adapter uses position/yaw setpoints, a 20 Hz heartbeat and bounded acknowledgement
waits. Connection never arms; explicit takeoff first holds the measured pose,
warms up offboard, confirms mode/arming and then changes the position target.
`land()` sends `VEHICLE_CMD_NAV_LAND` and confirms landing mode, not touchdown.
Keep stepping in real time and inspect measured land/arming status. Shutdown does
not auto-disarm; communication loss invokes the aircraft's configured failsafe.
See [flight semantics and coordinate frames](../simulation.md#drone-position-control-and-flight-commands).

Native ROS message serialization and node/executor/type support have been checked
locally on an isolated DDS domain. The automated PX4 transport tests replace the
ROS graph with an in-memory executor; they test warmup, setpoints, ACK/status
confirmation, rejection/timeouts, frame conversion, tag-pose ingestion and cleanup
without connecting to a flight controller. PX4 SITL, physical flight, radio/bridge timing and actual
flight-controller gains remain unverified. Simulation tracks the same API through
an illustrative controller; it does not run PX4 firmware.
