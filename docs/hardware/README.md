# Physical SO-101 adapters

`RealSO101` in `airtruck/raw_api/so101/controller.py` provides the physical implementation of Airtruck's shared position-control interface. It is tested against a fake transport; this preflight does not verify physical operation.

- `airtruck/raw_api/so101/controller.py`: `RealSO101` and `LeaderSO101` adapters.
- `airtruck/raw_api/so101/calibration.py`: immutable `JointCalibration` records and JSON load/save helpers.
- `airtruck/raw_api/so101/position.py`: immutable `JointPosition` values, units and conversions.
- `airtruck/raw_api/so101/utils.py`: command validation helpers.
- `airtruck/raw_api/so101/io.py`: `ServoTransport`, the contract a selected Feetech/serial driver must implement.
- `airtruck/raw_api/so101/tests/test_hardware.py`: calibration, command validation, communication-failure and enable/disable tests.

The caller supplies an already configured transport implementing `read_positions(servo_ids)`, `write_positions({servo_id: raw_ticks})` and `set_torque_enabled(servo_ids, enabled)`. It must raise on failed communication rather than fabricate feedback. The caller owns connection lifecycle, startup torque state, bus timing, firmware configuration and device errors. `FeetechTransport` in `airtruck/raw_api/so101/transport.py` provides this implementation using the locked `feetech-servo-sdk` dependency. Supply your actual serial port explicitly; no device scanning or automatic connection occurs.

Provide all six joint calibrations: actual servo ID, measured encoder tick at the hardware midpoint, direction (`-1` or `+1`), measured travel in hardware radians, and encoder counts per turn (4096 by default). Do not use the simulator's virtual offsets. Calibration crossing the absolute encoder wrap is rejected by this simple adapter; unwrap it in a more specialized driver before supporting such travel.

```python
from airtruck.raw_api.so101.controller import RealSO101
from airtruck.raw_api.so101.position import JointPosition, PositionUnit
from airtruck.raw_api.so101.calibration import load_joint_calibrations
from airtruck.raw_api.so101.transport import FeetechTransport

# port: your USB-to-TTL servo adapter's serial device path
# Load the complete mapping produced by the guided calibration script.
calibration = load_joint_calibrations("models/so101/runtime/calibration.json")
with FeetechTransport(port, baudrate=1_000_000) as transport:
    arm = RealSO101(transport, calibration)
    try:
        arm.set_enabled(True)  # Read current pose, queue it, then enable torque
        arm.set_positions({
            "shoulder_pan": JointPosition(0.0872665, PositionUnit.RADIANS, arm.positions["shoulder_pan"].limits_rad)
        })
        print(arm.positions)  # Feedback, not proof the requested motion has finished
        # Keep this scope open while your application monitors/controls the arm.
    finally:
        arm.set_enabled(False)
```


Position feedback is clamped to each joint’s calibrated limits. Enabling still checks
the unclamped feedback and refuses poses outside calibrated travel. The error lists
each offending joint, servo ID, current encoder tick and allowed tick interval;
no goal or torque-enable writes occur for an out-of-range pose. Check that the
follower port and calibration belong to the same arm. A calibration command that
refuses an existing output has not recalibrated the arm; use a new `--output` path
and update `FOLLOWER_CALIBRATION` to use the new file.
The main program prints full exception details, including individual servo failures
inside a torque-disable exception group and any earlier error that triggered cleanup.
Failed torque-disable requests leave the affected motors' torque state unconfirmed.
Position batches are validated before any write; successful writes update the adapter's remembered targets. Transport failures propagate; a real bus may have partially applied a failed batch, so reads and driver recovery are necessary before retrying. Torque state is the adapter's last successful request, not independent hardware telemetry. Disabling always sends the request, including directly after construction.

Test without hardware:

```sh
uv run pytest -k hardware
```

## Human-operated leader reader

`LeaderSO101` reads the same six named joints as `JointPosition` objects in
calibrated radians, carrying their units and ranges, and exposes
`positions` and `normalized_positions`. It has no position commands, targets, or torque-enable
method and does not implement `ArmController`. Construction performs no I/O.
Support the leader, then call `start()` to disable all six motors; reads are
blocked until that request succeeds. A failed start/restart blocks further reads
and propagates the transport error. Each `positions` access reads fresh feedback,
clamped to the taught limits (below LOW becomes LOW; above HIGH becomes HIGH).

```python
from airtruck.raw_api.so101.controller import LeaderSO101
from airtruck.raw_api.so101.calibration import load_joint_calibrations
from airtruck.raw_api.so101.transport import FeetechTransport

calibration = load_joint_calibrations("models/so101/runtime/leader-calibration.json")
with FeetechTransport(leader_port, baudrate=1_000_000) as transport:
    leader = LeaderSO101(transport, calibration)
    leader.start()  # Support the arm first; disables torque without commanding motion
    print(leader.positions)
```

The caller owns the connection and must call `start()` again after reconnecting.
Do not share the leader's motors with a controller that enables torque. A
successful disable is a bus acknowledgement, not independent torque telemetry.
Use the leader's own measured calibration, never the follower's file. New sweeps use increasing encoder ticks as positive on both arms; verify that
these correspond to matching physical directions before following. Their endpoint midpoints and
travel can differ; this reader does not implement follower teleoperation or an
automatic coordinate/range mapping. The [application entry point](../README.md#leaderfollower-control)
exchanges normalized `[-1, 1]` joint positions in both simulation and real mode.
`LeaderSO101.normalized_positions` encodes its own range; each follower’s
`set_normalized_positions()` decodes using its own limits. Radian APIs remain available.

## STS3215 serial transport

The [official SO-101 guide](https://github.com/huggingface/lerobot/blob/main/docs/source/so101.mdx)
specifies six STS3215 servos with 345:1 gearing for the follower. Leader gearing
is different. This project's user-reported follower uses the 7.4 V variant.
The [LeRobot register table](https://github.com/huggingface/lerobot/blob/v0.4.4/src/lerobot/motors/feetech/tables.py)
identifies SDK protocol 0, 4096 encoder counts, torque enable at address 40,
goal position at 42 (two bytes), and feedback position at 56 (two bytes).
The [SDK](https://github.com/Adam-Software/FEETECH-Servo-Python-SDK) handles
packet encoding, checksums and receive timeouts.

`FeetechTransport(port, baudrate=1_000_000)` explicitly inherits `ServoTransport`.
This protocol also accepts structurally compatible transports (including test
fakes). Declared subclasses must implement its abstract methods.
Construction does no I/O; `connect()` or context entry opens the serial port.
Use a suitable USB-to-TTL Feetech bus adapter and the installed servos' supply.
Motor IDs must already be unique, baudrates must match, servos must be in
single-turn position mode, and status responses must acknowledge writes.
The transport does not configure or verify those firmware settings, rewrite
EEPROM, set motor IDs, or invent calibration. Speed and acceleration registers
retain their existing hardware values; simulation tuning is not applied to them.

Each operation serializes access to the bus. Reads and writes use individual
acknowledged transactions; batch commands are not simultaneous or atomic.
All input IDs and target ticks are validated before transmission. IDs are 0..253;
broadcast is disallowed. Raw positions must be integers 0..4095. Signed or
multi-turn feedback is rejected instead of silently wrapped. Missing replies,
corrupt responses and device error flags outside the warning exceptions below
raise exceptions; writes are not retried
automatically because a timed-out write may already have applied.

Enabling stops on the first error; earlier motors may already be enabled.
Disabling attempts every requested motor and raises an `ExceptionGroup` for
failures. `close()` is idempotent and only closes serial I/O: it does not disable
torque. The caller must explicitly disable, including after partial enable
failure, as shown above. A lost connection cannot guarantee torque removal.

`tests/test_transport.py` under the SO-101 control folder tests SDK calls,
register addresses, protocol selection, validation, device/communication errors,
partial batches, cleanup, and integration with `RealSO101`. Tests replace the SDK
port/packet boundary; they open no serial device and do not establish physical
hardware operation. Run them with:

```sh
uv run --locked pytest airtruck/raw_api/so101/tests/test_transport.py
```

Successful replies carrying exactly status `1` (input voltage) print a console
warning and continue for all operations: position reads/writes and torque enable/
disable. This applies by default to calibration and both application modes.
Warnings print to stderr once per servo per connection; an optional
`voltage_warning=callback` customizes their presentation. Communication failures,
combined or other error flags, and invalid position values still fail, except that
real following configures `gripper_servo_id` from the follower calibration to warn
and continue on overload-only status `32` for that gripper. This includes torque-off
requests. Other joints still fail on overload; standalone transports remain strict
unless explicitly given a gripper ID. Overload warnings print once per connection.
Continuing
does not clear or override any protection applied by the servo’s firmware, or
establish whether the voltage fault is transient.

## Guided hardware calibration

Run the single interactive script from the project root in a macOS/Linux terminal:

```sh
uv run --locked python scripts/so101/so101_calibrate.py --arm both
```

`--arm both` teaches the follower first, closes its connection, then teaches the
leader with a separate adapter selection and support/torque-disable confirmation.
Use `--arm leader` or `--arm follower` for just one arm; omitting `--arm` preserves
the follower-only workflow. Each arm has its own six-ID mapping, so IDs may repeat
across separate buses.

Choose the USB adapter from the numbered serial-port list, or enter its `/dev/...`
path. For a single arm, `--port`, `--baudrate`, `--ids` and `--output` select that
arm's settings. In both mode those options select the follower, while
`--leader-port`, `--leader-baudrate`, `--leader-ids` and `--leader-output` select
the leader. Both baudrates default to 1,000,000 and both ID sets to 1..6.
Both output paths and ID sets are validated before either session starts.
Listing ports does not open them; connection follows your start confirmation.

The terminal UI has colored sections, numbered joint steps, a live most-moving
servo/tick indicator and per-ID cumulative travel in radians, plus saved direction
and limit summaries. The live line is clipped to the terminal width. Redirected
output stays plain without live redraws; `NO_COLOR=1` disables colors. All UI code
lives in the calibration script and uses no additional dependency.

The prompts follow the physical chain from its tip toward the fixed base:
**gripper → wrist roll → wrist flex → elbow flex → shoulder lift → shoulder pan**.
This is a distal-to-proximal order, independent of which joint is currently highest
in space. Each prompt names the joint and describes the mechanism to move.
Support the arm before confirming torque disable. Calibration commands no motion
and attempts to leave torque disabled, including on interruption.

Calibration prints a warning and stays connected for voltage-only status replies,
including the final read after Space and torque-disable requests. It uses the
returned position if it passes the usual range checks; it does not retry or use a
stale sample. The warning does not establish whether voltage is low or high, or
whether the condition is transient. Other failures still stop the session.

For each named joint, move it gently through its full safe travel in **both
directions**, in either order, then press **Space once** when done. The script
records the smallest and largest encoder values observed throughout the sweep;
you can return to the middle before finishing. There are no LOW/HIGH choices or
separate endpoint captures. Move only the named mechanism as much as possible.
Use safe usable limits for wrist roll; do not assume it has physical hard stops.
The script records observed travel; it cannot prove you reached the full range.

The script reads all six candidate motors during motion. Default candidate IDs
are 1..6; supply `--ids 7 8 9 10 11 12` for another configured set. It discovers
which ID belongs to each named joint by **accumulated absolute angular travel**
over the whole sweep, not by assuming the standard joint/ID assignment.
One-count jitter is ignored. Other motors can move accidentally; the selected
motor must move most, by at least 0.02 rad, with at least 0.05 rad total motion.
Ambiguity, insufficient endpoint separation, or selection of an already assigned
ID causes a retry. It requires replies from every candidate; it does not scan the
bus for unknown IDs or change IDs/baudrates.

Calibration is independent of CAD. Zero is the midpoint of the measured endpoints,
rounded to an encoder tick. New sweeps use direction +1: lower encoder ticks
are negative and higher ticks are positive, independent of sweep order. Encoder
feedback alone cannot infer a physical positive direction such as opening the
gripper or lifting the arm. Existing signed calibration files remain supported;
recalibrating an arm previously using direction -1 changes its angle convention. The two `limits_rad` values are computed relative to that zero using
4096 counts per revolution, the STS3215 encoder specification. Resolution cannot
be inferred from two arbitrary stops. No CAD angles, geometry, or simulation
zero offsets are consulted. Hardware angles require an explicit coordinate mapping
if they are later compared with simulation angles.

The follower default output is `models/so101/runtime/calibration.json`; the leader
default is `models/so101/runtime/leader-calibration.json`. Each is a direct
mapping from joint name to these fields (illustrative numbers only):

```json
{
  "gripper": {
    "servo_id": 6,
    "zero_tick": 2000,
    "direction": 1,
    "limits_rad": [-1.533980787885641, 1.533980787885641],
    "counts_per_revolution": 4096
  }
}
```

Each completed joint is atomically checkpointed into its arm’s file. Q or
Ctrl-C stops recording and preserves completed entries. Existing output files
are refused; choose another `--output` (or `--leader-output` in both mode) when
restarting or recalibrating. If interrupted during the leader session, the completed
follower file is retained; restart with `--arm leader` and a fresh output. There is
no automatic resume. `load_joint_calibrations(path)` returns a typed
`dict[JointName, JointCalibration]` only when all six unique joints and servo IDs
are present; partial, duplicate-key or invalid records are rejected.

The existing hardware controller represents only single-turn, non-wrapping
encoder intervals. A detected 0/4095 crossing is rejected for the selected motor;
the script does not wrap it into a plausible but incorrect calibration or rewrite
firmware homing offsets. Move slowly enough for continuous sampling. Tests cover
shuffled IDs, incidental motion, signed calibration conversion, either sweep order,
interruption checkpoints, JSON round trips and endpoint precision. The checked-in follower and leader calibration files contain arm-specific encoder
records; their physical endpoint coverage and matching directions were not
independently verified in this preflight.
