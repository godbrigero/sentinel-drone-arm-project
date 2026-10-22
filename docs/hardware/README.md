# Physical SO-101 adapter

`RealSO101` in `airtruck/raw_controls/so101/controller.py` provides the physical implementation of Airtruck's shared position-control interface. It is tested against a fake transport; no hardware communication or motion was performed during development.

- `airtruck/raw_controls/so101/controller.py`: `RealSO101` and immutable `JointCalibration` records.
- `airtruck/raw_controls/so101/io.py`: `ServoTransport`, the contract a selected Feetech/serial driver must implement.
- `airtruck/raw_controls/so101/tests/test_hardware.py`: calibration, command validation, communication-failure and enable/disable tests.

The caller supplies an already configured transport implementing `read_positions(servo_ids)`, `write_positions({servo_id: raw_ticks})` and `set_torque_enabled(servo_ids, enabled)`. It must raise on failed communication rather than fabricate feedback. The caller owns connection lifecycle, startup torque state, bus timing, firmware configuration and device errors. `FeetechTransport` in `airtruck/raw_controls/so101/transport.py` provides this implementation using the locked `feetech-servo-sdk` dependency. Supply your actual serial port explicitly; no device scanning or automatic connection occurs.

Provide all six joint calibrations: actual servo ID, measured encoder tick at the hardware midpoint, direction (`-1` or `+1`), measured travel in hardware radians, and encoder counts per turn (4096 by default). Do not use the simulator's virtual offsets. Calibration crossing the absolute encoder wrap is rejected by this simple adapter; unwrap it in a more specialized driver before supporting such travel.

```python
from airtruck.raw_controls.so101.controller import RealSO101, load_joint_calibrations
from airtruck.raw_controls.so101.transport import FeetechTransport

# port: your USB-to-TTL servo adapter's serial device path
# Load the complete mapping produced by the guided calibration script.
calibration = load_joint_calibrations("models/so101/runtime/calibration.json")
with FeetechTransport(port, baudrate=1_000_000) as transport:
    arm = RealSO101(transport, calibration)
    try:
        arm.set_enabled(True)  # Read current pose, queue it, then enable torque
        arm.set_positions({"shoulder_pan": 0.0872665})
        print(arm.positions)  # Feedback, not proof the requested motion has finished
        # Keep this scope open while your application monitors/controls the arm.
    finally:
        arm.set_enabled(False)
```


Enabling refuses feedback outside calibrated travel. Position batches are validated before any write; successful writes update the adapter's remembered targets. Transport failures propagate; a real bus may have partially applied a failed batch, so reads and driver recovery are necessary before retrying. Torque state is the adapter's last successful request, not independent hardware telemetry. Disabling always sends the request, including directly after construction.

Test without hardware:

```sh
uv run pytest -k hardware
```

## STS3215 serial transport

The [official SO-101 guide](https://github.com/huggingface/lerobot/blob/main/docs/source/so101.mdx)
specifies six STS3215 servos with 345:1 gearing for the follower. Leader gearing
is different. This project's user-reported follower uses the 7.4 V variant.
The [LeRobot register table](https://github.com/huggingface/lerobot/blob/v0.4.4/src/lerobot/motors/feetech/tables.py)
identifies SDK protocol 0, 4096 encoder counts, torque enable at address 40,
goal position at 42 (two bytes), and feedback position at 56 (two bytes).
The [SDK](https://github.com/Adam-Software/FEETECH-Servo-Python-SDK) handles
packet encoding, checksums and receive timeouts.

`FeetechTransport(port, baudrate=1_000_000)` implements `ServoTransport`.
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
corrupt responses and servo error bits raise exceptions; writes are not retried
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
uv run --locked pytest airtruck/raw_controls/so101/tests/test_transport.py
```

## Guided hardware calibration

Run the single interactive script from the project root in a macOS/Linux terminal:

```sh
uv run --locked python scripts/so101/so101_calibrate.py
```

Choose the USB adapter from the numbered serial-port list, or enter its `/dev/...`
path. You can also pass `--port /dev/your-actual-adapter` to skip the picker.
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
and leaves torque disabled, including on interruption.

For each joint, move gently to LOW and press **Space**, then HIGH and press
**Space**. To capture HIGH first, press **H** before the first Space; **L** switches
back to LOW. These are your chosen signed-angle limits, not raw encoder minimum
and maximum, and not vertical height. For example, you can choose closed as LOW
and open as HIGH for the gripper. Use safe usable limits for wrist roll; do not
assume it has physical hard stops. Hold still when capturing each endpoint.

The script reads all six candidate motors during motion. Default candidate IDs
are 1..6; supply `--ids 7 8 9 10 11 12` for another configured set. It discovers
which ID belongs to each named joint by **accumulated absolute angular travel**
over both recording phases, not by assuming the standard joint/ID assignment.
One-count jitter is ignored. Other motors can move accidentally; the selected
motor must move most, by at least 0.02 rad, with at least 0.05 rad total motion.
Ambiguity, insufficient endpoint separation, or selection of an already assigned
ID causes a retry. It requires replies from every candidate; it does not scan the
bus for unknown IDs or change IDs/baudrates.

Calibration is independent of CAD. Zero is the midpoint of the measured endpoints,
rounded to an encoder tick. Direction is +1 if ticks increase from LOW to HIGH,
otherwise -1. The two `limits_rad` values are computed relative to that zero using
4096 counts per revolution, the STS3215 encoder specification. Resolution cannot
be inferred from two arbitrary stops. No CAD angles, geometry, or simulation
zero offsets are consulted. Hardware angles require an explicit coordinate mapping
if they are later compared with simulation angles.

The default output is `models/so101/runtime/calibration.json`. It is a direct
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

Each completed joint is atomically checkpointed into this **one file**. Q or
Ctrl-C stops recording and preserves completed entries. Existing output files
are refused; choose another `--output` when restarting or recalibrating. There is
no automatic resume. `load_joint_calibrations(path)` returns a typed
`dict[JointName, JointCalibration]` only when all six unique joints and servo IDs
are present; partial, duplicate-key or invalid records are rejected.

The existing hardware controller represents only single-turn, non-wrapping
encoder intervals. A detected 0/4095 crossing is rejected for the selected motor;
the script does not wrap it into a plausible but incorrect calibration or rewrite
firmware homing offsets. Move slowly enough for continuous sampling. Tests cover
shuffled IDs, incidental motion, reversed direction, either endpoint order,
interruption checkpoints, JSON round trips and endpoint precision. Physical
calibration has not been run on an attached arm.
