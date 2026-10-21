# Physical SO-101 adapter

`RealSO101` in `airtruck/raw_controls/so101/controller.py` provides the physical implementation of Airtruck's shared position-control interface. It is tested against a fake transport; no hardware communication or motion was performed during development.

- `airtruck/raw_controls/so101/controller.py`: `RealSO101` and immutable `JointCalibration` records.
- `airtruck/raw_controls/so101/io.py`: `ServoTransport`, the contract a selected Feetech/serial driver must implement.
- `airtruck/raw_controls/so101/tests/test_hardware.py`: calibration, command validation, communication-failure and enable/disable tests.

The caller supplies an already configured transport implementing `read_positions(servo_ids)`, `write_positions({servo_id: raw_ticks})` and `set_torque_enabled(servo_ids, enabled)`. It must raise on failed communication rather than fabricate feedback. The caller owns connection lifecycle, startup torque state, bus timing, firmware configuration and device errors. There is no concrete serial driver here because the user's port and installed driver have not been specified.

Provide all six joint calibrations: actual servo ID, measured encoder tick at CAD zero, direction (`-1` or `+1`), measured travel in CAD radians, and encoder counts per turn (4096 by default). Do not use the simulator's virtual offsets. Calibration crossing the absolute encoder wrap is rejected by this simple adapter; unwrap it in a more specialized driver before supporting such travel.

```python
from airtruck.raw_controls.so101.controller import RealSO101 as SO101

# transport: your configured driver implementing ServoTransport
# calibration: dict[str, JointCalibration], measured for all six joints
arm = SO101(transport, calibration)  # No I/O on construction
arm.set_enabled(True)              # Read pose, queue matching targets, enable torque
arm.set_positions({"shoulder_pan": 0.0872665})
print(arm.positions)               # Actual encoder feedback, converted to CAD radians
arm.set_enabled(False)
```

Enabling refuses feedback outside calibrated travel. Position batches are validated before any write; successful writes update the adapter's remembered targets. Transport failures propagate; a real bus may have partially applied a failed batch, so reads and driver recovery are necessary before retrying. Torque state is the adapter's last successful request, not independent hardware telemetry. Disabling always sends the request, including directly after construction.

Test without hardware:

```sh
uv run pytest -k hardware
```
