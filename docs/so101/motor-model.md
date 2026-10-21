# Motor research and model fidelity

## Published evidence

The [LeRobot SO-101 guide](https://huggingface.co/docs/lerobot/so101) specifies six STS3215 **345:1** follower servos. The leader uses mixed gear ratios and is not this model.

The [Feetech STS3215 datasheet](https://www.feetechrc.com/Data/feetechrc/upload/file/20200611/6372749961523760249976542.pdf), edition A/0 dated 2020-04-10, section 5, specifies the following at 7.4 V: terminal resistance **2.5 Ω**, output torque constant **8 kgf·cm/A = 0.784532 N·m/A**, stall torque **19.5 kgf·cm = 1.91229675 N·m**, stall current **2.5 A**, no-load current **0.15 A**, and no-load speed **52 rpm**. Rated torque is **5 kgf·cm = 0.4903325 N·m**, at **0.65 A**. Stall torque is a brief peak, not a continuous duty rating. It also lists 0.192 s per π/3 rad; the rounded 52 rpm value is used consistently here.

[Feetech's product description](https://www.feetechrc.com/20210430-56680.html) confirms 345:1 gearing, 4096-count magnetic position sensing and acceleration/speed settings. [LeRobot's follower implementation](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py) uses IDs 1–6, position mode, Goal_Position/Present_Position, and reduced gripper current/torque settings. We imitate those semantics and a 50% gripper current cap; we do not claim firmware equivalence. No register PID numbers are copied into physical SI gains.

## Equations actually implemented

Use output-shaft coordinates; gear reduction is already embodied in the output constants, so the MJCF actuator gear is 1 (not another factor of 345).

- `R_motor = 2.5 Ω` — published terminal resistance.
- `R_series = 7.4/2.5 − R_motor = 0.46 Ω` — estimated effective driver/supply loss, not measured winding resistance.
- `Kt = 0.784532 N·m/A` — published output constant.
- `Ke = (V − (R_motor+R_series) I_no_load) / omega_no_load = 1.277402 V·s/rad` — fitted output-coordinate back-EMF constant.
- `tau_c = Kt I_stall − tau_stall = 0.04903325 N·m` — estimated Coulomb loss.
- `b = (Kt I_no_load − tau_c)/omega_no_load = 0.01260627 N·m·s/rad` — estimated viscous loss.

The generated general actuator implements `tau_e = Kt/(R_motor+R_series) * (V_command − Ke * joint_velocity)`, clamped to `±Kt*I_stall` (half for the gripper). `joint.frictionloss=tau_c` and `joint.damping=b` supply mechanical resistance. Voltage is limited to ±7.4 V. The published stall torque and no-load speed are reproduced by this fit, but the loss split is not independently identified. Effective geared Kt and Ke need not be equal because gear losses are folded into output constants. Motor inductance is omitted (quasi-static current approximation).

`custom/numeric` stores the named resistance and fitted values for inspection; those metadata fields alone have no physical effect. Their derived gain/bias and joint damping/friction coefficients are what affect MuJoCo dynamics. See [MuJoCo actuator documentation](https://mujoco.readthedocs.io/en/stable/XMLreference.html#actuator-general).

The outer controller is an approximate PI-D voltage controller, with virtual encoder quantization, anti-windup and bounded target speed/acceleration. Its gains are demo tuning, not identified Feetech firmware constants. Peak current is bounded; thermal overload timers, electrical transients, bus latency, saturation heating and supply interactions are not modeled. No physical rotor-inertia measurement was found; additional armature is omitted rather than assigned an invented motor inertia. Backlash and unpowered backdrive friction remain unverified.

## Masses and geometry

Native Onshape assembly revision: `39f0987c444b49d7bde72b28`. Part Studio geometry revision: `eaf0660831e6ce2c01dcb7b8`. Export preserves 257 parts, seven groups, 12 connector frames and six joint axes/ranges. Parent connector Z is negated for the child's positive angle, matching Onshape's mate-angle convention. XML geometry transforms use the folded saved pose as zero.

The source CAD has no assigned mass. Reference link masses/COMs/inertias come from [TheRobotStudio's SO-101 URDF](https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf), aligned to the CAD using printed-part frame fits; the final reference inertias are retained in `models/so101/source/cad-manifest.json`. They are approximations for this detailed model, not measured properties. All seven masses and principal inertias are positive; inertia triangle inequalities are checked. Explicit inertias prevent visual mesh shell settings from changing link mass.

Full contact travel and hardware-identical behavior are outside this demo's validation. To improve fidelity, measure installed supply voltage, current versus torque/speed, powered/unpowered friction, response to position steps, backlash, rotor inertia and each assembled link's inertial properties. Do not treat a successful 0.0872665-radian demonstration as validation for payload prediction or hardware control.

The two-arm push regression additionally verifies force transfer using convex-hull
contacts and a collision-disabled control run. Its normal-force and penetration
checks validate the configured simulation, not measured contact stiffness or
hardware force. See the [push-test guide](README.md#two-arm-physical-push-test).
