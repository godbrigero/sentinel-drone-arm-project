# Native position servos and model fidelity

The runtime arm uses **MuJoCo position actuators**, not a Python servo controller or an
electrical motor simulation. This keeps shared-world stepping fast and makes
joint-target actions suitable for Gymnasium and LeRobot ACT.

Each actuator applies a bounded native PD servo:

`torque = kp * (target_rad - position_rad) - kv * velocity_rad_s`

`scripts/so101/build_model.py` generates native actuator settings directly: `kp = 40
N·m/rad`, native damping ratio `1.0`, joint Coulomb friction `0.04903325 N·m`, and
damping `0.012606274336967262 N·m·s/rad`. MuJoCo derives each actuator's `kv` from its
joint inertia and the requested damping ratio. These are illustrative simulation values,
not measured or identified firmware parameters. There is no integral controller, encoder
quantization, target-motion profile, voltage command, back-EMF, current simulation,
thermal model or backlash.

The actuator's position control range is its CAD joint range in radians. The generated
actuator stores LeRobot action bounds in its native `user` fields. `SO101Arm` reads
those bounds and converts normalized travel to that range. Native MuJoCo position servos
run at every physics substep without Python callbacks.

## Manufacturer reference versus simulation settings

The [STS3215
datasheet](https://www.feetechrc.com/Data/feetechrc/upload/file/20200611/6372749961523760249976542.pdf)
lists 19.5 kgf·cm peak stall torque at 7.4 V, or **1.91229675 N·m**. That value bounds
each actuator; the gripper uses half of it (**0.956148375 N·m**). Stall torque is not a
continuous-duty rating. Manufacturer data and unmeasured quantities remain in the model
`information.json` as engineering references. There is no separate servo JSON.

The user reported the installed 7.4 V STS3215 variant. Installed supply voltage, real
servo response and payload capability were not measured here. The earlier fitted
electrical endpoint model and Python PI-D approximation have been removed. Changing
build-time servo settings requires rebuilding `so101.xml`.

## Geometry, mass and contact

The exported model retains 257 native CAD solids, seven rigid links and six mate
axes/ranges. The live Onshape assembly has an additional wrist-camera bracket; it is not
included in the pinned simulation export. CAD mass/COM/inertia remain unassigned.
Simulation link mass/COM/inertia come from the reference [TheRobotStudio SO-101
URDF](https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf),
aligned to the CAD. Their total mass is 0.632006 kg; they are not physical measurements
of this assembly. No invented rotor inertia is added.

The builder creates separate CoACD convex collision pieces with a 1 mm concavity target
while preserving the CAD visual meshes. Material patches are rejoined before
decomposition. Source components with no thickness remain visual-only; no thickness or
physical identity is inferred for them. These collision shapes are an approximation, not
exact triangle-level CAD contact.

MJCF enables nonadjacent self-collision, cross-arm contact and floor contact. Directly
mating parent/child links are explicitly excluded at their shaft/bearing interfaces. The
standalone verifier checks the saved model and floor support; the collision-disabled
regression checks that contacts change the trajectory. Neither test identifies real
stiffness, friction or force accuracy. A successful simulation or ACT rollout is not
hardware validation.

When hardware is active, the bridge reads measured positions and holds the virtual
arm with native joint tracking constraints while disabling its simulated motors.
These proxies support virtual contacts, not physical force feedback or identified
hardware dynamics. State exposes measured positions in both backends; simulated
velocities are not labeled as hardware observations.
