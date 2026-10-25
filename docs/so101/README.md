# SO-101 native CAD → MuJoCo → Gymnasium

The pinned simulation uses 257 native Onshape solids, seven rigid groups and six
revolute joint axes/ranges. `onshape-to-robot` 1.8.3 builds MJCF; MuJoCo 3.16.0 runs
native position servos. The official Sentinel aircraft is not part of this arm export.

## Current CAD and pinned simulation

The live follower assembly contains 258 solids, including the original [TheRobotStudio
wrist-camera
bracket](https://github.com/TheRobotStudio/SO-ARM100/tree/a758567c3978dfeefe282ede0500085a48fe8f78/Optional/SO101_Wrist_Cam_Hex-Nut_Mount_32x32_UVC_Module).
The bracket attaches to the wrist-roll housing with 8.1 mm hole spacing and supports the
published 32 × 32 mm UVC board with 27 × 27 mm camera mounting holes. The camera board
and screws are not modeled, and its exact camera model and measured dimensions are
unknown. CAD mass, COM and inertia remain unassigned.

The simulation stays pinned to the earlier 257-part export. It does not include the
newer bracket or a camera sensor. Rebuilding the XML does not fetch updated CAD. Onshape
remains the source of truth; source links and numeric records are in
`models/so101/information.json`. This migration preserves the exported visual geometry
and reference link inertias.

## Run and verify

```sh
uv sync --locked
uv run --locked python -m airtruck.examples.two_arms
uv run --locked mjpython -m airtruck.examples.two_arms --render human
uv run --locked python scripts/so101/build_model.py
uv run --locked python scripts/so101/verify.py
```

The two-arm example controls both objects before advancing one shared world.
It runs for eight simulated seconds and exits. `two_arms()` in the example creates
names, placements and initial joint poses. `MujocoDriver` attaches the objects to
`worlds/bench/scene.xml`; `--world` selects another base template. Instantiate one
object for a single arm. The standalone native scenes are retained for direct
MuJoCo model verification.

`build_model.py` checks source hashes, rejoins material patches, splits connected
components and runs CoACD to prepare convex collision pieces at a 1 mm concavity target.
It embeds those pieces and six mating-link exclusions directly in MJCF. It preserves
visual triangles and reference inertias, reports zero-thickness components left
visual-only, and compiles/checks a candidate before replacing the model. Building
requires the development dependencies and can take several minutes; normal simulation
performs none of this work. CoACD 1.0.9 can print “No more convex hulls to merge” after
its merge candidates are exhausted, even though no hull-count limit was requested. The
verifier independently checks the result; these upstream messages are not suppressed.

`verify.py` loads the saved XML directly through MuJoCo, without Gymnasium. It checks
source hashes, joint frames/ranges, reference masses/COM/inertias, collision
frames/masks and source-vertex coverage. It also verifies actuator transmissions,
control ranges, native action bounds and initial poses. Native MuJoCo warnings fail
verification; scene-only arena sizing is rejected in the reusable arm asset. It rejects
initial penetrations, tests each position actuator under gravity, and drops a free
object onto the floor. It also drives the elbow into a nonadjacent self-contact and runs
the two-arm motion with contacts enabled/disabled, requiring cross-arm forces and a
changed trajectory. The vertex coverage test is a sampled check, not a global
surface-error proof. The [simulation guide](../simulation.md) contains the complete API,
local LeRobot dataset/ACT example and vector-environment usage.

## Model files

- `source/cad-manifest.json`: pinned CAD revision, rigid memberships, axes, ranges, mesh hashes and reference inertias.
- `assets/`: required visual mesh inputs; retained triangles are unchanged.
- `so101.xml`: generated SI physics model with position servos, embedded collision pieces and mating-link exclusions.
- `information.json`: compact engineering record, not runtime application state.

The builder creates its exporter configuration in a temporary directory. It uses
retained CAD manifests and meshes without Onshape API keys or CAD writes. The model
owns its collisions and native servos. `SO101Arm` owns joint bindings and normalized
units. `SO101Follower` and `SO101Leader` add the native LeRobot roles and hardware
flag. `MujocoDriver` coordinates the objects; `as_gymnasium()` supplies the separate
training/episode interface. See [model fidelity](motor-model.md) and
[hardware usage](../hardware/README.md).
