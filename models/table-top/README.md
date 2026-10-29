# Solid tabletop reconstruction for MuJoCo

`leader.xml` places the articulated SO-101 at the recorded work surface, facing
directly away from the wall. Its local forward axis (-Y) points along world -Y
in horizontal projection, and its base follows the work sheet's slight tilt.
The mounting datum is **(-0.160, 0.267, 0.69016313) m**. The CAD-root offset is
accounted for; the base is fixed while all six native joints remain movable.
This orientation is the requested placement, not an estimate of the captured
arm's yaw. It uses the existing SO-101 geometry and servo model unchanged.

Open the scene with the arm on macOS:

```sh
uv run --locked mjpython -m mujoco.viewer --mjcf models/table-top/leader.xml
```

The arm starts in its existing zero-joint configuration. Its root transform,
mounting datum and forward vector are recorded under `reconstruction.leader` in
`information.json`. The saved scene was checked for zero initial contacts,
correct mounting and facing direction, a fixed base under gravity, and native
wrist motion without MuJoCo warnings.

`scene.xml` is the arm-free, static workshop reconstructed with solid Blender
shapes. It includes textured OBJ visual meshes, analytic collision shapes,
lighting and three cameras. `tabletop.blend` is the editable, full-detail source
with all textures packed. The robot arm, mounting hardware and attached cables
have been removed; two reference markers retain its modeled location.

Run from the repository root on macOS:

```sh
uv run --locked mjpython -m mujoco.viewer --mjcf models/table-top/scene.xml
```

For headless loading:

```python
import mujoco

model = mujoco.MjModel.from_xml_path("models/table-top/scene.xml")
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
mount = data.site_xpos[model.site("removed_arm_mount").id].copy()
```

The arm-free `scene.xml` can also be passed as `world=` to `SystemDriver` when
adding systems. `leader.xml` already contains the arm; do not attach a second
leader to it.
The environment is entirely static, including the desktop accessories. It has
no arm, joints, actuators or hardware connection. Moving an accessory requires
separating it into a dynamic body and assigning its mass and inertia.

## Detail allocation

This is one model with spatially varying detail, not camera-distance LOD switching.
Flat surfaces remain economical meshes; extra triangles are kept for their edges,
fasteners and adjacent structure. Textures carry the fine surface markings.

| Area | Detail | Exported triangles |
| --- | --- | ---: |
| Plywood platform, work sheet, shelf dividers and fasteners | Full source detail | 3,572 |
| Folding table and adjacent desk structure | Reduced bevel and cylinder detail | 5,004 |
| Floor, wall, boxes, chair and desktop accessories | Simplified background geometry | 6,530 |

The total is 15,106 triangles in 26 meshes grouped by material and detail region.
`meshes/` contains OBJ files with explicit corner normals and UVs. `textures/`
contains the full 768 × 512 tabletop transfer, the 1536 × 1024 generated material
atlas, and reduced carpet/screen maps. MuJoCo texture color interpretation is
explicitly `linear` to retain the PNG display colors in its OpenGL renderer.
Blender lighting, bump shaders and color management are not reproduced by MJCF.

## Arm reference and coordinate frame

The reconstruction and MJCF use the same right-handed, Z-up, nominal-meter frame.
They are rotated +25° about Z relative to the Z-up conversion of the original
scan. In terms of original glTF coordinates, reconstruction coordinates are
`(cos(25°)*x + sin(25°)*z, sin(25°)*x - cos(25°)*z, y)`.

- `removed_arm_mount`: center of the removed mounting foot's lower face,
  **(-0.160, 0.267, 0.689) m**, modeled quaternion **(1, 0, 0, 0)** in wxyz order.
- The foot's object origin was **(-0.160, 0.267, 0.697) m**; its thickness was 16 mm.
- `arm_support_surface`: the work sheet at the same XY, **z = 0.69016313 m**, with
  the work sheet's modeled orientation. The original reconstruction's foot was
  about 1.16 mm below that surface; both datums are retained without conflating them.

Both are Blender empties and invisible MuJoCo sites. Their values, source hash,
modeled joint centers and frame transform are in `information.json` under
`removed_reconstructed_arm` and `reconstruction`. These are reconstruction values,
not calibrated physical arm placement or joint angles. Physical scale and actual
arm orientation remain unverified. To place an articulated CAD arm, account for
that model's own mounting datum and orientation.

## Contact geometry and verification

There are 103 box/cylinder collision primitives. Work-sheet and tabletop planes
retain the source pose and thickness. Separate bottom, side, center and rear
boards leave both front shelf cavities open. Legs and major room objects have
coarser collision shapes. Thin cables, straps, screw heads and small decoration
are visual only; bevels are approximated by their supporting primitives.

Visuals have collision masks disabled and belong to geom group 1. Collision
primitives are transparent in group 3. Reference sites are transparent in group 4.
For collision inspection, enable group 3 and give those geoms nonzero alpha.
Contact friction (0.7 sliding, 0.005 torsional, 0.0001 rolling) is an illustrative
simulation setting. No measured mass, inertia or material response is asserted.

Verified with MuJoCo 3.16.0: native compilation, asset loading, arm removal,
reference-site placement, overview/top/tabletop renders, and stable dynamic box
contacts on the work sheet, exposed plywood, folding table, desk, floor and lower
shelf. A horizontal collision ray through a front shelf opening reaches the rear
board. Those checks produced no MuJoCo warnings.

MuJoCo uses convex hulls for mesh collision; the separate primitive collision
model avoids closing the shelf openings. See the [MuJoCo mesh
reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html#asset-mesh).

## Other table assets

`table-cleaned.glb`, `table.xml`, `assets/`, and `worlds/table_top/` describe the
separate scan-based scene. Their original scan frame and arm-position records
are preserved. This solid reconstruction is selected explicitly through
`models/table-top/scene.xml`; its rotated frame must be accounted for when moving
placements between the two scenes.
