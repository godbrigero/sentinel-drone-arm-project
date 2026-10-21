# MuJoCo typing overlay

These local `.pyi` files describe the **MuJoCo 3.16.0** Python bindings pinned in `pyproject.toml` and `uv.lock`. They improve autocomplete and static checks; they do not wrap MuJoCo, change physics, or add runtime classes. Keep `typings/` on the type checker's stub search path, **not** on `PYTHONPATH`.

## Coverage

- `mujoco.MjModel`, `MjData`, model options, contacts, camera/scene/render structs, scalar fields, and NumPy array dtypes.
- Named model/data views such as `model.joint(name).range`, `data.joint(name).qvel`, `data.body(name).xpos`, and `model.actuator(name).id`.
- Simulation/kinematics/Jacobian/state functions and vector, matrix, and quaternion helpers.
- All 79 exported binding enum classes and constants.
- Model specification/editing classes (`MjSpec`, `MjsBody`, `MjsJoint`, etc.).
- Typed control/passive, actuator, sensor, contact-filter, and timing callback signatures.
- `Renderer` context management and scene/render APIs; passive viewer `Handle`, locking, cameras, synchronization, overlays, and launch functions.

The installed wheel contains no `.pyi` files. Its Python annotations and C-extension signatures were inspected and used as the source. Initial declarations were extracted with `pybind11-stubgen`, then corrected for invalid C++ names, named-view return types, generated enum overload conflicts, optional arguments, callbacks, and context managers. Python implementation docstrings and platform-specific implementation internals were omitted. This is a checked-in overlay, not a claim that every API has been manually validated.

## Limitations

Array dtype is represented; dimensions, contiguity, writability, and physical units generally remain runtime constraints. Renderer output remains a general ndarray because RGB, depth, and segmentation modes produce different dtypes. Some opaque binding/specification and low-level memory callbacks deliberately retain `Any`; a fabricated precise signature would be misleading. `resolve_orientation` and `set_to_muscle` represent their ambiguous binding defaults using keyword arguments. Platform-specific rendering internals, MJX, and other optional extension packages are outside the overlay. Upgrade and revalidate the overlay when changing MuJoCo versions.

## Checking

The workspace typechecker configuration supplies this directory as the stub path. To check independently from the project root:

```sh
MYPYPATH=typings uv run mypy --explicit-package-bases typings/smoke.py airtruck/raw_controls/so101/controller.py
uv run mypy --follow-imports=skip typings/mujoco
```

`smoke.py` is a static usage example, not a simulation entry point. `assert_type` checks ensure important views and arrays remain concrete rather than silently becoming `Any`. An additional negative check confirmed that reversed model/data parameters, an invalid control callback, a string viewer flag, and assigning a joint array to a string are rejected. No runtime import of this directory is required.

## Attribution

Derived API declarations: MuJoCo, Copyright 2022 DeepMind Technologies Limited, Apache License 2.0. Upstream: https://github.com/google-deepmind/mujoco. The upstream wheel's license is retained as `LICENSE.mujoco`. Local generation, annotation corrections, and examples are maintained with this project; these are not official upstream type stubs.
