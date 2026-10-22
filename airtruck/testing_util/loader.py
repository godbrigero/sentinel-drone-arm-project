"""Validated world manifests and explicit adapter registration; no dynamic imports."""
from collections.abc import Callable, Mapping
import json
import math
from pathlib import Path
import re
from typing import cast
import mujoco
import numpy as np
from airtruck.raw_api.so101.controller import ServoSpec, SimSO101
from airtruck.testing_util.adapters import SO101Adapter, SystemAdapter
from airtruck.testing_util.mujoco_world import MujocoWorld, Placement
from airtruck.testing_util.types import Pose

AdapterFactory = Callable[[mujoco.MjModel, mujoco.MjData, str, Mapping[str, object]], SystemAdapter]


def _mapping(value: object, label: str) -> dict[str, object]:
    if not isinstance(value, dict) or not all(isinstance(k, str) for k in value):
        raise ValueError(f"{label} must be an object")
    return cast(dict[str, object], value)


def _read(path: Path) -> dict[str, object]:
    def unique(pairs: list[tuple[str, object]]) -> dict[str, object]:
        result: dict[str, object] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f"Duplicate JSON key {key!r} in {path}")
            result[key] = value
        return result
    return _mapping(json.loads(path.read_text(), object_pairs_hook=unique), str(path))


def _string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{label} must be a nonempty string")
    return value


def _fields(value: dict[str, object], allowed: set[str], label: str) -> None:
    if extra := value.keys() - allowed:
        raise ValueError(f"Unknown {label} fields: {sorted(extra)}")


def servo_spec(value: Mapping[str, object]) -> ServoSpec:
    """Validate the fields consumed by the servo loop, preserving no unchecked casts."""
    datasheet = _mapping(value.get("datasheet"), "datasheet")
    tuning = _mapping(value.get("controller_demo_tuning"), "controller_demo_tuning")

    def number(mapping: Mapping[str, object], key: str, *, positive: bool = True) -> float:
        v = mapping.get(key)
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
            raise ValueError(f"{key} must be finite")
        if v < 0 or (positive and v == 0):
            raise ValueError(f"{key} must be {'positive' if positive else 'nonnegative'}")
        return float(v)

    counts = number(datasheet, "encoder_counts")
    if not counts.is_integer() or counts != 4096:
        raise ValueError("SO101 virtual encoder requires 4096 counts")
    return {
        "datasheet": {"encoder_counts": int(counts), "voltage_V": number(datasheet, "voltage_V")},
        "controller_demo_tuning": {
            "target_speed_rad_s": number(tuning, "target_speed_rad_s"),
            "target_acceleration_rad_s2": number(tuning, "target_acceleration_rad_s2"),
            "kp_V_per_rad": number(tuning, "kp_V_per_rad", positive=False),
            "ki_V_per_rad_s": number(tuning, "ki_V_per_rad_s"),
            "kd_V_s_per_rad": number(tuning, "kd_V_s_per_rad", positive=False),
        },
    }


def _so101(model: mujoco.MjModel, data: mujoco.MjData, prefix: str,
           config: Mapping[str, object]) -> SystemAdapter:
    return SO101Adapter(SimSO101(model, data, servo_spec=servo_spec(config), prefix=prefix))


def _pose(value: object) -> Pose:
    obj = _mapping(value, "pose")
    _fields(obj, {"position_m", "quaternion_wxyz"}, "pose")
    p, q = obj.get("position_m", [0, 0, 0]), obj.get("quaternion_wxyz", [1, 0, 0, 0])
    if not isinstance(p, list) or len(p) != 3 or not isinstance(q, list) or len(q) != 4:
        raise ValueError("pose requires 3 position and 4 quaternion values")
    if any(isinstance(x, bool) or not isinstance(x, (int, float)) for x in (*p, *q)):
        raise ValueError("pose coordinates must be numbers")
    return Pose(tuple(p), tuple(q))


def load_world(directory: str | Path, *, adapters: Mapping[str, AdapterFactory] | None = None) -> MujocoWorld:
    root = Path(directory).resolve()
    manifest = _read(root / "world.json")
    _fields(manifest, {"schema_version", "scene", "systems", "entities", "inter_system_collisions"}, "world")
    if type(manifest.get("schema_version")) is not int or manifest["schema_version"] != 1:
        raise ValueError("Unsupported world schema_version")
    entries, entities = manifest.get("systems", []), manifest.get("entities", [])
    if not isinstance(entries, list) or not isinstance(entities, list):
        raise ValueError("systems and entities must be arrays")
    collisions = manifest.get("inter_system_collisions", False)
    if not isinstance(collisions, bool):
        raise ValueError("inter_system_collisions must be boolean")
    if collisions and len(entries) > 30:
        raise ValueError("At most 30 collision groups are supported")
    initial_positions: dict[str, float] = {}
    registry: dict[str, AdapterFactory] = {"so101": _so101}
    if adapters:
        registry.update(adapters)
    spec = mujoco.MjSpec.from_file(str(root / _string(manifest.get("scene"), "scene")))
    bindings: list[tuple[str, str, AdapterFactory, dict[str, object]]] = []
    entity_specs: dict[str, tuple[str, Placement]] = {}

    def declare(name: str, body: str, placement: object) -> None:
        if name in entity_specs:
            raise ValueError(f"Duplicate entity/system name {name!r}")
        if placement not in ("fixed", "free", "kinematic"):
            raise ValueError(f"Invalid placement for {name!r}")
        entity_specs[name] = (body, cast(Placement, placement))

    for system_index, raw in enumerate(entries):
        entry = _mapping(raw, "system")
        _fields(entry, {"name", "model", "root_body", "controller", "controller_config", "pose", "placement", "parent_body", "initial_joint_positions_rad"}, "system")
        name = _string(entry.get("name"), "system name")
        if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
            raise ValueError("System names must be identifiers")
        kind = _string(entry.get("controller"), "controller")
        if kind not in registry:
            raise ValueError(f"Unknown controller {kind!r}; register its adapter explicitly")
        prefix = name + "/"
        body_name = _string(entry.get("root_body"), "root_body")
        declare(name, prefix + body_name, entry.get("placement", "fixed"))
        config = _read(root / _string(entry.get("controller_config"), "controller_config"))
        child = mujoco.MjSpec.from_file(str(root / _string(entry.get("model"), "model")))
        if collisions:
            # A render mesh may be planar decoration. Only volumetric mesh hulls
            # can participate in contact; original visual vertices are preserved.
            reference = child.compile()
            for geom in child.geoms:
                geom.contype = geom.conaffinity = 0
                if geom.type != mujoco.mjtGeom.mjGEOM_MESH:
                    continue
                mesh_id = reference.mesh(geom.meshname).id
                start = reference.mesh_vertadr[mesh_id]
                count = reference.mesh_vertnum[mesh_id]
                vertices = reference.mesh_vert[start:start + count].astype(np.float64)
                if np.linalg.matrix_rank(vertices - vertices.mean(axis=0), tol=1e-8) < 3:
                    continue
                geom.solref[:] = spec.default.geom.solref
                geom.solimp[:] = spec.default.geom.solimp
                geom.friction[:] = spec.default.geom.friction
                geom.contype = 1 << system_index
                geom.conaffinity = ((1 << len(entries)) - 1) ^ geom.contype
        for local_name, value in _mapping(entry.get("initial_joint_positions_rad", {}), "initial joints").items():
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
                raise ValueError("Initial joint positions must be finite radians")
            initial_positions[prefix + local_name] = float(value)
        # The environment owns the integration timestep for all systems.
        child.option.timestep = spec.option.timestep
        body = child.body(body_name)
        if body is None:
            raise ValueError(f"Missing root body {body_name!r}")
        parent_name = entry.get("parent_body")
        parent = spec.worldbody if parent_name is None else spec.body(_string(parent_name, "parent_body"))
        if parent is None:
            raise ValueError(f"Missing parent body {parent_name!r}; declare parent systems first")
        pose = _pose(entry.get("pose", {}))
        frame = parent.add_frame(pos=pose.position_m, quat=pose.quaternion_wxyz)
        frame.attach_body(body, prefix=prefix)
        bindings.append((name, prefix, registry[kind], config))
    for raw in entities:
        entry = _mapping(raw, "entity")
        _fields(entry, {"name", "body", "placement"}, "entity")
        declare(_string(entry.get("name"), "entity name"),
                _string(entry.get("body"), "entity body"), entry.get("placement", "fixed"))
    model = spec.compile()
    if not math.isfinite(model.opt.timestep) or model.opt.timestep <= 0:
        raise ValueError("World timestep must be positive and finite")
    data = mujoco.MjData(model)
    for name, value in initial_positions.items():
        joint = model.joint(name).id
        if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError("Initial radian positions require hinge joints")
        if model.jnt_limited[joint] and not model.jnt_range[joint, 0] <= value <= model.jnt_range[joint, 1]:
            raise ValueError(f"Initial position outside limits: {name}")
        data.qpos[model.jnt_qposadr[joint]] = value
    systems: dict[str, SystemAdapter] = {}
    try:
        for name, prefix, factory, config in bindings:
            systems[name] = factory(model, data, prefix, config)
        resolved = {name: (model.body(body).id, placement)
                    for name, (body, placement) in entity_specs.items()}
        return MujocoWorld(model, data, systems, resolved)
    except BaseException as original:
        cleanup_errors: list[Exception] = []
        for adapter in systems.values():
            try:
                adapter.disable()
            except Exception as error:
                cleanup_errors.append(error)
        if cleanup_errors:
            raise BaseExceptionGroup("World loading and cleanup failed", [original, *cleanup_errors]) from None
        raise
