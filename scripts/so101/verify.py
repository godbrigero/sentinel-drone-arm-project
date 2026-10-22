"""Fresh-load checks for CAD export, motor constants, and individual servo motion."""

import sys, json, hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import mujoco

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
ROOT = Path(__file__).resolve().parents[2] / "models" / "so101"
from airtruck.raw_api.so101.controller import SimSO101
from airtruck.testing_util.mujoco_world import MujocoWorld
from airtruck.testing_util.adapters import SO101Adapter

c = json.loads((ROOT / "source/cad-manifest.json").read_text())
s = json.loads((ROOT / "runtime/servo-model.json").read_text())
# Derived parameters live in the generated model, not a second JSON copy.
fit = {
    node.attrib["name"]: float(node.attrib["data"])
    for node in ET.parse(ROOT / "so101.xml").findall("custom/numeric")
}
m = mujoco.MjModel.from_xml_path(str(ROOT / "so101.xml"))
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)
assert m.nq == m.nu == 6 and m.nbody == 8
assert len({p for l in c["links"] for p in l["parts"]}) == 257
for mesh in c["meshes"]:
    assert (
        hashlib.sha256((ROOT / mesh["file"]).read_bytes()).hexdigest() == mesh["sha256"]
    )
errors = []
for j in c["joints"]:
    idx = m.joint(j["name"]).id
    T = np.array(j["T_world_joint"])
    assert np.allclose(d.xanchor[idx], T[:3, 3], atol=1e-6)
    assert np.allclose(d.xaxis[idx], -T[:3, 2], atol=2e-6)
    assert np.allclose(m.jnt_range[idx], j["limits_rad"], atol=1e-9)
for body in range(1, m.nbody):
    assert m.body_mass[body] > 0
    inertia = m.body_inertia[body]
    assert np.all(inertia > 0) and 2 * max(inertia) <= sum(inertia) + 1e-12
R = fit["terminal_resistance_ohm"] + fit["estimated_series_resistance_ohm"]
Kt = fit["output_Kt_Nm_A"]
Ke = fit["estimated_output_Ke_V_s_rad"]
V = fit["voltage_V"]
fc = fit["estimated_coulomb_Nm"]
b = fit["estimated_viscous_Nm_s_rad"]
w = fit["no_load_speed_rad_s"]
assert abs(Kt * V / R - fc - 19.5 * 0.0980665) < 1e-10
assert abs(Kt * (V - Ke * w) / R - fc - b * w) < 1e-10
bus = SimSO101(m, d, servo_spec=s)
world = MujocoWorld(m, d, {"arm": SO101Adapter(bus)}, {})
for idx in range(6):
    world.reset()
    for _ in range(5000):
        world.step()
    baseline = d.qpos.copy()
    bus.command_radians(bus.names[idx], np.pi / 36)
    for _ in range(3000):
        world.step()
    error = float(abs(d.qpos[idx] - np.pi / 36))
    other = float(max(np.abs(np.delete(d.qpos - baseline, idx))))
    errors.append(
        {
            "joint": bus.names[idx],
            "target_rad": np.pi / 36,
            "actual_rad": float(d.qpos[idx]),
            "error_rad": error,
            "other_joint_change_max_rad": other,
            "final_max_speed_rad_s": float(max(abs(d.qvel))),
        }
    )
    assert np.isfinite(d.qpos).all() and error < np.pi / 180 and other < np.pi / 180, (idx, errors[-1])
    assert not any(x.number for x in d.warning)
# Enforce target limits at the bus and voltage/current bounds at actuators.
for n in bus.names:
    bus.command_radians(n, 10000)
assert np.all(bus.target <= m.jnt_range[:, 1])
for n in bus.names:
    bus.command_radians(n, -10000)
assert np.all(bus.target >= m.jnt_range[:, 0])
report = {
    "status": "passed",
    "mujoco_version": mujoco.__version__,
    "cad_microversion": c["microversion"],
    "solid_parts": 257,
    "rigid_links": 7,
    "joints": 6,
    "reference_total_mass_kg": float(m.body_mass.sum()),
    "independent_motion": errors,
    "electrical_endpoints": "Published stall torque and no-load speed reproduced algebraically by fitted model",
    "checks": [
        "fresh MJCF and mesh hashes",
        "pivot/axis/range versus native mates",
        "positive inertias and triangle inequality",
        "six dynamic position commands with gravity",
        "finite states and no MuJoCo warnings",
        "command limit clamping",
    ],
    "not_validated": [
        "hardware parameter identification",
        "thermal duty limits",
        "collision/contact behavior",
        "real encoder calibration",
        "real firmware PID equivalence",
    ],
}
print(json.dumps(report, indent=2))

world.close()
