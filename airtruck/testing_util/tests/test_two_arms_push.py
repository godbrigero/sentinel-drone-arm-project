"""A full-voltage arm physically pushes an extended, lightly driven arm.

The matched collision-disabled run establishes causality. No external force,
pose placement, or follower commands are applied during the push.
"""
from dataclasses import dataclass
import json
from pathlib import Path
import pytest
from airtruck.raw_api.so101.io import SimulatedArmController
from airtruck.testing_util import WorldControl, load_world
from airtruck.testing_util import simulation

ROOT = Path(__file__).resolve().parents[3]
WORLD = ROOT / "worlds/two_arms_push"


@dataclass(frozen=True)
class PushResult:
    displacement_x_m: float
    pan_change_rad: float
    peak_contact_force_n: float
    peak_penetration_m: float
    peak_drive_torque_nm: float
    contact_samples: int


def push(world: WorldControl) -> PushResult:
    weak = world.system("weak", SimulatedArmController)
    strong = world.system("strong", SimulatedArmController)
    world.advance(3.0)
    assert not world.contacts(), "The extended arms must start separated"
    start_x = world.pose(world.entity("weak_hand")).position_m[0]
    start_pan = weak.positions["shoulder_pan"]
    weak_target = weak.targets.copy()
    # Same direction, dramatically different effort. Other joints hold extension.
    weak.set_motor_voltages({"shoulder_pan": -0.2})
    strong.set_motor_voltages({"shoulder_pan": -7.4})
    peak_force = peak_penetration = peak_torque = 0.0
    samples = 0
    for _ in range(160):
        world.advance(0.005)
        assert weak.motor_voltages["shoulder_pan"] == pytest.approx(-0.2)
        assert strong.motor_voltages["shoulder_pan"] == pytest.approx(-7.4)
        assert weak.targets == weak_target
        for contact in world.contacts():
            assert {contact.body_a.split('/')[0], contact.body_b.split('/')[0]} == {"weak", "strong"}
            peak_force = max(peak_force, contact.normal_force_n)
            peak_penetration = max(peak_penetration, contact.penetration_m)
            if contact.normal_force_n > 0.1:
                samples += 1
        peak_torque = max(peak_torque, abs(strong.motor_torques_nm["shoulder_pan"]))
    return PushResult(world.pose(world.entity("weak_hand")).position_m[0] - start_x,
                      weak.positions["shoulder_pan"] - start_pan,
                      peak_force, peak_penetration, peak_torque, samples)


def without_collisions(directory: Path) -> None:
    manifest = json.loads((WORLD / "world.json").read_text())
    manifest["inter_system_collisions"] = False
    manifest["scene"] = str(WORLD / manifest["scene"])
    for system in manifest["systems"]:
        for key in ("model", "controller_config"):
            system[key] = str((WORLD / system[key]).resolve())
    (directory / "world.json").write_text(json.dumps(manifest))


@simulation("worlds/two_arms_push")
def test_two_arms_collision_push(world: WorldControl, tmp_path: Path) -> None:
    without_collisions(tmp_path)
    with load_world(tmp_path) as baseline_world:
        baseline = push(baseline_world)
    assert baseline.contact_samples == 0
    assert abs(baseline.displacement_x_m) < 0.01

    print("\nExtended arms: left motor at -0.2 V, right motor at full -7.4 V.", flush=True)
    result = push(world)
    assert result.contact_samples > 10
    assert result.peak_contact_force_n > 1
    assert result.peak_penetration_m < 0.003
    assert 1.8 < result.peak_drive_torque_nm <= 1.96133 + 1e-6
    assert result.displacement_x_m < -0.15
    assert result.pan_change_rad < -0.5
    assert result.displacement_x_m < baseline.displacement_x_m - 0.15
    print(f"Contact pushed the weak arm {-result.displacement_x_m * 1000:.1f} mm left; "
          f"without contact it moved {-baseline.displacement_x_m * 1000:.1f} mm. "
          f"Peak contact force {result.peak_contact_force_n:.1f} N.", flush=True)
    # End the brief full-drive pulse, then hold so the final pose can be inspected.
    for name in ("weak", "strong"):
        world.system(name, SimulatedArmController).use_position_control()
    world.advance(3.0)


@simulation("worlds/so101_bench")
def test_voltage_override_validation_and_reset(world: WorldControl) -> None:
    arm = world.system("arm", SimulatedArmController)
    for value in (7.5, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            arm.set_motor_voltages({"shoulder_pan": value})
    arm.set_motor_voltages({"shoulder_pan": -0.2})
    world.step()
    assert arm.motor_voltages["shoulder_pan"] == pytest.approx(-0.2)
    snapshot = arm.motor_voltages
    snapshot["shoulder_pan"] = 99
    assert arm.motor_voltages["shoulder_pan"] == pytest.approx(-0.2)
    arm.use_position_control()
    world.step()
    assert abs(arm.motor_voltages["shoulder_pan"]) < 0.1
    arm.set_motor_voltages({"shoulder_pan": -7.4})
    world.reset()
    world.step()
    assert abs(arm.motor_voltages["shoulder_pan"]) < 0.1
    world.close()
    with pytest.raises(RuntimeError, match='closed'):
        arm.set_motor_voltages({"shoulder_pan": 0})
    with pytest.raises(RuntimeError, match='closed'):
        world.contacts()
