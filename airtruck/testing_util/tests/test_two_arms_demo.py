"""Visual call-and-response: two arms face each other and mirror a greeting.

Run: uv run --locked mjpython -m pytest --sim-gui -s -k two_arms_greeting
The existing SO-101 model has no collision contacts; this is coordinated motion.
"""
import pytest
from airtruck.raw_controls.so101.io import ArmController
from airtruck.testing_util import WorldControl
from airtruck.testing_util import simulation


@simulation("worlds/two_arms")
def test_two_arms_greeting(world: WorldControl) -> None:
    left = world.system("left", ArmController)
    right = world.system("right", ArmController)

    print("\nTwo arms: settle into their starting poses.", flush=True)
    world.advance(3.0)
    initial_right = right.targets

    for angle in (0.45, -0.45):
        print("Left arm greets; right arm waits.", flush=True)
        left.set_positions({"wrist_roll": angle, "gripper": -0.5})
        world.advance(3.0)
        assert right.targets == initial_right
        assert left.positions["wrist_roll"] == pytest.approx(angle, abs=0.04)

        print("Right arm responds to the left arm's measured pose.", flush=True)
        # Feedback drives the response, rather than two independent timelines.
        response = -left.positions["wrist_roll"]
        right.set_positions({"wrist_roll": response, "gripper": left.positions["gripper"]})
        world.advance(3.0)
        assert right.positions["wrist_roll"] == pytest.approx(response, abs=0.04)

        print("Both arms release the greeting.", flush=True)
        left.set_positions({"gripper": 0.0})
        right.set_positions({"gripper": 0.0})
        world.advance(1.5)
        initial_right = right.targets

    print("Both arms return home.", flush=True)
    left.set_positions({"wrist_roll": 0.0, "gripper": 0.0})
    right.set_positions({"wrist_roll": 0.0, "gripper": 0.0})
    world.advance(3.0)
    for arm in (left, right):
        assert arm.positions["wrist_roll"] == pytest.approx(0.0, abs=0.04)
