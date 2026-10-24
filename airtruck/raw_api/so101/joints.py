"""SO-101 joint names, order and accepted position commands."""
from collections.abc import Mapping
from typing import Literal, TypeAlias
from airtruck.raw_api.so101.position import JointPosition

JointName: TypeAlias = Literal[
    "shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"
]
PositionCommand: TypeAlias = Mapping[JointName, JointPosition]

JOINTS: tuple[JointName, ...] = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
