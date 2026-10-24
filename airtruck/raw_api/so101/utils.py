"""Command validation independent of controllers and transport."""
from collections.abc import Collection, Mapping
from airtruck.raw_api.so101.joints import JointName, PositionCommand
from airtruck.raw_api.so101.position import JointPosition


def validate_position_command(values: PositionCommand, joints: Collection[JointName]) -> None:
    """Validate the whole command without converting or replacing its values."""
    if not isinstance(values, Mapping):
        raise TypeError("Position commands must be a mapping of joint names to JointPosition")
    for name, position in values.items():
        if name not in joints:
            raise ValueError(f"Unknown SO-101 joint: {name}")
        if not isinstance(position, JointPosition):
            raise TypeError("Position command values must be JointPosition")
