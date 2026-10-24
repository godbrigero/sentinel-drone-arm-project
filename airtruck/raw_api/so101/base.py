"""Shared position behavior, separate from the abstract I/O contracts."""
from collections.abc import Mapping
from airtruck.raw_api.so101.io import ArmReader, ArmController
from airtruck.raw_api.so101.joints import JointName, PositionCommand
from airtruck.raw_api.so101.position import JointPosition, PositionUnit


class PositionReader(ArmReader):
    @property
    def normalized_positions(self) -> dict[JointName, JointPosition]:
        return {name: position.to(PositionUnit.NORMALIZED).clamped()
                for name, position in self.positions.items()}


class PositionController(PositionReader, ArmController):
    def set_normalized_positions(self, positions: PositionCommand) -> None:
        if not isinstance(positions, Mapping):
            raise TypeError("Position commands must be a mapping of joint names to JointPosition")
        for position in positions.values():
            if not isinstance(position, JointPosition):
                raise TypeError("Position command values must be JointPosition")
            if position.unit != PositionUnit.NORMALIZED:
                raise ValueError("Expected normalized position units")
        self.set_positions(positions)
