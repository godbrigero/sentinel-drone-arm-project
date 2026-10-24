"""Backend adapters; public views keep physics buffers out of test controls."""
from collections.abc import Callable, Mapping
from typing import Protocol
from airtruck.raw_api.so101.position import JointPosition
from airtruck.raw_api.so101.controller import SimSO101
from airtruck.raw_api.so101.io import ArmController, SimulatedArmController, JointName, PositionCommand


class SystemAdapter(Protocol):
    @property
    def actuator_ids(self) -> tuple[int, ...]: ...
    def interface(self, require_open: Callable[[], None]) -> object: ...
    def update_control(self) -> None: ...
    def reset_control(self) -> None: ...
    def disable(self) -> None: ...


class ArmView(SimulatedArmController):
    def __init__(self, backend: SimSO101, require_open: Callable[[], None]) -> None:
        self._backend = backend
        self._require_open = require_open

    def set_positions(self, positions: PositionCommand) -> None:
        self._require_open()
        self._backend.set_positions(positions)

    def set_motor_voltages(self, voltages: Mapping[JointName, float]) -> None:
        self._require_open()
        self._backend.set_motor_voltages(voltages)

    def use_position_control(self) -> None:
        self._require_open()
        self._backend.use_position_control()

    @property
    def motor_voltages(self) -> dict[JointName, float]:
        self._require_open()
        return dict(zip(self._backend.names, map(float, self._backend.data.ctrl[self._backend._actuators])))

    @property
    def motor_torques_nm(self) -> dict[JointName, float]:
        self._require_open()
        return dict(zip(self._backend.names, map(float, self._backend.data.actuator_force[self._backend._actuators])))

    @property
    def positions(self) -> dict[JointName, JointPosition]:
        self._require_open()
        return self._backend.positions

    @property
    def targets(self) -> dict[JointName, JointPosition]:
        self._require_open()
        return self._backend.targets

    @property
    def normalized_positions(self) -> dict[JointName, JointPosition]:
        self._require_open()
        return self._backend.normalized_positions

    def set_normalized_positions(self, positions: PositionCommand) -> None:
        self._require_open()
        self._backend.set_normalized_positions(positions)

    @property
    def enabled(self) -> bool:
        self._require_open()
        return self._backend.enabled

    def set_enabled(self, enabled: bool) -> None:
        self._require_open()
        self._backend.set_enabled(enabled)


class SO101Adapter:
    def __init__(self, backend: SimSO101) -> None:
        self.backend = backend

    @property
    def actuator_ids(self) -> tuple[int, ...]:
        return tuple(int(i) for i in self.backend._actuators)

    def interface(self, require_open: Callable[[], None]) -> ArmController:
        return ArmView(self.backend, require_open)

    def update_control(self) -> None:
        self.backend.update_control()

    def reset_control(self) -> None:
        self.backend.reset_control()

    def disable(self) -> None:
        self.backend.set_enabled(False)
        self.backend.model.actuator_gainprm[self.backend._actuators] = 0
        self.backend.model.actuator_biasprm[self.backend._actuators] = 0
        self.backend.data.ctrl[self.backend._actuators] = 0
