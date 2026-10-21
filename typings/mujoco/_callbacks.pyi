# MuJoCo 3.16.0 API declarations; Copyright 2022 DeepMind Technologies Limited.
# Derived under Apache-2.0; see typings/LICENSE.mujoco and typings/README.md.
from __future__ import annotations
import typing
from ._structs import MjModel, MjData
ControlCallback = typing.Callable[[MjModel, MjData], None]
ActuatorCallback = typing.Callable[[MjModel, MjData, int], float]
ContactFilterCallback = typing.Callable[[MjModel, MjData, int, int], int]
SensorCallback = typing.Callable[[MjModel, MjData, int], None]
__all__: list[str] = ['get_mjcb_act_bias', 'get_mjcb_act_dyn', 'get_mjcb_act_gain', 'get_mjcb_contactfilter', 'get_mjcb_control', 'get_mjcb_passive', 'get_mjcb_sensor', 'get_mjcb_time', 'get_mju_user_free', 'get_mju_user_malloc', 'get_mju_user_warning', 'set_mjcb_act_bias', 'set_mjcb_act_dyn', 'set_mjcb_act_gain', 'set_mjcb_contactfilter', 'set_mjcb_control', 'set_mjcb_passive', 'set_mjcb_sensor', 'set_mjcb_time', 'set_mju_user_free', 'set_mju_user_malloc', 'set_mju_user_warning']

def get_mjcb_act_bias() -> ActuatorCallback | None:
    ...

def get_mjcb_act_dyn() -> ActuatorCallback | None:
    ...

def get_mjcb_act_gain() -> ActuatorCallback | None:
    ...

def get_mjcb_contactfilter() -> ContactFilterCallback | None:
    ...

def get_mjcb_control() -> ControlCallback | None:
    ...

def get_mjcb_passive() -> ControlCallback | None:
    ...

def get_mjcb_sensor() -> SensorCallback | None:
    ...

def get_mjcb_time() -> typing.Callable[[], float] | None:
    ...

def get_mju_user_free() -> typing.Any:
    ...

def get_mju_user_malloc() -> typing.Any:
    ...

def get_mju_user_warning() -> typing.Any:
    ...

def set_mjcb_act_bias(arg0: ActuatorCallback | None) -> None:
    ...

def set_mjcb_act_dyn(arg0: ActuatorCallback | None) -> None:
    ...

def set_mjcb_act_gain(arg0: ActuatorCallback | None) -> None:
    ...

def set_mjcb_contactfilter(arg0: ContactFilterCallback | None) -> None:
    ...

def set_mjcb_control(arg0: ControlCallback | None) -> None:
    ...

def set_mjcb_passive(arg0: ControlCallback | None) -> None:
    ...

def set_mjcb_sensor(arg0: SensorCallback | None) -> None:
    ...

def set_mjcb_time(arg0: typing.Callable[[], float] | None) -> None:
    ...

def set_mju_user_free(arg0: typing.Any) -> None:
    ...

def set_mju_user_malloc(arg0: typing.Any) -> None:
    ...

def set_mju_user_warning(arg0: typing.Any) -> None:
    ...
