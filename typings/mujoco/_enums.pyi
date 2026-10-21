# MuJoCo 3.16.0 API declarations; Copyright 2022 DeepMind Technologies Limited.
# Derived under Apache-2.0; see typings/LICENSE.mujoco and typings/README.md.
from __future__ import annotations
import typing
__all__: list[str] = ['mjrIndexType', 'mjrMeshPrimitiveType', 'mjrPixelFormat', 'mjrVertexAttributeType', 'mjrVertexAttributeUsage', 'mjtActuator', 'mjtAlignFree', 'mjtBias', 'mjtBuiltin', 'mjtButton', 'mjtCTimer', 'mjtCamLight', 'mjtCamOutBit', 'mjtCamera', 'mjtCatBit', 'mjtColorSpace', 'mjtConDataField', 'mjtCone', 'mjtConflict', 'mjtConstraint', 'mjtConstraintState', 'mjtCtrlChart', 'mjtCtrlInput', 'mjtDataType', 'mjtDepthMap', 'mjtDisableBit', 'mjtDyn', 'mjtEnableBit', 'mjtEq', 'mjtEvent', 'mjtFlexSelf', 'mjtFont', 'mjtFontScale', 'mjtFrame', 'mjtFramebuffer', 'mjtGain', 'mjtGeom', 'mjtGeomInertia', 'mjtGridPos', 'mjtInertiaFromGeom', 'mjtIntegrator', 'mjtItem', 'mjtJacobian', 'mjtJoint', 'mjtLRMode', 'mjtLabel', 'mjtLightType', 'mjtLimited', 'mjtLogLevel', 'mjtLogTopic', 'mjtMark', 'mjtMeshBuiltin', 'mjtMeshInertia', 'mjtMouse', 'mjtObj', 'mjtOrientation', 'mjtPertBit', 'mjtPluginCapabilityBit', 'mjtProjection', 'mjtRayDataField', 'mjtRndFlag', 'mjtSDFType', 'mjtSameFrame', 'mjtScaling', 'mjtSection', 'mjtSensor', 'mjtSleepPolicy', 'mjtSleepState', 'mjtSolver', 'mjtStage', 'mjtState', 'mjtStereo', 'mjtTexture', 'mjtTextureRole', 'mjtTimer', 'mjtTrn', 'mjtVisFlag', 'mjtWarning', 'mjtWrap']

class mjtDisableBit:
    __members__: typing.ClassVar[dict[str, mjtDisableBit]]
    mjDSBL_ACTUATION: typing.ClassVar[mjtDisableBit]
    mjDSBL_AUTORESET: typing.ClassVar[mjtDisableBit]
    mjDSBL_CLAMPCTRL: typing.ClassVar[mjtDisableBit]
    mjDSBL_CONSTRAINT: typing.ClassVar[mjtDisableBit]
    mjDSBL_CONTACT: typing.ClassVar[mjtDisableBit]
    mjDSBL_DAMPER: typing.ClassVar[mjtDisableBit]
    mjDSBL_EQUALITY: typing.ClassVar[mjtDisableBit]
    mjDSBL_EULERDAMP: typing.ClassVar[mjtDisableBit]
    mjDSBL_FILTERPARENT: typing.ClassVar[mjtDisableBit]
    mjDSBL_FRICTIONLOSS: typing.ClassVar[mjtDisableBit]
    mjDSBL_GRAVITY: typing.ClassVar[mjtDisableBit]
    mjDSBL_ISLAND: typing.ClassVar[mjtDisableBit]
    mjDSBL_LIMIT: typing.ClassVar[mjtDisableBit]
    mjDSBL_MIDPHASE: typing.ClassVar[mjtDisableBit]
    mjDSBL_MULTICCD: typing.ClassVar[mjtDisableBit]
    mjDSBL_NATIVECCD: typing.ClassVar[mjtDisableBit]
    mjDSBL_REFSAFE: typing.ClassVar[mjtDisableBit]
    mjDSBL_SENSOR: typing.ClassVar[mjtDisableBit]
    mjDSBL_SPRING: typing.ClassVar[mjtDisableBit]
    mjDSBL_WARMSTART: typing.ClassVar[mjtDisableBit]
    mjNDISABLE: typing.ClassVar[mjtDisableBit]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtEnableBit:
    __members__: typing.ClassVar[dict[str, mjtEnableBit]]
    mjENBL_DIAGEXACT: typing.ClassVar[mjtEnableBit]
    mjENBL_ENERGY: typing.ClassVar[mjtEnableBit]
    mjENBL_FWDINV: typing.ClassVar[mjtEnableBit]
    mjENBL_INVDISCRETE: typing.ClassVar[mjtEnableBit]
    mjENBL_IPC: typing.ClassVar[mjtEnableBit]
    mjENBL_OVERRIDE: typing.ClassVar[mjtEnableBit]
    mjENBL_SLEEP: typing.ClassVar[mjtEnableBit]
    mjNENABLE: typing.ClassVar[mjtEnableBit]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtJoint:
    __members__: typing.ClassVar[dict[str, mjtJoint]]
    mjJNT_BALL: typing.ClassVar[mjtJoint]
    mjJNT_FREE: typing.ClassVar[mjtJoint]
    mjJNT_HINGE: typing.ClassVar[mjtJoint]
    mjJNT_SLIDE: typing.ClassVar[mjtJoint]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtGeom:
    __members__: typing.ClassVar[dict[str, mjtGeom]]
    mjGEOM_ARROW: typing.ClassVar[mjtGeom]
    mjGEOM_ARROW1: typing.ClassVar[mjtGeom]
    mjGEOM_ARROW2: typing.ClassVar[mjtGeom]
    mjGEOM_BOX: typing.ClassVar[mjtGeom]
    mjGEOM_CAPSULE: typing.ClassVar[mjtGeom]
    mjGEOM_CYLINDER: typing.ClassVar[mjtGeom]
    mjGEOM_ELLIPSOID: typing.ClassVar[mjtGeom]
    mjGEOM_FLEX: typing.ClassVar[mjtGeom]
    mjGEOM_HFIELD: typing.ClassVar[mjtGeom]
    mjGEOM_LABEL: typing.ClassVar[mjtGeom]
    mjGEOM_LINE: typing.ClassVar[mjtGeom]
    mjGEOM_LINEBOX: typing.ClassVar[mjtGeom]
    mjGEOM_MESH: typing.ClassVar[mjtGeom]
    mjGEOM_NONE: typing.ClassVar[mjtGeom]
    mjGEOM_PLANE: typing.ClassVar[mjtGeom]
    mjGEOM_SDF: typing.ClassVar[mjtGeom]
    mjGEOM_SKIN: typing.ClassVar[mjtGeom]
    mjGEOM_SPHERE: typing.ClassVar[mjtGeom]
    mjGEOM_TRIANGLE: typing.ClassVar[mjtGeom]
    mjNGEOMTYPES: typing.ClassVar[mjtGeom]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtProjection:
    __members__: typing.ClassVar[dict[str, mjtProjection]]
    mjPROJ_ORTHOGRAPHIC: typing.ClassVar[mjtProjection]
    mjPROJ_PERSPECTIVE: typing.ClassVar[mjtProjection]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtCamLight:
    __members__: typing.ClassVar[dict[str, mjtCamLight]]
    mjCAMLIGHT_FIXED: typing.ClassVar[mjtCamLight]
    mjCAMLIGHT_TARGETBODY: typing.ClassVar[mjtCamLight]
    mjCAMLIGHT_TARGETBODYCOM: typing.ClassVar[mjtCamLight]
    mjCAMLIGHT_TRACK: typing.ClassVar[mjtCamLight]
    mjCAMLIGHT_TRACKCOM: typing.ClassVar[mjtCamLight]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtLightType:
    __members__: typing.ClassVar[dict[str, mjtLightType]]
    mjLIGHT_DIRECTIONAL: typing.ClassVar[mjtLightType]
    mjLIGHT_IMAGE: typing.ClassVar[mjtLightType]
    mjLIGHT_POINT: typing.ClassVar[mjtLightType]
    mjLIGHT_SPOT: typing.ClassVar[mjtLightType]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtTexture:
    __members__: typing.ClassVar[dict[str, mjtTexture]]
    mjTEXTURE_2D: typing.ClassVar[mjtTexture]
    mjTEXTURE_CUBE: typing.ClassVar[mjtTexture]
    mjTEXTURE_SKYBOX: typing.ClassVar[mjtTexture]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtTextureRole:
    __members__: typing.ClassVar[dict[str, mjtTextureRole]]
    mjNTEXROLE: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_EMISSIVE: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_METALLIC: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_NORMAL: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_OCCLUSION: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_OPACITY: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_ORM: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_RGB: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_RGBA: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_ROUGHNESS: typing.ClassVar[mjtTextureRole]
    mjTEXROLE_USER: typing.ClassVar[mjtTextureRole]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtColorSpace:
    __members__: typing.ClassVar[dict[str, mjtColorSpace]]
    mjCOLORSPACE_AUTO: typing.ClassVar[mjtColorSpace]
    mjCOLORSPACE_LINEAR: typing.ClassVar[mjtColorSpace]
    mjCOLORSPACE_SRGB: typing.ClassVar[mjtColorSpace]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtIntegrator:
    __members__: typing.ClassVar[dict[str, mjtIntegrator]]
    mjINT_DISCRETE: typing.ClassVar[mjtIntegrator]
    mjINT_EULER: typing.ClassVar[mjtIntegrator]
    mjINT_IMPLICIT: typing.ClassVar[mjtIntegrator]
    mjINT_IMPLICITFAST: typing.ClassVar[mjtIntegrator]
    mjINT_RK4: typing.ClassVar[mjtIntegrator]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtCone:
    __members__: typing.ClassVar[dict[str, mjtCone]]
    mjCONE_ELLIPTIC: typing.ClassVar[mjtCone]
    mjCONE_PYRAMIDAL: typing.ClassVar[mjtCone]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtJacobian:
    __members__: typing.ClassVar[dict[str, mjtJacobian]]
    mjJAC_AUTO: typing.ClassVar[mjtJacobian]
    mjJAC_DENSE: typing.ClassVar[mjtJacobian]
    mjJAC_SPARSE: typing.ClassVar[mjtJacobian]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtSolver:
    __members__: typing.ClassVar[dict[str, mjtSolver]]
    mjSOL_CG: typing.ClassVar[mjtSolver]
    mjSOL_NEWTON: typing.ClassVar[mjtSolver]
    mjSOL_PGS: typing.ClassVar[mjtSolver]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtEq:
    __members__: typing.ClassVar[dict[str, mjtEq]]
    mjEQ_CONNECT: typing.ClassVar[mjtEq]
    mjEQ_DISTANCE: typing.ClassVar[mjtEq]
    mjEQ_FLEX: typing.ClassVar[mjtEq]
    mjEQ_FLEXSTRAIN: typing.ClassVar[mjtEq]
    mjEQ_FLEXVERT: typing.ClassVar[mjtEq]
    mjEQ_JOINT: typing.ClassVar[mjtEq]
    mjEQ_TENDON: typing.ClassVar[mjtEq]
    mjEQ_WELD: typing.ClassVar[mjtEq]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtWrap:
    __members__: typing.ClassVar[dict[str, mjtWrap]]
    mjWRAP_CYLINDER: typing.ClassVar[mjtWrap]
    mjWRAP_JOINT: typing.ClassVar[mjtWrap]
    mjWRAP_NONE: typing.ClassVar[mjtWrap]
    mjWRAP_PULLEY: typing.ClassVar[mjtWrap]
    mjWRAP_SITE: typing.ClassVar[mjtWrap]
    mjWRAP_SPHERE: typing.ClassVar[mjtWrap]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtTrn:
    __members__: typing.ClassVar[dict[str, mjtTrn]]
    mjTRN_BODY: typing.ClassVar[mjtTrn]
    mjTRN_JOINT: typing.ClassVar[mjtTrn]
    mjTRN_JOINTINPARENT: typing.ClassVar[mjtTrn]
    mjTRN_SITE: typing.ClassVar[mjtTrn]
    mjTRN_SLIDERCRANK: typing.ClassVar[mjtTrn]
    mjTRN_SO3: typing.ClassVar[mjtTrn]
    mjTRN_TENDON: typing.ClassVar[mjtTrn]
    mjTRN_UNDEFINED: typing.ClassVar[mjtTrn]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtDyn:
    __members__: typing.ClassVar[dict[str, mjtDyn]]
    mjDYN_DCMOTOR: typing.ClassVar[mjtDyn]
    mjDYN_FILTER: typing.ClassVar[mjtDyn]
    mjDYN_FILTEREXACT: typing.ClassVar[mjtDyn]
    mjDYN_INTEGRATOR: typing.ClassVar[mjtDyn]
    mjDYN_MUSCLE: typing.ClassVar[mjtDyn]
    mjDYN_NONE: typing.ClassVar[mjtDyn]
    mjDYN_PID: typing.ClassVar[mjtDyn]
    mjDYN_USER: typing.ClassVar[mjtDyn]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtGain:
    __members__: typing.ClassVar[dict[str, mjtGain]]
    mjGAIN_AFFINE: typing.ClassVar[mjtGain]
    mjGAIN_DCMOTOR: typing.ClassVar[mjtGain]
    mjGAIN_FIXED: typing.ClassVar[mjtGain]
    mjGAIN_MUSCLE: typing.ClassVar[mjtGain]
    mjGAIN_PID: typing.ClassVar[mjtGain]
    mjGAIN_SO3: typing.ClassVar[mjtGain]
    mjGAIN_USER: typing.ClassVar[mjtGain]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtBias:
    __members__: typing.ClassVar[dict[str, mjtBias]]
    mjBIAS_AFFINE: typing.ClassVar[mjtBias]
    mjBIAS_DCMOTOR: typing.ClassVar[mjtBias]
    mjBIAS_MUSCLE: typing.ClassVar[mjtBias]
    mjBIAS_NONE: typing.ClassVar[mjtBias]
    mjBIAS_SO3: typing.ClassVar[mjtBias]
    mjBIAS_USER: typing.ClassVar[mjtBias]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtCtrlChart:
    __members__: typing.ClassVar[dict[str, mjtCtrlChart]]
    mjCHART_EXPMAP: typing.ClassVar[mjtCtrlChart]
    mjCHART_QUAT: typing.ClassVar[mjtCtrlChart]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtCtrlInput:
    __members__: typing.ClassVar[dict[str, mjtCtrlInput]]
    mjINPUT_FF: typing.ClassVar[mjtCtrlInput]
    mjINPUT_NONE: typing.ClassVar[mjtCtrlInput]
    mjINPUT_POS: typing.ClassVar[mjtCtrlInput]
    mjINPUT_PRESSURE: typing.ClassVar[mjtCtrlInput]
    mjINPUT_VEL: typing.ClassVar[mjtCtrlInput]
    mjINPUT_VOLTAGE: typing.ClassVar[mjtCtrlInput]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtObj:
    __members__: typing.ClassVar[dict[str, mjtObj]]
    mjNOBJECT: typing.ClassVar[mjtObj]
    mjOBJ_ACTUATOR: typing.ClassVar[mjtObj]
    mjOBJ_BODY: typing.ClassVar[mjtObj]
    mjOBJ_CAMERA: typing.ClassVar[mjtObj]
    mjOBJ_DEFAULT: typing.ClassVar[mjtObj]
    mjOBJ_DOF: typing.ClassVar[mjtObj]
    mjOBJ_EQUALITY: typing.ClassVar[mjtObj]
    mjOBJ_EXCLUDE: typing.ClassVar[mjtObj]
    mjOBJ_FLEX: typing.ClassVar[mjtObj]
    mjOBJ_FRAME: typing.ClassVar[mjtObj]
    mjOBJ_GEOM: typing.ClassVar[mjtObj]
    mjOBJ_HFIELD: typing.ClassVar[mjtObj]
    mjOBJ_JOINT: typing.ClassVar[mjtObj]
    mjOBJ_KEY: typing.ClassVar[mjtObj]
    mjOBJ_LIGHT: typing.ClassVar[mjtObj]
    mjOBJ_MATERIAL: typing.ClassVar[mjtObj]
    mjOBJ_MESH: typing.ClassVar[mjtObj]
    mjOBJ_MODEL: typing.ClassVar[mjtObj]
    mjOBJ_NUMERIC: typing.ClassVar[mjtObj]
    mjOBJ_PAIR: typing.ClassVar[mjtObj]
    mjOBJ_PLUGIN: typing.ClassVar[mjtObj]
    mjOBJ_SENSOR: typing.ClassVar[mjtObj]
    mjOBJ_SITE: typing.ClassVar[mjtObj]
    mjOBJ_SKIN: typing.ClassVar[mjtObj]
    mjOBJ_TENDON: typing.ClassVar[mjtObj]
    mjOBJ_TEXT: typing.ClassVar[mjtObj]
    mjOBJ_TEXTURE: typing.ClassVar[mjtObj]
    mjOBJ_TUPLE: typing.ClassVar[mjtObj]
    mjOBJ_UNKNOWN: typing.ClassVar[mjtObj]
    mjOBJ_XBODY: typing.ClassVar[mjtObj]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtSensor:
    __members__: typing.ClassVar[dict[str, mjtSensor]]
    mjSENS_ACCELEROMETER: typing.ClassVar[mjtSensor]
    mjSENS_ACTUATORFRC: typing.ClassVar[mjtSensor]
    mjSENS_ACTUATORPOS: typing.ClassVar[mjtSensor]
    mjSENS_ACTUATORVEL: typing.ClassVar[mjtSensor]
    mjSENS_BALLANGVEL: typing.ClassVar[mjtSensor]
    mjSENS_BALLQUAT: typing.ClassVar[mjtSensor]
    mjSENS_CAMPROJECTION: typing.ClassVar[mjtSensor]
    mjSENS_CLOCK: typing.ClassVar[mjtSensor]
    mjSENS_CONTACT: typing.ClassVar[mjtSensor]
    mjSENS_E_KINETIC: typing.ClassVar[mjtSensor]
    mjSENS_E_POTENTIAL: typing.ClassVar[mjtSensor]
    mjSENS_FORCE: typing.ClassVar[mjtSensor]
    mjSENS_FRAMEANGACC: typing.ClassVar[mjtSensor]
    mjSENS_FRAMEANGVEL: typing.ClassVar[mjtSensor]
    mjSENS_FRAMELINACC: typing.ClassVar[mjtSensor]
    mjSENS_FRAMELINVEL: typing.ClassVar[mjtSensor]
    mjSENS_FRAMEPOS: typing.ClassVar[mjtSensor]
    mjSENS_FRAMEQUAT: typing.ClassVar[mjtSensor]
    mjSENS_FRAMEXAXIS: typing.ClassVar[mjtSensor]
    mjSENS_FRAMEYAXIS: typing.ClassVar[mjtSensor]
    mjSENS_FRAMEZAXIS: typing.ClassVar[mjtSensor]
    mjSENS_GEOMDIST: typing.ClassVar[mjtSensor]
    mjSENS_GEOMFROMTO: typing.ClassVar[mjtSensor]
    mjSENS_GEOMNORMAL: typing.ClassVar[mjtSensor]
    mjSENS_GYRO: typing.ClassVar[mjtSensor]
    mjSENS_INSIDESITE: typing.ClassVar[mjtSensor]
    mjSENS_JOINTACTFRC: typing.ClassVar[mjtSensor]
    mjSENS_JOINTLIMITFRC: typing.ClassVar[mjtSensor]
    mjSENS_JOINTLIMITPOS: typing.ClassVar[mjtSensor]
    mjSENS_JOINTLIMITVEL: typing.ClassVar[mjtSensor]
    mjSENS_JOINTPOS: typing.ClassVar[mjtSensor]
    mjSENS_JOINTVEL: typing.ClassVar[mjtSensor]
    mjSENS_MAGNETOMETER: typing.ClassVar[mjtSensor]
    mjSENS_PLUGIN: typing.ClassVar[mjtSensor]
    mjSENS_RANGEFINDER: typing.ClassVar[mjtSensor]
    mjSENS_SUBTREEANGMOM: typing.ClassVar[mjtSensor]
    mjSENS_SUBTREECOM: typing.ClassVar[mjtSensor]
    mjSENS_SUBTREELINVEL: typing.ClassVar[mjtSensor]
    mjSENS_TACTILE: typing.ClassVar[mjtSensor]
    mjSENS_TENDONACTFRC: typing.ClassVar[mjtSensor]
    mjSENS_TENDONLIMITFRC: typing.ClassVar[mjtSensor]
    mjSENS_TENDONLIMITPOS: typing.ClassVar[mjtSensor]
    mjSENS_TENDONLIMITVEL: typing.ClassVar[mjtSensor]
    mjSENS_TENDONPOS: typing.ClassVar[mjtSensor]
    mjSENS_TENDONVEL: typing.ClassVar[mjtSensor]
    mjSENS_TORQUE: typing.ClassVar[mjtSensor]
    mjSENS_TOUCH: typing.ClassVar[mjtSensor]
    mjSENS_USER: typing.ClassVar[mjtSensor]
    mjSENS_VELOCIMETER: typing.ClassVar[mjtSensor]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtStage:
    __members__: typing.ClassVar[dict[str, mjtStage]]
    mjSTAGE_ACC: typing.ClassVar[mjtStage]
    mjSTAGE_NONE: typing.ClassVar[mjtStage]
    mjSTAGE_POS: typing.ClassVar[mjtStage]
    mjSTAGE_VEL: typing.ClassVar[mjtStage]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtDataType:
    __members__: typing.ClassVar[dict[str, mjtDataType]]
    mjDATATYPE_AXIS: typing.ClassVar[mjtDataType]
    mjDATATYPE_POSITIVE: typing.ClassVar[mjtDataType]
    mjDATATYPE_QUATERNION: typing.ClassVar[mjtDataType]
    mjDATATYPE_REAL: typing.ClassVar[mjtDataType]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtConDataField:
    __members__: typing.ClassVar[dict[str, mjtConDataField]]
    mjCONDATA_ANGVEL: typing.ClassVar[mjtConDataField]
    mjCONDATA_DIST: typing.ClassVar[mjtConDataField]
    mjCONDATA_FORCE: typing.ClassVar[mjtConDataField]
    mjCONDATA_FOUND: typing.ClassVar[mjtConDataField]
    mjCONDATA_LINVEL: typing.ClassVar[mjtConDataField]
    mjCONDATA_NORMAL: typing.ClassVar[mjtConDataField]
    mjCONDATA_POS: typing.ClassVar[mjtConDataField]
    mjCONDATA_TANGENT: typing.ClassVar[mjtConDataField]
    mjCONDATA_TORQUE: typing.ClassVar[mjtConDataField]
    mjNCONDATA: typing.ClassVar[mjtConDataField]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtRayDataField:
    __members__: typing.ClassVar[dict[str, mjtRayDataField]]
    mjNRAYDATA: typing.ClassVar[mjtRayDataField]
    mjRAYDATA_DEPTH: typing.ClassVar[mjtRayDataField]
    mjRAYDATA_DIR: typing.ClassVar[mjtRayDataField]
    mjRAYDATA_DIST: typing.ClassVar[mjtRayDataField]
    mjRAYDATA_NORMAL: typing.ClassVar[mjtRayDataField]
    mjRAYDATA_ORIGIN: typing.ClassVar[mjtRayDataField]
    mjRAYDATA_POINT: typing.ClassVar[mjtRayDataField]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtCamOutBit:
    __members__: typing.ClassVar[dict[str, mjtCamOutBit]]
    mjCAMOUT_DEPTH: typing.ClassVar[mjtCamOutBit]
    mjCAMOUT_DIST: typing.ClassVar[mjtCamOutBit]
    mjCAMOUT_NORMAL: typing.ClassVar[mjtCamOutBit]
    mjCAMOUT_RGB: typing.ClassVar[mjtCamOutBit]
    mjCAMOUT_SEG: typing.ClassVar[mjtCamOutBit]
    mjNCAMOUT: typing.ClassVar[mjtCamOutBit]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtSameFrame:
    __members__: typing.ClassVar[dict[str, mjtSameFrame]]
    mjSAMEFRAME_BODY: typing.ClassVar[mjtSameFrame]
    mjSAMEFRAME_BODYROT: typing.ClassVar[mjtSameFrame]
    mjSAMEFRAME_INERTIA: typing.ClassVar[mjtSameFrame]
    mjSAMEFRAME_INERTIAROT: typing.ClassVar[mjtSameFrame]
    mjSAMEFRAME_NONE: typing.ClassVar[mjtSameFrame]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtSleepPolicy:
    __members__: typing.ClassVar[dict[str, mjtSleepPolicy]]
    mjSLEEP_ALLOWED: typing.ClassVar[mjtSleepPolicy]
    mjSLEEP_AUTO: typing.ClassVar[mjtSleepPolicy]
    mjSLEEP_AUTO_ALLOWED: typing.ClassVar[mjtSleepPolicy]
    mjSLEEP_AUTO_NEVER: typing.ClassVar[mjtSleepPolicy]
    mjSLEEP_INIT: typing.ClassVar[mjtSleepPolicy]
    mjSLEEP_NEVER: typing.ClassVar[mjtSleepPolicy]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtLRMode:
    __members__: typing.ClassVar[dict[str, mjtLRMode]]
    mjLRMODE_ALL: typing.ClassVar[mjtLRMode]
    mjLRMODE_MUSCLE: typing.ClassVar[mjtLRMode]
    mjLRMODE_MUSCLEUSER: typing.ClassVar[mjtLRMode]
    mjLRMODE_NONE: typing.ClassVar[mjtLRMode]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtFlexSelf:
    __members__: typing.ClassVar[dict[str, mjtFlexSelf]]
    mjFLEXSELF_AUTO: typing.ClassVar[mjtFlexSelf]
    mjFLEXSELF_BVH: typing.ClassVar[mjtFlexSelf]
    mjFLEXSELF_NARROW: typing.ClassVar[mjtFlexSelf]
    mjFLEXSELF_NONE: typing.ClassVar[mjtFlexSelf]
    mjFLEXSELF_SAP: typing.ClassVar[mjtFlexSelf]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtSDFType:
    __members__: typing.ClassVar[dict[str, mjtSDFType]]
    mjSDFTYPE_COLLISION: typing.ClassVar[mjtSDFType]
    mjSDFTYPE_INTERSECTION: typing.ClassVar[mjtSDFType]
    mjSDFTYPE_MIDSURFACE: typing.ClassVar[mjtSDFType]
    mjSDFTYPE_SINGLE: typing.ClassVar[mjtSDFType]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtState:
    __members__: typing.ClassVar[dict[str, mjtState]]
    mjNSTATE: typing.ClassVar[mjtState]
    mjSTATE_ACT: typing.ClassVar[mjtState]
    mjSTATE_CTRL: typing.ClassVar[mjtState]
    mjSTATE_EQ_ACTIVE: typing.ClassVar[mjtState]
    mjSTATE_FULLPHYSICS: typing.ClassVar[mjtState]
    mjSTATE_HISTORY: typing.ClassVar[mjtState]
    mjSTATE_INTEGRATION: typing.ClassVar[mjtState]
    mjSTATE_MOCAP_POS: typing.ClassVar[mjtState]
    mjSTATE_MOCAP_QUAT: typing.ClassVar[mjtState]
    mjSTATE_PHYSICS: typing.ClassVar[mjtState]
    mjSTATE_PLUGIN: typing.ClassVar[mjtState]
    mjSTATE_QFRC_APPLIED: typing.ClassVar[mjtState]
    mjSTATE_QPOS: typing.ClassVar[mjtState]
    mjSTATE_QVEL: typing.ClassVar[mjtState]
    mjSTATE_TIME: typing.ClassVar[mjtState]
    mjSTATE_USER: typing.ClassVar[mjtState]
    mjSTATE_USERDATA: typing.ClassVar[mjtState]
    mjSTATE_WARMSTART: typing.ClassVar[mjtState]
    mjSTATE_XFRC_APPLIED: typing.ClassVar[mjtState]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtConstraint:
    __members__: typing.ClassVar[dict[str, mjtConstraint]]
    mjCNSTR_CONTACT_ELLIPTIC: typing.ClassVar[mjtConstraint]
    mjCNSTR_CONTACT_FRICTIONLESS: typing.ClassVar[mjtConstraint]
    mjCNSTR_CONTACT_PYRAMIDAL: typing.ClassVar[mjtConstraint]
    mjCNSTR_EQUALITY: typing.ClassVar[mjtConstraint]
    mjCNSTR_FRICTION_DOF: typing.ClassVar[mjtConstraint]
    mjCNSTR_FRICTION_TENDON: typing.ClassVar[mjtConstraint]
    mjCNSTR_LIMIT_JOINT: typing.ClassVar[mjtConstraint]
    mjCNSTR_LIMIT_TENDON: typing.ClassVar[mjtConstraint]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtConstraintState:
    __members__: typing.ClassVar[dict[str, mjtConstraintState]]
    mjCNSTRSTATE_CONE: typing.ClassVar[mjtConstraintState]
    mjCNSTRSTATE_LINEARNEG: typing.ClassVar[mjtConstraintState]
    mjCNSTRSTATE_LINEARPOS: typing.ClassVar[mjtConstraintState]
    mjCNSTRSTATE_QUADRATIC: typing.ClassVar[mjtConstraintState]
    mjCNSTRSTATE_SATISFIED: typing.ClassVar[mjtConstraintState]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtWarning:
    __members__: typing.ClassVar[dict[str, mjtWarning]]
    mjNWARNING: typing.ClassVar[mjtWarning]
    mjWARN_BADCTRL: typing.ClassVar[mjtWarning]
    mjWARN_BADQACC: typing.ClassVar[mjtWarning]
    mjWARN_BADQPOS: typing.ClassVar[mjtWarning]
    mjWARN_BADQVEL: typing.ClassVar[mjtWarning]
    mjWARN_CNSTRFULL: typing.ClassVar[mjtWarning]
    mjWARN_CONTACTFULL: typing.ClassVar[mjtWarning]
    mjWARN_INERTIA: typing.ClassVar[mjtWarning]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtTimer:
    __members__: typing.ClassVar[dict[str, mjtTimer]]
    mjNTIMER: typing.ClassVar[mjtTimer]
    mjTIMER_ACTUATION: typing.ClassVar[mjtTimer]
    mjTIMER_ADVANCE: typing.ClassVar[mjtTimer]
    mjTIMER_COL_BROAD: typing.ClassVar[mjtTimer]
    mjTIMER_COL_NARROW: typing.ClassVar[mjtTimer]
    mjTIMER_CONSTRAINT: typing.ClassVar[mjtTimer]
    mjTIMER_FORWARD: typing.ClassVar[mjtTimer]
    mjTIMER_INVERSE: typing.ClassVar[mjtTimer]
    mjTIMER_POSITION: typing.ClassVar[mjtTimer]
    mjTIMER_POS_COLLISION: typing.ClassVar[mjtTimer]
    mjTIMER_POS_INERTIA: typing.ClassVar[mjtTimer]
    mjTIMER_POS_KINEMATICS: typing.ClassVar[mjtTimer]
    mjTIMER_POS_MAKE: typing.ClassVar[mjtTimer]
    mjTIMER_POS_PROJECT: typing.ClassVar[mjtTimer]
    mjTIMER_STEP: typing.ClassVar[mjtTimer]
    mjTIMER_VELOCITY: typing.ClassVar[mjtTimer]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtSleepState:
    __members__: typing.ClassVar[dict[str, mjtSleepState]]
    mjS_ASLEEP: typing.ClassVar[mjtSleepState]
    mjS_AWAKE: typing.ClassVar[mjtSleepState]
    mjS_STATIC: typing.ClassVar[mjtSleepState]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtLogLevel:
    __members__: typing.ClassVar[dict[str, mjtLogLevel]]
    mjLOG_DEBUG: typing.ClassVar[mjtLogLevel]
    mjLOG_ERROR: typing.ClassVar[mjtLogLevel]
    mjLOG_INFO: typing.ClassVar[mjtLogLevel]
    mjLOG_WARNING: typing.ClassVar[mjtLogLevel]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtLogTopic:
    __members__: typing.ClassVar[dict[str, mjtLogTopic]]
    mjNTOPIC: typing.ClassVar[mjtLogTopic]
    mjTOPIC_NONE: typing.ClassVar[mjtLogTopic]
    mjTOPIC_SLEEP: typing.ClassVar[mjtLogTopic]
    mjTOPIC_TIME_CMP: typing.ClassVar[mjtLogTopic]
    mjTOPIC_TIME_STP: typing.ClassVar[mjtLogTopic]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtGeomInertia:
    __members__: typing.ClassVar[dict[str, mjtGeomInertia]]
    mjINERTIA_SHELL: typing.ClassVar[mjtGeomInertia]
    mjINERTIA_VOLUME: typing.ClassVar[mjtGeomInertia]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtMeshInertia:
    __members__: typing.ClassVar[dict[str, mjtMeshInertia]]
    mjMESH_INERTIA_CONVEX: typing.ClassVar[mjtMeshInertia]
    mjMESH_INERTIA_EXACT: typing.ClassVar[mjtMeshInertia]
    mjMESH_INERTIA_LEGACY: typing.ClassVar[mjtMeshInertia]
    mjMESH_INERTIA_SHELL: typing.ClassVar[mjtMeshInertia]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtMeshBuiltin:
    __members__: typing.ClassVar[dict[str, mjtMeshBuiltin]]
    mjMESH_BUILTIN_BOX: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_CONE: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_CYLINDER: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_DISC: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_ELLIPSOID: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_GRID: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_HEMISPHERE: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_NONE: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_PLATE: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_SPHERE: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_SQUARE: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_SUPERSPHERE: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_SUPERTORUS: typing.ClassVar[mjtMeshBuiltin]
    mjMESH_BUILTIN_WEDGE: typing.ClassVar[mjtMeshBuiltin]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtBuiltin:
    __members__: typing.ClassVar[dict[str, mjtBuiltin]]
    mjBUILTIN_CHECKER: typing.ClassVar[mjtBuiltin]
    mjBUILTIN_FLAT: typing.ClassVar[mjtBuiltin]
    mjBUILTIN_GRADIENT: typing.ClassVar[mjtBuiltin]
    mjBUILTIN_NONE: typing.ClassVar[mjtBuiltin]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtMark:
    __members__: typing.ClassVar[dict[str, mjtMark]]
    mjMARK_CROSS: typing.ClassVar[mjtMark]
    mjMARK_EDGE: typing.ClassVar[mjtMark]
    mjMARK_NONE: typing.ClassVar[mjtMark]
    mjMARK_RANDOM: typing.ClassVar[mjtMark]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtLimited:
    __members__: typing.ClassVar[dict[str, mjtLimited]]
    mjLIMITED_AUTO: typing.ClassVar[mjtLimited]
    mjLIMITED_FALSE: typing.ClassVar[mjtLimited]
    mjLIMITED_TRUE: typing.ClassVar[mjtLimited]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtAlignFree:
    __members__: typing.ClassVar[dict[str, mjtAlignFree]]
    mjALIGNFREE_AUTO: typing.ClassVar[mjtAlignFree]
    mjALIGNFREE_FALSE: typing.ClassVar[mjtAlignFree]
    mjALIGNFREE_TRUE: typing.ClassVar[mjtAlignFree]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtInertiaFromGeom:
    __members__: typing.ClassVar[dict[str, mjtInertiaFromGeom]]
    mjINERTIAFROMGEOM_AUTO: typing.ClassVar[mjtInertiaFromGeom]
    mjINERTIAFROMGEOM_FALSE: typing.ClassVar[mjtInertiaFromGeom]
    mjINERTIAFROMGEOM_TRUE: typing.ClassVar[mjtInertiaFromGeom]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtOrientation:
    __members__: typing.ClassVar[dict[str, mjtOrientation]]
    mjORIENTATION_AXISANGLE: typing.ClassVar[mjtOrientation]
    mjORIENTATION_EULER: typing.ClassVar[mjtOrientation]
    mjORIENTATION_QUAT: typing.ClassVar[mjtOrientation]
    mjORIENTATION_XYAXES: typing.ClassVar[mjtOrientation]
    mjORIENTATION_ZAXIS: typing.ClassVar[mjtOrientation]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtConflict:
    __members__: typing.ClassVar[dict[str, mjtConflict]]
    mjCONFLICT_ERROR: typing.ClassVar[mjtConflict]
    mjCONFLICT_MERGE: typing.ClassVar[mjtConflict]
    mjCONFLICT_WARNING: typing.ClassVar[mjtConflict]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtScaling:
    __members__: typing.ClassVar[dict[str, mjtScaling]]
    mjSCALING_GEOMETRY: typing.ClassVar[mjtScaling]
    mjSCALING_SIMILAR: typing.ClassVar[mjtScaling]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtActuator:
    __members__: typing.ClassVar[dict[str, mjtActuator]]
    mjACTUATOR_ADHESION: typing.ClassVar[mjtActuator]
    mjACTUATOR_CYLINDER: typing.ClassVar[mjtActuator]
    mjACTUATOR_DAMPER: typing.ClassVar[mjtActuator]
    mjACTUATOR_DCMOTOR: typing.ClassVar[mjtActuator]
    mjACTUATOR_GENERAL: typing.ClassVar[mjtActuator]
    mjACTUATOR_INTVELOCITY: typing.ClassVar[mjtActuator]
    mjACTUATOR_MOTOR: typing.ClassVar[mjtActuator]
    mjACTUATOR_MUSCLE: typing.ClassVar[mjtActuator]
    mjACTUATOR_ORIENTATION: typing.ClassVar[mjtActuator]
    mjACTUATOR_PID: typing.ClassVar[mjtActuator]
    mjACTUATOR_POSITION: typing.ClassVar[mjtActuator]
    mjACTUATOR_VELOCITY: typing.ClassVar[mjtActuator]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtCTimer:
    __members__: typing.ClassVar[dict[str, mjtCTimer]]
    mjCTIMER_ASSETS: typing.ClassVar[mjtCTimer]
    mjCTIMER_MESH_BVH: typing.ClassVar[mjtCTimer]
    mjCTIMER_MESH_HULL: typing.ClassVar[mjtCTimer]
    mjCTIMER_MESH_INERTIA: typing.ClassVar[mjtCTimer]
    mjCTIMER_MESH_LOAD: typing.ClassVar[mjtCTimer]
    mjCTIMER_MESH_OCTREE: typing.ClassVar[mjtCTimer]
    mjCTIMER_MESH_POLYGON: typing.ClassVar[mjtCTimer]
    mjCTIMER_TEXTURE: typing.ClassVar[mjtCTimer]
    mjCTIMER_TOTAL: typing.ClassVar[mjtCTimer]
    mjNCTIMER: typing.ClassVar[mjtCTimer]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtCatBit:
    __members__: typing.ClassVar[dict[str, mjtCatBit]]
    mjCAT_ALL: typing.ClassVar[mjtCatBit]
    mjCAT_DECOR: typing.ClassVar[mjtCatBit]
    mjCAT_DYNAMIC: typing.ClassVar[mjtCatBit]
    mjCAT_STATIC: typing.ClassVar[mjtCatBit]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtMouse:
    __members__: typing.ClassVar[dict[str, mjtMouse]]
    mjMOUSE_MOVE_H: typing.ClassVar[mjtMouse]
    mjMOUSE_MOVE_H_REL: typing.ClassVar[mjtMouse]
    mjMOUSE_MOVE_V: typing.ClassVar[mjtMouse]
    mjMOUSE_MOVE_V_REL: typing.ClassVar[mjtMouse]
    mjMOUSE_NONE: typing.ClassVar[mjtMouse]
    mjMOUSE_ROTATE_H: typing.ClassVar[mjtMouse]
    mjMOUSE_ROTATE_V: typing.ClassVar[mjtMouse]
    mjMOUSE_TURN_H: typing.ClassVar[mjtMouse]
    mjMOUSE_TURN_V: typing.ClassVar[mjtMouse]
    mjMOUSE_ZOOM: typing.ClassVar[mjtMouse]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtPertBit:
    __members__: typing.ClassVar[dict[str, mjtPertBit]]
    mjPERT_ROTATE: typing.ClassVar[mjtPertBit]
    mjPERT_TRANSLATE: typing.ClassVar[mjtPertBit]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtCamera:
    __members__: typing.ClassVar[dict[str, mjtCamera]]
    mjCAMERA_FIXED: typing.ClassVar[mjtCamera]
    mjCAMERA_FREE: typing.ClassVar[mjtCamera]
    mjCAMERA_TRACKING: typing.ClassVar[mjtCamera]
    mjCAMERA_USER: typing.ClassVar[mjtCamera]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtLabel:
    __members__: typing.ClassVar[dict[str, mjtLabel]]
    mjLABEL_ACTUATOR: typing.ClassVar[mjtLabel]
    mjLABEL_BODY: typing.ClassVar[mjtLabel]
    mjLABEL_CAMERA: typing.ClassVar[mjtLabel]
    mjLABEL_CONSTRAINT: typing.ClassVar[mjtLabel]
    mjLABEL_CONTACTFORCE: typing.ClassVar[mjtLabel]
    mjLABEL_CONTACTPOINT: typing.ClassVar[mjtLabel]
    mjLABEL_FLEX: typing.ClassVar[mjtLabel]
    mjLABEL_GEOM: typing.ClassVar[mjtLabel]
    mjLABEL_ISLAND: typing.ClassVar[mjtLabel]
    mjLABEL_JOINT: typing.ClassVar[mjtLabel]
    mjLABEL_LIGHT: typing.ClassVar[mjtLabel]
    mjLABEL_NONE: typing.ClassVar[mjtLabel]
    mjLABEL_SELECTION: typing.ClassVar[mjtLabel]
    mjLABEL_SELPNT: typing.ClassVar[mjtLabel]
    mjLABEL_SITE: typing.ClassVar[mjtLabel]
    mjLABEL_SKIN: typing.ClassVar[mjtLabel]
    mjLABEL_TENDON: typing.ClassVar[mjtLabel]
    mjNLABEL: typing.ClassVar[mjtLabel]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtFrame:
    __members__: typing.ClassVar[dict[str, mjtFrame]]
    mjFRAME_BODY: typing.ClassVar[mjtFrame]
    mjFRAME_CAMERA: typing.ClassVar[mjtFrame]
    mjFRAME_CONTACT: typing.ClassVar[mjtFrame]
    mjFRAME_GEOM: typing.ClassVar[mjtFrame]
    mjFRAME_LIGHT: typing.ClassVar[mjtFrame]
    mjFRAME_NONE: typing.ClassVar[mjtFrame]
    mjFRAME_SITE: typing.ClassVar[mjtFrame]
    mjFRAME_WORLD: typing.ClassVar[mjtFrame]
    mjNFRAME: typing.ClassVar[mjtFrame]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtVisFlag:
    __members__: typing.ClassVar[dict[str, mjtVisFlag]]
    mjNVISFLAG: typing.ClassVar[mjtVisFlag]
    mjVIS_ACTIVATION: typing.ClassVar[mjtVisFlag]
    mjVIS_ACTUATOR: typing.ClassVar[mjtVisFlag]
    mjVIS_AUTOCONNECT: typing.ClassVar[mjtVisFlag]
    mjVIS_BODYBVH: typing.ClassVar[mjtVisFlag]
    mjVIS_CAMERA: typing.ClassVar[mjtVisFlag]
    mjVIS_COM: typing.ClassVar[mjtVisFlag]
    mjVIS_CONSTRAINT: typing.ClassVar[mjtVisFlag]
    mjVIS_CONTACTFORCE: typing.ClassVar[mjtVisFlag]
    mjVIS_CONTACTPOINT: typing.ClassVar[mjtVisFlag]
    mjVIS_CONTACTSPLIT: typing.ClassVar[mjtVisFlag]
    mjVIS_CONVEXHULL: typing.ClassVar[mjtVisFlag]
    mjVIS_FLEXEDGE: typing.ClassVar[mjtVisFlag]
    mjVIS_FLEXFACE: typing.ClassVar[mjtVisFlag]
    mjVIS_FLEXSKIN: typing.ClassVar[mjtVisFlag]
    mjVIS_FLEXVERT: typing.ClassVar[mjtVisFlag]
    mjVIS_INERTIA: typing.ClassVar[mjtVisFlag]
    mjVIS_ISLAND: typing.ClassVar[mjtVisFlag]
    mjVIS_JOINT: typing.ClassVar[mjtVisFlag]
    mjVIS_LIGHT: typing.ClassVar[mjtVisFlag]
    mjVIS_MESHBVH: typing.ClassVar[mjtVisFlag]
    mjVIS_PERTFORCE: typing.ClassVar[mjtVisFlag]
    mjVIS_PERTOBJ: typing.ClassVar[mjtVisFlag]
    mjVIS_RANGEFINDER: typing.ClassVar[mjtVisFlag]
    mjVIS_SCLINERTIA: typing.ClassVar[mjtVisFlag]
    mjVIS_SDFITER: typing.ClassVar[mjtVisFlag]
    mjVIS_SELECT: typing.ClassVar[mjtVisFlag]
    mjVIS_SKIN: typing.ClassVar[mjtVisFlag]
    mjVIS_STATIC: typing.ClassVar[mjtVisFlag]
    mjVIS_TENDON: typing.ClassVar[mjtVisFlag]
    mjVIS_TEXTURE: typing.ClassVar[mjtVisFlag]
    mjVIS_TRANSPARENT: typing.ClassVar[mjtVisFlag]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtRndFlag:
    __members__: typing.ClassVar[dict[str, mjtRndFlag]]
    mjNRNDFLAG: typing.ClassVar[mjtRndFlag]
    mjRND_ADDITIVE: typing.ClassVar[mjtRndFlag]
    mjRND_CULL_FACE: typing.ClassVar[mjtRndFlag]
    mjRND_DEPTH: typing.ClassVar[mjtRndFlag]
    mjRND_FOG: typing.ClassVar[mjtRndFlag]
    mjRND_HAZE: typing.ClassVar[mjtRndFlag]
    mjRND_IDCOLOR: typing.ClassVar[mjtRndFlag]
    mjRND_REFLECTION: typing.ClassVar[mjtRndFlag]
    mjRND_SEGMENT: typing.ClassVar[mjtRndFlag]
    mjRND_SHADOW: typing.ClassVar[mjtRndFlag]
    mjRND_SKYBOX: typing.ClassVar[mjtRndFlag]
    mjRND_WIREFRAME: typing.ClassVar[mjtRndFlag]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtStereo:
    __members__: typing.ClassVar[dict[str, mjtStereo]]
    mjSTEREO_NONE: typing.ClassVar[mjtStereo]
    mjSTEREO_QUADBUFFERED: typing.ClassVar[mjtStereo]
    mjSTEREO_SIDEBYSIDE: typing.ClassVar[mjtStereo]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtPluginCapabilityBit:
    __members__: typing.ClassVar[dict[str, mjtPluginCapabilityBit]]
    mjPLUGIN_ACTUATOR: typing.ClassVar[mjtPluginCapabilityBit]
    mjPLUGIN_PASSIVE: typing.ClassVar[mjtPluginCapabilityBit]
    mjPLUGIN_SDF: typing.ClassVar[mjtPluginCapabilityBit]
    mjPLUGIN_SENSOR: typing.ClassVar[mjtPluginCapabilityBit]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtGridPos:
    __members__: typing.ClassVar[dict[str, mjtGridPos]]
    mjGRID_BOTTOM: typing.ClassVar[mjtGridPos]
    mjGRID_BOTTOMLEFT: typing.ClassVar[mjtGridPos]
    mjGRID_BOTTOMRIGHT: typing.ClassVar[mjtGridPos]
    mjGRID_LEFT: typing.ClassVar[mjtGridPos]
    mjGRID_RIGHT: typing.ClassVar[mjtGridPos]
    mjGRID_TOP: typing.ClassVar[mjtGridPos]
    mjGRID_TOPLEFT: typing.ClassVar[mjtGridPos]
    mjGRID_TOPRIGHT: typing.ClassVar[mjtGridPos]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtFramebuffer:
    __members__: typing.ClassVar[dict[str, mjtFramebuffer]]
    mjFB_OFFSCREEN: typing.ClassVar[mjtFramebuffer]
    mjFB_WINDOW: typing.ClassVar[mjtFramebuffer]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtDepthMap:
    __members__: typing.ClassVar[dict[str, mjtDepthMap]]
    mjDEPTH_ZEROFAR: typing.ClassVar[mjtDepthMap]
    mjDEPTH_ZERONEAR: typing.ClassVar[mjtDepthMap]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtFontScale:
    __members__: typing.ClassVar[dict[str, mjtFontScale]]
    mjFONTSCALE_100: typing.ClassVar[mjtFontScale]
    mjFONTSCALE_150: typing.ClassVar[mjtFontScale]
    mjFONTSCALE_200: typing.ClassVar[mjtFontScale]
    mjFONTSCALE_250: typing.ClassVar[mjtFontScale]
    mjFONTSCALE_300: typing.ClassVar[mjtFontScale]
    mjFONTSCALE_50: typing.ClassVar[mjtFontScale]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtFont:
    __members__: typing.ClassVar[dict[str, mjtFont]]
    mjFONT_BIG: typing.ClassVar[mjtFont]
    mjFONT_NORMAL: typing.ClassVar[mjtFont]
    mjFONT_SHADOW: typing.ClassVar[mjtFont]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjrPixelFormat:
    __members__: typing.ClassVar[dict[str, mjrPixelFormat]]
    mjPIXEL_FORMAT_DEPTH32F: typing.ClassVar[mjrPixelFormat]
    mjPIXEL_FORMAT_KTX: typing.ClassVar[mjrPixelFormat]
    mjPIXEL_FORMAT_R32F: typing.ClassVar[mjrPixelFormat]
    mjPIXEL_FORMAT_R8: typing.ClassVar[mjrPixelFormat]
    mjPIXEL_FORMAT_RGB8: typing.ClassVar[mjrPixelFormat]
    mjPIXEL_FORMAT_RGBA8: typing.ClassVar[mjrPixelFormat]
    mjPIXEL_FORMAT_UNKNOWN: typing.ClassVar[mjrPixelFormat]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjrVertexAttributeUsage:
    __members__: typing.ClassVar[dict[str, mjrVertexAttributeUsage]]
    mjVERTEX_ATTRIBUTE_USAGE_COLOR: typing.ClassVar[mjrVertexAttributeUsage]
    mjVERTEX_ATTRIBUTE_USAGE_NORMAL: typing.ClassVar[mjrVertexAttributeUsage]
    mjVERTEX_ATTRIBUTE_USAGE_POSITION: typing.ClassVar[mjrVertexAttributeUsage]
    mjVERTEX_ATTRIBUTE_USAGE_TANGENTS: typing.ClassVar[mjrVertexAttributeUsage]
    mjVERTEX_ATTRIBUTE_USAGE_UV: typing.ClassVar[mjrVertexAttributeUsage]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjrVertexAttributeType:
    __members__: typing.ClassVar[dict[str, mjrVertexAttributeType]]
    mjVERTEX_ATTRIBUTE_TYPE_FLOAT2: typing.ClassVar[mjrVertexAttributeType]
    mjVERTEX_ATTRIBUTE_TYPE_FLOAT3: typing.ClassVar[mjrVertexAttributeType]
    mjVERTEX_ATTRIBUTE_TYPE_FLOAT4: typing.ClassVar[mjrVertexAttributeType]
    mjVERTEX_ATTRIBUTE_TYPE_UBYTE4: typing.ClassVar[mjrVertexAttributeType]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjrIndexType:
    __members__: typing.ClassVar[dict[str, mjrIndexType]]
    mjINDEX_TYPE_U16: typing.ClassVar[mjrIndexType]
    mjINDEX_TYPE_U32: typing.ClassVar[mjrIndexType]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjrMeshPrimitiveType:
    __members__: typing.ClassVar[dict[str, mjrMeshPrimitiveType]]
    mjMESH_PRIMITIVE_TYPE_LINES: typing.ClassVar[mjrMeshPrimitiveType]
    mjMESH_PRIMITIVE_TYPE_TRIANGLES: typing.ClassVar[mjrMeshPrimitiveType]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtButton:
    __members__: typing.ClassVar[dict[str, mjtButton]]
    mjBUTTON_LEFT: typing.ClassVar[mjtButton]
    mjBUTTON_MIDDLE: typing.ClassVar[mjtButton]
    mjBUTTON_NONE: typing.ClassVar[mjtButton]
    mjBUTTON_RIGHT: typing.ClassVar[mjtButton]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtEvent:
    __members__: typing.ClassVar[dict[str, mjtEvent]]
    mjEVENT_FILESDROP: typing.ClassVar[mjtEvent]
    mjEVENT_KEY: typing.ClassVar[mjtEvent]
    mjEVENT_MOVE: typing.ClassVar[mjtEvent]
    mjEVENT_NONE: typing.ClassVar[mjtEvent]
    mjEVENT_PRESS: typing.ClassVar[mjtEvent]
    mjEVENT_REDRAW: typing.ClassVar[mjtEvent]
    mjEVENT_RELEASE: typing.ClassVar[mjtEvent]
    mjEVENT_RESIZE: typing.ClassVar[mjtEvent]
    mjEVENT_SCROLL: typing.ClassVar[mjtEvent]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtItem:
    __members__: typing.ClassVar[dict[str, mjtItem]]
    mjITEM_BUTTON: typing.ClassVar[mjtItem]
    mjITEM_CHECKBYTE: typing.ClassVar[mjtItem]
    mjITEM_CHECKINT: typing.ClassVar[mjtItem]
    mjITEM_EDITFLOAT: typing.ClassVar[mjtItem]
    mjITEM_EDITINT: typing.ClassVar[mjtItem]
    mjITEM_EDITNUM: typing.ClassVar[mjtItem]
    mjITEM_EDITTXT: typing.ClassVar[mjtItem]
    mjITEM_END: typing.ClassVar[mjtItem]
    mjITEM_RADIO: typing.ClassVar[mjtItem]
    mjITEM_RADIOLINE: typing.ClassVar[mjtItem]
    mjITEM_SECTION: typing.ClassVar[mjtItem]
    mjITEM_SELECT: typing.ClassVar[mjtItem]
    mjITEM_SEPARATOR: typing.ClassVar[mjtItem]
    mjITEM_SLIDERINT: typing.ClassVar[mjtItem]
    mjITEM_SLIDERNUM: typing.ClassVar[mjtItem]
    mjITEM_STATIC: typing.ClassVar[mjtItem]
    mjNITEM: typing.ClassVar[mjtItem]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...

class mjtSection:
    __members__: typing.ClassVar[dict[str, mjtSection]]
    mjSECT_CLOSED: typing.ClassVar[mjtSection]
    mjSECT_FIXED: typing.ClassVar[mjtSection]
    mjSECT_OPEN: typing.ClassVar[mjtSection]

    def __add__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __and__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __eq__(self, other: object) -> bool:
        ...

    def __floordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __getstate__(self) -> int:
        ...

    def __hash__(self) -> int:
        ...

    def __index__(self) -> int:
        ...

    def __init__(self, value: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __int__(self) -> int:
        ...

    def __lshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __mul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ne__(self, other: object) -> bool:
        ...

    def __neg__(self) -> int:
        ...

    def __or__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __radd__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rand__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __repr__(self) -> str:
        ...

    def __rfloordiv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rmod__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rmul__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __ror__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rshift__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __rsub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rtruediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __rxor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    def __setstate__(self, state: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def __str__(self) -> str:
        ...

    def __sub__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __truediv__(self, arg0: typing.SupportsFloat | typing.SupportsIndex) -> int | float:
        ...

    def __xor__(self, arg0: typing.SupportsInt | typing.SupportsIndex) -> int:
        ...

    @property
    def name(self) -> str:
        ...

    @property
    def value(self) -> int:
        ...
