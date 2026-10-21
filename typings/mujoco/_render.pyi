# MuJoCo 3.16.0 API declarations; Copyright 2022 DeepMind Technologies Limited.
# Derived under Apache-2.0; see typings/LICENSE.mujoco and typings/README.md.
from __future__ import annotations
import mujoco._structs
import numpy
import numpy.typing
import typing
__all__: list[str] = ['MjrContext', 'MjrRendererInfo', 'mjr_addAux', 'mjr_blitAux', 'mjr_blitBuffer', 'mjr_changeFont', 'mjr_defaultRendererInfo', 'mjr_drawPixels', 'mjr_figure', 'mjr_findRect', 'mjr_finish', 'mjr_getError', 'mjr_getRendererInfo', 'mjr_label', 'mjr_maxViewport', 'mjr_overlay', 'mjr_readPixels', 'mjr_rectangle', 'mjr_render', 'mjr_resizeOffscreen', 'mjr_restoreBuffer', 'mjr_setAux', 'mjr_setBuffer', 'mjr_text', 'mjr_uploadHField', 'mjr_uploadMesh', 'mjr_uploadTexture']

class MjrContext:

    @typing.overload
    def __init__(self) -> None:
        ...

    @typing.overload
    def __init__(self, arg0: mujoco._structs.MjModel, arg1: typing.SupportsInt | typing.SupportsIndex) -> None:
        ...

    def free(self) -> None:
        ...

    @property
    def auxColor(self) -> numpy.typing.NDArray[numpy.uint32]:
        ...

    @auxColor.setter
    def auxColor(self, arg1: typing.Any) -> None:
        ...

    @property
    def auxColor_r(self) -> numpy.typing.NDArray[numpy.uint32]:
        ...

    @auxColor_r.setter
    def auxColor_r(self, arg1: typing.Any) -> None:
        ...

    @property
    def auxFBO(self) -> numpy.typing.NDArray[numpy.uint32]:
        ...

    @auxFBO.setter
    def auxFBO(self, arg1: typing.Any) -> None:
        ...

    @property
    def auxFBO_r(self) -> numpy.typing.NDArray[numpy.uint32]:
        ...

    @auxFBO_r.setter
    def auxFBO_r(self, arg1: typing.Any) -> None:
        ...

    @property
    def auxHeight(self) -> numpy.typing.NDArray[numpy.int32]:
        ...

    @auxHeight.setter
    def auxHeight(self, arg1: typing.Any) -> None:
        ...

    @property
    def auxSamples(self) -> numpy.typing.NDArray[numpy.int32]:
        ...

    @auxSamples.setter
    def auxSamples(self, arg1: typing.Any) -> None:
        ...

    @property
    def auxWidth(self) -> numpy.typing.NDArray[numpy.int32]:
        ...

    @auxWidth.setter
    def auxWidth(self, arg1: typing.Any) -> None:
        ...

    @property
    def baseBuiltin(self) -> int:
        ...

    @baseBuiltin.setter
    def baseBuiltin(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def baseFontBig(self) -> int:
        ...

    @baseFontBig.setter
    def baseFontBig(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def baseFontNormal(self) -> int:
        ...

    @baseFontNormal.setter
    def baseFontNormal(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def baseFontShadow(self) -> int:
        ...

    @baseFontShadow.setter
    def baseFontShadow(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def baseHField(self) -> int:
        ...

    @baseHField.setter
    def baseHField(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def baseMesh(self) -> int:
        ...

    @baseMesh.setter
    def baseMesh(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def basePlane(self) -> int:
        ...

    @basePlane.setter
    def basePlane(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def charHeight(self) -> int:
        ...

    @charHeight.setter
    def charHeight(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def charHeightBig(self) -> int:
        ...

    @charHeightBig.setter
    def charHeightBig(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def charWidth(self) -> numpy.typing.NDArray[numpy.int32]:
        ...

    @charWidth.setter
    def charWidth(self, arg1: typing.Any) -> None:
        ...

    @property
    def charWidthBig(self) -> numpy.typing.NDArray[numpy.int32]:
        ...

    @charWidthBig.setter
    def charWidthBig(self, arg1: typing.Any) -> None:
        ...

    @property
    def currentBuffer(self) -> int:
        ...

    @currentBuffer.setter
    def currentBuffer(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def fogEnd(self) -> float:
        ...

    @fogEnd.setter
    def fogEnd(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def fogRGBA(self) -> numpy.typing.NDArray[numpy.float32]:
        ...

    @fogRGBA.setter
    def fogRGBA(self, arg1: typing.Any) -> None:
        ...

    @property
    def fogStart(self) -> float:
        ...

    @fogStart.setter
    def fogStart(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def fontScale(self) -> int:
        ...

    @fontScale.setter
    def fontScale(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def glInitialized(self) -> int:
        ...

    @glInitialized.setter
    def glInitialized(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def lineWidth(self) -> float:
        ...

    @lineWidth.setter
    def lineWidth(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def mat_texid(self) -> numpy.typing.NDArray[numpy.int32]:
        ...

    @mat_texid.setter
    def mat_texid(self, arg1: typing.Any) -> None:
        ...

    @property
    def mat_texrepeat(self) -> numpy.typing.NDArray[numpy.float32]:
        ...

    @mat_texrepeat.setter
    def mat_texrepeat(self, arg1: typing.Any) -> None:
        ...

    @property
    def mat_texuniform(self) -> numpy.typing.NDArray[numpy.int32]:
        ...

    @mat_texuniform.setter
    def mat_texuniform(self, arg1: typing.Any) -> None:
        ...

    @property
    def nskin(self) -> int:
        ...

    @property
    def ntexture(self) -> int:
        ...

    @ntexture.setter
    def ntexture(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def offColor(self) -> int:
        ...

    @offColor.setter
    def offColor(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def offColor_r(self) -> int:
        ...

    @offColor_r.setter
    def offColor_r(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def offDepthStencil(self) -> int:
        ...

    @offDepthStencil.setter
    def offDepthStencil(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def offDepthStencil_r(self) -> int:
        ...

    @offDepthStencil_r.setter
    def offDepthStencil_r(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def offFBO(self) -> int:
        ...

    @offFBO.setter
    def offFBO(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def offFBO_r(self) -> int:
        ...

    @offFBO_r.setter
    def offFBO_r(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def offHeight(self) -> int:
        ...

    @offHeight.setter
    def offHeight(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def offSamples(self) -> int:
        ...

    @offSamples.setter
    def offSamples(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def offWidth(self) -> int:
        ...

    @offWidth.setter
    def offWidth(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def rangeBuiltin(self) -> int:
        ...

    @rangeBuiltin.setter
    def rangeBuiltin(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def rangeFont(self) -> int:
        ...

    @rangeFont.setter
    def rangeFont(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def rangeHField(self) -> int:
        ...

    @rangeHField.setter
    def rangeHField(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def rangeMesh(self) -> int:
        ...

    @rangeMesh.setter
    def rangeMesh(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def rangePlane(self) -> int:
        ...

    @rangePlane.setter
    def rangePlane(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def readDepthMap(self) -> int:
        ...

    @readDepthMap.setter
    def readDepthMap(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def readPixelFormat(self) -> int:
        ...

    @readPixelFormat.setter
    def readPixelFormat(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def shadowClip(self) -> float:
        ...

    @shadowClip.setter
    def shadowClip(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def shadowFBO(self) -> int:
        ...

    @shadowFBO.setter
    def shadowFBO(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def shadowScale(self) -> float:
        ...

    @shadowScale.setter
    def shadowScale(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def shadowSize(self) -> int:
        ...

    @shadowSize.setter
    def shadowSize(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def shadowTex(self) -> int:
        ...

    @shadowTex.setter
    def shadowTex(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def skinfaceVBO(self) -> tuple:
        ...

    @property
    def skinnormalVBO(self) -> tuple:
        ...

    @property
    def skintexcoordVBO(self) -> tuple:
        ...

    @property
    def skinvertVBO(self) -> tuple:
        ...

    @property
    def texture(self) -> numpy.typing.NDArray[numpy.uint32]:
        ...

    @texture.setter
    def texture(self, arg1: typing.Any) -> None:
        ...

    @property
    def textureType(self) -> numpy.typing.NDArray[numpy.int32]:
        ...

    @textureType.setter
    def textureType(self, arg1: typing.Any) -> None:
        ...

    @property
    def windowAvailable(self) -> int:
        ...

    @windowAvailable.setter
    def windowAvailable(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def windowDoublebuffer(self) -> int:
        ...

    @windowDoublebuffer.setter
    def windowDoublebuffer(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def windowSamples(self) -> int:
        ...

    @windowSamples.setter
    def windowSamples(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

    @property
    def windowStereo(self) -> int:
        ...

    @windowStereo.setter
    def windowStereo(self, arg1: typing.SupportsFloat | typing.SupportsIndex) -> None:
        ...

class MjrRendererInfo:
    __hash__: typing.ClassVar[typing.Any]

    def __copy__(self) -> MjrRendererInfo:
        ...

    def __deepcopy__(self, arg0: dict) -> MjrRendererInfo:
        ...

    def __eq__(self, arg0: typing.Any) -> bool:
        ...

    def __init__(self) -> None:
        ...

    def __repr__(self) -> str:
        ...

    @property
    def backend(self) -> str:
        ...

    @property
    def renderer(self) -> str:
        ...

def mjr_addAux(index: typing.SupportsInt | typing.SupportsIndex, width: typing.SupportsInt | typing.SupportsIndex, height: typing.SupportsInt | typing.SupportsIndex, samples: typing.SupportsInt | typing.SupportsIndex, con: MjrContext) -> None:
    ...

def mjr_blitAux(index: typing.SupportsInt | typing.SupportsIndex, src: mujoco._structs.MjrRect, left: typing.SupportsInt | typing.SupportsIndex, bottom: typing.SupportsInt | typing.SupportsIndex, con: MjrContext) -> None:
    ...

def mjr_blitBuffer(src: mujoco._structs.MjrRect, dst: mujoco._structs.MjrRect, flg_color: typing.SupportsInt | typing.SupportsIndex, flg_depth: typing.SupportsInt | typing.SupportsIndex, con: MjrContext) -> None:
    ...

def mjr_changeFont(fontscale: typing.SupportsInt | typing.SupportsIndex, con: MjrContext) -> None:
    ...

def mjr_defaultRendererInfo(info: MjrRendererInfo) -> None:
    ...

def mjr_drawPixels(rgb: typing.Annotated[numpy.typing.NDArray[numpy.uint8], '[m, 1]'] | None, depth: typing.Annotated[numpy.typing.NDArray[numpy.float32], '[m, 1]'] | None, viewport: mujoco._structs.MjrRect, con: MjrContext) -> None:
    ...

def mjr_figure(viewport: mujoco._structs.MjrRect, fig: mujoco._structs.MjvFigure, con: MjrContext) -> None:
    ...

def mjr_findRect(x: typing.SupportsInt | typing.SupportsIndex, y: typing.SupportsInt | typing.SupportsIndex, nrect: typing.SupportsInt | typing.SupportsIndex, rect: mujoco._structs.MjrRect) -> int:
    ...

def mjr_finish() -> None:
    ...

def mjr_getError() -> int:
    ...

def mjr_getRendererInfo(info: MjrRendererInfo) -> None:
    ...

def mjr_label(viewport: mujoco._structs.MjrRect, font: typing.SupportsInt | typing.SupportsIndex, txt: str, r: typing.SupportsFloat | typing.SupportsIndex, g: typing.SupportsFloat | typing.SupportsIndex, b: typing.SupportsFloat | typing.SupportsIndex, a: typing.SupportsFloat | typing.SupportsIndex, rt: typing.SupportsFloat | typing.SupportsIndex, gt: typing.SupportsFloat | typing.SupportsIndex, bt: typing.SupportsFloat | typing.SupportsIndex, con: MjrContext) -> None:
    ...

def mjr_maxViewport(con: MjrContext) -> mujoco._structs.MjrRect:
    ...

def mjr_overlay(font: typing.SupportsInt | typing.SupportsIndex, gridpos: typing.SupportsInt | typing.SupportsIndex, viewport: mujoco._structs.MjrRect, overlay: str, overlay2: str, con: MjrContext) -> None:
    ...

def mjr_readPixels(rgb: typing.Annotated[numpy.typing.ArrayLike, numpy.uint8] | None, depth: typing.Annotated[numpy.typing.ArrayLike, numpy.float32] | None, viewport: mujoco._structs.MjrRect, con: MjrContext) -> None:
    ...

def mjr_rectangle(viewport: mujoco._structs.MjrRect, r: typing.SupportsFloat | typing.SupportsIndex, g: typing.SupportsFloat | typing.SupportsIndex, b: typing.SupportsFloat | typing.SupportsIndex, a: typing.SupportsFloat | typing.SupportsIndex) -> None:
    ...

def mjr_render(viewport: mujoco._structs.MjrRect, scn: mujoco._structs.MjvScene, con: MjrContext) -> None:
    ...

def mjr_resizeOffscreen(width: typing.SupportsInt | typing.SupportsIndex, height: typing.SupportsInt | typing.SupportsIndex, con: MjrContext) -> None:
    ...

def mjr_restoreBuffer(con: MjrContext) -> None:
    ...

def mjr_setAux(index: typing.SupportsInt | typing.SupportsIndex, con: MjrContext) -> None:
    ...

def mjr_setBuffer(framebuffer: typing.SupportsInt | typing.SupportsIndex, con: MjrContext) -> None:
    ...

def mjr_text(font: typing.SupportsInt | typing.SupportsIndex, txt: str, con: MjrContext, x: typing.SupportsFloat | typing.SupportsIndex, y: typing.SupportsFloat | typing.SupportsIndex, r: typing.SupportsFloat | typing.SupportsIndex, g: typing.SupportsFloat | typing.SupportsIndex, b: typing.SupportsFloat | typing.SupportsIndex) -> None:
    ...

def mjr_uploadHField(m: mujoco._structs.MjModel, con: MjrContext, hfieldid: typing.SupportsInt | typing.SupportsIndex) -> None:
    ...

def mjr_uploadMesh(m: mujoco._structs.MjModel, con: MjrContext, meshid: typing.SupportsInt | typing.SupportsIndex) -> None:
    ...

def mjr_uploadTexture(m: mujoco._structs.MjModel, con: MjrContext, texid: typing.SupportsInt | typing.SupportsIndex) -> None:
    ...
