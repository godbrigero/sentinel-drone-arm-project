"""Static-only usage examples; does not open a viewer or load external assets."""
from typing import assert_type
import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray

def check_api(model: mujoco.MjModel, data: mujoco.MjData,
              handle: mujoco.viewer.Handle) -> None:
    assert_type(data.qpos, NDArray[np.float64])
    assert_type(model.body_mass, NDArray[np.float64])
    assert_type(data.joint("elbow").qvel, NDArray[np.float64])
    assert_type(model.actuator("elbow").id, int)
    assert_type(data.body("hand").xpos, NDArray[np.float64])
    assert_type(handle.cam, mujoco.MjvCamera)
    assert_type(handle.is_running(), bool)
    with handle.lock():
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data, nstep=10)
    handle.sync()
    mujoco.mj_forward(model, data)
    assert_type(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "elbow"), int)
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_axisAngle2Quat(quat, np.array([0., 0., 1.]), .1)
    mujoco.set_mjcb_control(controller)
    mujoco.set_mjcb_control(None)
    with mujoco.Renderer(model) as renderer:
        renderer.update_scene(data)
        pixels = renderer.render()
        assert_type(pixels, np.ndarray)  # Modes select RGB, depth, or segmentation dtype.

def controller(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    data.ctrl[:] = 0.0
