"""Static examples for native MuJoCo and Gymnasium access."""

from collections.abc import Sequence
from typing import assert_type

import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import NDArray
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data

from airtruck.simulation.driver import MujocoDriver
from airtruck.systems.base import Values


def check_api(
    model: mujoco.MjModel, data: mujoco.MjData, handle: mujoco.viewer.Handle
) -> None:
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
    mujoco.mju_axisAngle2Quat(quat, np.array([0.0, 0.0, 1.0]), 0.1)
    mujoco.set_mjcb_control(controller)
    mujoco.set_mjcb_control(None)
    with mujoco.Renderer(model) as renderer:
        renderer.update_scene(data)
        pixels = renderer.render()
        assert_type(
            pixels, np.ndarray
        )  # Modes select RGB, depth, or segmentation dtype.


def controller(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    data.ctrl[:] = 0.0


def check_driver(driver: MujocoDriver) -> None:
    assert_type(driver.data.qpos, NDArray[np.float64])
    driver.reset(seed=42)
    observations = driver.step()
    assert_type(observations, dict[str, Values])
    env = driver.as_gymnasium(flatten=True)
    env.reset(seed=42)
    env.step(env.action_space.sample())


def check_px4(node: Node) -> None:
    target = TrajectorySetpoint()
    assert_type(target.timestamp, int)
    assert_type(target.position, Sequence[float])
    assert_type(target.yaw, float)
    publisher = node.create_publisher(
        TrajectorySetpoint, "/typing_only", qos_profile_sensor_data
    )
    assert_type(publisher, Publisher[TrajectorySetpoint])
    publisher.publish(target)
    # These ignores must stay necessary: fields and publisher payloads are typed.
    target.yaw = "north"  # type: ignore[assignment]
    publisher.publish(VehicleCommand())  # type: ignore[arg-type]
