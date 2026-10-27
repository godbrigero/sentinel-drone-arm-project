"""Actual transport code on an in-memory ROS executor; no ROS graph or aircraft."""

import copy
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

import cv2

import airtruck.systems.px4 as px4
from airtruck.runtime.driver import SystemDriver
from airtruck.systems.apriltag import TAGS, AprilTagConfig, apriltag_image
from airtruck.systems.drone import Drone
from airtruck.systems.sentinel import SentinelDrone


class Message:
    MESSAGE_VERSION = 0

    def __init__(self, **values: Any) -> None:
        self.__dict__.update(values)


class Status(Message):
    arming_state: int
    nav_state: int
    MESSAGE_VERSION = 1
    ARMING_STATE_ARMED, ARMING_STATE_DISARMED = 2, 1
    NAVIGATION_STATE_OFFBOARD, NAVIGATION_STATE_AUTO_LAND = 14, 18


class Odometry(Message):
    reset_counter: int
    q: list[float]
    velocity_frame: int
    velocity: list[float]
    angular_velocity: list[float]
    POSE_FRAME_NED = VELOCITY_FRAME_NED = 1
    VELOCITY_FRAME_BODY_FRD = 3


class Command(Message):
    VEHICLE_CMD_DO_SET_MODE, VEHICLE_CMD_COMPONENT_ARM_DISARM, VEHICLE_CMD_NAV_LAND = (
        176,
        400,
        21,
    )


class Ack(Message):
    VEHICLE_CMD_RESULT_ACCEPTED, VEHICLE_CMD_RESULT_IN_PROGRESS = 0, 5


class ROSGraph:
    def __init__(self) -> None:
        self.now = 0.0
        self.telemetry = True
        self.reject = False
        self.acknowledge = True
        self.change_status = True
        self.pending: list[tuple[str, Any]] = []
        self.sent: list[tuple[float, str, Any]] = []
        self.subscriptions: dict[str, Any] = {}
        self.timers: list[Any] = []
        self.timer_due = 0.0
        self.destroyed = self.closed = False
        self.odom = Odometry(
            pose_frame=1,
            velocity_frame=1,
            position=[1.0, 2.0, -0.5],
            q=[1.0, 0, 0, 0],
            velocity=[0.0] * 3,
            angular_velocity=[0.0] * 3,
            reset_counter=0,
        )
        self.status = Status(arming_state=1, nav_state=0)
        self.land = Message(landed=False)

    def create_publisher(self, message: Any, topic: str, qos: Any) -> Any:
        return SimpleNamespace(publish=lambda value: self.publish(topic, value))

    def create_subscription(
        self, message: Any, topic: str, callback: Any, qos: Any
    ) -> None:
        self.subscriptions[topic] = callback

    def create_timer(self, interval: float, callback: Any) -> None:
        self.timers.append(callback)

    def get_clock(self) -> Any:
        return SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(self.now * 1e9))
        )

    def publish(self, topic: str, message: Any) -> None:
        self.sent.append((self.now, topic, copy.deepcopy(message)))
        if not topic.endswith("/vehicle_command"):
            return
        if self.change_status and not self.reject:
            if message.command == 176:
                self.status.nav_state = 14
            elif message.command == 400:
                self.status.arming_state = 2 if message.param1 else 1
            elif message.command == 21:
                self.status.nav_state = 18
        if self.acknowledge:
            self.pending.append(
                (
                    "vehicle_command_ack",
                    Ack(
                        command=message.command,
                        result=2 if self.reject else 0,
                        target_system=message.source_system,
                        target_component=message.source_component,
                    ),
                )
            )

    def spin_once(self, timeout_sec: float = 0) -> None:
        self.now += max(timeout_sec, 0.001)
        if self.telemetry:
            for topic, callback in self.subscriptions.items():
                if "/vehicle_odometry" in topic:
                    callback(self.odom)
                elif "/vehicle_status" in topic:
                    callback(self.status)
                elif "/vehicle_land_detected" in topic:
                    callback(self.land)
        if self.now >= self.timer_due:
            for callback in self.timers:
                callback()
            self.timer_due = self.now + 0.05
        while self.pending:
            name, message = self.pending.pop(0)
            for topic, callback in self.subscriptions.items():
                if topic.endswith("/" + name):
                    callback(message)

    def add_node(self, node: Any) -> None:
        pass

    def shutdown(self) -> None:
        self.closed = True

    def destroy_node(self) -> None:
        self.destroyed = True


@pytest.fixture
def graph(monkeypatch: pytest.MonkeyPatch) -> ROSGraph:
    graph = ROSGraph()
    monkeypatch.setattr(px4.time, "monotonic", lambda: graph.now)
    for key, value in {
        "Context": lambda: SimpleNamespace(try_shutdown=lambda: None),
        "rclpy": SimpleNamespace(
            init=lambda **kwargs: None, create_node=lambda *args, **kwargs: graph
        ),
        "SingleThreadedExecutor": lambda **kwargs: graph,
        "QoSProfile": lambda **kwargs: kwargs,
        "ReliabilityPolicy": SimpleNamespace(BEST_EFFORT=1),
        "DurabilityPolicy": SimpleNamespace(TRANSIENT_LOCAL=1),
        "HistoryPolicy": SimpleNamespace(KEEP_LAST=1),
        "qos_profile_sensor_data": object(),
        "OffboardControlMode": Message,
        "TrajectorySetpoint": Message,
        "VehicleCommand": Command,
        "VehicleCommandAck": Ack,
        "VehicleStatus": Status,
        "VehicleOdometry": Odometry,
        "VehicleLandDetected": Message,
    }.items():
        monkeypatch.setattr(px4, key, value, raising=False)
    return graph


def test_px4_streams_hold_before_mode_and_confirms_flight_commands(
    graph: ROSGraph,
) -> None:
    backend = px4.PX4Backend(config=px4.PX4Config(namespace="px4_1"))
    backend.connect()
    assert not any(topic.endswith("/vehicle_command") for _, topic, _ in graph.sent)
    assert "/px4_1/fmu/out/vehicle_status_v1" in graph.subscriptions
    backend.engage_offboard_mode()
    backend.arm()
    observations = backend.get_observation()
    assert observations["armed"] and observations["offboard"]
    first_command = next(
        index
        for index, (_, topic, _) in enumerate(graph.sent)
        if topic.endswith("/vehicle_command")
    )
    targets = [
        (stamp, message)
        for stamp, topic, message in graph.sent[:first_command]
        if topic.endswith("/trajectory_setpoint")
    ]
    assert len(targets) >= 20 and targets[-1][0] - targets[0][0] >= 1.0
    np.testing.assert_allclose(targets[0][1].position, [1, 2, -0.5])
    assert np.isnan(targets[0][1].velocity).all()
    backend.send_action({"position": np.array([2.0, 3.0, -1]), "yaw": 0.3})
    for _ in range(10):
        backend.get_observation()
    target = next(
        message
        for _, topic, message in reversed(graph.sent)
        if topic.endswith("/trajectory_setpoint")
    )
    np.testing.assert_allclose(target.position, [2, 3, -1])
    assert target.yaw == pytest.approx(0.3)
    backend.land()
    assert not backend.get_observation()["offboard"]
    backend.disarm()
    assert not backend.get_observation()["armed"]
    commands = [
        message
        for _, topic, message in graph.sent
        if topic.endswith("/vehicle_command")
    ]
    assert [message.command for message in commands] == [176, 400, 21, 400]
    assert commands[-2].target_system == 1 and commands[-2].from_external
    backend.disconnect()
    assert graph.destroyed and graph.closed


@pytest.mark.parametrize("failure", ["rejected", "no_ack", "no_status"])
def test_px4_never_assumes_a_command_succeeded(graph: ROSGraph, failure: str) -> None:
    backend = px4.PX4Backend(config=px4.PX4Config(timeout_s=0.2))
    backend.connect()
    graph.reject = failure == "rejected"
    graph.acknowledge = failure != "no_ack"
    graph.change_status = failure != "no_status"
    with pytest.raises((RuntimeError, TimeoutError)):
        backend.arm()
    backend.disconnect()


def test_stale_telemetry_stops_heartbeat_and_reset_origin_is_rejected(
    graph: ROSGraph,
) -> None:
    backend = px4.PX4Backend()
    backend.connect()
    backend.get_observation()
    graph.telemetry = False
    graph.now += 2
    count = len(graph.sent)
    with pytest.raises(TimeoutError, match="stale"):
        backend.get_observation()
    assert len(graph.sent) == count
    backend.disconnect()
    graph.telemetry = True
    backend.connect()
    graph.odom.reset_counter += 1
    with pytest.raises(RuntimeError, match="origin reset"):
        backend.get_observation()
    assert backend._target is None
    backend.disconnect()


def test_body_velocity_and_attitude_are_returned_in_ned(graph: ROSGraph) -> None:
    graph.odom.q = [2**-0.5, 0, 0, 2**-0.5]
    graph.odom.velocity_frame = 3
    graph.odom.velocity = [1, 0, 0]
    graph.odom.angular_velocity = [1, 0, 0]
    backend = px4.PX4Backend()
    backend.connect()
    state = backend.get_observation()
    np.testing.assert_allclose(state["linear_velocity_m_s"], [0, 1, 0], atol=1e-6)
    np.testing.assert_allclose(state["angular_velocity_rad_s"], [0, 1, 0], atol=1e-6)
    backend.disconnect()


def test_partial_connect_timeout_cleans_up_the_private_ros_context(
    graph: ROSGraph,
) -> None:
    graph.telemetry = False
    backend = px4.PX4Backend(config=px4.PX4Config(timeout_s=0.1))
    with pytest.raises(TimeoutError, match="telemetry"):
        backend.connect()
    assert graph.closed and graph.destroyed and backend._context is None
    assert not graph.sent


def test_offboard_reentry_requires_a_fresh_continuous_heartbeat_stream(
    graph: ROSGraph,
) -> None:
    backend = px4.PX4Backend()
    backend.connect()
    backend.engage_offboard_mode()
    graph.now += 2.0
    started = graph.now
    backend.engage_offboard_mode()
    assert graph.now - started >= 1.1
    backend.disconnect()


def test_drone_hardware_uses_the_same_targets_and_confirmed_state(
    graph: ROSGraph, drone_model: Path
) -> None:
    backend = px4.PX4Backend()
    drone = Drone("drone", model=drone_model, hardware=True, backend=backend)
    with SystemDriver([drone]) as driver:
        driver.reset()
        assert not drone.state["armed"]
        drone.takeoff(0.7)
        assert drone.state["armed"] and drone.state["offboard"]
        np.testing.assert_allclose(drone.controls["position"], [1, 2, -1.2])
        assert driver.data.time == 0
        driver.step()
        assert backend._target is not None
        np.testing.assert_allclose(backend._target["position"], np.array([1, 2, -1.2]))
        np.testing.assert_allclose(driver.data.qpos[:3], [1, -2, 0.5], atol=0.003)
        drone.hold()
        np.testing.assert_allclose(drone.controls["position"], [1, 2, -0.5])
        drone.land()
        assert not drone.state["offboard"]
        drone.hardware = False
        driver.step()
        assert not drone.hardware_active and graph.destroyed


def tag_pose(x: float, y: float, z: float, stamp: float = 12.5) -> Any:
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(sec=int(stamp), nanosec=int(stamp % 1 * 1e9)),
            frame_id="tracking",
        ),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def test_ros_tag_detections_update_the_drone_and_global_board(
    graph: ROSGraph,
) -> None:
    backend = px4.PX4Backend()
    drone = SentinelDrone(
        "tagged",
        hardware=True,
        backend=backend,
    )
    with SystemDriver([drone]) as driver:
        driver.reset()
        assert "/tag_detections" in graph.subscriptions
        assert drone.latest_tag() is None
        # The same PoseStamped topic and camera-frame meaning as the onboard node.
        graph.subscriptions["/tag_detections"](tag_pose(0.1, 0.2, 1.5))
        driver.step()
        detection = drone.latest_tag()
        assert detection is TAGS.latest(vehicle="tagged")
        assert detection is not None and detection.source == "ros"
        assert detection.camera == "front_down" and detection.tag_id is None
        np.testing.assert_allclose(detection.position_m, [0.1, 0.2, 1.5])
        assert detection.timestamp_s == pytest.approx(12.5)
        # The vehicle is level at NED [1, 2, -0.5]; front_down looks forward/down.
        camera = drone.cameras["front_down"]
        expected = np.array([1, 2, -0.5]) + camera.body_point([0.1, 0.2, 1.5])
        assert detection.position_ned_m is not None
        np.testing.assert_allclose(detection.position_ned_m, expected, atol=1e-5)
        assert expected[0] > 1.9 and expected[2] > 0.5  # Ahead of and below the drone.
        assert backend.pop_tag_detections() == []


def test_hardware_frames_can_be_published_for_tag_detection(graph: ROSGraph) -> None:
    drone = SentinelDrone(
        "framed",
        hardware=True,
        backend=px4.PX4Backend(),
        apriltags=AprilTagConfig(tag_size_m=0.16, cameras=("rear_left",)),
    )
    with SystemDriver([drone]) as driver:
        driver.reset()
        with pytest.raises(RuntimeError, match="No frame"):
            drone.capture("rear_left")
        frame = np.full((480, 640, 3), 128, np.uint8)
        tag = cv2.resize(apriltag_image(9), (200, 200), interpolation=cv2.INTER_NEAREST)
        frame[140:340, 220:420] = tag
        drone.cameras["rear_left"].publish(frame, timestamp_s=3.0)
        assert drone.capture("rear_left") is drone.cameras["rear_left"].frame
        graph.now += 1.0  # Next detection period on the hardware clock.
        driver.step()
        detection = drone.latest_tag(tag_id=9)
        assert detection is not None and detection.source == "camera"
        assert detection.timestamp_s == 3.0
        # Centered, facing the camera: 0.16 m tag over 160 px at f=349.2 px.
        np.testing.assert_allclose(detection.position_m, [0, 0, 0.349], atol=0.01)


def test_sentinel_hardware_uses_the_identical_api(graph: ROSGraph) -> None:
    backend = px4.PX4Backend()
    drone = SentinelDrone("real", hardware=True, backend=backend)
    with SystemDriver([drone]) as driver:
        assert drone.is_connected and drone.hardware_active  # Opened on entry.
        driver.reset()
        assert not drone.state["armed"]  # Connecting never arms.
        accepted = drone.send_action({"position": np.array([1.0, 2.0, -1.5])})
        np.testing.assert_allclose(accepted["position"], [1, 2, -1.5])
        assert backend._target is not None
        np.testing.assert_allclose(backend._target["position"], [1, 2, -1.5])
        assert drone.get_observation().keys() == drone.state.keys()
        assert set(drone.cameras) == {"front_down", "rear_left"}
        drone.disconnect()
        assert not drone.is_connected and graph.destroyed
