"""ROS 2 PX4 position offboard transport; optional ROS workspace dependency."""

from __future__ import annotations

import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

from airtruck.systems.base import Values, space_values
from airtruck.systems.flight import (
    flight_action_space,
    flight_observation_space,
    rotation,
    yaw_angle,
)

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLandDetected,
    VehicleOdometry,
    VehicleStatus,
)


@dataclass(frozen=True)
class PX4Config:
    namespace: str = ""
    system_id: int = 1
    component_id: int = 1
    source_system: int = 255
    source_component: int = 191
    timeout_s: float = 5.0
    telemetry_timeout_s: float = 1.0
    # Onboard tag detector output (absolute topic, camera optical frame).
    tag_topic: str = "/tag_detections"
    tag_camera: str = "front_down"

    def __post_init__(self) -> None:
        if any(
            type(value) is not int or not 1 <= value <= 255
            for value in (
                self.system_id,
                self.component_id,
                self.source_system,
                self.source_component,
            )
        ):
            raise ValueError("PX4 system/component IDs must be integers in [1,255]")
        if any(
            not np.isfinite(value) or value <= 0
            for value in (self.timeout_s, self.telemetry_timeout_s)
        ):
            raise ValueError("PX4 timeouts must be positive and finite")
        if not self.tag_topic.startswith("/") or not self.tag_camera:
            raise ValueError("tag_topic must be absolute and tag_camera nonempty")


class PX4Backend:
    """Pumped by driver reads/commands; no background thread or global ROS shutdown.

    Connection starts no flight mode and never arms. Explicit flight commands wait
    for both ACK and measured status. ROS timers stream targets at 20 Hz while the
    driver runs or a bounded command wait spins this backend's private context.
    """

    def __init__(self, name: str = "drone", config: PX4Config | None = None) -> None:
        self.name, self.config = name, config or PX4Config()
        self._node: Node | None = None
        self._context: Context | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._odometry: VehicleOdometry | None = None
        self._status: VehicleStatus | None = None
        self._land: VehicleLandDetected | None = None
        self._received: dict[str, float] = {}
        self._acks: dict[int, VehicleCommandAck] = {}
        self._target: Values | None = None
        self._stream_started: float | None = None
        self._last_heartbeat: float | None = None
        self._reset_counter: int | None = None
        self._tags: deque[Values] = deque(maxlen=64)

    @property
    def node(self) -> Node:
        if self._node is None:
            raise RuntimeError("PX4 backend is disconnected")
        return self._node

    @property
    def status(self) -> VehicleStatus:
        if self._status is None:
            raise RuntimeError("PX4 vehicle status is not available")
        return self._status

    def _topic(self, direction: str, name: str, message: type[object]) -> str:
        version = getattr(message, "MESSAGE_VERSION", 0)
        suffix = f"_v{version}" if version else ""
        prefix = self.config.namespace.strip("/")
        return f"/{prefix + '/' if prefix else ''}fmu/{direction}/{name}{suffix}"

    def connect(self) -> None:
        if self._node is not None:
            raise RuntimeError("PX4 backend is already connected")
        try:
            self._context = Context()
            rclpy.init(context=self._context)
            self._node = rclpy.create_node(
                f"{self.name}_offboard", context=self._context
            )
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self._mode_publisher = self.node.create_publisher(
                OffboardControlMode,
                self._topic("in", "offboard_control_mode", OffboardControlMode),
                qos,
            )
            self._target_publisher = self.node.create_publisher(
                TrajectorySetpoint,
                self._topic("in", "trajectory_setpoint", TrajectorySetpoint),
                qos,
            )
            self._command_publisher = self.node.create_publisher(
                VehicleCommand,
                self._topic("in", "vehicle_command", VehicleCommand),
                qos,
            )
            self.node.create_subscription(
                VehicleOdometry,
                self._topic("out", "vehicle_odometry", VehicleOdometry),
                self._on_odometry,
                qos_profile_sensor_data,
            )
            self.node.create_subscription(
                VehicleStatus,
                self._topic("out", "vehicle_status", VehicleStatus),
                self._on_status,
                qos_profile_sensor_data,
            )
            self.node.create_subscription(
                VehicleLandDetected,
                self._topic("out", "vehicle_land_detected", VehicleLandDetected),
                self._on_land,
                qos_profile_sensor_data,
            )
            self.node.create_subscription(
                PoseStamped,
                self.config.tag_topic,
                self._on_tag,
                qos_profile_sensor_data,
            )
            self.node.create_subscription(
                VehicleCommandAck,
                self._topic("out", "vehicle_command_ack", VehicleCommandAck),
                self._on_ack,
                qos_profile_sensor_data,
            )
            self._timer = self.node.create_timer(0.05, self._publish_target)
            self._wait(
                lambda: self._odometry is not None
                and self._status is not None
                and self._land is not None,
                "valid PX4 telemetry",
            )
            observation = self.get_observation()
            self.send_action(
                {
                    "position": observation["position"],
                    "yaw": yaw_angle(observation["quaternion"]),
                }
            )
        except BaseException:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        try:
            try:
                if self._executor is not None:
                    self._executor.shutdown()
            finally:
                if self._node is not None:
                    self._node.destroy_node()
        finally:
            self._node = None
            self._executor = None
            try:
                if self._context is not None:
                    self._context.try_shutdown()
            finally:
                self._context = None
                self._odometry = self._status = self._land = None
                self._received.clear()
                self._acks.clear()
                self._tags.clear()
                self._target = self._stream_started = self._reset_counter = None
                self._last_heartbeat = None

    def _spin(self, timeout: float = 0.0) -> None:
        if self._executor is None:
            raise RuntimeError("PX4 backend is disconnected")
        self._executor.spin_once(timeout_sec=timeout)

    def _wait(self, condition: Callable[[], bool], description: str) -> None:
        deadline = time.monotonic() + self.config.timeout_s
        while not condition():
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Timed out waiting for {description}")
            self._spin(0.01)

    def _on_odometry(self, message: VehicleOdometry) -> None:
        if message.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            raise ValueError("PX4 odometry must use the local NED pose frame")
        if (
            self._reset_counter is not None
            and message.reset_counter != self._reset_counter
        ):
            # Reusing an old absolute target after an estimator reset can cause a jump.
            self._target = None
            raise RuntimeError(
                "PX4 estimator origin reset; reconnect to seed a fresh hold target"
            )
        self._reset_counter = message.reset_counter
        self._odometry = message
        self._received["odometry"] = time.monotonic()

    def _on_status(self, message: VehicleStatus) -> None:
        self._status = message
        self._received["status"] = time.monotonic()

    def _on_land(self, message: VehicleLandDetected) -> None:
        self._land = message
        self._received["land"] = time.monotonic()

    def _on_tag(self, message: PoseStamped) -> None:
        pose, stamp = message.pose, message.header.stamp
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        self._tags.append(
            {
                "camera": self.config.tag_camera,
                "tag_id": None,  # PoseStamped carries no tag ID.
                "position": [pose.position.x, pose.position.y, pose.position.z],
                "quaternion": [
                    pose.orientation.w,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                ],
                "timestamp_s": timestamp
                or self.node.get_clock().now().nanoseconds * 1e-9,
            }
        )

    def pop_tag_detections(self) -> list[Values]:
        """Tag poses received while spinning since the previous call."""
        records = list(self._tags)
        self._tags.clear()
        return records

    def _on_ack(self, message: VehicleCommandAck) -> None:
        if (
            message.target_system == self.config.source_system
            and message.target_component == self.config.source_component
        ):
            self._acks[message.command] = message

    def get_observation(self) -> Values:
        for _ in range(8):
            self._spin()
        now = time.monotonic()
        if any(
            now - self._received.get(key, -np.inf) > self.config.telemetry_timeout_s
            for key in ("odometry", "status", "land")
        ):
            self._target = None
            raise TimeoutError("PX4 telemetry is missing or stale")
        odometry, land = self._odometry, self._land
        if odometry is None or land is None:
            raise RuntimeError("PX4 telemetry is not available")
        quaternion = np.asarray(odometry.q, dtype=float)
        if not np.isclose(np.linalg.norm(quaternion), 1):
            raise ValueError("PX4 attitude must be a valid unit quaternion")
        attitude = rotation(quaternion)
        velocity = np.asarray(odometry.velocity, dtype=float)
        if odometry.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
            velocity = attitude @ velocity
        elif odometry.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            raise ValueError("PX4 odometry velocity must use NED or body FRD")
        return space_values(
            flight_observation_space(),
            {
                "position": odometry.position,
                "quaternion": quaternion,
                "linear_velocity_m_s": velocity,
                "angular_velocity_rad_s": attitude
                @ np.asarray(odometry.angular_velocity),
                "armed": self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
                "offboard": self.status.nav_state
                == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
                "landed": land.landed,
            },
        )

    def send_action(self, action: Values) -> Values:
        target: Values = space_values(flight_action_space(), action)
        self._target = target
        return dict(target)

    def _publish_target(self) -> None:
        if self._target is None:
            return
        # Stop proof-of-life when state feedback is stale, rather than fly blind.
        if any(
            time.monotonic() - self._received.get(key, -np.inf)
            > self.config.telemetry_timeout_s
            for key in ("odometry", "status", "land")
        ):
            self._stream_started = self._last_heartbeat = None
            return
        timestamp = self.node.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode()
        mode.timestamp, mode.position = timestamp, True
        target = TrajectorySetpoint()
        target.timestamp = timestamp
        target.position = [float(value) for value in self._target["position"]]
        target.yaw = float(self._target["yaw"])
        target.velocity = [float("nan")] * 3
        target.acceleration = [float("nan")] * 3
        target.jerk = [float("nan")] * 3
        target.yawspeed = float("nan")
        self._mode_publisher.publish(mode)
        self._target_publisher.publish(target)
        now = time.monotonic()
        if self._last_heartbeat is None or now - self._last_heartbeat > 0.2:
            self._stream_started = now
        self._last_heartbeat = now

    def _command(
        self, command: int, condition: Callable[[], bool], **params: float
    ) -> None:
        self.get_observation()
        self._acks.pop(command, None)
        message = VehicleCommand()
        message.timestamp = self.node.get_clock().now().nanoseconds // 1000
        message.command, message.from_external = command, True
        for key, value in params.items():
            setattr(message, key, value)
        message.target_system, message.target_component = (
            self.config.system_id,
            self.config.component_id,
        )
        message.source_system, message.source_component = (
            self.config.source_system,
            self.config.source_component,
        )
        self._command_publisher.publish(message)

        def accepted() -> bool:
            ack = self._acks.get(command)
            if ack is None:
                return False
            if ack.result not in (
                VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
                VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
            ):
                raise RuntimeError(
                    f"PX4 rejected command {command}: result={ack.result}"
                )
            return (
                ack.result == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED
                and condition()
            )

        self._wait(
            accepted, f"PX4 command {command} acknowledgement and vehicle status"
        )

    def engage_offboard_mode(self) -> None:
        self.get_observation()
        if self._target is None:
            raise RuntimeError("Seed a position target before engaging offboard")
        self._wait(
            lambda: self._stream_started is not None
            and time.monotonic() - self._stream_started >= 1.1,
            "one second of offboard heartbeats and hold setpoints",
        )
        self._command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            lambda: self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            param1=1.0,
            param2=6.0,
        )

    def arm(self) -> None:
        self._command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            lambda: self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            param1=1.0,
        )

    def disarm(self) -> None:
        self._command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            lambda: self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED,
            param1=0.0,
        )

    def land(self) -> None:
        self._command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            lambda: self.status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
            param4=float("nan"),
            param5=float("nan"),
            param6=float("nan"),
            param7=float("nan"),
        )
