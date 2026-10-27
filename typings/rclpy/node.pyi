"""Focused rclpy 7.1.12 overlay; see typings/README.md."""
from collections.abc import Callable
from typing import TypeVar
from .clock import Clock
from .publisher import Publisher
from .qos import QoSProfile
from .subscription import Subscription
from .timer import Timer

MessageT = TypeVar("MessageT")

class Node:
    def create_publisher(self, msg_type: type[MessageT], topic: str, qos_profile: QoSProfile) -> Publisher[MessageT]: ...
    def create_subscription(self, msg_type: type[MessageT], topic: str, callback: Callable[[MessageT], None], qos_profile: QoSProfile) -> Subscription[MessageT]: ...
    def create_timer(self, timer_period_sec: float, callback: Callable[[], None]) -> Timer: ...
    def get_clock(self) -> Clock: ...
    def destroy_node(self) -> None: ...
