"""Focused rclpy 3.3.21 overlay; see typings/README.md."""
from typing import Generic, TypeVar

MessageT = TypeVar("MessageT")

class Subscription(Generic[MessageT]): ...
