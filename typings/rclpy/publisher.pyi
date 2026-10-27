"""Focused rclpy 7.1.12 overlay; see typings/README.md."""
from typing import Generic, TypeVar

MessageT = TypeVar("MessageT")

class Publisher(Generic[MessageT]):
    def publish(self, message: MessageT) -> None: ...
