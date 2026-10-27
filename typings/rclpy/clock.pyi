"""Focused rclpy 7.1.12 overlay; see typings/README.md."""
from .time import Time

class Clock:
    def now(self) -> Time: ...
