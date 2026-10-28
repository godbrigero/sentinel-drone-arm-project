"""Focused rclpy 3.3.21 overlay; see typings/README.md."""
from .time import Time

class Clock:
    def now(self) -> Time: ...
