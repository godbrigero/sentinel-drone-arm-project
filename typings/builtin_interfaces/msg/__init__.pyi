"""builtin_interfaces/Time, ROS 2 Humble."""
from typing import Any

class Time:
    def __init__(self, **kwargs: Any) -> None: ...
    sec: int
    nanosec: int
