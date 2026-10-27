"""builtin_interfaces/Time, ROS 2 Jazzy."""
from typing import Any

class Time:
    def __init__(self, **kwargs: Any) -> None: ...
    sec: int
    nanosec: int
