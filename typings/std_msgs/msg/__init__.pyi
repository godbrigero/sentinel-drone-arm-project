"""std_msgs/Header, ROS 2 Jazzy."""
from typing import Any

from builtin_interfaces.msg import Time

class Header:
    def __init__(self, **kwargs: Any) -> None: ...
    stamp: Time
    frame_id: str
