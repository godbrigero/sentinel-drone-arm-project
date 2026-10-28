"""geometry_msgs 4.9.1 pose messages, ROS 2 Humble."""
from typing import Any

from std_msgs.msg import Header

class Point:
    def __init__(self, **kwargs: Any) -> None: ...
    x: float
    y: float
    z: float

class Quaternion:
    def __init__(self, **kwargs: Any) -> None: ...
    x: float
    y: float
    z: float
    w: float

class Pose:
    def __init__(self, **kwargs: Any) -> None: ...
    position: Point
    orientation: Quaternion

class PoseStamped:
    def __init__(self, **kwargs: Any) -> None: ...
    header: Header
    pose: Pose
