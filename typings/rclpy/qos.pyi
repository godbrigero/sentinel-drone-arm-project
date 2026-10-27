"""Focused rclpy 7.1.12 overlay; see typings/README.md."""
from enum import IntEnum

class ReliabilityPolicy(IntEnum):
    RELIABLE = 1
    BEST_EFFORT = 2

class DurabilityPolicy(IntEnum):
    TRANSIENT_LOCAL = 1
    VOLATILE = 2

class HistoryPolicy(IntEnum):
    KEEP_LAST = 1
    KEEP_ALL = 2

class QoSProfile:
    def __init__(self, *, reliability: ReliabilityPolicy, durability: DurabilityPolicy, history: HistoryPolicy, depth: int) -> None: ...

qos_profile_sensor_data: QoSProfile
