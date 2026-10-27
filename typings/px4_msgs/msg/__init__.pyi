"""PX4 msg declarations, revision 392e831c1f659429ca83902e66820d7094591410."""
from collections.abc import Sequence
from typing import Any, ClassVar

class OffboardControlMode:
    def __init__(self, **kwargs: Any) -> None: ...
    timestamp: int
    position: bool
    velocity: bool
    acceleration: bool
    attitude: bool
    body_rate: bool
    thrust_and_torque: bool
    direct_actuator: bool

class TrajectorySetpoint:
    def __init__(self, **kwargs: Any) -> None: ...
    MESSAGE_VERSION: ClassVar[int] = 0
    timestamp: int
    position: Sequence[float]
    velocity: Sequence[float]
    acceleration: Sequence[float]
    jerk: Sequence[float]
    yaw: float
    yawspeed: float

class VehicleCommand:
    def __init__(self, **kwargs: Any) -> None: ...
    MESSAGE_VERSION: ClassVar[int] = 0
    timestamp: int
    VEHICLE_CMD_CUSTOM_0: ClassVar[int] = 0
    VEHICLE_CMD_CUSTOM_1: ClassVar[int] = 1
    VEHICLE_CMD_CUSTOM_2: ClassVar[int] = 2
    VEHICLE_CMD_NAV_WAYPOINT: ClassVar[int] = 16
    VEHICLE_CMD_NAV_LOITER_UNLIM: ClassVar[int] = 17
    VEHICLE_CMD_NAV_LOITER_TURNS: ClassVar[int] = 18
    VEHICLE_CMD_NAV_LOITER_TIME: ClassVar[int] = 19
    VEHICLE_CMD_NAV_RETURN_TO_LAUNCH: ClassVar[int] = 20
    VEHICLE_CMD_NAV_LAND: ClassVar[int] = 21
    VEHICLE_CMD_NAV_TAKEOFF: ClassVar[int] = 22
    VEHICLE_CMD_NAV_PRECLAND: ClassVar[int] = 23
    VEHICLE_CMD_DO_ORBIT: ClassVar[int] = 34
    VEHICLE_CMD_DO_FIGUREEIGHT: ClassVar[int] = 35
    VEHICLE_CMD_NAV_ROI: ClassVar[int] = 80
    VEHICLE_CMD_NAV_PATHPLANNING: ClassVar[int] = 81
    VEHICLE_CMD_NAV_VTOL_TAKEOFF: ClassVar[int] = 84
    VEHICLE_CMD_NAV_VTOL_LAND: ClassVar[int] = 85
    VEHICLE_CMD_NAV_GUIDED_LIMITS: ClassVar[int] = 90
    VEHICLE_CMD_NAV_GUIDED_MASTER: ClassVar[int] = 91
    VEHICLE_CMD_NAV_DELAY: ClassVar[int] = 93
    VEHICLE_CMD_NAV_LAST: ClassVar[int] = 95
    VEHICLE_CMD_CONDITION_DELAY: ClassVar[int] = 112
    VEHICLE_CMD_CONDITION_CHANGE_ALT: ClassVar[int] = 113
    VEHICLE_CMD_CONDITION_DISTANCE: ClassVar[int] = 114
    VEHICLE_CMD_CONDITION_YAW: ClassVar[int] = 115
    VEHICLE_CMD_CONDITION_LAST: ClassVar[int] = 159
    VEHICLE_CMD_CONDITION_GATE: ClassVar[int] = 4501
    VEHICLE_CMD_DO_SET_MODE: ClassVar[int] = 176
    VEHICLE_CMD_DO_JUMP: ClassVar[int] = 177
    VEHICLE_CMD_DO_CHANGE_SPEED: ClassVar[int] = 178
    VEHICLE_CMD_DO_SET_HOME: ClassVar[int] = 179
    VEHICLE_CMD_DO_SET_PARAMETER: ClassVar[int] = 180
    VEHICLE_CMD_DO_SET_RELAY: ClassVar[int] = 181
    VEHICLE_CMD_DO_REPEAT_RELAY: ClassVar[int] = 182
    VEHICLE_CMD_DO_REPEAT_SERVO: ClassVar[int] = 184
    VEHICLE_CMD_DO_FLIGHTTERMINATION: ClassVar[int] = 185
    VEHICLE_CMD_DO_CHANGE_ALTITUDE: ClassVar[int] = 186
    VEHICLE_CMD_DO_SET_ACTUATOR: ClassVar[int] = 187
    VEHICLE_CMD_DO_LAND_START: ClassVar[int] = 189
    VEHICLE_CMD_DO_GO_AROUND: ClassVar[int] = 191
    VEHICLE_CMD_DO_REPOSITION: ClassVar[int] = 192
    VEHICLE_CMD_DO_PAUSE_CONTINUE: ClassVar[int] = 193
    VEHICLE_CMD_DO_SET_ROI_LOCATION: ClassVar[int] = 195
    VEHICLE_CMD_DO_SET_ROI_WPNEXT_OFFSET: ClassVar[int] = 196
    VEHICLE_CMD_DO_SET_ROI_NONE: ClassVar[int] = 197
    VEHICLE_CMD_DO_CONTROL_VIDEO: ClassVar[int] = 200
    VEHICLE_CMD_DO_SET_ROI: ClassVar[int] = 201
    VEHICLE_CMD_DO_DIGICAM_CONTROL: ClassVar[int] = 203
    VEHICLE_CMD_DO_MOUNT_CONFIGURE: ClassVar[int] = 204
    VEHICLE_CMD_DO_MOUNT_CONTROL: ClassVar[int] = 205
    VEHICLE_CMD_DO_SET_CAM_TRIGG_DIST: ClassVar[int] = 206
    VEHICLE_CMD_DO_FENCE_ENABLE: ClassVar[int] = 207
    VEHICLE_CMD_DO_PARACHUTE: ClassVar[int] = 208
    VEHICLE_CMD_DO_MOTOR_TEST: ClassVar[int] = 209
    VEHICLE_CMD_DO_INVERTED_FLIGHT: ClassVar[int] = 210
    VEHICLE_CMD_DO_GRIPPER: ClassVar[int] = 211
    VEHICLE_CMD_DO_SET_CAM_TRIGG_INTERVAL: ClassVar[int] = 214
    VEHICLE_CMD_DO_MOUNT_CONTROL_QUAT: ClassVar[int] = 220
    VEHICLE_CMD_DO_GUIDED_MASTER: ClassVar[int] = 221
    VEHICLE_CMD_DO_GUIDED_LIMITS: ClassVar[int] = 222
    VEHICLE_CMD_DO_LAST: ClassVar[int] = 240
    VEHICLE_CMD_PREFLIGHT_CALIBRATION: ClassVar[int] = 241
    PREFLIGHT_CALIBRATION_TEMPERATURE_CALIBRATION: ClassVar[int] = 3
    VEHICLE_CMD_PREFLIGHT_SET_SENSOR_OFFSETS: ClassVar[int] = 242
    VEHICLE_CMD_PREFLIGHT_UAVCAN: ClassVar[int] = 243
    VEHICLE_CMD_PREFLIGHT_STORAGE: ClassVar[int] = 245
    VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN: ClassVar[int] = 246
    VEHICLE_CMD_OBLIQUE_SURVEY: ClassVar[int] = 260
    VEHICLE_CMD_DO_SET_STANDARD_MODE: ClassVar[int] = 262
    VEHICLE_CMD_GIMBAL_DEVICE_INFORMATION: ClassVar[int] = 283
    VEHICLE_CMD_MISSION_START: ClassVar[int] = 300
    VEHICLE_CMD_ACTUATOR_TEST: ClassVar[int] = 310
    VEHICLE_CMD_CONFIGURE_ACTUATOR: ClassVar[int] = 311
    VEHICLE_CMD_COMPONENT_ARM_DISARM: ClassVar[int] = 400
    VEHICLE_CMD_RUN_PREARM_CHECKS: ClassVar[int] = 401
    VEHICLE_CMD_INJECT_FAILURE: ClassVar[int] = 420
    VEHICLE_CMD_START_RX_PAIR: ClassVar[int] = 500
    VEHICLE_CMD_REQUEST_MESSAGE: ClassVar[int] = 512
    VEHICLE_CMD_REQUEST_CAMERA_INFORMATION: ClassVar[int] = 521
    VEHICLE_CMD_SET_CAMERA_MODE: ClassVar[int] = 530
    VEHICLE_CMD_SET_CAMERA_ZOOM: ClassVar[int] = 531
    VEHICLE_CMD_SET_CAMERA_FOCUS: ClassVar[int] = 532
    VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW: ClassVar[int] = 1000
    VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE: ClassVar[int] = 1001
    VEHICLE_CMD_IMAGE_START_CAPTURE: ClassVar[int] = 2000
    VEHICLE_CMD_DO_TRIGGER_CONTROL: ClassVar[int] = 2003
    VEHICLE_CMD_VIDEO_START_CAPTURE: ClassVar[int] = 2500
    VEHICLE_CMD_VIDEO_STOP_CAPTURE: ClassVar[int] = 2501
    VEHICLE_CMD_LOGGING_START: ClassVar[int] = 2510
    VEHICLE_CMD_LOGGING_STOP: ClassVar[int] = 2511
    VEHICLE_CMD_CONTROL_HIGH_LATENCY: ClassVar[int] = 2600
    VEHICLE_CMD_DO_VTOL_TRANSITION: ClassVar[int] = 3000
    VEHICLE_CMD_ARM_AUTHORIZATION_REQUEST: ClassVar[int] = 3001
    VEHICLE_CMD_PAYLOAD_PREPARE_DEPLOY: ClassVar[int] = 30001
    VEHICLE_CMD_PAYLOAD_CONTROL_DEPLOY: ClassVar[int] = 30002
    VEHICLE_CMD_FIXED_MAG_CAL_YAW: ClassVar[int] = 42006
    VEHICLE_CMD_DO_WINCH: ClassVar[int] = 42600
    VEHICLE_CMD_EXTERNAL_POSITION_ESTIMATE: ClassVar[int] = 43003
    VEHICLE_CMD_EXTERNAL_WIND_ESTIMATE: ClassVar[int] = 43004
    VEHICLE_CMD_PX4_INTERNAL_START: ClassVar[int] = 65537
    VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN: ClassVar[int] = 100000
    VEHICLE_CMD_SET_NAV_STATE: ClassVar[int] = 100001
    VEHICLE_MOUNT_MODE_RETRACT: ClassVar[int] = 0
    VEHICLE_MOUNT_MODE_NEUTRAL: ClassVar[int] = 1
    VEHICLE_MOUNT_MODE_MAVLINK_TARGETING: ClassVar[int] = 2
    VEHICLE_MOUNT_MODE_RC_TARGETING: ClassVar[int] = 3
    VEHICLE_MOUNT_MODE_GPS_POINT: ClassVar[int] = 4
    VEHICLE_MOUNT_MODE_ENUM_END: ClassVar[int] = 5
    VEHICLE_ROI_NONE: ClassVar[int] = 0
    VEHICLE_ROI_WPNEXT: ClassVar[int] = 1
    VEHICLE_ROI_WPINDEX: ClassVar[int] = 2
    VEHICLE_ROI_LOCATION: ClassVar[int] = 3
    VEHICLE_ROI_TARGET: ClassVar[int] = 4
    VEHICLE_ROI_ENUM_END: ClassVar[int] = 5
    PARACHUTE_ACTION_DISABLE: ClassVar[int] = 0
    PARACHUTE_ACTION_ENABLE: ClassVar[int] = 1
    PARACHUTE_ACTION_RELEASE: ClassVar[int] = 2
    FAILURE_UNIT_SENSOR_GYRO: ClassVar[int] = 0
    FAILURE_UNIT_SENSOR_ACCEL: ClassVar[int] = 1
    FAILURE_UNIT_SENSOR_MAG: ClassVar[int] = 2
    FAILURE_UNIT_SENSOR_BARO: ClassVar[int] = 3
    FAILURE_UNIT_SENSOR_GPS: ClassVar[int] = 4
    FAILURE_UNIT_SENSOR_OPTICAL_FLOW: ClassVar[int] = 5
    FAILURE_UNIT_SENSOR_VIO: ClassVar[int] = 6
    FAILURE_UNIT_SENSOR_DISTANCE_SENSOR: ClassVar[int] = 7
    FAILURE_UNIT_SENSOR_AIRSPEED: ClassVar[int] = 8
    FAILURE_UNIT_SYSTEM_BATTERY: ClassVar[int] = 100
    FAILURE_UNIT_SYSTEM_MOTOR: ClassVar[int] = 101
    FAILURE_UNIT_SYSTEM_SERVO: ClassVar[int] = 102
    FAILURE_UNIT_SYSTEM_AVOIDANCE: ClassVar[int] = 103
    FAILURE_UNIT_SYSTEM_RC_SIGNAL: ClassVar[int] = 104
    FAILURE_UNIT_SYSTEM_MAVLINK_SIGNAL: ClassVar[int] = 105
    FAILURE_TYPE_OK: ClassVar[int] = 0
    FAILURE_TYPE_OFF: ClassVar[int] = 1
    FAILURE_TYPE_STUCK: ClassVar[int] = 2
    FAILURE_TYPE_GARBAGE: ClassVar[int] = 3
    FAILURE_TYPE_WRONG: ClassVar[int] = 4
    FAILURE_TYPE_SLOW: ClassVar[int] = 5
    FAILURE_TYPE_DELAYED: ClassVar[int] = 6
    FAILURE_TYPE_INTERMITTENT: ClassVar[int] = 7
    SPEED_TYPE_AIRSPEED: ClassVar[int] = 0
    SPEED_TYPE_GROUNDSPEED: ClassVar[int] = 1
    SPEED_TYPE_CLIMB_SPEED: ClassVar[int] = 2
    SPEED_TYPE_DESCEND_SPEED: ClassVar[int] = 3
    ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER: ClassVar[int] = 0
    ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING: ClassVar[int] = 1
    ORBIT_YAW_BEHAVIOUR_UNCONTROLLED: ClassVar[int] = 2
    ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE: ClassVar[int] = 3
    ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED: ClassVar[int] = 4
    ORBIT_YAW_BEHAVIOUR_UNCHANGED: ClassVar[int] = 5
    ARMING_ACTION_DISARM: ClassVar[int] = 0
    ARMING_ACTION_ARM: ClassVar[int] = 1
    GRIPPER_ACTION_RELEASE: ClassVar[int] = 0
    GRIPPER_ACTION_GRAB: ClassVar[int] = 1
    ORB_QUEUE_LENGTH: ClassVar[int] = 8
    param1: float
    param2: float
    param3: float
    param4: float
    param5: float
    param6: float
    param7: float
    command: int
    target_system: int
    target_component: int
    source_system: int
    source_component: int
    confirmation: int
    from_external: bool
    COMPONENT_MODE_EXECUTOR_START: ClassVar[int] = 1000

class VehicleCommandAck:
    def __init__(self, **kwargs: Any) -> None: ...
    MESSAGE_VERSION: ClassVar[int] = 0
    timestamp: int
    VEHICLE_CMD_RESULT_ACCEPTED: ClassVar[int] = 0
    VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: ClassVar[int] = 1
    VEHICLE_CMD_RESULT_DENIED: ClassVar[int] = 2
    VEHICLE_CMD_RESULT_UNSUPPORTED: ClassVar[int] = 3
    VEHICLE_CMD_RESULT_FAILED: ClassVar[int] = 4
    VEHICLE_CMD_RESULT_IN_PROGRESS: ClassVar[int] = 5
    VEHICLE_CMD_RESULT_CANCELLED: ClassVar[int] = 6
    ARM_AUTH_DENIED_REASON_GENERIC: ClassVar[int] = 0
    ARM_AUTH_DENIED_REASON_NONE: ClassVar[int] = 1
    ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT: ClassVar[int] = 2
    ARM_AUTH_DENIED_REASON_TIMEOUT: ClassVar[int] = 3
    ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE: ClassVar[int] = 4
    ARM_AUTH_DENIED_REASON_BAD_WEATHER: ClassVar[int] = 5
    ORB_QUEUE_LENGTH: ClassVar[int] = 4
    command: int
    result: int
    result_param1: int
    result_param2: int
    target_system: int
    target_component: int
    from_external: bool

class VehicleLandDetected:
    def __init__(self, **kwargs: Any) -> None: ...
    MESSAGE_VERSION: ClassVar[int] = 0
    timestamp: int
    freefall: bool
    ground_contact: bool
    maybe_landed: bool
    landed: bool
    in_ground_effect: bool
    in_descend: bool
    has_low_throttle: bool
    vertical_movement: bool
    horizontal_movement: bool
    rotational_movement: bool
    close_to_ground_or_skipped_check: bool
    at_rest: bool

class VehicleOdometry:
    def __init__(self, **kwargs: Any) -> None: ...
    MESSAGE_VERSION: ClassVar[int] = 0
    timestamp: int
    timestamp_sample: int
    POSE_FRAME_UNKNOWN: ClassVar[int] = 0
    POSE_FRAME_NED: ClassVar[int] = 1
    POSE_FRAME_FRD: ClassVar[int] = 2
    pose_frame: int
    position: Sequence[float]
    q: Sequence[float]
    VELOCITY_FRAME_UNKNOWN: ClassVar[int] = 0
    VELOCITY_FRAME_NED: ClassVar[int] = 1
    VELOCITY_FRAME_FRD: ClassVar[int] = 2
    VELOCITY_FRAME_BODY_FRD: ClassVar[int] = 3
    velocity_frame: int
    velocity: Sequence[float]
    angular_velocity: Sequence[float]
    position_variance: Sequence[float]
    orientation_variance: Sequence[float]
    velocity_variance: Sequence[float]
    reset_counter: int
    quality: int

class VehicleStatus:
    def __init__(self, **kwargs: Any) -> None: ...
    MESSAGE_VERSION: ClassVar[int] = 1
    timestamp: int
    armed_time: int
    takeoff_time: int
    arming_state: int
    ARMING_STATE_DISARMED: ClassVar[int] = 1
    ARMING_STATE_ARMED: ClassVar[int] = 2
    latest_arming_reason: int
    latest_disarming_reason: int
    ARM_DISARM_REASON_TRANSITION_TO_STANDBY: ClassVar[int] = 0
    ARM_DISARM_REASON_STICK_GESTURE: ClassVar[int] = 1
    ARM_DISARM_REASON_RC_SWITCH: ClassVar[int] = 2
    ARM_DISARM_REASON_COMMAND_INTERNAL: ClassVar[int] = 3
    ARM_DISARM_REASON_COMMAND_EXTERNAL: ClassVar[int] = 4
    ARM_DISARM_REASON_MISSION_START: ClassVar[int] = 5
    ARM_DISARM_REASON_SAFETY_BUTTON: ClassVar[int] = 6
    ARM_DISARM_REASON_AUTO_DISARM_LAND: ClassVar[int] = 7
    ARM_DISARM_REASON_AUTO_DISARM_PREFLIGHT: ClassVar[int] = 8
    ARM_DISARM_REASON_KILL_SWITCH: ClassVar[int] = 9
    ARM_DISARM_REASON_LOCKDOWN: ClassVar[int] = 10
    ARM_DISARM_REASON_FAILURE_DETECTOR: ClassVar[int] = 11
    ARM_DISARM_REASON_SHUTDOWN: ClassVar[int] = 12
    ARM_DISARM_REASON_UNIT_TEST: ClassVar[int] = 13
    nav_state_timestamp: int
    nav_state_user_intention: int
    nav_state: int
    NAVIGATION_STATE_MANUAL: ClassVar[int] = 0
    NAVIGATION_STATE_ALTCTL: ClassVar[int] = 1
    NAVIGATION_STATE_POSCTL: ClassVar[int] = 2
    NAVIGATION_STATE_AUTO_MISSION: ClassVar[int] = 3
    NAVIGATION_STATE_AUTO_LOITER: ClassVar[int] = 4
    NAVIGATION_STATE_AUTO_RTL: ClassVar[int] = 5
    NAVIGATION_STATE_POSITION_SLOW: ClassVar[int] = 6
    NAVIGATION_STATE_FREE5: ClassVar[int] = 7
    NAVIGATION_STATE_FREE4: ClassVar[int] = 8
    NAVIGATION_STATE_FREE3: ClassVar[int] = 9
    NAVIGATION_STATE_ACRO: ClassVar[int] = 10
    NAVIGATION_STATE_FREE2: ClassVar[int] = 11
    NAVIGATION_STATE_DESCEND: ClassVar[int] = 12
    NAVIGATION_STATE_TERMINATION: ClassVar[int] = 13
    NAVIGATION_STATE_OFFBOARD: ClassVar[int] = 14
    NAVIGATION_STATE_STAB: ClassVar[int] = 15
    NAVIGATION_STATE_FREE1: ClassVar[int] = 16
    NAVIGATION_STATE_AUTO_TAKEOFF: ClassVar[int] = 17
    NAVIGATION_STATE_AUTO_LAND: ClassVar[int] = 18
    NAVIGATION_STATE_AUTO_FOLLOW_TARGET: ClassVar[int] = 19
    NAVIGATION_STATE_AUTO_PRECLAND: ClassVar[int] = 20
    NAVIGATION_STATE_ORBIT: ClassVar[int] = 21
    NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: ClassVar[int] = 22
    NAVIGATION_STATE_EXTERNAL1: ClassVar[int] = 23
    NAVIGATION_STATE_EXTERNAL2: ClassVar[int] = 24
    NAVIGATION_STATE_EXTERNAL3: ClassVar[int] = 25
    NAVIGATION_STATE_EXTERNAL4: ClassVar[int] = 26
    NAVIGATION_STATE_EXTERNAL5: ClassVar[int] = 27
    NAVIGATION_STATE_EXTERNAL6: ClassVar[int] = 28
    NAVIGATION_STATE_EXTERNAL7: ClassVar[int] = 29
    NAVIGATION_STATE_EXTERNAL8: ClassVar[int] = 30
    NAVIGATION_STATE_MAX: ClassVar[int] = 31
    executor_in_charge: int
    valid_nav_states_mask: int
    can_set_nav_states_mask: int
    failure_detector_status: int
    FAILURE_NONE: ClassVar[int] = 0
    FAILURE_ROLL: ClassVar[int] = 1
    FAILURE_PITCH: ClassVar[int] = 2
    FAILURE_ALT: ClassVar[int] = 4
    FAILURE_EXT: ClassVar[int] = 8
    FAILURE_ARM_ESC: ClassVar[int] = 16
    FAILURE_BATTERY: ClassVar[int] = 32
    FAILURE_IMBALANCED_PROP: ClassVar[int] = 64
    FAILURE_MOTOR: ClassVar[int] = 128
    hil_state: int
    HIL_STATE_OFF: ClassVar[int] = 0
    HIL_STATE_ON: ClassVar[int] = 1
    vehicle_type: int
    VEHICLE_TYPE_ROTARY_WING: ClassVar[int] = 1
    VEHICLE_TYPE_FIXED_WING: ClassVar[int] = 2
    VEHICLE_TYPE_ROVER: ClassVar[int] = 3
    FAILSAFE_DEFER_STATE_DISABLED: ClassVar[int] = 0
    FAILSAFE_DEFER_STATE_ENABLED: ClassVar[int] = 1
    FAILSAFE_DEFER_STATE_WOULD_FAILSAFE: ClassVar[int] = 2
    failsafe: bool
    failsafe_and_user_took_over: bool
    failsafe_defer_state: int
    gcs_connection_lost: bool
    gcs_connection_lost_counter: int
    high_latency_data_link_lost: bool
    is_vtol: bool
    is_vtol_tailsitter: bool
    in_transition_mode: bool
    in_transition_to_fw: bool
    system_type: int
    system_id: int
    component_id: int
    safety_button_available: bool
    safety_off: bool
    power_input_valid: bool
    usb_connected: bool
    open_drone_id_system_present: bool
    open_drone_id_system_healthy: bool
    parachute_system_present: bool
    parachute_system_healthy: bool
    rc_calibration_in_progress: bool
    calibration_enabled: bool
    pre_flight_checks_pass: bool

