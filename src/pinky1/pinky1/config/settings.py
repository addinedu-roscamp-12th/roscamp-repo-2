# config/settings.py
# 모든 설정값을 한 곳에서 관리
# 수정할 때 이 파일만 수정하면 됨


# ════════════════════════════════════════════════════════════
# 로봇 설정
# ════════════════════════════════════════════════════════════

# ── 로봇 설정 ──────────────────────────────────────────────────
ROBOT_CONFIG = {
    "pinky1": {
        "ip":        "192.168.1.96",
        "namespace": "pinky1",
        "name":      "핑키1 (적재 담당)",
    },
}

ROS_DOMAIN_ID = 99


# ════════════════════════════════════════════════════════════
# 좌표 데이터
# ════════════════════════════════════════════════════════════

# ── 장소 좌표 (지도 기준) ──────────────────────────────────────
LOCATIONS = {
    "home":         {"x": -0.0092, "y": -0.0233, "yaw": 0.0},

    # ── 상차 대기 구역 (두 모드 공통) ────────────────
    # TODO: 실제 지도 좌표로 교체
    "load_wait_1":  {"x": -1.0442, "y": -0.1919, "yaw": 3.1416},
    "load_wait_2":  {"x": 0.0,    "y": 0.0,    "yaw": 0.0},

    # ── 기존 존 방식 구역 (zone 모드 전용) ───────────
    "outbound_zone": {"x": 0.0503, "y": -0.7542, "yaw": 0.0},
    "zone_1":       {"x": -0.4976, "y": -0.0311, "yaw": 0.0},
    "zone_2":       {"x": -0.5217, "y": -0.4008, "yaw": 0.0},
    "zone_3":       {"x": -0.5128, "y": -0.7697, "yaw": 0.0},
}


# ════════════════════════════════════════════════════════════
# 모듈 설정
# ════════════════════════════════════════════════════════════

# ── ArUco 설정 ────────────────────────────────────────────────
ARUCO_CONFIG = {
    "dict":              "DICT_4X4_250",
    "marker_size":       0.1,    # 실제 마커 크기 (m)
    "marker_map": {              # 마커ID → 장소 매핑
        0: "load_wait_1",
        # 1: "load_wait_2",
    },
}

# ── zone 이동 출발지별 cmd_vel 중간 좌표 ──────────────
# zone_1/2/3 명령 시 이 좌표까지 cmd_vel 전진 후 Nav2 시작
# x,y 는 해당 출발지와 동일, yaw만 변경
ZONE_DEPART_POINTS = {
    "load_wait_1": {"x": LOCATIONS["load_wait_1"]["x"], "y": LOCATIONS["load_wait_1"]["y"], "yaw": 0.0},
    "home":        {"x": LOCATIONS["home"]["x"],        "y": LOCATIONS["home"]["y"],        "yaw": 3.1416},
}

# ── 도착 후 초음파 감지까지 전진할 위치 ──────────────────
# Nav2 도착 후 초음파 ≤ 6.5cm 될 때까지 cmd_vel 전진
ARRIVAL_US_STOP = set()

# ── 도착 후 cmd_vel로 yaw 정렬할 위치 ────────────────────
# {location: target_yaw_rad} — Nav2 도착 후 cmd_vel로 해당 yaw로 회전
ARRIVAL_YAW_ROTATE = {
    "outbound_zone": 3.14,
    "zone_1":        0.02,
    "zone_2":        0.02,
    "zone_3":        0.02,
}

# ── 도착 후 초음파 전진 → yaw 정렬할 위치 ────────────────
# {location: target_yaw} — Nav2 도착 후 us_forward → 해당 yaw로 회전
ARRIVAL_US_FORWARD = {"load_wait_1": 0.0}

# ── YOLO 설정 ──────────────────────────────────────
YOLO_CONFIG = {
    "model":        "yolov8n.pt",
    "confidence":   0.4,
    "enabled":      False,      # 커스텀 모델 준비 후 True로 변경
    "classes":      None,       # None = 전체 클래스 감지
}

# ── Nav2 설정 ──────────────────────────────────────
NAV2_CONFIG = {
    "goal_tolerance": 0.1,       # 목적지 허용 오차 (m)
    "nav_timeout":    60.0,      # 이동 타임아웃 (초)
    "dock_timeout":   30.0,      # 도킹 타임아웃 (초)
}

# ── SLAM 설정 ──────────────────────────────────────
SLAM_CONFIG = {
    "map_frame":     "map",
    "odom_frame":    "odom",
    "base_frame":    "base_footprint",
    "map_save_path": "./maps/my_map",
}


# ════════════════════════════════════════════════════════════
# 메타데이터
# ════════════════════════════════════════════════════════════

# ── 로봇 토픽/서비스/액션 정의 ─────────────────────────────────
# 로봇이 제공하는 인터페이스 목록
ROBOT_INTERFACES = {
    "topics": {
        "sub": {
            "cmd_vel":        "geometry_msgs/Twist",
        },
        "pub": {
            "odom":           "nav_msgs/Odometry",
            "scan":           "sensor_msgs/LaserScan",
            "camera_image":   "sensor_msgs/Image",
            "camera_info":    "sensor_msgs/CameraInfo",
            "imu":            "sensor_msgs/Imu",
            "battery":        "sensor_msgs/BatteryState",
            "us_sensor":      "sensor_msgs/Range",
            "ir_sensor":      "std_msgs/UInt16MultiArray",
            "robot_status":   "pinky_interfaces/RobotStatus",
            "mission_status": "pinky_interfaces/MissionStatus",
            "person_detected":"pinky_interfaces/PersonDetected",
        },
    },
    "services": {
        "set_emotion":    "pinky_interfaces/Emotion",
        "set_led":        "pinky_interfaces/SetLed",
        "set_brightness": "pinky_interfaces/SetBrightness",
        "set_lamp":       "pinky_interfaces/SetLamp",
        "emergency_stop": "pinky_interfaces/EmergencyStop",
    },
    "actions": {
        "navigate_to_pose":       "nav2_msgs/NavigateToPose",
        "navigate_through_poses": "nav2_msgs/NavigateThroughPoses",
        "transport_mission":      "pinky_interfaces/TransportMission",
        "dock_to_marker":         "pinky_interfaces/DockToMarker",
    },
}
