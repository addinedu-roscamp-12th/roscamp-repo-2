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
    "pinky2": {
        "ip":        "192.168.1.97",
        "namespace": "pinky2",
        "name":      "핑키2 (보조)",
    },
}

ROS_DOMAIN_ID = 99


# ════════════════════════════════════════════════════════════
# 좌표 데이터
# ════════════════════════════════════════════════════════════

# ── 장소 좌표 (지도 기준) ──────────────────────────────────────
LOCATIONS = {
    "home":         {"x": 0.0,    "y": 0.0,    "yaw": 0.0},

    # ── 상차 대기 구역 (두 모드 공통) ────────────────
    # TODO: 실제 지도 좌표로 교체
    "load_wait_1":  {"x": 0.0,    "y": 0.0,    "yaw": 0.0},
    "load_wait_2":  {"x": 0.0,    "y": 0.0,    "yaw": 0.0},

    # ── 렉 구역 (waypoint 모드 전용) ─────────────────
    # TODO: 실제 지도 좌표로 교체
    "rack_1":       {"x": 0.0,    "y": 0.0,    "yaw": 0.0},
    "rack_2":       {"x": 0.0,    "y": 0.0,    "yaw": 0.0},
    "rack_3":       {"x": 0.0,    "y": 0.0,    "yaw": 0.0},

    # ── 경로 웨이포인트 (waypoint 모드 경유지) ─────────
    # TODO: 실제 지도 좌표로 교체
    "wp_1":         {"x": 0.0,    "y": 0.0,    "yaw": 0.0},
    "wp_2":         {"x": 0.0,    "y": 0.0,    "yaw": 0.0},
    "wp_3":         {"x": 0.0,    "y": 0.0,    "yaw": 0.0},
    "wp_4":         {"x": 0.0,    "y": 0.0,    "yaw": 0.0},
    "wp_5":         {"x": 0.0,    "y": 0.0,    "yaw": 0.0},
    "wp_6":         {"x": 0.0,    "y": 0.0,    "yaw": 0.0},

    # ── 기존 존 방식 구역 (zone 모드 전용) ───────────
    "loading_zone": {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_1":       {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_2":       {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_3":       {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_4":       {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_5":       {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_6":       {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_7":       {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_8":       {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_9":       {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_10":      {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_11":      {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_12":      {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_13":      {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_14":      {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_15":      {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
    "zone_16":      {"x": 1.6577, "y": 0.2709, "yaw": -2.6691},
}

# ── 웨이포인트 경로 (지도 기준) ───────────────────────
# 이 dict에 등록된 목적지는 NavigateThroughPoses(경유지 순서 이동)를 사용.
# 등록되지 않은 목적지는 NavigateToPose(직접 이동)를 사용.
#
# waypoint 모드 사용 시:
#   · load_wait_1/2 : HOME → 경유지 → load_wait 최종 도착 포즈
#   · rack_1/2/3    : load_wait → 경유지 → rack 최종 도착 포즈
#   마지막 항목이 실제 목적지 포즈가 되어야 함.
#
# TODO: 실제 지도 좌표로 교체
WAYPOINTS = {
    # ── HOME → load_wait_1 경로 ──────────────────────
    "load_wait_1": [
        LOCATIONS["wp_1"],
        LOCATIONS["wp_3"],
        LOCATIONS["load_wait_1"],
    ],
    # ── HOME → load_wait_2 경로 ──────────────────────
    "load_wait_2": [
        LOCATIONS["wp_2"],
        LOCATIONS["wp_4"],
        LOCATIONS["load_wait_2"],
    ],
    # ── load_wait → rack_1 경로 ──────────────────────
    "rack_1": [
        LOCATIONS["wp_5"],
        LOCATIONS["rack_1"],
    ],
    # ── load_wait → rack_2 경로 ──────────────────────
    "rack_2": [
        LOCATIONS["wp_5"],
        LOCATIONS["rack_2"],
    ],
    # ── load_wait → rack_3 경로 ──────────────────────
    "rack_3": [
        LOCATIONS["wp_6"],
        LOCATIONS["rack_3"],
    ],
    # ── 기존 테스트 경로 ──────────────────────────────
    "zone_A": [
        {"x": 0.5069,  "y": 1.1749,  "yaw": 1.6246},
        {"x": 0.3,     "y": 0.8,     "yaw": 1.57},
        {"x": 0.0,     "y": 0.3,     "yaw": 3.14},
        {"x": -0.3,    "y": 0.0,     "yaw": -1.57},
        {"x": -0.6,    "y": -0.5,    "yaw": -1.57},
        {"x": -1.0361, "y": -0.9091, "yaw": 0.5682},
    ],
}

# ════════════════════════════════════════════════════════════
# 모듈 설정
# ════════════════════════════════════════════════════════════

# ── ArUco 설정 ────────────────────────────────────────────────
ARUCO_CONFIG = {
    "dict":              "DICT_4X4_250",
    "marker_size":       0.1,    # 실제 마커 크기 (m)
    "approach_distance": 0.3,    # 마커 앞 정차 거리 (m)
    "marker_map": {              # 마커ID → 장소 매핑
        # 0: "loading_zone",
        # 1: "unloading_zone",
        # 2: "zone_A",
        # 3: "zone_B",
        # 4: "zone_C",
    },
}

# ── YOLO 설정 ──────────────────────────────────────
YOLO_CONFIG = {
    "model":        "yolov8n.pt",
    "confidence":   0.6,
    "enabled":      False,       # 커스텀 모델 준비 후 True로 변경
    "classes":      [],
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
