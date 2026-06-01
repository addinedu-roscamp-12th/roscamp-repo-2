# sensors/sensor_manager.py
# 로봇이 제공하는 모든 센서 토픽 수신

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import (
    Image, LaserScan, Imu,
    BatteryState, CameraInfo, Range
)
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16MultiArray

from config.settings import ROBOT_INTERFACES
from utils.logger import RobotLogger

try:
    from pinky_interfaces.msg import (
        RobotStatus, MissionStatus, PersonDetected)
    _PINKY_AVAILABLE = True
except ImportError:
    _PINKY_AVAILABLE = False


class SensorManager:
    """
    로봇 제공 토픽 전체 구독
    ──────────────────────────────
    /cmd_vel          sub  이동 속도 명령
    /odom             pub  현재 위치/속도
    /scan             pub  라이다 거리 데이터
    /camera/image_raw pub  카메라 이미지
    /camera/camera_info pub 카메라 파라미터
    /imu_raw          pub  IMU 가속도
    /batt_state       pub  배터리 잔량
    /us_sensor/range  pub  초음파 센서
    /ir_sensor/range  pub  적외선 센서
    /robot_status     pub  로봇 전체 상태
    /mission_status   pub  미션 진행 상태
    /person_detected  pub  사람 감지 결과
    """

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        self.ns   = f"/{namespace}" if namespace else ""
        self.log  = RobotLogger(node)

        # ── 최신 데이터 저장 ───────────────────────
        self.scan           = None
        self.odom           = None
        self.image          = None
        self.camera_info    = None
        self.imu            = None
        self.battery        = None
        self.us_range       = None
        self.ir_range       = None
        self.robot_status   = None
        self.mission_status = None
        self.person_detected= None

        # 외부 콜백 (필요시 등록)
        self.on_image          = None
        self.on_person_detected= None
        self.on_robot_status   = None

        self._setup_subscribers()

    def _setup_subscribers(self):
        ns = self.ns
        sub = self.node.create_subscription

        # 기본 센서
        sub(LaserScan,        f"{ns}/scan",
            self._cb_scan,   10)
        sub(Odometry,         f"{ns}/odom",
            self._cb_odom,   10)
        sub(Image,            f"{ns}/camera/image_raw",
            self._cb_image,  10)
        sub(CameraInfo,       f"{ns}/camera/camera_info",
            self._cb_cam_info, 10)
        sub(Imu,              f"{ns}/imu_raw",
            self._cb_imu,    10)
        sub(BatteryState,     f"{ns}/batt_state",
            self._cb_battery, 10)
        sub(Range,            f"{ns}/us_sensor/range",
            self._cb_us,     10)
        sub(UInt16MultiArray, f"{ns}/ir_sensor/range",
            self._cb_ir,     10)

        # 로봇 상태 (pinky_interfaces)
        if _PINKY_AVAILABLE:
            sub(RobotStatus,    f"{ns}/robot_status",
                self._cb_robot_status,    10)
            sub(MissionStatus,  f"{ns}/mission_status",
                self._cb_mission_status,  10)
            sub(PersonDetected, f"{ns}/person_detected",
                self._cb_person_detected, 10)
        else:
            self.log.warn("SENSOR",
                "pinky_interfaces 없음 — 상태 토픽 비활성화")

        self.log.info("SENSOR",
            f"센서 구독 완료 (ns={self.ns})")

    # ── 콜백 ───────────────────────────────────────
    def _cb_scan(self, msg):
        self.scan = msg

    def _cb_odom(self, msg):
        self.odom = msg

    def _cb_image(self, msg):
        if self.image is None:
            self.log.info("SENSOR", "카메라 이미지 첫 수신!")
        self.image = msg
        if self.on_image:
            self.on_image(msg)

    def _cb_cam_info(self, msg):
        self.camera_info = msg

    def _cb_imu(self, msg):
        self.imu = msg

    def _cb_battery(self, msg):
        self.battery = msg
        pct = msg.percentage * 100
        if pct < 20:
            self.log.warn("SENSOR",
                f"배터리 부족! {pct:.0f}%")

    def _cb_us(self, msg):
        self.us_range = msg

    def _cb_ir(self, msg):
        self.ir_range = msg

    def _cb_robot_status(self, msg):
        self.robot_status = msg
        if self.on_robot_status:
            self.on_robot_status(msg)

    def _cb_mission_status(self, msg):
        self.mission_status = msg

    def _cb_person_detected(self, msg):
        self.person_detected = msg
        if self.on_person_detected:
            self.on_person_detected(msg)

    # ── 편의 프로퍼티 ──────────────────────────────
    @property
    def position(self):
        """현재 위치 반환 {x, y, z}"""
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return {"x": round(p.x, 3),
                "y": round(p.y, 3),
                "z": round(p.z, 3)}

    @property
    def battery_pct(self):
        """배터리 잔량 (%)"""
        if self.battery is None:
            return None
        return round(self.battery.percentage * 100, 1)

    @property
    def camera_matrix(self):
        """카메라 내부 파라미터 행렬 (3x3)"""
        if self.camera_info is None:
            return None
        return np.array(self.camera_info.k).reshape(3, 3)

    @property
    def dist_coeffs(self):
        """카메라 왜곡 계수"""
        if self.camera_info is None:
            return None
        return np.array(self.camera_info.d)

    @property
    def us_distance(self):
        """초음파 센서 거리 (m)"""
        if self.us_range is None:
            return None
        return round(self.us_range.range, 3)
