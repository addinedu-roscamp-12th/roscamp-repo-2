# core/visual_dock.py
# ArUco 마커 픽셀 좌표 기반 시각 서보 정밀 정차

import time

from geometry_msgs.msg import Twist
from utils.logger import RobotLogger


IMAGE_WIDTH      = 640   # 카메라 해상도 가로
CENTER_TOLERANCE = 20    # px — 좌우 허용 오차
PERP_TOLERANCE   = 8     # px — 좌우 높이 차 허용 오차 (정면 판단)
TARGET_SIZE      = 220   # px — 이 크기 이상이면 도착으로 판단
MIN_DOCK_SIZE    = 80    # px — 이 크기 이상일 때만 시각 서보 시작
ANG_SPEED        = 0.3   # 회전 속도 (rad/s)
LIN_SPEED        = 0.1   # 전진 속도 (m/s)


class VisualDock:
    """
    마커가 보이는 동안 매 프레임 update() 호출
    1단계: 마커 좌우 정렬 (각도 제어)
    2단계: 정렬 완료 후 전진h
    3단계: 마커 픽셀 크기 목표치 도달 시 정지
    """

    def __init__(self, node):
        self._pub   = node.create_publisher(Twist, "/cmd_vel", 10)
        self.log    = RobotLogger(node)
        self.active = False
        self._done_cb   = None
        self._state     = None  # "align" | "forward" | None
        self._last_log  = 0.0

    def start(self, done_callback=None):
        self.active   = True
        self._done_cb = done_callback
        self._state   = None

    def stop(self):
        self.active = False
        self._send(0.0, 0.0)

    def update(self, marker: dict) -> bool:
        if not self.active:
            return False

        cx         = marker["cx"]
        pixel_size = marker["pixel_size"]
        error      = cx - (IMAGE_WIDTH // 2)

        if pixel_size >= TARGET_SIZE:
            self._send(0.0, 0.0)
            self.active = False
            self._state = None
            self.log.info("DOCK", "정밀 정차 완료")
            if self._done_cb:
                self._done_cb()
            return True

        # 좌우 높이 차로 정면 각도 계산
        corners   = marker.get("corners")
        perp_err  = 0
        if corners:
            left_h  = abs(corners[3][1] - corners[0][1])  # 왼쪽 변 높이
            right_h = abs(corners[2][1] - corners[1][1])  # 오른쪽 변 높이
            perp_err = left_h - right_h  # 양수=왼쪽 치우침, 음수=오른쪽 치우침

        # 중앙 정렬 + 정면 정렬 합산
        cx_err   = -(error / (IMAGE_WIDTH // 2)) * ANG_SPEED
        perp_ang = -(perp_err / 30.0) * ANG_SPEED * 0.5
        angular  = cx_err + perp_ang

        # 중앙 정렬 + 정면 정렬 둘 다 됐을 때만 전진
        centered = abs(error) <= CENTER_TOLERANCE * 3
        perpendicular = abs(perp_err) <= PERP_TOLERANCE

        if centered and perpendicular:
            linear = LIN_SPEED
            self._state = "forward"
        elif centered:
            linear = 0.0
            self._state = "perp"  # 정면 정렬 중
        else:
            linear = 0.0
            self._state = "align"

        now = time.time()
        if now - self._last_log >= 1.0:
            self._last_log = now
            self.log.info("DOCK",
                f"{self._state} | pixel={pixel_size}/{TARGET_SIZE} cx_err={error:+d} perp_err={perp_err:+.1f}")

        self._send(linear, angular)
        return False

    def _send(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._pub.publish(msg)
