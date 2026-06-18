# core/visual_dock.py
# ArUco 마커 픽셀 좌표 기반 시각 서보 정밀 정차

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pinky1.utils.logger import RobotLogger
from tf2_ros import Buffer, TransformListener


# ════════════════════════════════════════════════════════════
# TUNING PARAMETERS
# ════════════════════════════════════════════════════════════
IMAGE_WIDTH      = 640   # 카메라 해상도 가로
CENTER_TOLERANCE = 20    # px — 좌우 허용 오차
MIN_DOCK_SIZE    = 0     # px — 크기 제한 없음 (감지 즉시 시작)
ANG_SPEED        = 0.3   # rad/s — 정렬 회전 속도
LIN_SPEED        = 0.05  # m/s  — 전진 속도
US_STOP_DIST     = 0.065 # m    — 초음파 정지 거리 (6.5cm)
YAW_TARGET       = 0.0   # rad  — 정지 후 정렬할 map 기준 yaw
YAW_TOLERANCE    = math.radians(2.0)  # 2도 허용 오차
YAW_SPEED        = 0.3   # rad/s — yaw 정렬 회전 속도 (왼쪽 고정)


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d


class VisualDock:
    """
    1단계 (align):   마커 좌우 중앙 + 정면 각도 정렬
    2단계 (forward): 정렬 완료 후 전진
    3단계 (us_stop): 초음파 ≤ 6.5cm 도달 시 정지
    4단계 (yaw):     map 기준 yaw=0 방향으로 왼쪽 회전 후 완료
    """

    def __init__(self, node, sensors=None, tf_buf=None):
        ns = getattr(node, 'ns', 'pinky1')
        self._node    = node
        self._sensors = sensors
        self._pub     = node.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        self.log      = RobotLogger(node)
        self.active      = False
        self._done_cb    = None
        self._state      = None   # "align" | "perp" | "forward" | "yaw" | None
        self._last_log   = 0.0
        self._start_time = 0.0
        self._wait_timer = None
        self._us_timer   = None   # 전진 중 독립 US 모니터링 타이머

        # map 기준 yaw 읽기용 TF
        if tf_buf is not None:
            self._tf_buf = tf_buf
        else:
            self._tf_buf      = Buffer()
            self._tf_listener = TransformListener(self._tf_buf, node)
        self._odom_yaw    = None
        node.create_subscription(
            Odometry, f"/{ns}/odom", self._cb_odom, 10)

        # yaw 정렬 타이머 (4단계 전용)
        self._yaw_timer = None

    MARKER_WAIT_TIMEOUT = 10.0  # 마커 미감지 시 타임아웃 (초)

    # ── 외부 인터페이스 ────────────────────────────────────────
    def start(self, done_callback=None, target_yaw=None):
        self.active      = True
        self._done_cb    = done_callback
        self._yaw_target = target_yaw if target_yaw is not None else YAW_TARGET
        self._state      = None
        self._start_time = time.time()
        self._wait_timer = self._node.create_timer(1.0, self._wait_timeout_tick)
        self._start_us_monitor()

    def stop(self):
        self.active = False
        self._cancel_yaw_timer()
        self._cancel_wait_timer()
        self._cancel_us_timer()
        self._send(0.0, 0.0)

    # ── odom 콜백 (yaw 폴백용) ─────────────────────────────────
    def _cb_odom(self, msg: Odometry):
        self._odom_yaw = _yaw_from_quat(msg.pose.pose.orientation)

    # ── 제어 루프 (매 ArUco 프레임마다 호출) ──────────────────
    def update(self, marker: dict) -> bool:
        if not self.active or self._state == "yaw":
            return False

        # 첫 마커 감지 — 타임아웃 타이머 해제
        if self._state is None and self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None

        cx         = marker["cx"]
        pixel_size = marker["pixel_size"]
        error      = cx - (IMAGE_WIDTH // 2)

        # 초음파 정지 조건 확인
        us = self._us_distance()
        if us is not None and 0 < us <= US_STOP_DIST:
            self._send(0.0, 0.0)
            self.log.info("DOCK", f"초음파 정지 — {us*100:.1f}cm → yaw 정렬 시작")
            self._start_yaw_align()
            return False

        # 좌우 높이 차로 정면 각도 계산
        corners  = marker.get("corners")
        perp_err = 0
        if corners:
            left_h   = abs(corners[3][1] - corners[0][1])
            right_h  = abs(corners[2][1] - corners[1][1])
            perp_err = left_h - right_h

        cx_err_ang = -(error / (IMAGE_WIDTH // 2)) * ANG_SPEED
        perp_ang   = -(perp_err / 30.0) * ANG_SPEED * 0.5

        centered = abs(error) <= CENTER_TOLERANCE * 3

        if centered:
            # 중앙 정렬 완료 → 전진 (perp 보정도 같이 적용)
            linear  = LIN_SPEED
            angular = cx_err_ang + perp_ang
            self._state = "forward"
        else:
            # 중앙 미정렬 → 제자리 회전 (perp 보정 제외 — 상쇄 방지)
            linear  = 0.0
            angular = cx_err_ang
            self._state = "align"

        now = time.time()
        if now - self._last_log >= 1.0:
            self._last_log = now
            us_str = f"{us*100:.1f}cm" if us is not None else "N/A"
            self.log.info("DOCK",
                f"{self._state} | us={us_str} cx_err={error:+d} perp_err={perp_err:+.1f}")

        self._send(linear, angular)
        return False

    # ── 4단계: yaw 정렬 (왼쪽 방향 고정) ─────────────────────
    def _start_yaw_align(self):
        self._state = "yaw"
        if self._yaw_timer is None:
            self._yaw_timer = self._node.create_timer(0.1, self._yaw_tick)

    def _yaw_tick(self):
        yaw = self._map_yaw()
        if yaw is None:
            self.log.warn("DOCK", "yaw 값 없음 (TF/odom 미수신) — 대기 중")
            return

        remaining = _angle_diff(self._yaw_target, yaw)
        now = time.time()
        if now - self._last_log >= 0.5:
            self._last_log = now
            self.log.info("DOCK",
                f"yaw 정렬 | 현재={math.degrees(yaw):+.1f}° 목표={math.degrees(self._yaw_target):+.1f}° 남은={math.degrees(remaining):+.1f}°")

        if abs(remaining) <= YAW_TOLERANCE:
            self._send(0.0, 0.0)
            self._cancel_yaw_timer()
            self.active = False
            self._state = None
            self.log.info("DOCK", "yaw 정렬 완료 → 도킹 완료")
            if self._done_cb:
                self._done_cb(success=True)
            return

        # 왼쪽(반시계) 고정 — angular.z 양수
        speed = max(0.15, min(YAW_SPEED, abs(remaining) * 0.5))
        self._send(0.0, speed)

    def _cancel_yaw_timer(self):
        if self._yaw_timer is not None:
            self._yaw_timer.cancel()
            self._yaw_timer = None

    def _start_us_monitor(self):
        if self._us_timer is None:
            self._us_timer = self._node.create_timer(0.05, self._us_monitor_tick)

    def _cancel_us_timer(self):
        if self._us_timer is not None:
            self._us_timer.cancel()
            self._us_timer = None

    def _us_monitor_tick(self):
        if not self.active:
            self._cancel_us_timer()
            return
        if self._state != "forward":
            return  # align/yaw 중에는 체크 안 함
        us = self._us_distance()
        if us is not None and 0 < us <= US_STOP_DIST:
            self._send(0.0, 0.0)
            self._cancel_us_timer()
            self.log.info("DOCK", f"초음파 정지 — {us*100:.1f}cm → yaw 정렬 시작")
            self._start_yaw_align()

    def _cancel_wait_timer(self):
        if self._wait_timer is not None:
            self._wait_timer.cancel()
            self._wait_timer = None

    def _wait_timeout_tick(self):
        if not self.active or self._state is not None:
            self._cancel_wait_timer()
            return
        elapsed = time.time() - self._start_time
        if elapsed >= self.MARKER_WAIT_TIMEOUT:
            self.log.warn("DOCK",
                f"마커 {self.MARKER_WAIT_TIMEOUT:.0f}초 미감지 → 도킹 건너뜀")
            self._cancel_wait_timer()
            self._cancel_us_timer()
            self.active = False
            self._state = None
            if self._done_cb:
                # 타임아웃임을 호출자에게 알리기 위해 success=False 전달
                self._done_cb(success=False)

    # ── 헬퍼 ───────────────────────────────────────────────────
    def _us_distance(self) -> float | None:
        if self._sensors is None:
            return None
        return self._sensors.us_distance

    def _map_yaw(self) -> float | None:
        ns = getattr(self._node, 'ns', 'pinky1')
        try:
            t = self._tf_buf.lookup_transform(
                "map", f"{ns}/base_footprint",
                rclpy.time.Time())
            return _yaw_from_quat(t.transform.rotation)
        except Exception:
            return self._odom_yaw

    def _send(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._pub.publish(msg)
