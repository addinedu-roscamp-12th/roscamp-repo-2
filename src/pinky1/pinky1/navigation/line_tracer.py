# navigation/line_tracer.py
# IR 센서 기반 라인트레이싱 + 초음파 센서 주차 + 180도 회전

import math
import enum
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pinky1.utils.logger import RobotLogger


class _State(enum.Enum):
    IDLE           = 0  # 대기 (Nav2 제어 중)
    LINE_FOLLOWING = 1  # 라인 추종 중
    ROTATING       = 2  # 180도 회전 중
    PARKED         = 3  # 주차 완료


class LineTracer:
    """
    IR 센서 3개로 흰 라인을 감지하고 추종.
    초음파 센서로 장애물 감지 시 정지 → 180도 회전 → 주차 완료.

    SensorManager.ir_range  — UInt16MultiArray [left, center, right]
    SensorManager.us_range  — Range (초음파 거리)
    """

    # 제어 루프 주기 (초) — 타이머와 D항 계산에 공통 사용
    _control_dt = 0.05

    def __init__(self, node: Node, sensors, nav,
                 line_threshold: int   = 2500,
                 linear_speed:   float = 0.06,
                 angular_gain:   float = 0.8,
                 angular_d_gain: float = 0.0,
                 stop_distance:  float = 0.055,
                 rotate_speed:   float = 0.5):

        self.node    = node
        self.sensors = sensors
        self.nav     = nav
        self.log     = RobotLogger(node)

        # ── 튜닝 파라미터 ──────────────────────────
        self.line_threshold = line_threshold
        self.linear_speed   = linear_speed
        self.angular_gain   = angular_gain
        self.angular_d_gain = angular_d_gain
        self.stop_distance  = stop_distance
        self.rotate_speed   = rotate_speed

        # ── 상태 변수 ──────────────────────────────
        self._state      = _State.IDLE
        self._timer      = None
        self._prev_error = 0.0  # 직전 오차 (D항 계산용)

        # ── 회전 타이밍 ────────────────────────────
        self._rotate_start    = None
        self._rotate_duration = math.pi / rotate_speed  # 180도 회전 시간(초)

        # ── 퍼블리셔 · 콜백 ────────────────────────
        ns = getattr(node, 'ns', 'pinky1')
        self._cmd_pub = node.create_publisher(Twist, f"/{ns}/cmd_vel", 10)

        self.on_parked: callable = None  # 주차 완료 콜백

    # ── 외부 인터페이스 ────────────────────────────────────────
    def start(self):
        """라인트레이싱 시작"""
        if self._state == _State.LINE_FOLLOWING:
            return
        self._cancel_timer()
        self._state        = _State.LINE_FOLLOWING
        self._rotate_start = None
        self._prev_error   = 0.0  # D항 누적 방지를 위해 리셋
        self.nav.cancel_navigation()
        self._timer = self.node.create_timer(self._control_dt, self._loop)  # 20Hz
        self.log.info("LINE", "라인트레이싱 시작")

    def stop(self):
        """라인트레이싱 중단"""
        self._cancel_timer()
        self._publish_stop()
        self._state = _State.IDLE
        self.log.info("LINE", "라인트레이싱 중단")

    @property
    def is_active(self):
        return self._state == _State.LINE_FOLLOWING

    @property
    def is_parked(self):
        return self._state == _State.PARKED

    # ── 내부 루프 ──────────────────────────────────────────────
    def _loop(self):
        if self._state == _State.ROTATING:
            self._do_rotate()
            return

        if self._state != _State.LINE_FOLLOWING:
            return

        # 초음파 정지 조건 (연속 감지 debounce)
        us = self.sensors.us_distance
        if us is not None and us <= self.stop_distance:
            self._publish_stop()
            self._state        = _State.ROTATING
            self._rotate_start = self.node.get_clock().now()
            self.log.info("LINE", f"장애물 감지 → 180도 회전 시작 (거리: {us:.2f}m)")
            return

        # IR 센서 읽기
        ir = self.sensors.ir_range
        if ir is None or len(ir.data) < 3:
            return

        self._follow(ir.data[0], ir.data[1], ir.data[2])

    def _do_rotate(self):
        """180도 회전 처리"""
        elapsed = (self.node.get_clock().now() - self._rotate_start).nanoseconds / 1e9
        if elapsed >= self._rotate_duration:
            self._publish_stop()
            self._cancel_timer()
            self._state = _State.PARKED
            self.log.info("LINE", "180도 회전 완료 → 주차 완료")
            if self.on_parked:
                self.on_parked()
        else:
            twist = Twist()
            twist.angular.z = self.rotate_speed
            self._cmd_pub.publish(twist)

    def _follow(self, left: int, center: int, right: int):
        thr = self.line_threshold

        # 흰선 위 = 낮은 값, 바닥 = 높은 값 → threshold보다 낮을수록 라인
        w_left   = max(0, thr - left)
        w_center = max(0, thr - center)
        w_right  = max(0, thr - right)
        total    = w_left + w_center + w_right

        twist = Twist()

        if total == 0:
            # 라인 잃음 → 저속 직진 탐색
            twist.linear.x  = self.linear_speed * 0.3
            twist.angular.z = 0.0
            self._prev_error = 0.0  # 라인 복귀 시 D항 튐 방지
            self.log.warn("LINE", "라인 미감지 — 탐색 중")
        else:
            # 가중 평균 오차: -1(왼쪽) ~ 0(정중앙) ~ 1(오른쪽)
            error   = (-1.0 * w_left + 0.0 * w_center + 1.0 * w_right) / total
            d_error = (error - self._prev_error) / self._control_dt  # 오차 변화율
            twist.linear.x   = self.linear_speed * (1.0 - 0.5 * abs(error))
            twist.angular.z  = -(error * self.angular_gain + d_error * self.angular_d_gain)
            self._prev_error = error

        self._cmd_pub.publish(twist)

    # ── 헬퍼 ───────────────────────────────────────────────────
    def _publish_stop(self):
        self._cmd_pub.publish(Twist())

    def _cancel_timer(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None
