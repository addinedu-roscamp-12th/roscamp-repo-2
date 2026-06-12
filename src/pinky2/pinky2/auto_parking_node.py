#!/usr/bin/env python3
"""
시나리오: go_home → IR 흰선 주차 → 초음파 정밀 주차 → 180도 회전
1. Nav2로 목표 좌표 이동
2. IR 센서로 흰선 감지 → 정지
3. 초음파로 정밀 주차 → 정지
4. 오도메트리 기반 180도 회전 → 종료
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import UInt16MultiArray, Bool
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

ODOM_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

GOAL_YAW_DEG = 175.3  # go_home 도착 목표 방향 (흰선과 정렬되는 각도, AMCL 측정값)

WHITE_THRESHOLD = 2000
LINEAR_VEL  = 0.05
ANGULAR_VEL = 0.5   # 회전 속도 [rad/s]
US_STOP_DIST = 0.065  # 초음파 정지 거리 [m]

LINE_KP = -0.0005          # 라인트레이싱 비례 게인 (좌우 IR raw 차이 → 각속도)
LINE_ANGULAR_LIMIT = 0.3  # 라인트레이싱 최대 각속도 [rad/s]

LEFT  = 0
MID   = 1
RIGHT = 2


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AutoParkingNode(Node):
    def __init__(self):
        super().__init__('auto_parking_node')

        self.declare_parameter('goal_x', -0.004)
        self.declare_parameter('goal_y', 0.018)
        self.declare_parameter('white_threshold', WHITE_THRESHOLD)
        self.declare_parameter('us_stop_dist', US_STOP_DIST)
        self.declare_parameter('line_kp', LINE_KP)
        self.declare_parameter('line_angular_limit', LINE_ANGULAR_LIMIT)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.white_threshold = self.get_parameter('white_threshold').value
        self.us_stop_dist = self.get_parameter('us_stop_dist').value
        self.line_kp = self.get_parameter('line_kp').value
        self.line_angular_limit = self.get_parameter('line_angular_limit').value

        self.started    = False
        self.ir_active  = False
        self.us_active  = False
        self.rotating   = False
        self.done       = False

        self.last_yaw        = None
        self.accumulated_yaw = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'pinky2/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, 'pinky2/parking/auto_done', 10)
        self._nav_client = ActionClient(self, NavigateToPose, 'pinky2/navigate_to_pose')

        self.create_subscription(Odometry, '/pinky2/odom', self._odom_callback, ODOM_QOS)
        self.create_subscription(UInt16MultiArray, 'pinky2/ir_sensor/range', self._ir_callback, 10)
        self.create_subscription(Range, 'pinky2/us_sensor/range', self._us_callback, 10)
        self.create_subscription(Bool, 'pinky2/parking/auto_start', self._start_callback, 10)

        self.get_logger().info(
            f'AutoParkingNode 시작. 목표=({self.goal_x}, {self.goal_y}), '
            f'white_threshold={self.white_threshold}, us_stop_dist={self.us_stop_dist}m, '
            f'line_kp={self.line_kp}, line_angular_limit={self.line_angular_limit}'
        )
        ###self._go_home() 단순테스트용
        
    def _start_callback(self, msg: Bool):
        if msg.data and not self.started:
            self.started = True
            self.get_logger().info('주차 진입 시작 신호 수신.')
            self._go_home()

    def _finish(self, success: bool):
        self.status_pub.publish(Bool(data=success))
        time.sleep(0.2)  # 메시지 전송 여유
        self.done = True
        rclpy.shutdown()

    # ── Phase 1: 네비게이션 ──────────────────────────────────────

    def _go_home(self):
        self.get_logger().info('Nav2 서버 대기 중...')
        self._nav_client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y

        self.get_logger().info(f'목표 전송: ({self.goal_x}, {self.goal_y}), yaw={GOAL_YAW_DEG}°')
        future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=self._nav_feedback,
        )
        future.add_done_callback(self._nav_goal_response)

    def _nav_feedback(self, feedback_msg: NavigateToPose.Feedback):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'이동 중... 남은 거리: {dist:.2f} m', throttle_duration_sec=1.0)

    def _nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('목표 거부됨. 종료.')
            self._finish(success=False)
            return
        self.get_logger().info('목표 수락됨.')
        handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info('도착. IR 주차 모드 시작.')
            self._start_ir_parking()
        else:
            self.get_logger().error(f'네비게이션 실패 (status={status}). 종료.')
            self._finish(success=False)

    # ── Phase 2: IR 흰선 감지 ────────────────────────────────────

    def _start_ir_parking(self):
        self.ir_active = True

    def _ir_callback(self, msg: UInt16MultiArray):
        if not self.ir_active:
            return

        data = msg.data
        left_raw, mid_raw, right_raw = data[LEFT], data[MID], data[RIGHT]

        if mid_raw <= self.white_threshold:
            self.cmd_pub.publish(Twist())
            self.ir_active = False
            self.get_logger().info(
                f'흰선 감지 완료. 초음파 정밀 주차 시작. (L={left_raw}, M={mid_raw}, R={right_raw})'
            )
            self._start_us_parking()
            return

        # 좌우 raw 값 차이를 비례 오차로 사용해 직진하며 조향 보정
        error = float(left_raw - right_raw)
        angular_z = self.line_kp * error
        angular_z = max(-self.line_angular_limit, min(self.line_angular_limit, angular_z))

        self.get_logger().info(
            f'라인트레이싱: L={left_raw}, M={mid_raw}, R={right_raw}, '
            f'error={error:.0f}, angular_z={angular_z:.3f}',
            throttle_duration_sec=0.5,
        )

        twist = Twist()
        twist.linear.x = LINEAR_VEL
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    # ── Phase 3: 초음파 정밀 주차 ────────────────────────────────

    def _start_us_parking(self):
        self.us_active = True

    def _us_callback(self, msg: Range):
        if not self.us_active:
            return

        dist = msg.range
        self.get_logger().info(f'초음파 거리: {dist:.3f} m', throttle_duration_sec=1.0)

        if dist <= self.us_stop_dist:
            self.cmd_pub.publish(Twist())
            self.us_active = False
            self.get_logger().info(f'정밀 주차 완료 — 장애물까지 {dist:.3f} m. 180도 회전 시작.')
            self._start_rotation()
            return

        twist = Twist()
        twist.linear.x = LINEAR_VEL
        self.cmd_pub.publish(twist)

    # ── Phase 4: 180도 회전 ──────────────────────────────────────

    def _start_rotation(self):
        self.rotating = True
        self.last_yaw = None
        self.accumulated_yaw = 0.0

    def _odom_callback(self, msg: Odometry):
        if not self.rotating or self.done:
            return

        current_yaw = quat_to_yaw(msg.pose.pose.orientation)

        if self.last_yaw is None:
            self.last_yaw = current_yaw
            self.get_logger().info(f'회전 시작 yaw: {math.degrees(current_yaw):.1f}°')
            return

        # 한 스텝 각도 변화 (작은 값이므로 wrapping 안전)
        delta = current_yaw - self.last_yaw
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi

        self.accumulated_yaw += delta
        self.last_yaw = current_yaw

        self.get_logger().info(
            f'회전 누적: {math.degrees(self.accumulated_yaw):.1f}°', throttle_duration_sec=0.5
        )

        if abs(self.accumulated_yaw) >= math.pi:  # 180도 달성
            self.cmd_pub.publish(Twist())
            self.rotating = False
            self.get_logger().info('180도 회전 완료. 시나리오 종료.')
            self._finish(success=True)
            return

        twist = Twist()
        twist.angular.z = ANGULAR_VEL
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoParkingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
