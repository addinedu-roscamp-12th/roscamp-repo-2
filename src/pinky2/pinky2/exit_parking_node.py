#!/usr/bin/env python3
"""
탈출 시나리오: amcl_pose로 탈출 좌표까지 전진 → 360도 회전(로컬라이제이션) → Nav2로 목적지 이동
1. 타이머로 cmd_vel 지속 발행, amcl_pose로 위치 체크
2. 탈출 좌표 도달 → 정지
3. 오도메트리 기반 360도 회전 (AMCL 위치 보정)
4. Nav2로 최종 목적지 이동 (현재 미사용)
"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

LINEAR_VEL     = 0.05
ANGULAR_VEL    = 0.5
EXIT_TOLERANCE = 0.10  # 탈출 좌표 도달 판정 반경 [m]

AMCL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ODOM_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ExitParkingNode(Node):
    def __init__(self):
        super().__init__('exit_parking_node')

        self.declare_parameter('exit_x', -0.18112303207757627)
        self.declare_parameter('exit_y', -0.035069745872950836)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('exit_tolerance', EXIT_TOLERANCE)

        self.exit_x        = self.get_parameter('exit_x').value
        self.exit_y        = self.get_parameter('exit_y').value
        self.goal_x        = self.get_parameter('goal_x').value
        self.goal_y        = self.get_parameter('goal_y').value
        self.exit_tolerance = self.get_parameter('exit_tolerance').value

        self.current_x   = None
        self.current_y   = None
        self.current_yaw = None
        self.exiting     = False
        self.rotating    = False
        self.done        = False

        self.last_yaw        = None
        self.accumulated_yaw = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'pinky2/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, 'pinky2/parking/exit_done', 10)
        self._nav_client = ActionClient(self, NavigateToPose, 'pinky2/navigate_to_pose')

        self.create_subscription(
            PoseWithCovarianceStamped, '/pinky2/amcl_pose', self._pose_callback, AMCL_QOS
        )
        self.create_subscription(Odometry, '/pinky2/odom', self._odom_callback, ODOM_QOS)
        self.create_subscription(Bool, 'pinky2/parking/exit_start', self._start_callback, 10)

        # 10Hz로 cmd_vel 발행
        self.create_timer(0.1, self._timer_callback)

        self.get_logger().info(
            f'ExitParkingNode 시작. 탈출 좌표=({self.exit_x:.3f}, {self.exit_y:.3f})'
        )
        # self.exiting = True 단순테스트용

    def _start_callback(self, msg: Bool):
        if msg.data and not self.exiting:
            self.exiting = True
            self.get_logger().info('탈출 시작 신호 수신. 탈출 시퀀스 시작.')

    def _finish(self, success: bool):
        self.done_pub.publish(Bool(data=success))
        time.sleep(0.2)  # 메시지 전송 여유
        self.done = True
        rclpy.shutdown()

    # ── amcl_pose 수신 → 현재 위치 갱신 ────────────────────────

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    # ── 타이머: 10Hz로 cmd_vel 발행 ─────────────────────────────

    def _timer_callback(self):
        if not self.exiting or self.done:
            return
        if self.current_x is None:
            return

        dist = math.sqrt(
            (self.current_x - self.exit_x) ** 2 +
            (self.current_y - self.exit_y) ** 2
        )

        self.get_logger().info(
            f'현재=({self.current_x:.3f}, {self.current_y:.3f}) 탈출좌표까지 {dist:.3f} m',
            throttle_duration_sec=0.5,
        )

        if dist <= self.exit_tolerance:
            self.cmd_pub.publish(Twist())
            self.exiting = False
            self.get_logger().info('탈출 완료. 로컬라이제이션을 위해 360도 회전 시작.')
            self._start_rotation()
            return

        target_angle = math.atan2(
            self.exit_y - self.current_y,
            self.exit_x - self.current_x,
        )
        angle_err = target_angle - self.current_yaw
        if angle_err > math.pi:
            angle_err -= 2 * math.pi
        elif angle_err < -math.pi:
            angle_err += 2 * math.pi

        angular = max(-ANGULAR_VEL, min(ANGULAR_VEL, ANGULAR_VEL * angle_err))

        twist = Twist()
        twist.linear.x  = LINEAR_VEL
        twist.angular.z = angular
        self.cmd_pub.publish(twist)

    # ── Phase 2: 로컬라이제이션을 위한 360도 회전 ────────────────

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

        if abs(self.accumulated_yaw) >= 2 * math.pi:  # 360도 달성
            self.cmd_pub.publish(Twist())
            self.rotating = False
            self.get_logger().info('360도 회전 완료. 시나리오 종료.')
            self._finish(success=True)
            return

        twist = Twist()
        twist.angular.z = ANGULAR_VEL
        self.cmd_pub.publish(twist)

    # ── Phase 3: Nav2로 최종 목적지 (현재 미사용) ────────────────

    def _go_to_goal(self):
        self.get_logger().info('Nav2 서버 대기 중...')
        self._nav_client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        # 현재 방향 유지 (Nav2가 도착 후 강제 회전하지 않도록)
        half = self.current_yaw / 2.0
        goal.pose.pose.orientation.z = math.sin(half)
        goal.pose.pose.orientation.w = math.cos(half)

        self.get_logger().info(f'목적지 전송: ({self.goal_x}, {self.goal_y})')
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback
        )
        future.add_done_callback(self._nav_goal_response)

    def _nav_feedback(self, feedback_msg: NavigateToPose.Feedback):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'이동 중... 남은 거리: {dist:.2f} m', throttle_duration_sec=1.0)

    def _nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('목표 거부됨. 종료.')
            rclpy.shutdown()
            return
        self.get_logger().info('목표 수락됨.')
        handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('목적지 도착. 시나리오 종료.')
        else:
            self.get_logger().error(f'네비게이션 실패 (status={status}).')
        self.done = True
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ExitParkingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
