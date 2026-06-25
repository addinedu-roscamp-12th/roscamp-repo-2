#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import UInt16MultiArray, Bool
from sensor_msgs.msg import Range

AMCL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

WHITE_THRESHOLD = 2000
LINEAR_VEL  = 0.05
ANGULAR_VEL = 0.5
ROTATION_YAW_TOLERANCE = math.radians(3)
US_STOP_DIST = 0.065

LINE_KP = -0.0005
LINE_ANGULAR_LIMIT = 0.3

LEFT  = 0
MID   = 1
RIGHT = 2


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AutoParkingNode(Node):
    def __init__(self):
        super().__init__('auto_parking_node')

        #self.declare_parameter('goal_x', 0.001)
        #self.declare_parameter('goal_y', -0.005)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
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

        self.current_yaw = None
        self.target_yaw  = None

        self.cmd_pub    = self.create_publisher(Twist, '/pinky1/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/pinky1/parking/auto_done', 10)
        self._nav_client = ActionClient(self, NavigateToPose, '/pinky1/navigate_to_pose')

        self.create_subscription(
            PoseWithCovarianceStamped, '/pinky1/amcl_pose', self._pose_callback, AMCL_QOS
        )
        self.create_subscription(UInt16MultiArray, '/pinky1/ir_sensor/range', self._ir_callback, 10)
        self.create_subscription(Range, '/pinky1/us_sensor/range', self._us_callback, 10)
        self.create_subscription(Bool, '/pinky1/parking/auto_start', self._start_callback, 10)

        self.create_timer(0.1, self._rotation_tick)

        # wait_for_server 블로킹 방지 → 타이머로 Nav2 서버 준비 확인 후 시작
        self._nav_check_timer = self.create_timer(1.0, self._check_nav_server)

        self.get_logger().info(
            f'AutoParkingNode 시작. 목표=({self.goal_x}, {self.goal_y})'
        )
        self._go_home()
    def _check_nav_server(self):
        """Nav2 서버가 준비되면 자동으로 시작"""
        if self._nav_client.server_is_ready():
            self.get_logger().info('Nav2 서버 준비 완료. 이동 시작.')
            self.destroy_timer(self._nav_check_timer)
            self.started = True
            self._go_home()

    def _start_callback(self, msg: Bool):
        if msg.data and not self.started:
            self.started = True
            self.get_logger().info('주차 진입 시작 신호 수신.')
            self._go_home()

    def _finish(self, success: bool):
        self.status_pub.publish(Bool(data=success))
        self.done = True
        # 타이머로 약간 딜레이 후 shutdown (메시지 전송 여유)
        self.create_timer(0.3, self._shutdown)

    def _shutdown(self):
        rclpy.shutdown()

    # ── Phase 1: 네비게이션 ──────────────────────────────────────

    def _go_home(self):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y

        self.get_logger().info(f'목표 전송: ({self.goal_x}, {self.goal_y})')
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
            self._finish(success=False)
            return
        self.get_logger().info('목표 수락됨.')
        handle.get_result_async().add_done_callback(self._nav_result)

    def _nav_result(self, future):
        status = future.result().status
        if status == 4:
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

    # ── amcl_pose 수신 ───────────────────────────────────────────

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_yaw = quat_to_yaw(msg.pose.pose.orientation)

    # ── Phase 4: 180도 회전 ──────────────────────────────────────

    def _start_rotation(self):
        self.rotating = True
        self.target_yaw = None

    def _rotation_tick(self):
        if not self.rotating or self.done:
            return
        if self.current_yaw is None:
            return

        if self.target_yaw is None:
            self.target_yaw = self.current_yaw + math.pi
            if self.target_yaw > math.pi:
                self.target_yaw -= 2 * math.pi
            elif self.target_yaw < -math.pi:
                self.target_yaw += 2 * math.pi
            self.get_logger().info(f'회전 목표 yaw: {math.degrees(self.target_yaw):.1f}°')

        angle_err = self.target_yaw - self.current_yaw
        if angle_err > math.pi:
            angle_err -= 2 * math.pi
        elif angle_err < -math.pi:
            angle_err += 2 * math.pi

        self.get_logger().info(
            f'회전 중... 현재={math.degrees(self.current_yaw):.1f}°, '
            f'목표={math.degrees(self.target_yaw):.1f}°, 오차={math.degrees(angle_err):.1f}°',
            throttle_duration_sec=0.5,
        )

        if abs(angle_err) <= ROTATION_YAW_TOLERANCE:
            self.cmd_pub.publish(Twist())
            self.rotating = False
            self.get_logger().info('180도 회전 완료. 시나리오 종료.')
            self._finish(success=True)
            return

        angular_z = max(-ANGULAR_VEL, min(ANGULAR_VEL, ANGULAR_VEL * angle_err))
        twist = Twist()
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoParkingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()