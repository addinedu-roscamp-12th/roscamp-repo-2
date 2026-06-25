#!/usr/bin/env python3
import math
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
EXIT_TOLERANCE = 0.10
ROTATION_YAW_TOLERANCE = math.radians(3)

AMCL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ODOM_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

ZONE_POSES = {
    1: {"x": -0.497, "y": -0.0542},
    2: {"x": -0.492, "y": -0.424},
    3: {"x": -0.481, "y": -0.773},
}

FINAL_GOAL = {"x": 0.0612, "y": -0.7896, "yaw": math.pi}


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quaternion(yaw: float):
    return {"z": math.sin(yaw / 2.0), "w": math.cos(yaw / 2.0)}


class ExitParkingNode(Node):
    def __init__(self):
        super().__init__('exit_parking_node')

        self.declare_parameter('exit_x', -0.18112303207757627)
        self.declare_parameter('exit_y', -0.035069745872950836)
        self.declare_parameter('exit_tolerance', EXIT_TOLERANCE)
        self.declare_parameter('zone', 1)

        self.exit_x         = self.get_parameter('exit_x').value
        self.exit_y         = self.get_parameter('exit_y').value
        self.exit_tolerance = self.get_parameter('exit_tolerance').value
        self.zone           = self.get_parameter('zone').value

        if self.zone not in ZONE_POSES:
            self.get_logger().error(f'유효하지 않은 존 번호: {self.zone}. 1, 2, 3 중 하나여야 합니다.')
            raise ValueError(f'유효하지 않은 존 번호: {self.zone}')

        self.zone_x = ZONE_POSES[self.zone]["x"]
        self.zone_y = ZONE_POSES[self.zone]["y"]

        self.current_x      = None
        self.current_y      = None
        self.current_yaw    = None
        self.exiting        = False
        self.rotating       = False
        self.final_aligning = False
        self.done           = False
        self.wait_timer     = None

        self.last_yaw        = None
        self.accumulated_yaw = 0.0

        self.cmd_pub  = self.create_publisher(Twist, '/pinky2/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, '/pinky2/parking/exit_done', 10)
        self._nav_client = ActionClient(self, NavigateToPose, '/pinky2/navigate_to_pose')

        self.create_subscription(
            PoseWithCovarianceStamped, '/pinky2/amcl_pose', self._pose_callback, AMCL_QOS
        )
        self.create_subscription(Odometry, '/pinky2/odom', self._odom_callback, ODOM_QOS)
        self.create_subscription(Bool, '/pinky2/parking/exit_start', self._start_callback, 10)

        self.create_timer(0.1, self._timer_callback)

        self.get_logger().info(
            f'ExitParkingNode 시작. 탈출 좌표=({self.exit_x:.3f}, {self.exit_y:.3f}), 존={self.zone}'
        )
        self.exiting = True
    def _start_callback(self, msg: Bool):
        if msg.data and not self.exiting:
            self.exiting = True
            self.get_logger().info('탈출 시작 신호 수신. 탈출 시퀀스 시작.')

    def _finish(self, success: bool):
        self.done_pub.publish(Bool(data=success))
        self.done = True
        rclpy.shutdown()

    # ── amcl_pose 수신 ───────────────────────────────────────────

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_yaw = quat_to_yaw(msg.pose.pose.orientation)

    # ── 타이머: Phase 1 직진 ─────────────────────────────────────

    def _timer_callback(self):
        if self.done:
            return
        if self.final_aligning:
            self._final_alignment_tick()
            return
        if not self.exiting or self.rotating:
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
            self._go_to_zone()
            #self._start_rotation()
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

    # ── Phase 2: 360도 회전 ──────────────────────────────────────

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

        if abs(self.accumulated_yaw) >= 2 * math.pi:
            self.cmd_pub.publish(Twist())
            self.rotating = False
            self.get_logger().info(f'360도 회전 완료. 존 {self.zone} 으로 이동 시작.')
            self._go_to_zone()
            return

        twist = Twist()
        twist.angular.z = ANGULAR_VEL
        self.cmd_pub.publish(twist)

    # ── Phase 3a: 존으로 이동 ────────────────────────────────────

    def _go_to_zone(self):
        self.get_logger().info(f'존 {self.zone} 으로 이동: ({self.zone_x:.3f}, {self.zone_y:.3f})')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.zone_x
        goal.pose.pose.position.y = self.zone_y
        goal.pose.pose.orientation.z = 0.0
        goal.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback)
        future.add_done_callback(self._zone_goal_response)

    def _zone_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('존 이동 목표 거부됨.')
            rclpy.shutdown()
            return
        self.get_logger().info('존 이동 목표 수락됨.')
        handle.get_result_async().add_done_callback(self._zone_result)

    def _zone_result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info(f'존 {self.zone} 도착. 3초 대기 중...')
            # time.sleep 대신 타이머로 대기
            self.wait_timer = self.create_timer(3.0, self._on_wait_done)
            return
        self.get_logger().error(f'존 이동 실패 (status={status}).')
        rclpy.shutdown()

    def _on_wait_done(self):
        # 타이머 한 번만 실행
        self.wait_timer.cancel()
        self.wait_timer = None
        self.get_logger().info('3초 대기 완료. 최종 목적지로 이동 중...')
        self._go_to_final()

    # ── Phase 3b: 최종 목적지로 이동 ─────────────────────────────

    def _go_to_final(self):
        q = yaw_to_quaternion(FINAL_GOAL["yaw"])

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = FINAL_GOAL["x"]
        goal.pose.pose.position.y = FINAL_GOAL["y"]
        goal.pose.pose.orientation.z = q["z"]
        goal.pose.pose.orientation.w = q["w"]

        self.get_logger().info(f'최종 목적지로 이동: ({FINAL_GOAL["x"]}, {FINAL_GOAL["y"]})')
        future = self._nav_client.send_goal_async(goal, feedback_callback=self._nav_feedback)
        future.add_done_callback(self._final_goal_response)

    def _final_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('최종 목적지 목표 거부됨.')
            rclpy.shutdown()
            return
        self.get_logger().info('최종 목적지 목표 수락됨.')
        handle.get_result_async().add_done_callback(self._final_result)

    def _final_result(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info('최종 목적지 도착. yaw 정렬 시작.')
            self._start_final_alignment()
            return
        self.get_logger().error(f'최종 목적지 이동 실패 (status={status}).')
        rclpy.shutdown()

    # ── Phase 3c: 최종 yaw 정렬 ──────────────────────────────────

    def _start_final_alignment(self):
        self.final_aligning = True

    def _final_alignment_tick(self):
        if self.current_yaw is None:
            return

        angle_err = FINAL_GOAL["yaw"] - self.current_yaw
        if angle_err > math.pi:
            angle_err -= 2 * math.pi
        elif angle_err < -math.pi:
            angle_err += 2 * math.pi

        self.get_logger().info(
            f'최종 정렬 중... 현재={math.degrees(self.current_yaw):.1f}°, '
            f'목표={math.degrees(FINAL_GOAL["yaw"]):.1f}°, 오차={math.degrees(angle_err):.1f}°',
            throttle_duration_sec=0.5,
        )

        if abs(angle_err) <= ROTATION_YAW_TOLERANCE:
            self.cmd_pub.publish(Twist())
            self.final_aligning = False
            self.get_logger().info('최종 yaw 정렬 완료. 시나리오 종료.')
            self._finish(True)
            return

        angular_z = max(-ANGULAR_VEL, min(ANGULAR_VEL, ANGULAR_VEL * angle_err))
        twist = Twist()
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

    # ── 공통 ────────────────────────────────────────────────────

    def _nav_feedback(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'이동 중... 남은 거리: {dist:.2f} m', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = ExitParkingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()