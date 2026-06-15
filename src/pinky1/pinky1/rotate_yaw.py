#!/usr/bin/env python3
"""
yaw 기반 제자리 회전 유틸리티 (map 프레임 기준)
사용법:
  ros2 run pinky1 rotate_yaw -- --angle -90          # 상대: 오른쪽 90도
  ros2 run pinky1 rotate_yaw -- --angle 180           # 상대: 왼쪽 180도
  ros2 run pinky1 rotate_yaw -- --target 0            # 절대: map 기준 yaw 0도로 정렬
  ros2 run pinky1 rotate_yaw -- --target 90           # 절대: map 기준 yaw 90도로 정렬
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


# ════════════════════════════════════════════════════════════
# HELPERS
# ════════════════════════════════════════════════════════════
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


def _get_map_yaw(tf_buffer: Buffer, ns: str) -> float | None:
    """map → base_footprint TF에서 map 기준 yaw 반환."""
    try:
        t = tf_buffer.lookup_transform(
            "map", f"{ns}/base_footprint",
            rclpy.time.Time())
        return _yaw_from_quat(t.transform.rotation)
    except Exception:
        return None


# ════════════════════════════════════════════════════════════
# NODE
# ════════════════════════════════════════════════════════════
class RotateNode(Node):
    def __init__(self, ns: str, angle_deg: float = None, target_deg: float = None):
        super().__init__("rotate_yaw_util", namespace=ns)
        self._ns         = ns
        self._angle_deg  = angle_deg
        self._target_rad = math.radians(target_deg) if target_deg is not None else None
        self._start_yaw  = None
        self._done       = False

        self._pub = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)

        # map 기준 TF
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # TF 실패 시 odom 폴백
        self._sub = self.create_subscription(
            Odometry, f"/{ns}/odom", self._cb_odom, 10)

        self.create_timer(0.1, self._tick)

        if target_deg is not None:
            self.get_logger().info(
                f"절대 yaw {target_deg:+.1f}도로 정렬 — map 프레임 기준 (namespace={ns})")
        else:
            self.get_logger().info(f"상대 회전 {angle_deg:+.1f}도 (namespace={ns})")

        self._odom_yaw = None

    def _cb_odom(self, msg: Odometry):
        self._odom_yaw = _yaw_from_quat(msg.pose.pose.orientation)

    def _current_yaw(self) -> float | None:
        y = _get_map_yaw(self._tf_buf, self._ns)
        if y is not None:
            return y
        return self._odom_yaw  # TF 없으면 odom 폴백

    def _tick(self):
        if self._done:
            return

        yaw = self._current_yaw()
        if yaw is None:
            return

        # 절대 yaw 모드
        if self._target_rad is not None:
            remaining = _angle_diff(self._target_rad, yaw)
            self.get_logger().info(
                f"현재 yaw={math.degrees(yaw):+.1f}도  남은 각도={math.degrees(remaining):+.1f}도",
                throttle_duration_sec=0.5)
            if abs(remaining) < math.radians(2.0):
                self._stop()
                return
            speed = math.copysign(max(0.15, min(0.4, abs(remaining) * 0.5)), remaining)
            twist = Twist()
            twist.angular.z = speed
            self._pub.publish(twist)
            return

        # 상대 회전 모드
        if self._start_yaw is None:
            self._start_yaw = yaw
            return
        target_diff = math.radians(self._angle_deg)
        rotated     = _angle_diff(yaw, self._start_yaw)
        remaining   = target_diff - rotated
        if abs(remaining) < math.radians(2.0):
            self._stop()
            return
        speed = math.copysign(max(0.15, min(0.4, abs(remaining) * 0.5)), target_diff)
        twist = Twist()
        twist.angular.z = speed
        self._pub.publish(twist)

    def _stop(self):
        self._pub.publish(Twist())
        self.get_logger().info("회전 완료 — 정지")
        self._done = True


# ════════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--angle",  type=float, help="상대 회전 (도). 양수=왼쪽, 음수=오른쪽")
    group.add_argument("--target", type=float, help="절대 yaw 목표 (도). 지도 기준 방향으로 정렬")
    parser.add_argument("--ns", type=str, default="pinky1",
                        help="로봇 네임스페이스 (기본값: pinky1)")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = RotateNode(ns=args.ns, angle_deg=args.angle, target_deg=args.target)
    timeout = time.time() + 30.0

    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.05)
        if time.time() > timeout:
            node.get_logger().error("타임아웃 — 정지")
            node._pub.publish(Twist())
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
