#!/usr/bin/env python3
# 현재 로봇 위치 평균 측정 스크립트
# 사용법: python3 get_pose.py [측정횟수]

import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped


COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 5


class PoseReader(Node):
    def __init__(self):
        super().__init__("pose_reader")
        self._samples = []
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._cb,
            qos)
        self.create_timer(3.0, self._timeout)
        print(f"측정 중... ({COUNT}회 수집, 또는 3초 후 자동 출력)")

    def _cb(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = 2 * math.atan2(o.z, o.w)
        self._samples.append((p.x, p.y, yaw))
        print(f"  [{len(self._samples)}/{COUNT}] x={p.x:.4f} y={p.y:.4f} yaw={yaw:.4f}")
        if len(self._samples) >= COUNT:
            self._print_result()
            rclpy.shutdown()

    def _timeout(self):
        if self._samples:
            self._print_result()
        else:
            print("수신된 데이터 없음 — AMCL 실행 중인지 확인해줘")
        rclpy.shutdown()

    def _print_result(self):
        xs   = [s[0] for s in self._samples]
        ys   = [s[1] for s in self._samples]
        yaws = [s[2] for s in self._samples]
        x   = round(sum(xs)   / len(xs),   4)
        y   = round(sum(ys)   / len(ys),   4)
        yaw = round(sum(yaws) / len(yaws), 4)
        print(f"\n=== 평균 좌표 ({len(self._samples)}회) ===")
        print(f'  "location_name": {{"x": {x}, "y": {y}, "yaw": {yaw}}},')


def main():
    rclpy.init()
    node = PoseReader()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
