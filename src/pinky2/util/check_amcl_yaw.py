#!/usr/bin/env python3
"""
/amcl_pose를 구독하여 현재 로봇의 위치(x, y)와 yaw(도) 값을 실시간으로 출력하는 측정용 스크립트.
사용법: python3 check_amcl_yaw.py  (워크스페이스 source 후 실행)
로봇을 흰선과 평행하게 정렬시킨 뒤 출력되는 yaw 값을 확인한다.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped

AMCL_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class CheckAmclYaw(Node):
    def __init__(self):
        super().__init__('check_amcl_yaw')
        self.create_subscription(
            PoseWithCovarianceStamped, '/pinky2/amcl_pose', self._callback, AMCL_QOS
        )
        self.get_logger().info('AMCL pose 대기 중... 로봇을 흰선과 정렬시키고 yaw 값을 확인하세요.')

    def _callback(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.get_logger().info(
            f'x={x:.3f}, y={y:.3f}, yaw={math.degrees(yaw):.1f}°',
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CheckAmclYaw()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
