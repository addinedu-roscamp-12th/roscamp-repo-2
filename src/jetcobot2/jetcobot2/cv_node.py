#!/usr/bin/env python3
"""
cv_node.py ─ 카메라 + 블럭 검출 Action Client 노드

역할:
  - camera/image 토픽을 구독해 프레임 수신
  - cv_lib.process_frame()으로 흰 블럭 좌표 검출
  - 블럭 감지 시 Action Server로 Goal 전송
  - /cv_mode 토픽으로 action_type 동적 변경
"""

import json
import uuid

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String

from jetcobot_interfaces.action import RobotCommand
from jetcobot2.cv_lib import process_frame


class CvNode(Node):
    def __init__(self):
        super().__init__('cv_node')

        self._bridge      = CvBridge()
        self._busy        = False
        self._action_type = "do_load"  # 기본값
        self._target_item = "물병"     # 출고 시 품목

        self._action_client = ActionClient(self, RobotCommand, 'jetcobot2/command')

        # 카메라 토픽 구독
        self.create_subscription(Image, 'camera/image', self._image_callback, 10)

        # 모드 변경 토픽 구독
        self.create_subscription(String, '/cv_mode', self._mode_callback, 10)

        self.get_logger().info("cv_node 시작 — camera/image 구독 중")

    def _mode_callback(self, msg):
        """
        /cv_mode 토픽으로 모드 변경
        예: '{"action_type": "do_load"}'
            '{"action_type": "do_unload", "target_item": "물병"}'
        """
        try:
            data = json.loads(msg.data)
            self._action_type = data.get("action_type", "do_load")
            self._target_item = data.get("target_item", "물병")
            self.get_logger().info(
                f"모드 변경 → action_type={self._action_type}, "
                f"target_item={self._target_item}"
            )
        except Exception as e:
            self.get_logger().error(f"모드 변경 실패: {msg.data}, 오류: {e}")

    def _image_callback(self, msg: Image):
        """카메라 프레임 수신 → 블럭 검출 → Goal 전송"""
        if self._busy:
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detected, cx, cy, area, hor, ver, ar, _ = process_frame(frame)

        if detected:
            self._busy = True
            self.get_logger().info(
                f"블럭 감지! cx={cx:.1f}, cy={cy:.1f}, area={area:.0f}, "
                f"hor={hor:.0f}, ver={ver:.0f}, ar={ar:.2f} → Goal 전송"
            )
            self._send_goal(detected, cx, cy, area, hor, ver, ar)

    def _send_goal(self, detected, cx, cy, area, hor, ver, ar):
        """Action Server로 Goal 전송"""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("arm_node 가 아직 준비 안 됨 — 스킵")
            self._busy = False
            return

        goal_msg                 = RobotCommand.Goal()
        goal_msg.action_type     = self._action_type
        goal_msg.task_id         = str(uuid.uuid4())
        goal_msg.parameters_json = json.dumps({
            "detected":    detected,
            "result_cx":   round(cx,   2),
            "result_cy":   round(cy,   2),
            "result_area": round(area, 2),
            "result_hor":  round(hor,  2),
            "result_ver":  round(ver,  2),
            "result_ar":   round(ar,   2),
            "target_item": self._target_item,
        })

        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Goal 수락/거절 처리"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal 거절됨")
            self._busy = False
            return

        self.get_logger().info("Goal 수락됨 — 동작 시작 대기")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """arm 상태 피드백 수신"""
        self.get_logger().info(f"arm 상태: {feedback_msg.feedback.status}")

    def _result_callback(self, future):
        """동작 완료 후 잠금 해제"""
        result = future.result().result

        if result.event == "done":
            self.get_logger().info(f"완료: {result.message}")
        elif result.event == "error":
            self.get_logger().error(f"실패: {result.message}")
        elif result.event == "canceled":
            self.get_logger().warn("취소됨")

        if result.event in ["done", "error", "canceled"]:
            self._busy = False
            self.get_logger().info("다음 블럭 감지 대기 중...")
        else:
            self.get_logger().warn(f"예상치 못한 event: '{result.event}' → busy 유지")


def main(args=None):
    rclpy.init(args=args)
    node = CvNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()