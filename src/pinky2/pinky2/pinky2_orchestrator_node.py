#!/usr/bin/env python3
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

from common_interfaces.action import RobotCommand

ACTION_NAME = '/command'   # task_manager와 합의 후 변경 가능
EXPECTED_ACTION_TYPE = 'navigate'  # task_manager와 합의 후 변경 가능
WAIT_POLL_SEC = 1.0


class ParkingOrchestratorNode(Node):
    def __init__(self):
        super().__init__('pinky2_orchestrator_node')

        cb_group = ReentrantCallbackGroup()

        self.exit_start_pub = self.create_publisher(Bool, 'pinky2/parking/exit_start', 10)
        self.auto_start_pub = self.create_publisher(Bool, 'pinky2/parking/auto_start', 10)

        self._exit_done_event = threading.Event()
        self._exit_done_success = False
        self._auto_done_event = threading.Event()
        self._auto_done_success = False

        self.create_subscription(
            Bool, 'pinky2/parking/exit_done', self._exit_done_callback, 10,
            callback_group=cb_group,
        )
        self.create_subscription(
            Bool, 'pinky2/parking/auto_done', self._auto_done_callback, 10,
            callback_group=cb_group,
        )

        self._action_server = ActionServer(
            self, RobotCommand, ACTION_NAME,
            execute_callback=self._execute_callback,
            callback_group=cb_group,
        )

        self.get_logger().info(f'ParkingOrchestratorNode 시작. 액션 서버: {ACTION_NAME}')

    def _exit_done_callback(self, msg: Bool):
        self._exit_done_success = msg.data
        self._exit_done_event.set()

    def _auto_done_callback(self, msg: Bool):
        self._auto_done_success = msg.data
        self._auto_done_event.set()

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(
            f'액션 골 수신: action_type={goal.action_type}, task_id={goal.task_id}'
        )

        result = RobotCommand.Result()

        if goal.action_type != EXPECTED_ACTION_TYPE:
            self.get_logger().error(f'지원하지 않는 action_type: {goal.action_type}')
            goal_handle.abort()
            result.event = 'unsupported_action_type'
            result.message = f'지원하지 않는 action_type: {goal.action_type}'
            return result

        # 1단계: 탈출 시퀀스
        self._exit_done_event.clear()
        self.get_logger().info('탈출 시퀀스 시작 신호 전송.')
        self.exit_start_pub.publish(Bool(data=True))
        self._publish_feedback(goal_handle, '탈출 시퀀스 진행 중')

        while not self._exit_done_event.wait(timeout=WAIT_POLL_SEC):
            self._publish_feedback(goal_handle, '탈출 시퀀스 진행 중')

        if not self._exit_done_success:
            self.get_logger().error('탈출 시퀀스 실패.')
            goal_handle.abort()
            result.event = 'exit_failed'
            result.message = '탈출 시퀀스 실패'
            return result

        # 2단계: 주차 진입 시퀀스
        self._auto_done_event.clear()
        self.get_logger().info('주차 진입 시퀀스 시작 신호 전송.')
        self.auto_start_pub.publish(Bool(data=True))
        self._publish_feedback(goal_handle, '주차 진입 시퀀스 진행 중')

        while not self._auto_done_event.wait(timeout=WAIT_POLL_SEC):
            self._publish_feedback(goal_handle, '주차 진입 시퀀스 진행 중')

        if not self._auto_done_success:
            self.get_logger().error('주차 진입 시퀀스 실패.')
            goal_handle.abort()
            result.event = 'auto_failed'
            result.message = '주차 진입 시퀀스 실패'
            return result

        goal_handle.succeed()
        result.event = 'success'
        result.message = '탈출 및 재주차 시퀀스 완료'
        return result

    def _publish_feedback(self, goal_handle, status: str):
        feedback = RobotCommand.Feedback()
        feedback.status = status
        goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingOrchestratorNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    finally:
        node.destroy_node()
        rclpy.shutdown()
