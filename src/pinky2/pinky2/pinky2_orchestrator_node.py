#!/usr/bin/env python3
import json
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool

from msgs.action import RobotCommand

ACTION_NAME = 'pinky2/command'   # task_manager와 합의 후 변경 가능
EXPECTED_ACTION_TYPE = 'navigate'  # task_manager와 합의 후 변경 가능
WAIT_POLL_SEC = 1.0


class ParkingOrchestratorNode(Node):
    def __init__(self):
        super().__init__('pinky2_orchestrator_node')

        cb_group = ReentrantCallbackGroup()

        self.exit_start_pub = self.create_publisher(Bool, 'pinky2/parking/exit_start', 10)

        self._exit_done_event = threading.Event()
        self._exit_done_success = False

        self.create_subscription(
            Bool, 'pinky2/parking/exit_done', self._exit_done_callback, 10,
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

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(
            f'액션 골 수신: action_type={goal.action_type}, params={goal.parameters_json}'
        )

        result = RobotCommand.Result()

        if goal.action_type != EXPECTED_ACTION_TYPE:
            self.get_logger().error(f'지원하지 않는 action_type: {goal.action_type}')
            goal_handle.abort()
            result.event = 'unsupported_action_type'
            result.message = f'지원하지 않는 action_type: {goal.action_type}'
            return result

        try:
            params = json.loads(goal.parameters_json)
        except json.JSONDecodeError:
            goal_handle.abort()
            result.event = 'invalid_params'
            result.message = 'parameters_json 파싱 실패'
            return result

        location = params.get('location', '')

        if location == 'load_wait_2':
            return self._run_exit_sequence(goal_handle, result)
        else:
            goal_handle.abort()
            result.event = 'unknown_location'
            result.message = f'알 수 없는 location: {location}'
            return result

    def _run_exit_sequence(self, goal_handle, result):
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

        goal_handle.succeed()
        result.event = 'success'
        result.message = '탈출 시퀀스 완료'
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
