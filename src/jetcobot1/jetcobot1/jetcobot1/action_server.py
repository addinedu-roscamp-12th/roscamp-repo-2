"""
JetCobot1 ROS2 액션 서버 (Jazzy)
액션명: jetcobot1/command
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import json
import time
import logging
import threading

from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle

from pinky_msgs.action import RobotCommand

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("jetcobot1")


class RobotController:
    def __init__(self):
        self.mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.thread_lock = True
        logger.info("로봇 연결 완료")

    def do_pickup(self, feedback_cb):
        feedback_cb("pickup 시작")
        logger.info("pickup 동작 시작")
        time.sleep(1)
        feedback_cb("pickup 완료")
        logger.info("pickup 동작 완료")

    def do_load(self, target_pinky: str, feedback_cb):
        feedback_cb(f"load 시작: target_pinky={target_pinky}")
        logger.info(f"load 동작 시작 → target_pinky={target_pinky}")
        time.sleep(1)
        feedback_cb("load 완료")
        logger.info("load 동작 완료")

    def do_unload(self, zone: str, source_pinky: str, feedback_cb):
        feedback_cb(f"unload 시작: zone={zone}, source_pinky={source_pinky}")
        logger.info(f"unload 동작 시작 → zone={zone}, source_pinky={source_pinky}")
        time.sleep(1)
        feedback_cb("unload 완료")
        logger.info("unload 동작 완료")


class JetCobot1ActionServer(Node):
    def __init__(self):
        super().__init__('jetcobot1_action_server')

        self.robot = RobotController()

        self._action_server = ActionServer(
            self,
            RobotCommand,
            'jetcobot1/command',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info("액션 서버 시작: jetcobot1/command")

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Goal 수신 → action_type={goal_request.action_type}, "
            f"task_id={goal_request.task_id}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("취소 요청 수신")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        req         = goal_handle.request
        task_id     = req.task_id
        action_type = req.action_type
        parameters  = json.loads(req.parameters_json) if req.parameters_json else {}

        self.get_logger().info(
            f"[{task_id}] 실행 시작 → action_type={action_type}, params={parameters}"
        )

        def send_feedback(status: str):
            fb = RobotCommand.Feedback()
            fb.status = status
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f"[{task_id}] Feedback: {status}")

        result = RobotCommand.Result()

        try:
            if goal_handle.is_cancel_requested:
                result.event   = "canceled"
                result.message = "취소됨"
                goal_handle.canceled()
                return result

            if action_type == "pickup":
                self.robot.do_pickup(send_feedback)

            elif action_type == "load":
                target_pinky = parameters.get("target_pinky", "unknown")
                self.robot.do_load(target_pinky, send_feedback)

            elif action_type == "unload":
                zone         = parameters.get("zone", "unknown")
                source_pinky = parameters.get("source_pinky", "unknown")
                self.robot.do_unload(zone, source_pinky, send_feedback)

            else:
                self.get_logger().warning(f"[{task_id}] 알 수 없는 action_type: {action_type}")
                result.event   = "error"
                result.message = f"Unknown action_type: {action_type}"
                goal_handle.abort()
                return result

            # ── result 먼저 설정 후 succeed() 호출 ──
            result.event   = "done"
            result.message = f"action '{action_type}' 완료"
            goal_handle.succeed()
            self.get_logger().info(f"[{task_id}] 완료 → event=done 전송")

        except Exception as e:
            self.get_logger().error(f"[{task_id}] 오류: {e}")
            result.event   = "error"
            result.message = str(e)
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)

    node     = JetCobot1ActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
