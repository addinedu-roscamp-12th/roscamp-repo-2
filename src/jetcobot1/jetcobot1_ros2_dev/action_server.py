"""
JetCobot1 ROS2 액션 서버 (Jazzy)
액션명: jetcobot1/command

────────────────────────────────────────────────────────────────
역할:
  Task Manager가 보내는 RobotCommand 액션(Goal)을 받아
  pick_and_place.py(pnp) / weight_aging.py(wt) / box_tracker.py(tracker)의
  실제 동작으로 위임한다. 이 노드 자체는 MyCobot280 시리얼 연결이나
  로봇 제어 로직을 직접 갖지 않는다 (main.py가 생성한 인스턴스를 주입받음).

action_type별 매핑:
  "pickup" → 입고 1회 사이클
             tracker.activate() (카메라 오픈)
             → pnp.run() (박스 감지 대기 + 정렬 + 픽업 + 플레이스)
             → tracker.deactivate() (카메라 해제, 성공/실패 무관 항상 실행)
             기존 main.py의 'I' 키 + inbound_trigger 로직을 그대로 흡수.

  "load"   → 핑키가 적재 위치에 도착했다는 의미로 출고 트리거
             wt.simulate_pinky_arrive(target_pinky)
             기존 main.py의 'P' 키를 흡수.

  "unload" → 출고 트리거 (zone 정보는 현재 로직에서 사용하지 않음 — 필요 시 확장)
             wt.simulate_pinky_arrive(source_pinky)

  pause/resume(R)은 이 액션과 분리되어 pause_control.PauseService
  (/jetcobot1/set_pause 서비스)에서 별도로 처리된다.
────────────────────────────────────────────────────────────────
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import json
import logging

from pinky_msgs.action import RobotCommand

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("jetcobot1")


class JetCobot1ActionServer(Node):
    """
    pnp(PickAndPlace), wt(WaitingTable), tracker(BoxTracker)는
    main.py에서 생성된 인스턴스를 그대로 주입받는다.
    이 노드는 자체적으로 mc(MyCobot280)나 카메라 디바이스를 새로 열지 않는다.
    """

    def __init__(self, pnp, wt, tracker):
        super().__init__('jetcobot1_action_server')

        self.pnp = pnp
        self.wt = wt
        self.tracker = tracker

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

    # ──────────────────────────────────────────────────────────
    # 액션 서버 콜백
    # ──────────────────────────────────────────────────────────
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
                self._do_pickup(send_feedback)

            elif action_type == "load":
                target_pinky = parameters.get("target_pinky", "unknown")
                self._do_load(target_pinky, send_feedback)

            elif action_type == "unload":
                zone         = parameters.get("zone", "unknown")
                source_pinky = parameters.get("source_pinky", "unknown")
                self._do_unload(zone, source_pinky, send_feedback)

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

    # ──────────────────────────────────────────────────────────
    # 실제 동작 위임
    # ──────────────────────────────────────────────────────────
    def _do_pickup(self, feedback_cb):
        """
        입고 1회 사이클. 기존 main.py의 'I' 키 + inbound_trigger + tracker
        activate/deactivate 로직을 그대로 흡수한다.

        카메라는 pnp.run() 성공/실패와 무관하게 항상 deactivate 한다
        (리소스 누수 방지 — try/finally로 보장).
        """
        feedback_cb("pickup 시작 — 카메라 활성화")
        logger.info("pickup 동작 시작 → tracker.activate()")
        self.tracker.activate()

        try:
            feedback_cb("박스 감지/픽업/플레이스 진행 중")
            success = self.pnp.run()
        finally:
            self.tracker.deactivate()
            logger.info("tracker.deactivate() 완료 — 카메라 리소스 해제")

        if success:
            feedback_cb("pickup 완료")
            logger.info("pickup 동작 완료")
        else:
            feedback_cb("pickup 실패")
            logger.warning("pickup 동작 실패 (run() == False)")
            raise RuntimeError("pickup 실패 — pnp.run()이 False 반환")

    def _do_load(self, target_pinky: str, feedback_cb):
        """핑키가 적재 위치에 도착 → 출고 트리거. 기존 'P' 키를 흡수."""
        feedback_cb(f"load 시작: target_pinky={target_pinky}")
        logger.info(f"load 동작 시작 → target_pinky={target_pinky}")

        self.wt.simulate_pinky_arrive(target_pinky)

        feedback_cb("load 완료")
        logger.info("load 동작 완료")

    def _do_unload(self, zone: str, source_pinky: str, feedback_cb):
        feedback_cb(f"unload 시작: zone={zone}, source_pinky={source_pinky}")
        logger.info(f"unload 동작 시작 → zone={zone}, source_pinky={source_pinky}")

        self.wt.simulate_pinky_arrive(source_pinky)

        feedback_cb("unload 완료")
        logger.info("unload 동작 완료")


def main(args=None):
    """
    단독 실행용 (테스트). 실제 운영에서는 main.py에서
    pnp, wt, tracker를 생성한 뒤 JetCobot1ActionServer(pnp, wt, tracker)로
    주입해서 사용.
    """
    import argparse
    from box_tracker import BoxTracker, CameraSetup
    from pick_and_place import PickAndPlace, PLACE_ANGLES_LIST
    from weight_aging import WaitingTable

    parser = argparse.ArgumentParser()
    parser.add_argument("--flip-x", action="store_true")
    cli_args = parser.parse_args()

    rclpy.init(args=args)

    setup = CameraSetup()
    setup.prepare()
    tracker = BoxTracker()
    tracker.start()  # 감시 스레드만 시작, 카메라는 닫힌 상태 — activate()는 pickup 시에만

    wt = WaitingTable(place_angles_list=PLACE_ANGLES_LIST)
    pnp = PickAndPlace(mc=setup.mc, tracker=tracker, flip_x=cli_args.flip_x)
    pnp.set_waiting_table(wt)
    wt.set_pick_and_place(pnp)

    action_node = JetCobot1ActionServer(pnp=pnp, wt=wt, tracker=tracker)

    executor = MultiThreadedExecutor()
    executor.add_node(action_node)
    executor.add_node(wt)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop()
        action_node.destroy_node()
        wt.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
