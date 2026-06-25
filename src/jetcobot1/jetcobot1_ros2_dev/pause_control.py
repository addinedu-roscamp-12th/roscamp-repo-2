"""
pause_control.py - 관리자 일시정지/재시작 공용 모듈

목적:
  비상정지(E-stop)가 아닌, 관리자가 청소/정리 등을 위해
  "다음 동작 시작 전에" 로봇을 잠시 멈춰두는 운영성 일시정지.

사용 패턴:
  - 동작 단계 "경계"에서 wait_if_paused() 호출
  - 이미 나간 모션 명령(send_angles 등) 자체를 끊지는 않음
  - 음성/키보드/HTTP/ROS2 등 어떤 입력이든 pause()/resume() 만 호출하면 됨

main.py, pick_and_place_d.py, weight_azing.py 가 전부 이 모듈을
import해서 같은 PAUSE_EVENT를 공유한다.

────────────────────────────────────────────────────────────────
ROS2 연동 (신규):
  PauseService 노드가 pinky_msgs/srv/SetPause 서비스를 제공.
  voice_pause_client.py 등 외부 클라이언트가 이 서비스를 호출하면
  내부적으로 pause()/resume()을 호출 → 기존 PAUSE_EVENT를 그대로 갱신.

  main.py에서:
      from pause_control import PauseService
      pause_srv_node = PauseService()
      executor.add_node(pause_srv_node)   # 기존 executor에 같이 등록
────────────────────────────────────────────────────────────────
"""

import time
import logging
import threading

logger = logging.getLogger("jetcobot1.pause_control")

PAUSE_EVENT = threading.Event()

# 일시정지 사유 (로그/UI 표시용, 필수 아님)
_pause_reason = ""


def pause(reason: str = ""):
    global _pause_reason
    _pause_reason = reason
    PAUSE_EVENT.set()
    logger.warning(f"⏸ 일시정지 ON{f' ({reason})' if reason else ''}")


def resume():
    global _pause_reason
    _pause_reason = ""
    PAUSE_EVENT.clear()
    logger.info("▶ 재시작 (일시정지 해제)")


def is_paused() -> bool:
    return PAUSE_EVENT.is_set()


def get_pause_reason() -> str:
    return _pause_reason


def wait_if_paused(poll: float = 0.1):
    """
    동작 단계 경계에서 호출. 정지 상태면 해제될 때까지 블록.
    이미 시작된 모션 자체를 끊지는 않으며, 다음 단계로 넘어가기 전에만 막는다.
    """
    if not PAUSE_EVENT.is_set():
        return
    logger.warning(
        f"⏸ 일시정지 중 — 다음 동작 대기{f' ({_pause_reason})' if _pause_reason else ''}"
    )
    while PAUSE_EVENT.is_set():
        time.sleep(poll)
    logger.info("▶ 재시작됨 — 다음 동작 진행")


# ────────────────────────────────────────────────────────────────
# ROS2 서비스 서버 — pinky_msgs/srv/SetPause
# 기존 HTTP(voice_command_server.py)를 대체.
# ────────────────────────────────────────────────────────────────
try:
    from rclpy.node import Node
    from pinky_msgs.srv import SetPause
    _ROS2_SRV_AVAILABLE = True
except ImportError:
    # pinky_msgs가 아직 빌드 안 됐거나 rclpy 환경이 아닐 때도
    # pause_control 자체(PAUSE_EVENT)는 계속 import 가능하도록 처리
    Node = object
    _ROS2_SRV_AVAILABLE = False
    logger.warning(
        "pinky_msgs.srv.SetPause import 실패 — "
        "ROS2 서비스 서버 비활성화 (PAUSE_EVENT는 정상 동작)"
    )


if _ROS2_SRV_AVAILABLE:
    class PauseService(Node):
        """
        /jetcobot1/set_pause 서비스 서버.

        요청:
            paused: bool   — True면 정지, False면 재시작
            reason: string — 정지 사유 (선택, paused=False일 때는 무시됨)
        응답:
            success: bool
            current_paused: bool
            current_reason: string

        main.py에서 다른 노드들과 같은 MultiThreadedExecutor에 등록해서 사용.
        """

        def __init__(self, service_name: str = "/jetcobot1/set_pause"):
            super().__init__("pause_service")
            self._srv = self.create_service(
                SetPause, service_name, self._handle_set_pause
            )
            self.get_logger().info(f"PauseService 시작: {service_name}")

        def _handle_set_pause(self, request, response):
            if request.paused:
                pause(request.reason or "ROS2 서비스 요청")
                self.get_logger().warning(
                    f"⏸ 일시정지 ON (서비스 요청, reason={request.reason!r})"
                )
            else:
                resume()
                self.get_logger().info("▶ 재시작 (서비스 요청)")

            response.success = True
            response.current_paused = is_paused()
            response.current_reason = get_pause_reason()
            return response
else:
    PauseService = None  # rclpy/pinky_msgs 없는 환경에서 import 에러 방지용
