#!/usr/bin/env python3
"""
main.py - JetCobot1 통합 런처 (ROS2 통합 버전)

pick_and_place_d.py + weight_azing.py + action_server.py + pause_control.py
를 하나의 프로세스, 하나의 MultiThreadedExecutor에서 실행.

────────────────────────────────────────────────────────────────
변경 사항 (키보드 시뮬레이션 → ROS2):
  기존 키보드 명령은 "실제로 Task Manager가 ROS2로 보낼 신호"를
  임시로 키보드로 대체한 것이었음. 이제 다음과 같이 교체됨:

    I (입고 트리거)   → jetcobot1/command 액션, action_type="pickup"
                        (action_server.py의 _do_pickup이
                         tracker.activate() → pnp.run() → tracker.deactivate()
                         순서로 기존 카메라 리소스 관리 로직을 그대로 수행)
    P (핑키 도착)      → jetcobot1/command 액션, action_type="load"/"unload"
    R (일시정지/재시작) → /jetcobot1/set_pause 서비스 (pause_control.PauseService)
                        Task Manager가 음성 STT 텍스트를 LLM으로 분류한 뒤
                        이 서비스를 호출해 즉시 반영 (액션 큐와 무관)

  S(상태조회), Q(종료)는 운영 중 로컬 디버그 용도로만 남겨둠.
────────────────────────────────────────────────────────────────

실행:
    python3 main.py
    python3 main.py --flip-x
    python3 main.py --stream

키보드 명령 (디버그용, 선택):
    S  → 대기 테이블 현황 출력
    Q  → 종료
"""

import argparse
import threading
import time
import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor

from box_tracker import BoxTracker, CameraSetup, StreamServer
from pick_and_place import PickAndPlace, PLACE_ANGLES_LIST
from weight_aging import WaitingTable
from pause_control import PauseService
from action_server import JetCobot1ActionServer


def _keyboard_listener(wt: WaitingTable, stop_event: threading.Event):
    """
    디버그용 키보드 입력 (운영 중 모니터링/종료 용도).
    I/P/R은 ROS2 액션/서비스로 대체되었으므로 더 이상 여기서 처리하지 않음.
    """
    print("\n[키보드] S=테이블현황  Q=종료")
    while not stop_event.is_set():
        try:
            key = input().strip().upper()
            if key == 'S':
                wt._log_table()
                wt._log_pinky_queue()
            elif key == 'Q':
                print("\n[키보드] 종료 요청")
                stop_event.set()
            else:
                print("[키보드] S=테이블현황  Q=종료 "
                      "(I/P는 jetcobot1/command 액션, R은 /jetcobot1/set_pause 서비스로 대체됨)")
        except EOFError:
            break


def main():
    parser = argparse.ArgumentParser(description="JetCobot1 통합 런처 (ROS2)")
    parser.add_argument("--flip-x", action="store_true", help="J1 회전 방향 반전")
    parser.add_argument("--stream", action="store_true", help="MJPEG 스트리밍 서버 시작")
    args = parser.parse_args()

    stop_event = threading.Event()

    # ── ROS2 초기화 ───────────────────────────────────────────────
    rclpy.init()
    wt = WaitingTable(place_angles_list=PLACE_ANGLES_LIST)

    # ── 로봇 + 카메라 초기화 ─────────────────────────────────────
    print("[main] 로봇팔 초기화 중...")
    setup = CameraSetup()
    setup.prepare()

    tracker = BoxTracker()
    tracker.start()  # 감시 스레드만 시작 — 카메라는 닫힌 상태.
                      # pickup 액션이 들어올 때 action_server.py가 activate() 호출.

    if args.stream:
        StreamServer(tracker).start()
        print("[main] MJPEG 스트리밍 서버 시작됨")

    # ── PickAndPlace 생성 + WaitingTable 연동 ────────────────────
    pnp = PickAndPlace(mc=setup.mc, tracker=tracker, flip_x=args.flip_x)
    pnp.set_waiting_table(wt)   # pick_and_place → waiting_table 알림
    wt.set_pick_and_place(pnp)  # waiting_table → 실제 로봇 출고 동작
    print("[main] PickAndPlace ↔ WaitingTable 연동 완료")

    # ── 액션서버 + pause 서비스 노드 생성 ────────────────────────
    action_node = JetCobot1ActionServer(pnp=pnp, wt=wt, tracker=tracker)
    pause_node = PauseService() if PauseService is not None else None
    if pause_node is None:
        print("[main] ⚠️ PauseService 비활성화 — pinky_msgs 빌드/설치 확인 필요")

    # ── 단일 MultiThreadedExecutor에 모든 노드 등록 ──────────────
    executor = MultiThreadedExecutor()
    executor.add_node(wt)
    executor.add_node(action_node)
    if pause_node is not None:
        executor.add_node(pause_node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    print("[main] ROS2 executor 백그라운드 실행 중 "
          "(WaitingTable + ActionServer + PauseService)...")

    # 키보드 입력 백그라운드 스레드 (디버그용)
    kb_thread = threading.Thread(
        target=_keyboard_listener, args=(wt, stop_event), daemon=True
    )
    kb_thread.start()

    # ── 메인 스레드는 종료 신호 대기만 ────────────────────────────
    # 실제 동작(입고/출고)은 모두 action_server.py의 execute_callback
    # (ReentrantCallbackGroup, executor 스레드풀) 안에서 처리되므로
    # main()에는 더 이상 박스 polling 루프가 필요 없다.
    print("\n[main] 준비 완료 — Task Manager의 ROS2 명령 대기 중")
    print("[main] jetcobot1/command(액션) / /jetcobot1/set_pause(서비스)")
    print("[main] 키보드(디버그): S=테이블현황  Q=종료\n")

    try:
        while not stop_event.is_set():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[main] 중단됨 (Ctrl+C)")

    finally:
        stop_event.set()
        tracker.stop()
        executor.shutdown()
        action_node.destroy_node()
        if pause_node is not None:
            pause_node.destroy_node()
        wt.destroy_node()
        rclpy.shutdown()
        print("[main] 종료")


if __name__ == "__main__":
    main()
