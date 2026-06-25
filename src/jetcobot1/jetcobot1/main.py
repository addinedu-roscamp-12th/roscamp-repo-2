#!/usr/bin/env python3
"""
main.py - JetCobot1 통합 런처

pick_and_place.py + weight_aging.py 를 하나의 프로세스에서 실행.

실행:
    python3 main.py
    python3 main.py --flip-x
    python3 main.py --stream
    python3 main.py --voice-server        # 음성 명령 수신 서버(8011) 같이 실행

키보드 명령 (pick & place 대기 중):
    I  → 입고 트리거 ON (박스 감지 → pick & place 시작 + 카메라 활성화)
    P  → 핑키 도착 트리거 (출고 큐 정렬 + 순서대로 적재)
    S  → 대기 테이블 현황 출력
    R  → 일시정지 / 재시작 토글
    Q  → 종료

카메라 리소스 관리:
    입고 트리거(I)가 켜질 때 tracker.activate() → 카메라 오픈 + 프레임 수신 시작
    pick & place 1회(성공/실패 무관) 완료 시 tracker.deactivate() → 카메라 해제
    그 사이 모든 재시도/대기(공잡이 재시도, 낙하 감지 등)는 같은 입고 사이클로 간주하여
    카메라를 계속 켜둔 채로 처리한다.
    핑키 출고 트리거는 이 사이클과 무관 — 카메라 on/off는 입고 트리거에만 연동된다.
"""

import argparse
import threading
import time
import sys
import rclpy

from box_tracker import BoxTracker, CameraSetup, StreamServer, ANGLES_CAMERA_DOWN, ROBOT_SPEED
from pick_and_place import PickAndPlace, PLACE_ANGLES_LIST, PLACE_MAX_COUNT
from weight_aging import WaitingTable
from pause_control import pause, resume, is_paused, get_pause_reason


def _keyboard_listener(wt: WaitingTable, stop_event: threading.Event,
                        inbound_trigger: threading.Event, tracker: BoxTracker):
    """
    백그라운드에서 키보드 입력을 감지해 핑키 도착 트리거 등을 처리.
    I: 입고 트리거 (박스 감지 → pick & place 허용) — 동시에 카메라 활성화
    P: 핑키 도착 시뮬레이션
    S: 대기 테이블 현황 출력
    R: 일시정지 / 재시작 토글 (관리자용 — 비상정지 아님)
    Q: 종료
    """
    print("\n[키보드] I=입고시작  S=테이블현황  P=핑키상태  R=일시정지/재시작  Q=종료")
    while not stop_event.is_set():
        try:
            key = input().strip().upper()
            if key == 'I':
                if inbound_trigger.is_set():
                    print("[키보드] 이미 입고 대기 중입니다.")
                else:
                    inbound_trigger.set()
                    tracker.activate()
                    print("[키보드] 📦 입고 트리거 ON — 카메라 활성화, 박스 감지 시작")
            elif key == 'S':
                wt._log_table()
                wt._log_pinky_queue()
            elif key == 'P':
                print(f"[핑키 상태] {wt._pinky_status}")
                print(f"[핑키 큐]   {wt._pinky_queue}")
            elif key == 'R':
                if is_paused():
                    resume()
                    print("[키보드] ▶ 재시작됨")
                else:
                    pause("관리자 키보드 명령")
                    print("[키보드] ⏸ 일시정지됨 — 현재 진행 중인 동작 완료 후 다음 단계에서 정지")
            elif key == 'Q':
                print("\n[키보드] 종료 요청")
                stop_event.set()
            else:
                print("[키보드] I=입고시작  S=테이블현황  P=핑키상태  R=일시정지/재시작  Q=종료")
        except EOFError:
            break


def main():
    parser = argparse.ArgumentParser(description="JetCobot1 통합 런처")
    parser.add_argument("--flip-x", action="store_true", help="J1 회전 방향 반전")
    parser.add_argument("--stream", action="store_true", help="MJPEG 스트리밍 서버 시작")
    parser.add_argument("--voice-server", action="store_true",
                         help="음성 명령 수신 서버(포트 8011) 백그라운드 실행")
    parser.add_argument("--voice-port", type=int, default=8011,
                         help="음성 명령 수신 서버 포트 (기본 8011)")
    args = parser.parse_args()

    stop_event = threading.Event()
    inbound_trigger = threading.Event()  # 입고 트리거 (I 키로 set)

    # ── ROS2 초기화 ───────────────────────────────────────────────
    rclpy.init()
    wt = WaitingTable(place_angles_list=PLACE_ANGLES_LIST)

    # ROS2 spin 백그라운드 스레드
    spin_thread = threading.Thread(target=rclpy.spin, args=(wt,), daemon=True)
    spin_thread.start()
    print("[main] ROS2 WaitingTable 노드 백그라운드 실행 중...")

    # ── 로봇 + 카메라 초기화 ─────────────────────────────────────
    print("[main] 로봇팔 초기화 중...")
    setup = CameraSetup()
    setup.prepare()

    tracker = BoxTracker()
    tracker.start()

    # 키보드 입력 백그라운드 스레드 (tracker 준비된 후 시작 — I 키가 activate() 호출하므로)
    kb_thread = threading.Thread(
        target=_keyboard_listener, args=(wt, stop_event, inbound_trigger, tracker), daemon=True
    )
    kb_thread.start()

    if args.stream:
        StreamServer(tracker).start()
        print("[main] MJPEG 스트리밍 서버 시작됨")

    if args.voice_server:
        from voice_command_server import start_voice_server_in_background
        start_voice_server_in_background(port=args.voice_port)
        print(f"[main] 음성 명령 수신 서버 시작됨 (포트 {args.voice_port})")

    # ── PickAndPlace 생성 + WaitingTable 연동 ────────────────────
    pnp = PickAndPlace(mc=setup.mc, tracker=tracker, flip_x=args.flip_x)
    pnp.set_waiting_table(wt)   # pick_and_place → waiting_table 알림
    wt.set_pick_and_place(pnp)  # waiting_table → 실제 로봇 출고 동작
    print("[main] PickAndPlace ↔ WaitingTable 연동 완료")

    # ── 메인 루프 ────────────────────────────────────────────────
    print("\n[main] 시작! 박스를 컨베이어에 올려주세요.")
    print(f"[main] 대기 슬롯 {PLACE_MAX_COUNT}개 순환 (0 → 1 → 2 → 0 ...)")
    print("[main] 키보드: I=입고시작  S=테이블현황  P=핑키상태  R=일시정지/재시작  Q=종료\n")

    try:
        while not stop_event.is_set():
            # 관리자 일시정지 (비상정지 아님 — 다음 동작 시작 전에만 막음)
            if is_paused():
                reason = get_pause_reason()
                print(f"[main] ⏸ 일시정지 중{f' ({reason})' if reason else ''}...", end='\r')
                time.sleep(0.3)
                continue

            # 출고 중이면 대기 (로봇 제어 충돌 방지)
            if wt._is_loading:
                print("[main] 출고 중... pick & place 대기")
                time.sleep(1.0)
                continue

            # align_stop이 set된 상태면 핑키 트리거가 들어온 것
            # pnp.run()이 중단됐으니 출고 처리될 때까지 대기
            if wt._align_stop.is_set():
                print("[main] 핑키 트리거 감지 — 출고 대기 중...", end='\r')
                time.sleep(0.5)
                continue

            # 입고 트리거가 없으면 카메라 비활성 상태이므로 박스 감지 자체를 시도하지 않음
            if not inbound_trigger.is_set():
                print("[main] 입고 대기 중 (I 키로 시작)...", end='\r')
                time.sleep(0.5)
                continue

            # 카메라에 박스가 감지될 때까지 대기
            box = tracker.get_best_box()
            if box is None:
                print("[main] 박스 감지 대기 중...", end='\r')
                time.sleep(0.5)
                continue

            slot_id = pnp._place_count % PLACE_MAX_COUNT
            print(f"\n{'='*50}")
            print(f"[main] 박스 감지! 슬롯 {slot_id}에 적재 시작 (완료: {pnp._place_count}개)")
            print(f"{'='*50}")

            success = pnp.run()

            if stop_event.is_set():
                break

            # pnp.run() 완료 후 align_stop 초기화
            wt._align_stop.clear()

            # 입고 사이클 종료 (성공/실패 무관) — 트리거 해제 + 카메라 비활성화
            inbound_trigger.clear()
            tracker.deactivate()

            if success:
                print(f"[main] ✅ 완료 — 슬롯 {slot_id} 적재, aging 시작")
            else:
                print("[main] ❌ 실패 — 박스 재감지 대기 중...")
                setup.mc.send_angles(list(ANGLES_CAMERA_DOWN), ROBOT_SPEED)
                time.sleep(2.5)

    except KeyboardInterrupt:
        print("\n[main] 중단됨 (Ctrl+C)")

    finally:
        stop_event.set()
        tracker.stop()
        wt.destroy_node()
        rclpy.shutdown()
        print("[main] 종료")


if __name__ == "__main__":
    main()