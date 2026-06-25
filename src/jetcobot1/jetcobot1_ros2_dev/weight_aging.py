#!/usr/bin/env python3
"""
weight_aging.py - 대기 테이블 관리 + Priority Level Aging

동작 흐름:
  1. pick_and_place.py의 _place() 완료 후 on_box_placed(slot_id) 호출
  2. 각 박스는 priority_level=1로 시작
  3. AGING_INTERVAL(10초)마다 priority_level +1 (최대 MAX_PRIORITY=5)
  4. 핑키 도착 신호(simulate_pinky_arrive) 수신 시
     → priority_level 높은 순 (동점이면 FIFO) 으로 출고 큐 정렬
     → 슬롯 각도로 이동 → 카메라 디텍팅 → 박스 집기 → 제자리에 놓기
  5. 출고 완료 후 슬롯 해제

TODO (실제 연동 시 교체):
  - simulate_pinky_arrive() → Task Manager ROS2 액션 수신
  - _notify_pinky_depart()  → ROS2 액션으로 PinkyPro 출발 신호

연동 방법 (main.py):
  wt  = WaitingTable(place_angles_list=PLACE_ANGLES_LIST)
  pnp = PickAndPlace(mc, tracker)
  pnp.set_waiting_table(wt)
  wt.set_pick_and_place(pnp)    # ← 출고 시 실제 로봇 동작 연동
"""

import time
import threading
import rclpy
from rclpy.node import Node

from pause_control import wait_if_paused

try:
    from pick_and_place import PLACE_ANGLES_LIST, PLACE_J1_SPEED
    _USE_PLACE_ANGLES = True
except ImportError:
    _USE_PLACE_ANGLES = False


# ── 설정 ──────────────────────────────────────────────────────────
AGING_INTERVAL      = 10.0    # 초: priority_level 상승 주기
MAX_PRIORITY        = 5       # priority_level 최댓값
PINKY_SIM_DELAY     = 99999.0 # 자동 시뮬 비활성화 (main.py 키보드로 트리거)
NUM_SLOTS           = 3

# 대기 테이블 높이 보정 (mm)
TABLE_HEIGHT_OFFSET = 0.0

# 핑키별 적재 각도
# 핑키 도착 시 박스를 집어 이 각도로 이동 후 그리퍼 열기
PINKY_LOAD_ANGLES = {
    "pinky1": [38.23,  -29.35, -12.48, -31.90, -14.67,  -6.32],
    "pinky2": [ 0.08,  -31.20, -12.39, -25.66, -15.29, -50.71],
}


# ── 박스 정보 ─────────────────────────────────────────────────────
class BoxInfo:
    def __init__(self, box_id: int, slot_id: int, arrived_at: float):
        self.id             = box_id
        self.slot_id        = slot_id
        self.arrived_at     = arrived_at
        self.priority_level = 1
        self.visible        = True
        self.pinky_id       = None   # 출고 시 할당될 핑키 ID

    def wait_sec(self) -> float:
        return time.time() - self.arrived_at

    def __repr__(self):
        return (
            f"Box(id={self.id}, slot={self.slot_id}, "
            f"priority={self.priority_level}, wait={self.wait_sec():.1f}s)"
        )


# ── 대기 테이블 노드 ──────────────────────────────────────────────
class WaitingTable(Node):

    def __init__(self, place_angles_list=None):
        super().__init__('waiting_table')

        self._boxes: dict[int, BoxInfo] = {}
        self._slot_occupied: dict[int, int | None] = {
            i: None for i in range(NUM_SLOTS)
        }
        self._next_id     = 0
        self._is_loading  = False
        self._pnp         = None
        self._align_stop  = threading.Event()
        self._pinky_queue: list[str] = []   # 핑키 도착 순서 큐

        # 핑키 상태 관리
        # idle    : 대기 중 (출발 가능)
        # moving  : 이동 중 (출발 명령 전송 후 도착 전)
        # loading : 적재 중 (도착 후 적재 완료 전)
        self._pinky_status = {
            "pinky1": "idle",
            "pinky2": "idle",
        }
        self._pinky_order = ["pinky1", "pinky2"]  # 빈 핑키 없을 때 순서

        self._place_angles_list = place_angles_list or (
            PLACE_ANGLES_LIST if _USE_PLACE_ANGLES else None
        )

        self.create_timer(AGING_INTERVAL, self._age_all_boxes)

        self.get_logger().info(
            f'WaitingTable 시작 | 슬롯={NUM_SLOTS}개 | '
            f'Aging={AGING_INTERVAL}s 주기 | MAX_PRIORITY={MAX_PRIORITY}'
        )
        if self._place_angles_list:
            self.get_logger().info(f'PLACE_ANGLES_LIST 연동: {NUM_SLOTS}개 슬롯 확인')

    # ── 연동 주입 ────────────────────────────────────────────────
    def set_pick_and_place(self, pnp):
        """
        PickAndPlace 인스턴스 주입.
        출고 시 실제 로봇 동작에 사용.
        main.py에서 호출:
            wt.set_pick_and_place(pnp)
        """
        self._pnp = pnp
        self.get_logger().info('PickAndPlace 연동 완료 → 실제 로봇 출고 동작 활성화')

    # ── one-shot 타이머 헬퍼 ────────────────────────────────────
    def _once(self, delay: float, callback):
        timer = None
        def _cb():
            timer.cancel()
            callback()
        timer = self.create_timer(delay, _cb)
        return timer

    # ── 박스 입고 ────────────────────────────────────────────────
    def on_box_placed(self, slot_id: int):
        if self._slot_occupied.get(slot_id) is not None:
            self.get_logger().warn(
                f'슬롯 {slot_id}에 이미 박스 있음! '
                f'(box_id={self._slot_occupied[slot_id]})'
            )
            return

        box_id = self._next_id
        self._next_id += 1
        box = BoxInfo(box_id, slot_id, time.time())
        self._boxes[box_id]          = box
        self._slot_occupied[slot_id] = box_id

        angles_info = self._angles_info(slot_id)
        self.get_logger().info(
            f'[입고] 박스 {box_id} → 슬롯 {slot_id}{angles_info} | '
            f'priority_level={box.priority_level}'
        )
        self._log_table()

        # 적재 완료 → 빈 핑키에 자동 출발 명령
        self._dispatch_pinky()

    # ── 핑키 자동 디스패치 ────────────────────────────────────────
    def _dispatch_pinky(self):
        """
        빈 핑키(idle)를 찾아 출발 명령 전송.
        둘 다 idle이면 _pinky_order 순서대로 선택.
        둘 다 busy이면 대기 (도착 시 자동 처리).
        TODO: 실제 Task Manager ROS2 액션으로 교체
        """
        idle_pinkies = [
            p for p in self._pinky_order
            if self._pinky_status[p] == "idle"
        ]

        if not idle_pinkies:
            self.get_logger().info(
                f'모든 핑키 작업 중 '
                f'({self._pinky_status}) — 완료 후 자동 처리'
            )
            return

        # 빈 핑키 중 순서 앞인 것 선택
        pinky_id = idle_pinkies[0]
        self._pinky_status[pinky_id] = "moving"
        self.get_logger().info(
            f'[디스패치] {pinky_id} 출발 명령 전송 '
            f'(상태: {self._pinky_status})'
        )
        # TODO: Task Manager ROS2 액션으로 핑키 출발 명령 전송
        # 테스트용: 출발 명령 후 바로 도착 트리거 (실제로는 핑키 이동 시간 있음)
        self._on_pinky_arrive(pinky_id)

    def _on_pinky_arrive(self, pinky_id: str):
        """
        핑키 도착 처리.
        TODO: Task Manager에서 도착 신호 수신으로 교체
        """
        self._pinky_status[pinky_id] = "loading"
        self.get_logger().info(
            f'[도착] {pinky_id} 도착 확인 '
            f'(상태: {self._pinky_status})'
        )
        self.simulate_pinky_arrive(pinky_id)

    # ── Aging ────────────────────────────────────────────────────
    def _age_all_boxes(self):
        """AGING_INTERVAL마다 호출. 대기 중인 박스의 priority_level을 올린다."""
        aged_ids = []
        for box in self._boxes.values():
            if box.visible and box.priority_level < MAX_PRIORITY:
                box.priority_level += 1
                aged_ids.append(box.id)

        if aged_ids:
            self.get_logger().info(f'[Aging] priority_level 상승: 박스 {aged_ids}')
            self._log_table()

    # ── 핑키 도착 처리 ───────────────────────────────────────────
    def simulate_pinky_arrive(self, pinky_id: str = "pinky1"):
        """
        핑키 도착 트리거.
        핑키 큐에 추가 후 처리 중이 아니면 즉시 시작.
        TODO: Task Manager ROS2 액션 수신으로 교체

        Args:
            pinky_id: 핑키 식별자 ("pinky1" or "pinky2")
        """
        self._pinky_queue.append(pinky_id)
        self.get_logger().info(
            f'=== {pinky_id} 도착! 큐 등록 '
            f'(대기 {len(self._pinky_queue)}번째) ==='
        )
        self._log_pinky_queue()

        # 현재 처리 중이 아니면 바로 시작
        if not self._is_loading:
            self._process_next_pinky()

    def _log_pinky_queue(self):
        if self._pinky_queue:
            self.get_logger().info(
                f'── 핑키 대기 큐: {self._pinky_queue}'
            )

    def _process_next_pinky(self):
        """핑키 큐에서 다음 핑키를 꺼내 출고 시작."""
        if not self._pinky_queue:
            self.get_logger().info('핑키 큐 비어있음.')
            return

        pinky_id = self._pinky_queue.pop(0)
        self.get_logger().info(
            f'=== {pinky_id} 출고 시작 '
            f'(남은 큐: {self._pinky_queue}) ==='
        )
        self._start_loading(pinky_id)

    def _start_loading(self, pinky_id: str = "pinky1"):
        visible = [b for b in self._boxes.values() if b.visible]
        if not visible:
            self.get_logger().info(f'[{pinky_id}] 대기 중인 박스 없음.')
            # 다음 핑키 처리 시도
            self._process_next_pinky()
            return

        # priority_level 높은 순 → 동점이면 FIFO
        queue = sorted(visible, key=lambda b: (-b.priority_level, b.arrived_at))

        self.get_logger().info(f'── [{pinky_id}] 출고 큐 ──')
        for rank, box in enumerate(queue, 1):
            self.get_logger().info(
                f'  {rank}번: 박스 {box.id} | 슬롯 {box.slot_id}'
                f'{self._angles_info(box.slot_id)} | '
                f'priority_level={box.priority_level} | '
                f'대기 {box.wait_sec():.1f}s'
            )

        self._is_loading = True
        self._align_stop.clear()

        threading.Thread(
            target=self._process_queue_thread,
            args=(queue, pinky_id),
            daemon=True
        ).start()

    def _process_queue_thread(self, queue: list, pinky_id: str):
        """핑키 도착 1회당 priority 가장 높은 박스 1개만 처리."""
        if not queue:
            self._is_loading = False
            self._process_next_pinky()
            return

        wait_if_paused()

        box = queue[0]
        box.pinky_id = pinky_id   # 출고 핑키 ID 할당
        self.get_logger().info(
            f'[{pinky_id}] 출고 박스 {box.id} | '
            f'슬롯 {box.slot_id}{self._angles_info(box.slot_id)} | '
            f'priority_level={box.priority_level}'
        )

        success = self._pick_from_slot(box)

        if success:
            box.visible = False
            self._slot_occupied[box.slot_id] = None
            self.get_logger().info(
                f'  [{pinky_id}] 박스 {box.id} 출고 완료 → 슬롯 {box.slot_id} 해제'
            )
        else:
            self.get_logger().warn(
                f'  [{pinky_id}] 박스 {box.id} 출고 실패 → 슬롯 {box.slot_id} 유지'
            )

        self._is_loading = False
        self.get_logger().info(f'=== [{pinky_id}] 출고 완료 ===')

        # 출고 완료 후 카메라 하향 자세로 복귀
        if self._pnp is not None:
            try:
                from box_tracker import ANGLES_CAMERA_DOWN, ROBOT_SPEED
                self._pnp.mc.send_angles(list(ANGLES_CAMERA_DOWN), ROBOT_SPEED)
                time.sleep(2.5)
                self.get_logger().info('카메라 하향 자세 복귀 완료 → 입고 재개 가능')
            except Exception as e:
                self.get_logger().error(f'복귀 중 오류: {e}')

        self._notify_pinky_depart(pinky_id)

        # 핑키 상태 idle로 복귀
        self._pinky_status[pinky_id] = "idle"
        self.get_logger().info(
            f'[{pinky_id}] 상태 idle 복귀 (상태: {self._pinky_status})'
        )

        # align_stop 초기화 → 메인 루프 입고 재개
        self._align_stop.clear()
        self.get_logger().info('입고 재개 가능 → align_stop 초기화')

        # 다음 핑키 처리 (큐에 대기 중인 핑키 있으면)
        self._process_next_pinky()

        # 대기 중인 박스 있으면 빈 핑키로 자동 디스패치
        visible = [b for b in self._boxes.values() if b.visible]
        if visible:
            self._dispatch_pinky()

    def _pick_from_slot(self, box: BoxInfo) -> bool:
        """
        대기 슬롯에서 박스를 집어 핑키 적재 위치에 내려놓는다.
        PLACE_ANGLES_LIST[slot_id] 각도로 x/y/z가 모두 결정되므로 Z 계산은 불필요.
        """
        if self._pnp is None:
            self.get_logger().warn('PickAndPlace 미연동 — 로봇 동작 생략')
            return False

        mc = self._pnp.mc

        try:
            from pick_and_place import (
                ANGLES_ZERO, PLACE_J1_SPEED,
                _wait_move, _gripper_open, _gripper_close,
                GRIPPER_CLOSE_FALLBACK,
            )

            target_angles = list(self._place_angles_list[box.slot_id])

            # ── 1) ANGLES_ZERO 경유 ───────────────────────────
            wait_if_paused()
            self.get_logger().info('  ANGLES_ZERO 경유 중...')
            mc.send_angles(list(ANGLES_ZERO), PLACE_J1_SPEED)
            _wait_move(mc, timeout=8.0)
            time.sleep(0.3)

            # ── 2) J1 먼저 선회 ───────────────────────────────
            j1_only    = list(ANGLES_ZERO)
            j1_only[0] = target_angles[0]
            mc.send_angles(j1_only, PLACE_J1_SPEED)
            _wait_move(mc, timeout=8.0)
            time.sleep(0.3)

            # ── 3) 슬롯 각도로 이동 ───────────────────────────
            wait_if_paused()
            self.get_logger().info(
                f'  슬롯 {box.slot_id} 이동 (J1={target_angles[0]}°)'
            )
            _gripper_open(mc)
            mc.send_angles(target_angles, PLACE_J1_SPEED)
            _wait_move(mc, timeout=8.0)
            time.sleep(0.5)

            # ── 4) 그리퍼 닫기 ────────────────────────────────
            self.get_logger().info('  그리퍼 닫기 (박스 집기)')
            _gripper_close(mc, value=GRIPPER_CLOSE_FALLBACK)

            # ── 5) 핑키 적재 위치로 이동 후 그리퍼 열기 ─────
            wait_if_paused()
            pinky_angles = PINKY_LOAD_ANGLES.get(box.pinky_id)
            if pinky_angles is None:
                self.get_logger().warn(
                    f'  {box.pinky_id} 적재 각도 미정의 → ANGLES_ZERO 복귀'
                )
                mc.send_angles(list(ANGLES_ZERO), PLACE_J1_SPEED)
                _wait_move(mc, timeout=8.0)
            else:
                self.get_logger().info(
                    f'  {box.pinky_id} 적재 위치로 이동: {pinky_angles}'
                )
                # ANGLES_ZERO 경유 후 핑키 적재 위치로
                mc.send_angles(list(ANGLES_ZERO), PLACE_J1_SPEED)
                _wait_move(mc, timeout=8.0)
                time.sleep(0.3)

                j1_only    = list(ANGLES_ZERO)
                j1_only[0] = pinky_angles[0]
                mc.send_angles(j1_only, PLACE_J1_SPEED)
                _wait_move(mc, timeout=8.0)
                time.sleep(0.3)

                mc.send_angles(list(pinky_angles), PLACE_J1_SPEED)
                _wait_move(mc, timeout=8.0)
                time.sleep(0.3)

            _gripper_open(mc)
            self.get_logger().info(f'  {box.pinky_id} 적재 완료!')

            # ── 6) ANGLES_ZERO 복귀 ───────────────────────────
            mc.send_angles(list(ANGLES_ZERO), PLACE_J1_SPEED)
            _wait_move(mc, timeout=8.0)

            return True

        except Exception as e:
            self.get_logger().error(f'  출고 중 오류: {e}')
            return False

    # ── stub ────────────────────────────────────────────────────
    def _notify_pinky_depart(self, pinky_id: str = "pinky1"):
        """TODO: ROS2 액션으로 PinkyPro 출발 신호"""
        self.get_logger().info(f'[stub] {pinky_id} 출발 신호 전송 (ROS2 액션 미연동)')

    # ── 유틸 ────────────────────────────────────────────────────
    def get_free_slot(self) -> int | None:
        """
        비어있는 슬롯 중 가장 작은 번호를 반환.
        모든 슬롯이 꽉 찼으면 None 반환.
        pick_and_place.py의 run()에서 다음 입고 슬롯 결정에 사용.
        """
        for slot_id in range(NUM_SLOTS):
            if self._slot_occupied[slot_id] is None:
                return slot_id
        return None
    def _angles_info(self, slot_id: int) -> str:
        if self._place_angles_list and slot_id < len(self._place_angles_list):
            return f" | J1={self._place_angles_list[slot_id][0]}°"
        return ""

    def _log_table(self):
        self.get_logger().info('── 대기 테이블 현황 ──')
        for slot_id in range(NUM_SLOTS):
            box_id = self._slot_occupied[slot_id]
            if box_id is None:
                self.get_logger().info(
                    f'  슬롯 {slot_id}{self._angles_info(slot_id)}: 비어있음'
                )
            else:
                box = self._boxes[box_id]
                self.get_logger().info(
                    f'  슬롯 {slot_id}{self._angles_info(slot_id)}: '
                    f'박스 {box.id} | priority_level={box.priority_level} | '
                    f'대기 {box.wait_sec():.1f}s'
                )


# ────────────────────────────────────────────────────────────────
# 단독 실행
# ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    try:
        from pick_and_place import PLACE_ANGLES_LIST as pal
    except ImportError:
        pal = None

    node = WaitingTable(place_angles_list=pal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
