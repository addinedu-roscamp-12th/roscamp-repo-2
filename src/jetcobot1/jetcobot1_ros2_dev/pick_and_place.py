"""
픽앤플레이스 모듈 — Z 자동 계산 + 그리퍼 크기 자동 조절

weight_aging.py 연동: PLACE_ANGLES_LIST 인덱스(0~2) = 대기 슬롯 ID.
  _place() 완료 후 waiting_table.on_box_placed(slot_id) 호출 (set_waiting_table로 주입).

픽업 실패(공잡이) 시: 관리자 대기 후 재시도, PICK_RETRY_MAX 초과 시 PickupFailedError.
플레이스 후 낙하 감지: 슬롯에서 박스 미감지 시 BoxDropError.
"""

import time
import math
import logging
import argparse
import statistics

from pymycobot.mycobot280 import MyCobot280
from box_tracker import (
    BoxTracker,
    BoxInfo,
    CameraSetup,
    GripperAligner,
    StreamServer,
    ROBOT_SPEED,
    ANGLES_ZERO,
    ANGLES_CAMERA_DOWN,
)
from pause_control import wait_if_paused

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("jetcobot1.pick_and_place")


# ────────────────────────────────────────────────────────────────
# 커스텀 예외
# ────────────────────────────────────────────────────────────────
class PickupFailedError(Exception):
    """픽업을 PICK_RETRY_MAX회 시도했으나 모두 실패했을 때 발생."""
    pass


class SlotFullError(Exception):
    """모든 대기 슬롯이 꽉 찼을 때 발생."""
    pass


class BoxDropError(Exception):
    """플레이스 후 슬롯에서 박스가 감지되지 않을 때 발생 (낙하 추정)."""
    pass


class RobotMoveError(Exception):
    """좌표/각도 읽기 또는 이동 실패 시 발생."""
    pass


# ────────────────────────────────────────────────────────────────
# Z 자동 계산 캘리브레이션 값
# ────────────────────────────────────────────────────────────────
REF_AREA        = 54989.5
REF_DISTANCE    = 140.0
FOCAL_LENGTH_PX = 783.9
GRIPPER_OFFSET  = 25.0
APPROACH_MARGIN = 100.0
CAMERA_GRIPPER_Y_OFFSET  = 30

# ────────────────────────────────────────────────────────────────
# 플레이스 설정
# ※ 인덱스 0,1,2 가 weight_aging.py 의 슬롯 ID 0,1,2 와 1:1 매핑
# ────────────────────────────────────────────────────────────────
PLACE_J1_SPEED = 20

PLACE_ANGLES_LIST = [
    [125.0, -27.68, -41.66, -16.69, -1.23,  -3.69],   # 슬롯 0
    [100.0, -32.25, -41.57, -12.04, -1.14, -27.86],   # 슬롯 1
    [ 75.0, -44.20, -21.44, -15.73, -5.53, -56.95],   # 슬롯 2
]
PLACE_MAX_COUNT = len(PLACE_ANGLES_LIST)  # 3

# 슬롯별 박스 감지용 각도 (slot_calib.py로 실측한 값)
# 플레이스 완료 후 이 자세로 이동 → 박스 감지 → 홈 복귀
SLOT_CHECK_ANGLES_LIST = [
    [114.78, -20.00, -2.63, -50.53, -1.40, -18.01],   # 슬롯 0
    [ 88.36, -20.00, -2.63, -37.00, -4.39, -35.68],   # 슬롯 1
    [ 50.25, -20.00, -2.63, -34.45,  6.67, -37.52],   # 슬롯 2
]

# ────────────────────────────────────────────────────────────────
# 그리퍼
# ────────────────────────────────────────────────────────────────
GRIPPER_OPEN           = 100
GRIPPER_SPEED          = 80
GRIPPER_PHYS_MIN_MM    = 20.0
GRIPPER_PHYS_MAX_MM    = 60.0
GRIPPER_VALUE_MIN      = 10
GRIPPER_VALUE_MAX      = 60
GRIPPER_MARGIN_MM      = 5.0
GRIPPER_CLOSE_FALLBACK = 15

# ────────────────────────────────────────────────────────────────
# 이동 속도
# ────────────────────────────────────────────────────────────────
MOVE_SPEED    = 70
DESCEND_SPEED = 30

# 플레이스 후 J2 상승 각도 (도)
# 박스 내려놓은 후 J2를 음수 방향으로 조정해 팔을 위로 들어올림
# 박스를 건드리면 값을 키우고, 충분하면 줄여서 조정
PLACE_LIFT_J2 = -20.0

# 박스 간 충돌 방지용 Z 리프트 높이 (mm)
LIFT_HEIGHT = 50.0

# ────────────────────────────────────────────────────────────────
# 픽업 실패 감지 설정
# ────────────────────────────────────────────────────────────────
PICK_RETRY_MAX      = 3     # 최대 재시도 횟수 (초과 시 PickupFailedError)
PICK_GRIP_MARGIN    = 5     # 공잡이 판정 마진 (value 단위, 판정 원리는 _check_grip_success 참고)
PICK_GRIP_READ_WAIT = 0.5   # 그리퍼 닫힌 후 값 읽기 전 대기 (초)

# ────────────────────────────────────────────────────────────────
# 플레이스 낙하 감지 설정
# ────────────────────────────────────────────────────────────────
BOX_DROP_CHECK_WAIT    = 1.0  # 플레이스 후 감지 전 대기 (초) — 박스 안착 기다림
BOX_DROP_CHECK_SAMPLES = 5    # 감지 시도 횟수 (한 번이라도 감지되면 성공)


# ────────────────────────────────────────────────────────────────
# Z 추정
# ────────────────────────────────────────────────────────────────
def estimate_z_distance(area: float) -> float:
    distance = REF_DISTANCE * math.sqrt(REF_AREA / area)
    if area > REF_AREA:
        distance += (area - REF_AREA) / REF_AREA * 67.0
    elif area < REF_AREA:
        distance -= (REF_AREA - area) / REF_AREA * 120.0
    logger.info(f"Z 추정: area={area:.0f}px² distance={distance:.1f}mm")
    return distance


def estimate_gripper_close_value(pixel_width: float, z_distance: float) -> int:
    if pixel_width is None or pixel_width <= 0 or z_distance <= 0:
        return GRIPPER_CLOSE_FALLBACK
    real_width_mm = (pixel_width * z_distance) / FOCAL_LENGTH_PX
    target_mm = real_width_mm - GRIPPER_MARGIN_MM
    t = (target_mm - GRIPPER_PHYS_MIN_MM) / (GRIPPER_PHYS_MAX_MM - GRIPPER_PHYS_MIN_MM)
    t = max(0.0, min(1.0, t))
    value = int(GRIPPER_VALUE_MIN + t * (GRIPPER_VALUE_MAX - GRIPPER_VALUE_MIN))
    logger.info(f"그리퍼 닫힘 값: {value}")
    return value


def get_stable_box_info(tracker, n_samples=15):
    areas, widths = [], []
    print(f"  박스 정보 측정 중 ({n_samples}프레임)...", end="", flush=True)
    for _ in range(n_samples):
        box = tracker.get_best_box()
        if box is not None:
            areas.append(box.area)
            w = box.width if box.width is not None else math.sqrt(box.area)
            widths.append(w)
        time.sleep(0.05)
        print(".", end="", flush=True)
    print()
    if len(areas) < n_samples // 2:
        return None, None
    return statistics.median(areas), statistics.median(widths)


# ────────────────────────────────────────────────────────────────
# 유틸
# ────────────────────────────────────────────────────────────────
def _wait_move(mc, timeout=6.0, poll=0.1):
    start = time.time()
    while time.time() - start < timeout:
        if mc.is_moving() == 0:
            return
        time.sleep(poll)
    logger.warning("이동 타임아웃")


def _get_coords(mc, retries=5):
    for i in range(retries):
        coords = mc.get_coords()
        if coords and len(coords) >= 6:
            return coords
        logger.warning(f"좌표 읽기 실패 ({i+1}/{retries})")
        time.sleep(0.3)
    return None


def _send_coords(mc, x, y, z, speed=MOVE_SPEED):
    coords = _get_coords(mc)
    if coords is None:
        raise RobotMoveError("좌표 읽기 실패 — 이동 불가")
    mc.send_coords([x, y, z, coords[3], coords[4], coords[5]], speed, 0)


def _gripper_open(mc):
    mc.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
    time.sleep(0.8)


def _gripper_close(mc, value=GRIPPER_CLOSE_FALLBACK):
    clamped = max(GRIPPER_VALUE_MIN, min(GRIPPER_VALUE_MAX, value))
    mc.set_gripper_value(clamped, GRIPPER_SPEED)
    time.sleep(0.8)


def _check_grip_success(mc: MyCobot280, target_value: int) -> bool:
    """
    그리퍼 닫힘 후 실제 값을 읽어 박스를 잡았는지 판정한다.

    원리:
      박스 있음 → 그리퍼가 박스에 막혀 target보다 덜 닫힘 (actual 큼)
      박스 없음 → 그리퍼가 완전히 닫혀 target에 근접 (actual 작음)
      → target - actual >= PICK_GRIP_MARGIN 이면 박스 잡은 것

    Returns:
        True  = 박스 잡음
        False = 공잡이 (박스 없음)
    """
    time.sleep(PICK_GRIP_READ_WAIT)
    actual = mc.get_gripper_value()

    if actual is None:
        logger.warning("그리퍼 값 읽기 실패 → 성공으로 간주")
        return True

    diff = actual - target_value
    result = diff >= PICK_GRIP_MARGIN
    logger.info(
        f"그리퍼 감지: target={target_value}  actual={actual}  diff={diff}  "
        f"→ {'✅ 박스 잡음' if result else '❌ 공잡이'}"
    )
    return result


def _check_box_in_slot(mc, tracker, slot_id: int) -> bool:
    """
    플레이스 완료 후 슬롯 감지 자세로 이동해 박스 낙하 여부를 확인하고,
    결과와 무관하게 ANGLES_ZERO 경유로 홈 자세에 복귀한다.

    Returns:
        True  = 슬롯에 박스 있음 (정상)
        False = 박스 없음 (낙하 추정)
    """
    target_angles = list(SLOT_CHECK_ANGLES_LIST[slot_id])
    logger.info(
        f"낙하 감지: 슬롯 {slot_id} 감지 자세로 이동 "
        f"(J1={target_angles[0]}°)"
    )

    # ── 1) 감지 자세로 이동 ───────────────────────────────────
    mc.send_angles(target_angles, PLACE_J1_SPEED)
    _wait_move(mc, timeout=8.0)
    time.sleep(BOX_DROP_CHECK_WAIT)

    # ── 2) 박스 감지 시도 ─────────────────────────────────────
    detected = False
    for i in range(BOX_DROP_CHECK_SAMPLES):
        box = tracker.get_best_box()
        if box is not None:
            logger.info(
                f"낙하 감지: 슬롯 {slot_id} 박스 확인 ✅ "
                f"(시도 {i+1}/{BOX_DROP_CHECK_SAMPLES}  area={box.area:.0f}px²)"
            )
            detected = True
            break
        time.sleep(0.1)

    if not detected:
        logger.warning(f"낙하 감지: 슬롯 {slot_id} 박스 없음 ❌ — 낙하 추정")

    # ── 3) 홈 복귀 (감지 결과 무관하게 항상 복귀) ────────────
    logger.info("감지 완료 → ANGLES_ZERO 경유 → 홈 복귀")
    j1_only    = list(ANGLES_ZERO)
    j1_only[0] = target_angles[0]
    mc.send_angles(j1_only, PLACE_J1_SPEED)
    _wait_move(mc, timeout=8.0)
    time.sleep(0.3)

    mc.send_angles(list(ANGLES_ZERO), PLACE_J1_SPEED)
    _wait_move(mc, timeout=8.0)

    return detected


# ────────────────────────────────────────────────────────────────
# 메인 클래스
# ────────────────────────────────────────────────────────────────
class PickAndPlace:
    """
    중앙 정렬 → Z 자동 계산 → 픽업 → 대기 슬롯에 플레이스.

    공잡이 감지 시 홈 복귀 후 관리자 대기(Enter로 재시도),
    PICK_RETRY_MAX회 모두 실패 시 PickupFailedError.
    set_waiting_table(wt)로 WaitingTable을 주입하면 _place() 완료 후
    wt.on_box_placed(slot_id)가 자동 호출된다.
    """

    def __init__(self, mc, tracker, flip_x=False):
        self.mc             = mc
        self.tracker        = tracker
        self._aligner       = GripperAligner(mc, tracker, flip_x=flip_x)
        self._place_count   = 0
        self._waiting_table = None

    def set_waiting_table(self, waiting_table):
        self._waiting_table = waiting_table
        logger.info("WaitingTable 연동 완료")

    def run(self, align_timeout=30.0) -> bool:
        """
        전체 픽앤플레이스 시퀀스. 성공 시 True.

        발생 가능 예외:
          PickupFailedError : 3회 재시도 모두 공잡이
          SlotFullError     : 대기 슬롯 꽉 참
          RobotMoveError    : 좌표 읽기/이동 실패
        """
        try:
            # ── Step 1: 중앙 정렬 ─────────────────────────────
            wait_if_paused()
            self._print_step(1, 5, "박스 중앙 정렬 중...")
            if not self._aligner.align(timeout=align_timeout):
                print("\n[오류] 정렬 실패 — 박스를 카메라 시야 내에 놓아주세요.")
                return False
            print("\n  정렬 완료!")

            # ── Step 2: 좌표 + Z + 그리퍼 값 계산 ────────────
            wait_if_paused()
            self._print_step(2, 5, "픽업 좌표 및 Z / 그리퍼 값 계산 중...")
            coords = _get_coords(self.mc)
            if coords is None:
                raise RobotMoveError("좌표 읽기 실패")
            pick_x, pick_y, current_z = coords[0], coords[1], coords[2]

            area, pixel_width = get_stable_box_info(self.tracker)
            if area is None:
                print("[오류] 박스 정보 측정 실패")
                return False

            box = self.tracker.get_best_box()
            box_angle = box.angle if (box and box.angle is not None) else 0.0

            z_distance  = estimate_z_distance(area)
            grasp_z     = current_z - z_distance - GRIPPER_OFFSET
            approach_z  = grasp_z + APPROACH_MARGIN
            gripper_val = estimate_gripper_close_value(pixel_width, z_distance)
            real_w_mm   = (pixel_width * z_distance) / FOCAL_LENGTH_PX

            print(f"\n  픽업 위치   : x={pick_x:.1f}  y={pick_y:.1f}")
            print(f"  파지 Z      : {grasp_z:.1f}mm  /  접근 Z: {approach_z:.1f}mm")
            print(f"  박스 실폭   : {real_w_mm:.1f}mm  /  그리퍼: {gripper_val}")
            print(f"  박스 각도   : {box_angle:.1f}°")

            # ── Step 3: 픽업 (공잡이 시 관리자 개입 + 재시도) ─
            wait_if_paused()
            self._print_step(3, 5, "박스 픽업 중...")
            self._pick_with_retry(pick_x, pick_y, grasp_z, approach_z, gripper_val, box_angle)
            print("  픽업 완료!")

            # ── Step 4: 플레이스 → weight_aging 알림 ─────────
            if self._waiting_table is not None:
                slot_id = self._waiting_table.get_free_slot()
                if slot_id is None:
                    raise SlotFullError("모든 대기 슬롯이 꽉 찼습니다!")
            else:
                slot_id = self._place_count % PLACE_MAX_COUNT

            self._print_step(4, 5, f"대기 슬롯 {slot_id}에 플레이스 중...")
            wait_if_paused()
            self._place(slot_id)
            print(f"  플레이스 완료! (슬롯 {slot_id})")

            # ── 낙하 감지: 슬롯에 박스가 있는지 확인 ────────
            # 감지 자세 이동 → 박스 확인 → ANGLES_ZERO → 홈 복귀까지 내부 처리
            print("  슬롯 박스 감지 중...")
            if not _check_box_in_slot(self.mc, self.tracker, slot_id):
                raise BoxDropError(
                    f"슬롯 {slot_id}에 박스가 감지되지 않습니다 — 낙하 추정"
                )
            print(f"  슬롯 {slot_id} 박스 확인 완료! ✅")

            if self._waiting_table is not None:
                self._waiting_table.on_box_placed(slot_id)
                logger.info(f"WaitingTable.on_box_placed({slot_id}) 호출")
            else:
                logger.warning("WaitingTable 미연동 — on_box_placed 호출 생략")

            self._place_count += 1

            # ── Step 5: 홈 복귀 ───────────────────────────────
            # _check_box_in_slot() 내부에서 ANGLES_ZERO → 홈 복귀까지 처리됨
            self._print_step(5, 5, "홈 자세 복귀 중...")
            wait_if_paused()
            self._home()
            print("  홈 복귀 완료!")

            print("\n" + "=" * 50)
            print(f"  픽앤플레이스 성공! (총 {self._place_count}개 완료)")
            print("=" * 50)
            return True

        except BoxDropError as e:
            logger.error(f"[낙하 감지] {e}")
            print(f"\n{'!'*50}")
            print(f"  ⚠️  [경고] 박스 낙하 감지!")
            print(f"  {e}")
            print(f"  박스를 확인하고 필요 시 수동으로 슬롯에 올려주세요.")
            print(f"{'!'*50}")
            self._home()
            return False

        except PickupFailedError as e:
            logger.error(f"[픽업 최종 실패] {e}")
            print(f"\n{'!'*50}")
            print(f"  [픽업 최종 실패] {PICK_RETRY_MAX}회 모두 공잡이")
            print(f"  로봇을 정지합니다. 관리자 확인 후 재시작하세요.")
            print(f"{'!'*50}")
            self._home()
            return False

        except SlotFullError as e:
            logger.error(f"[슬롯 꽉 참] {e}")
            print(f"\n[오류] {e}")
            return False

        except RobotMoveError as e:
            logger.error(f"[이동 오류] {e}")
            print(f"\n[오류] 로봇 이동 실패: {e}")
            self._home()
            return False

        except Exception as e:
            logger.error(f"[예기치 못한 오류] {e}")
            print(f"\n[오류] {e}")
            self._home()
            return False

    # ──────────────────────────────────────────────────────────
    # 픽업 (관리자 재시도 포함)
    # ──────────────────────────────────────────────────────────
    def _pick_with_retry(self, pick_x, pick_y, grasp_z, approach_z,
                         gripper_close_value, box_angle):
        """
        픽업 시도 + 공잡이 감지. 실패 시 그리퍼 열고 홈 복귀 후
        관리자가 박스 위치를 조정하고 Enter를 누르면 재시도한다.
        PICK_RETRY_MAX회 초과 시 PickupFailedError 발생.
        """
        for attempt in range(1, PICK_RETRY_MAX + 1):
            wait_if_paused()
            if attempt > 1:
                print(f"\n  재시도 {attempt}/{PICK_RETRY_MAX}...")

            # 픽업 동작 실행
            self._pick(pick_x, pick_y, grasp_z, approach_z,
                       gripper_close_value, box_angle)

            # 공잡이 여부 판정
            success = _check_grip_success(self.mc, gripper_close_value)
            if success:
                logger.info(f"픽업 성공 (시도 {attempt}/{PICK_RETRY_MAX})")
                return  # 성공 → 정상 종료

            # ── 공잡이 감지 ───────────────────────────────────
            logger.warning(
                f"[공잡이 감지] 박스를 잡지 못했습니다 "
                f"(시도 {attempt}/{PICK_RETRY_MAX})"
            )
            print(f"\n{'!'*50}")
            print(f"  ⚠️  [경고] 공잡이 감지 — 박스를 잡지 못했습니다!")
            print(f"  시도 {attempt}/{PICK_RETRY_MAX}")
            print(f"{'!'*50}")

            if attempt >= PICK_RETRY_MAX:
                # 최대 횟수 초과 → 예외 발생
                raise PickupFailedError(
                    f"{PICK_RETRY_MAX}회 모두 공잡이 — 관리자 확인 필요"
                )

            # ── 홈 복귀 + 관리자 대기 ─────────────────────────
            print("  그리퍼를 열고 홈 자세로 복귀합니다...")
            _gripper_open(self.mc)
            self._home()

            print("\n  📋 박스 위치를 조정해주세요.")
            print(f"  조정 완료 후 Enter를 누르면 재시도합니다. "
                  f"(남은 횟수: {PICK_RETRY_MAX - attempt}회)")
            try:
                input("  >> ")
            except EOFError:
                # 비대화형 환경 (자동화 실행) 에서는 짧게 대기 후 재시도
                logger.warning("비대화형 환경 — 5초 후 자동 재시도")
                time.sleep(5.0)

            # 재시도 전 카메라 하향 자세로 복귀
            print("  카메라 하향 자세로 복귀 중...")
            self.mc.send_angles(list(ANGLES_CAMERA_DOWN), ROBOT_SPEED)
            time.sleep(2.5)

    # ──────────────────────────────────────────────────────────
    # 내부 동작
    # ──────────────────────────────────────────────────────────
    def _pick(self, pick_x, pick_y, grasp_z, approach_z, gripper_close_value, box_angle):
        _send_coords(self.mc, pick_x, pick_y, approach_z, MOVE_SPEED)
        _wait_move(self.mc, timeout=5.0)
        time.sleep(0.3)

        _gripper_open(self.mc)

        gripper_y = pick_y + CAMERA_GRIPPER_Y_OFFSET
        _send_coords(self.mc, pick_x, gripper_y, approach_z, MOVE_SPEED)
        _wait_move(self.mc, timeout=4.0)
        time.sleep(0.2)

        j6_angles = self.mc.get_angles()
        if j6_angles and len(j6_angles) >= 6:
            target_j6 = j6_angles[:]
            target_j6[5] = max(-175.0, min(175.0, j6_angles[5] + box_angle))
            self.mc.send_angles(target_j6, MOVE_SPEED)
            _wait_move(self.mc, timeout=3.0)
            time.sleep(0.3)

        _send_coords(self.mc, pick_x, gripper_y, grasp_z, DESCEND_SPEED)
        _wait_move(self.mc, timeout=6.0)
        time.sleep(0.2)

        _gripper_close(self.mc, value=gripper_close_value)

        _send_coords(self.mc, pick_x, gripper_y, approach_z, DESCEND_SPEED)
        _wait_move(self.mc, timeout=5.0)

    def _place(self, slot_id: int):
        target_angles = list(PLACE_ANGLES_LIST[slot_id])
        logger.info(f"플레이스 → 슬롯 {slot_id}: {target_angles}")

        current_angles = self.mc.get_angles()
        zero_with_j6    = list(ANGLES_ZERO)
        zero_with_j6[5] = current_angles[5]
        self.mc.send_angles(zero_with_j6, PLACE_J1_SPEED)
        _wait_move(self.mc, timeout=8.0)
        time.sleep(0.5)

        j1_only    = list(ANGLES_ZERO)
        j1_only[0] = target_angles[0]
        j1_only[5] = current_angles[5]
        self.mc.send_angles(j1_only, PLACE_J1_SPEED)
        _wait_move(self.mc, timeout=8.0)
        time.sleep(0.3)

        target_angles[5] = current_angles[5]
        self.mc.send_angles(target_angles, PLACE_J1_SPEED)
        _wait_move(self.mc, timeout=8.0)
        time.sleep(0.3)

        _gripper_open(self.mc)

        # J2 상승으로 그리퍼를 박스 위로 들기 (J4 고정 유지)
        current_angles_lift = self.mc.get_angles()
        if current_angles_lift and len(current_angles_lift) >= 6:
            lift_angles    = list(current_angles_lift)
            lift_angles[1] = current_angles_lift[1] - PLACE_LIFT_J2
            logger.info(
                f"J2 리프트: {current_angles_lift[1]:.1f}° → {lift_angles[1]:.1f}° "
                f"(-{PLACE_LIFT_J2}°)"
            )
            self.mc.send_angles(lift_angles, PLACE_J1_SPEED)
            _wait_move(self.mc, timeout=5.0)
            time.sleep(0.2)
        # 이후 _check_box_in_slot()에서 검출 자세 → ANGLES_ZERO 복귀 처리

    def _home(self):
        self.mc.send_angles(list(ANGLES_CAMERA_DOWN), ROBOT_SPEED)
        time.sleep(2.5)

    @staticmethod
    def _print_step(current, total, msg):
        print(f"\n{'='*50}\n[{current}/{total}] {msg}\n{'='*50}")


# ────────────────────────────────────────────────────────────────
# 단독 실행 (weight_aging 없이 테스트)
# ────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--flip-x", action="store_true")
    parser.add_argument("--stream", action="store_true")
    args = parser.parse_args()

    setup = CameraSetup()
    setup.prepare()
    tracker = BoxTracker()
    tracker.start()

    if args.stream:
        StreamServer(tracker).start()

    pnp = PickAndPlace(mc=setup.mc, tracker=tracker, flip_x=args.flip_x)

    try:
        while True:
            slot_id = pnp._place_count % PLACE_MAX_COUNT
            print(f"\n박스를 올려주세요. (다음 슬롯: {slot_id}, 완료: {pnp._place_count}개)")
            pnp.run()
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        tracker.stop()


if __name__ == "__main__":
    main()