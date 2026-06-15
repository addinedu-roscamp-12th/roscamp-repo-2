"""
픽앤플레이스 모듈 — Z 자동 계산 + 그리퍼 크기 자동 조절
────────────────────────────────────────────────────────────────
box_tracker.py의 GripperAligner로 중앙 정렬 완료 후
박스 픽셀 면적으로 카메라~박스 거리를 추정해
Z 하강량을 자동 계산한다.

박스 픽셀 폭(box.width = minAreaRect short_side) + Z 거리로
실제 박스 폭(mm)을 추정해 그리퍼 닫힘 값을 자동 조절한다.

Z 추정 원리:
  카메라가 수직 하향 고정 구조에서
  픽셀 면적 ∝ 1 / distance²
  → Z_rel = REF_DISTANCE * sqrt(REF_AREA / current_area)
  → grasp_z = current_z - Z_rel - GRIPPER_OFFSET

그리퍼 닫힘 값 추정 원리 (핀홀 카메라 모델):
  real_width_mm = (pixel_width × z_distance) / FOCAL_LENGTH_PX
  → gripper_value = 선형 매핑(real_width_mm)

실행:
    python pick_and_place.py
    python pick_and_place.py --stream
    python pick_and_place.py --flip-x

캘리브레이션:
    파일 상단의 REF_AREA, REF_DISTANCE 를 한 번 측정 후 입력
    측정 방법: 박스를 카메라 정중앙 바닥에 놓고
              터미널에 출력되는 area 값과 자로 잰 거리를 입력

    FOCAL_LENGTH_PX 측정 방법:
      알려진 폭의 물체를 알려진 거리에 놓고
      FOCAL_LENGTH_PX = pixel_width × z_distance / real_width_mm

의존:
    box_tracker.py (BoxTracker, CameraSetup, GripperAligner)
    pymycobot MyCobot280
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

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("jetcobot1.pick_and_place")


# ────────────────────────────────────────────────────────────────
# Z 자동 계산 캘리브레이션 값
# ── 측정 방법 ──────────────────────────────────────────────────
#   1. 박스를 카메라 정중앙 바닥에 놓는다
#   2. python pick_and_place.py --calib 실행
#   3. 터미널에 출력되는 area 값을 REF_AREA에 입력
#   4. 자로 카메라~박스 윗면까지 거리(mm)를 재서 REF_DISTANCE에 입력
# ────────────────────────────────────────────────────────────────
# REF_AREA     = 21875.0  # 기준 픽셀 면적 (측정 후 입력)
REF_AREA     = 39865.0  # 기준 픽셀 면적 (측정 후 입력)
REF_DISTANCE = 130.0    # 기준 카메라~박스 거리 mm (측정 후 입력)
FOCAL_LENGTH_PX = 854.9  # 카메라 초점거리 (픽셀 단위) — 그리퍼 닫힘 값 자동 계산용

# 그리퍼 오프셋: 그리퍼 끝이 박스 윗면보다 얼마나 더 내려가야 하는지 (mm)
GRIPPER_OFFSET = 50.0

# 접근 높이 여유: grasp_z 위 얼마에서 천천히 내려갈지 (mm)
APPROACH_MARGIN = 100.0

# 카메라~그리퍼 Y축 오프셋 (mm)
# 카메라가 그리퍼보다 Y축 방향으로 떨어진 거리
# 하강 직전에 이 값만큼 Y를 보정해 그리퍼가 박스 중앙에 오도록 함
# 방향이 반대면 음수(-35.0)로 변경
CAMERA_GRIPPER_Y_OFFSET = 35.0

# ────────────────────────────────────────────────────────────────
# 플레이스 설정
# ────────────────────────────────────────────────────────────────
PLACE_J1_ANGLE   = 130.0  # 플레이스 방향 J1 목표 각도 (도)
PLACE_J1_SPEED   = 30     # J1 회전 속도 — 박스 떨어지지 않게 느리게

PLACE_Z_DROP     = 187.0  # 테이블 윗면 Z (mm) — 실측값
PLACE_Z_APPROACH = 267.0  # 플레이스 접근 높이 (PLACE_Z_DROP + 80mm)

PLACE_X_OFFSET   = 30.0   # J1 회전 후 X축으로 추가 이동 거리 (mm) — 더 깊이 놓으려면 늘림
LIFT_MARGIN      = 50.0   # 픽업 후 회전 전 상승 여유 (mm) — 장애물 여유있게 높이면 늘림
LIFT_X_RETRACT = +50.0 # X 후퇴 거리 (mm) — 음수: 베이스 방향, 양수: 반대 방향

# ────────────────────────────────────────────────────────────────
# 그리퍼
# ────────────────────────────────────────────────────────────────
GRIPPER_OPEN  = 100
GRIPPER_SPEED = 80

# ── 그리퍼 닫힘 값 자동 계산용 ─────────────────────────────────
# 카메라 초점거리 (픽셀 단위)
# 측정법: 실폭을 아는 물체를 실거리를 아는 위치에 놓고
# FOCAL_LENGTH_PX = pixel_width × z_distance / real_width_mm

# 그리퍼 물리 개폐 범위 (mm) — 실측
# 각 value에서 그리퍼 핑거가 실제로 벌어지는 폭
GRIPPER_PHYS_MIN_MM = 20.0   # GRIPPER_VALUE_MIN일 때 실제 벌어진 폭'
GRIPPER_PHYS_MAX_MM = 60.0   # GRIPPER_VALUE_MAX일 때 실제 벌어진 폭

# 그리퍼 value 매핑 범위
GRIPPER_VALUE_MIN = 10        # 가장 많이 닫힌 값 (작은 박스)
GRIPPER_VALUE_MAX = 60        # 가장 덜 닫힌 값 (큰 박스)

# 파지 여유: 박스 실폭보다 얼마나 더 열어서 진입 후 닫을지 (mm)
# 값이 크면 박스 진입이 쉽지만 파지력이 약해짐
GRIPPER_MARGIN_MM = 5.0

# 폴백: 박스 폭 추정 실패 시 사용할 기본 닫힘 값
GRIPPER_CLOSE_FALLBACK = 15

# ────────────────────────────────────────────────────────────────
# 이동 속도
# ────────────────────────────────────────────────────────────────
MOVE_SPEED    = 60
DESCEND_SPEED = 25


# ────────────────────────────────────────────────────────────────
# Z 추정 함수
# ────────────────────────────────────────────────────────────────

def estimate_z_distance(area: float) -> float:
    """
    박스 픽셀 면적으로 카메라~박스 거리(mm)를 추정한다.

    원리:
      면적 ∝ 1/distance²
      → distance = REF_DISTANCE × sqrt(REF_AREA / area)

    Args:
        area: 현재 프레임의 박스 픽셀 면적

    Returns:
        카메라~박스 윗면까지 추정 거리 (mm)
    """
    if area <= 0:
        raise ValueError(f"유효하지 않은 면적: {area}")
    distance = REF_DISTANCE * math.sqrt(REF_AREA / area)
    logger.info(f"Z 추정: area={area:.0f}px²  distance={distance:.1f}mm")
    return distance


# ────────────────────────────────────────────────────────────────
# 그리퍼 닫힘 값 자동 계산
# ────────────────────────────────────────────────────────────────

def estimate_gripper_close_value(pixel_width: float, z_distance: float) -> int:
    """
    박스 픽셀 폭(box.width = minAreaRect short_side)과
    카메라~박스 거리로 그리퍼 닫힘 값을 자동 계산한다.

    원리 (핀홀 카메라 모델):
      real_width_mm = (pixel_width × z_distance) / FOCAL_LENGTH_PX
      target_mm     = real_width_mm - GRIPPER_MARGIN_MM
      gripper_value = 선형 매핑(target_mm)

    Args:
        pixel_width : 박스 픽셀 폭 (box.width, minAreaRect short_side)
        z_distance  : 카메라~박스 윗면 거리 (mm)

    Returns:
        그리퍼 닫힘 value (GRIPPER_VALUE_MIN ~ GRIPPER_VALUE_MAX)
        추정 실패 시 GRIPPER_CLOSE_FALLBACK
    """
    if pixel_width is None or pixel_width <= 0 or z_distance <= 0:
        logger.warning(
            f"그리퍼 계산 입력 이상: pixel_width={pixel_width}, "
            f"z_distance={z_distance} → 폴백({GRIPPER_CLOSE_FALLBACK}) 사용"
        )
        return GRIPPER_CLOSE_FALLBACK

    # 1) 픽셀 폭 → 실제 폭 (mm)
    real_width_mm = (pixel_width * z_distance) / FOCAL_LENGTH_PX

    # 2) 파지 목표폭: 박스 실폭보다 GRIPPER_MARGIN_MM 만큼 더 열고 닫기
    target_mm = real_width_mm - GRIPPER_MARGIN_MM

    logger.info(
        f"그리퍼 계산: pixel_w={pixel_width:.1f}px  "
        f"z={z_distance:.1f}mm  real_w={real_width_mm:.1f}mm  "
        f"target={target_mm:.1f}mm"
    )

    # 3) mm → gripper value 선형 매핑
    #    GRIPPER_PHYS_MIN_MM → GRIPPER_VALUE_MIN  (가장 많이 닫힘)
    #    GRIPPER_PHYS_MAX_MM → GRIPPER_VALUE_MAX  (덜 닫힘)
    t = (target_mm - GRIPPER_PHYS_MIN_MM) / (GRIPPER_PHYS_MAX_MM - GRIPPER_PHYS_MIN_MM)
    t = max(0.0, min(1.0, t))  # 클램프 [0, 1]
    value = int(GRIPPER_VALUE_MIN + t * (GRIPPER_VALUE_MAX - GRIPPER_VALUE_MIN))

    logger.info(f"그리퍼 닫힘 값: {value}  (t={t:.2f})")
    return value


# ────────────────────────────────────────────────────────────────
# 안정적 박스 정보 측정 (면적 + 픽셀 폭 동시 수집)
# ────────────────────────────────────────────────────────────────

def get_stable_box_info(
    tracker: BoxTracker,
    n_samples: int = 15,
) -> tuple[float, float] | tuple[None, None]:
    """
    n_samples 프레임의 중앙값으로 안정적인 박스 면적과 픽셀 폭을 반환한다.
    노이즈로 튀는 단일 프레임 면적 대신 중앙값 사용.

    box.width = minAreaRect short_side (box_tracker.py에서 추가된 필드)
    box.width가 None이면 sqrt(area)로 정사각형 근사.

    Returns:
        (median_area, median_pixel_width) 또는 (None, None)
    """
    areas  = []
    widths = []

    print(f"  박스 정보 측정 중 ({n_samples}프레임)...", end="", flush=True)
    for _ in range(n_samples):
        box = tracker.get_best_box()
        if box is not None:
            areas.append(box.area)
            # box.width가 None이면 정사각형으로 근사
            w = box.width if box.width is not None else math.sqrt(box.area)
            widths.append(w)
        time.sleep(0.05)
        print(".", end="", flush=True)
    print()

    if len(areas) < n_samples // 2:
        logger.error(f"샘플 부족: {len(areas)}/{n_samples}")
        return None, None

    median_area  = statistics.median(areas)
    median_width = statistics.median(widths)
    logger.info(
        f"면적 중앙값: {median_area:.0f}px²  "
        f"폭 중앙값: {median_width:.1f}px  "
        f"(샘플: {len(areas)}개)"
    )
    return median_area, median_width


# ────────────────────────────────────────────────────────────────
# 유틸
# ────────────────────────────────────────────────────────────────

def _wait_move(mc: MyCobot280, timeout: float = 6.0, poll: float = 0.1):
    start = time.time()
    while time.time() - start < timeout:
        if mc.is_moving() == 0:
            return
        time.sleep(poll)
    logger.warning("이동 타임아웃")


def _get_coords(mc: MyCobot280, retries: int = 5) -> list | None:
    for i in range(retries):
        coords = mc.get_coords()
        if coords and len(coords) >= 6:
            return coords
        logger.warning(f"좌표 읽기 실패 ({i + 1}/{retries})")
        time.sleep(0.3)
    logger.error("좌표 읽기 최종 실패")
    return None


def _send_coords(mc: MyCobot280, x: float, y: float, z: float,
                 speed: int = MOVE_SPEED):
    """현재 자세각을 유지하면서 x, y, z로 직선 이동 (mode=1)."""
    coords = _get_coords(mc)
    if coords is None:
        raise RuntimeError("좌표 읽기 실패 — 이동 불가")
    target = [x, y, z, coords[3], coords[4], coords[5]]
    mc.send_coords(target, speed, 0)


def _gripper_open(mc: MyCobot280):
    mc.set_gripper_value(GRIPPER_OPEN, GRIPPER_SPEED)
    time.sleep(0.8)


def _gripper_close(mc: MyCobot280, value: int = GRIPPER_CLOSE_FALLBACK):
    """
    그리퍼를 지정한 value로 닫는다.
    범위 초과 시 자동 클램프.

    Args:
        value: 닫힘 강도 (GRIPPER_VALUE_MIN ~ GRIPPER_VALUE_MAX)
               기본값은 GRIPPER_CLOSE_FALLBACK (박스 크기 추정 실패 시)
    """
    clamped = max(GRIPPER_VALUE_MIN, min(GRIPPER_VALUE_MAX, value))
    if clamped != value:
        logger.warning(f"그리퍼 값 클램프: {value} → {clamped}")
    logger.info(f"그리퍼 닫기: value={clamped}")
    mc.set_gripper_value(clamped, GRIPPER_SPEED)
    time.sleep(0.8)


# ────────────────────────────────────────────────────────────────
# 메인 클래스
# ────────────────────────────────────────────────────────────────

class PickAndPlace:
    """
    중앙 정렬 완료 후 박스 픽셀 면적으로 Z를 자동 계산하고,
    박스 픽셀 폭으로 그리퍼 닫힘 값을 자동 조절해 픽앤플레이스를 수행한다.

    Z 계산 흐름:
        박스 픽셀 면적 (안정적 중앙값)
              ↓
        estimate_z_distance() → 카메라~박스 거리
              ↓
        현재 Z - distance - GRIPPER_OFFSET = grasp_z
              ↓
        grasp_z + APPROACH_MARGIN = approach_z (접근 높이)

    그리퍼 닫힘 값 계산 흐름:
        박스 픽셀 폭 (안정적 중앙값) + z_distance
              ↓
        estimate_gripper_close_value()
              ↓
        real_width_mm = pixel_width × z_distance / FOCAL_LENGTH_PX
              ↓
        선형 매핑 → gripper value

    플레이스 흐름:
        픽업 위치에서 안전 높이 상승
              ↓
        J1을 PLACE_J1_ANGLE로 회전 (테이블 방향)
              ↓
        회전 후 현재 XY 그대로 접근 높이 이동
              ↓
        PLACE_Z_DROP까지 하강 → 그리퍼 열기 → 상승
    """

    def __init__(
        self,
        mc: MyCobot280,
        tracker: BoxTracker,
        flip_x: bool = False,
    ):
        self.mc       = mc
        self.tracker  = tracker
        self._aligner = GripperAligner(mc, tracker, flip_x=flip_x)

    def run(self, align_timeout: float = 30.0) -> bool:
        """전체 픽앤플레이스 시퀀스 실행. 성공 시 True."""
        try:
            # ── Step 1: 중앙 정렬 ──────────────────────────────
            self._print_step(1, 5, "박스 중앙 정렬 중...")
            aligned = self._aligner.align(timeout=align_timeout)
            if not aligned:
                print("\n[오류] 정렬 실패 — 박스를 카메라 시야 내에 놓아주세요.")
                return False
            print("\n  정렬 완료!")

            # ── Step 2: 픽업 좌표 + Z + 그리퍼 값 계산 ────────
            self._print_step(2, 5, "픽업 좌표 및 Z / 그리퍼 값 계산 중...")

            coords = _get_coords(self.mc)
            if coords is None:
                print("[오류] 좌표 읽기 실패")
                return False
            pick_x, pick_y, current_z = coords[0], coords[1], coords[2]

            # 박스 면적 + 픽셀 폭 동시 측정
            area, pixel_width = get_stable_box_info(self.tracker)
            if area is None:
                print("[오류] 박스 정보 측정 실패")
                return False

            # Z 거리 추정
            z_distance = estimate_z_distance(area)
            grasp_z    = current_z - z_distance - GRIPPER_OFFSET
            approach_z = grasp_z + APPROACH_MARGIN

            # 그리퍼 닫힘 값 자동 계산
            gripper_close_value = estimate_gripper_close_value(pixel_width, z_distance)
            real_width_mm = (pixel_width * z_distance) / FOCAL_LENGTH_PX

            print(f"\n  픽업 위치    : x={pick_x:.1f}  y={pick_y:.1f}")
            print(f"  현재 Z       : {current_z:.1f}mm")
            print(f"  박스 면적    : {area:.0f}px²")
            print(f"  박스 픽셀 폭 : {pixel_width:.1f}px")
            print(f"  박스 실폭    : {real_width_mm:.1f}mm")
            print(f"  추정 거리    : {z_distance:.1f}mm")
            print(f"  접근 Z       : {approach_z:.1f}mm")
            print(f"  파지 Z       : {grasp_z:.1f}mm")
            print(f"  그리퍼 닫힘  : {gripper_close_value}  (박스 {real_width_mm:.1f}mm 기준)")

            # ── Step 3: 픽업 ───────────────────────────────────
            self._print_step(3, 5, "박스 픽업 중...")
            self._pick(pick_x, pick_y, grasp_z, approach_z, gripper_close_value)
            print("  픽업 완료!")

            # ── Step 4: 플레이스 ───────────────────────────────
            self._print_step(4, 5, f"플레이스 이동 중... (J1={PLACE_J1_ANGLE}°)")
            self._place(approach_z)
            print("  플레이스 완료!")

            # ── Step 5: 홈 복귀 ────────────────────────────────
            self._print_step(5, 5, "홈 자세 복귀 중...")
            self._home()
            print("  홈 복귀 완료!")

            print("\n" + "=" * 50)
            print("  픽앤플레이스 성공!")
            print("=" * 50)
            return True

        except Exception as e:
            logger.error(f"픽앤플레이스 오류: {e}")
            print(f"\n[오류] {e}")
            return False

    # ──────────────────────────────────────────────────────────
    # 내부 동작
    # ──────────────────────────────────────────────────────────

    def _pick(self, pick_x: float, pick_y: float,
              grasp_z: float, approach_z: float,
              gripper_close_value: int):
        """
        픽업 시퀀스:
          접근 높이(approach_z) 이동
          → 그리퍼 열기
          → Y축 오프셋 보정 (카메라~그리퍼 거리만큼 이동)
          → 파지 높이(grasp_z) 하강
          → 그리퍼 닫기 (박스 크기에 맞춘 value)
          → 접근 높이 상승
        """
        # 1) 접근 높이로 이동 (카메라 정렬 위치 그대로)
        logger.info(f"접근 높이 이동: z={approach_z:.1f}mm")
        _send_coords(self.mc, pick_x, pick_y, approach_z, MOVE_SPEED)
        _wait_move(self.mc, timeout=5.0)
        time.sleep(0.3)

        # 2) 그리퍼 열기
        logger.info("그리퍼 열기")
        _gripper_open(self.mc)

        # 3) Y축 오프셋 보정 — 그리퍼를 박스 중앙 위로 정렬
        #    카메라와 그리퍼가 Y축으로 35mm 떨어져 있으므로
        #    하강 전에 Y를 보정해야 그리퍼가 박스 중앙에 위치
        gripper_y = pick_y + CAMERA_GRIPPER_Y_OFFSET
        logger.info(f"Y 오프셋 보정: {pick_y:.1f} → {gripper_y:.1f}mm (+{CAMERA_GRIPPER_Y_OFFSET}mm)")
        _send_coords(self.mc, pick_x, gripper_y, approach_z, MOVE_SPEED)
        _wait_move(self.mc, timeout=4.0)
        time.sleep(0.2)

        # 4) 파지 높이로 하강
        logger.info(f"파지 높이 하강: z={grasp_z:.1f}mm")
        _send_coords(self.mc, pick_x, gripper_y, grasp_z, DESCEND_SPEED)
        _wait_move(self.mc, timeout=6.0)
        time.sleep(0.2)

        # 5) 그리퍼 닫기 — 박스 크기에 맞춘 value 사용
        logger.info(f"그리퍼 닫기 (파지): value={gripper_close_value}")
        _gripper_close(self.mc, value=gripper_close_value)

        # 6) 접근 높이로 상승
        logger.info(f"상승: z={approach_z:.1f}mm")
        _send_coords(self.mc, pick_x, gripper_y, approach_z, DESCEND_SPEED)
        _wait_move(self.mc, timeout=5.0)

    def _place(self, approach_z: float):
        """
        플레이스 시퀀스 (J1 회전 방식):
          1. 픽업 위치에서 안전 높이(approach_z) 확보
          2. J1을 PLACE_J1_ANGLE로 회전 → 테이블 방향으로 선회
          3. 회전 후 현재 XY 그대로 읽어 접근 높이 유지
          4. PLACE_Z_DROP까지 하강 → 그리퍼 열기 → 상승
        """
        # 1) 현재 XY에서 안전 높이 상승 (X 후퇴 후 Z 상승)
        coords = _get_coords(self.mc)
        if coords is None:
            raise RuntimeError("좌표 읽기 실패")
        cur_x, cur_y = coords[0], coords[1]
        lift_z = approach_z + LIFT_MARGIN

        # X 먼저 후퇴 (Z 유지)
        logger.info(f"X 후퇴: x={cur_x:.1f} → {cur_x + LIFT_X_RETRACT:.1f}mm")
        _send_coords(self.mc, cur_x + LIFT_X_RETRACT, cur_y, approach_z, DESCEND_SPEED)
        _wait_move(self.mc, timeout=4.0)
        time.sleep(0.2)

        # Z 상승
        logger.info(f"안전 높이 상승: z={lift_z:.1f}mm (approach_z={approach_z:.1f} + LIFT_MARGIN={LIFT_MARGIN})")
        _send_coords(self.mc, cur_x + LIFT_X_RETRACT, cur_y, lift_z, DESCEND_SPEED)
        _wait_move(self.mc, timeout=5.0)
        time.sleep(0.3)

        # 2) J1 회전 — 나머지 관절 자세 유지
        angles = self.mc.get_angles()
        if not angles or len(angles) < 6:
            raise RuntimeError("각도 읽기 실패")
        target_angles = angles[:]
        target_angles[0] = PLACE_J1_ANGLE
        logger.info(f"J1 회전: {angles[0]:.1f}° → {PLACE_J1_ANGLE:.1f}°")
        self.mc.send_angles(target_angles, PLACE_J1_SPEED)
        _wait_move(self.mc, timeout=6.0)
        time.sleep(0.5)

        # 3) 회전 후 현재 XY 읽기 + X 오프셋 적용 (테이블 안쪽으로 이동)
        coords_after = _get_coords(self.mc)
        if coords_after is None:
            raise RuntimeError("회전 후 좌표 읽기 실패")
        place_x = coords_after[0] + PLACE_X_OFFSET   # X축으로 더 깊이
        place_y = coords_after[1]
        logger.info(f"J1 회전 후 좌표: x={coords_after[0]:.1f}→{place_x:.1f}  y={place_y:.1f}  z={coords_after[2]:.1f}")

        # 4) 플레이스 접근 높이 이동 (회전 후 XY 그대로, Z만 PLACE_Z_APPROACH)
        logger.info(f"플레이스 접근: x={place_x:.1f}  y={place_y:.1f}  z={PLACE_Z_APPROACH:.1f}")
        _send_coords(self.mc, place_x, place_y, PLACE_Z_APPROACH, MOVE_SPEED)
        _wait_move(self.mc, timeout=6.0)
        time.sleep(0.3)

        # 5) 테이블 위로 하강
        logger.info(f"드롭 하강: z={PLACE_Z_DROP:.1f}mm")
        _send_coords(self.mc, place_x, place_y, PLACE_Z_DROP, DESCEND_SPEED)
        _wait_move(self.mc, timeout=5.0)
        time.sleep(0.2)

        # 6) 그리퍼 열기 (박스 놓기)
        logger.info("그리퍼 열기 (박스 놓기)")
        _gripper_open(self.mc)

        # 7) 복귀 상승
        logger.info(f"복귀 상승: z={PLACE_Z_APPROACH:.1f}mm")
        _send_coords(self.mc, place_x, place_y, PLACE_Z_APPROACH, MOVE_SPEED)
        _wait_move(self.mc, timeout=5.0)

    def _home(self):
        """J1을 0으로 먼저 복귀 → 카메라 하향 자세 경유 → 홈([0,0,0,0,0,0])으로 복귀."""
        # 1) 현재 자세 그대로 J1만 0으로 복귀
        angles = self.mc.get_angles()
        if angles and len(angles) >= 6:
            target_angles = angles[:]
            target_angles[0] = 0.0
            logger.info(f"J1 복귀: {angles[0]:.1f}° → 0°")
            self.mc.send_angles(target_angles, PLACE_J1_SPEED)
            _wait_move(self.mc, timeout=6.0)
            time.sleep(0.5)
        else:
            logger.warning("J1 복귀용 각도 읽기 실패 — 스킵")

        # 2) 카메라 하향 자세 경유
        self.mc.send_angles(ANGLES_CAMERA_DOWN, ROBOT_SPEED)
        time.sleep(2.5)

        # 3) 홈
        self.mc.send_angles(ANGLES_ZERO, ROBOT_SPEED)
        time.sleep(2.5)

    @staticmethod
    def _print_step(current: int, total: int, msg: str):
        print(f"\n{'=' * 50}")
        print(f"[{current}/{total}] {msg}")
        print("=" * 50)


# ────────────────────────────────────────────────────────────────
# 캘리브레이션 모드
# ────────────────────────────────────────────────────────────────

def run_calib(tracker: BoxTracker):
    """
    REF_AREA / REF_DISTANCE / FOCAL_LENGTH_PX 측정을 도와주는 캘리브레이션 모드.
    박스를 카메라 정중앙 바닥에 놓고 실행한다.
    """
    print("\n" + "=" * 50)
    print("  캘리브레이션 모드")
    print("=" * 50)
    print("박스를 카메라 정중앙 바닥에 놓아주세요.")
    input("준비되면 Enter를 누르세요...")

    area, pixel_width = get_stable_box_info(tracker, n_samples=30)
    if area is None:
        print("[오류] 박스 감지 실패")
        return

    distance   = float(input("\n자로 카메라~박스 윗면까지 거리(mm)를 입력하세요: "))
    real_width = float(input("자로 박스 실폭(mm)을 입력하세요: "))

    focal_length = (pixel_width * distance) / real_width

    print("\n" + "=" * 50)
    print("  아래 값을 pick_and_place.py 상단에 입력하세요:")
    print(f"  REF_AREA        = {area:.1f}")
    print(f"  REF_DISTANCE    = {distance:.1f}")
    print(f"  FOCAL_LENGTH_PX = {focal_length:.1f}")
    print("=" * 50)

    # 역검증
    test_area = area * 0.8   # 박스가 20% 멀어진 경우 시뮬레이션
    test_dist = estimate_z_distance(test_area)
    test_grip = estimate_gripper_close_value(pixel_width, test_dist)
    print(f"\n  역검증: area={test_area:.0f}px² → distance={test_dist:.1f}mm")
    print(f"  (실제 거리의 약 {test_dist/distance*100:.0f}% — 1/√0.8 ≈ 112%가 기대값)")
    print(f"  그리퍼 닫힘 값 예시: {test_grip}  (박스 실폭 {real_width:.0f}mm 기준)")


# ────────────────────────────────────────────────────────────────
# 단독 실행
# ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="픽앤플레이스 — Z 자동 계산 + 그리퍼 크기 자동 조절")
    parser.add_argument("--flip-x",  action="store_true",
                        help="J1 회전 방향 반전")
    parser.add_argument("--stream",  action="store_true",
                        help="브라우저 MJPEG 스트리밍 서버 시작")
    parser.add_argument("--calib",   action="store_true",
                        help="캘리브레이션 모드 (REF_AREA, REF_DISTANCE, FOCAL_LENGTH_PX 측정)")
    args = parser.parse_args()

    # ── 로봇 초기화 ─────────────────────────────────────────
    print("로봇팔 초기화 중...")
    setup = CameraSetup()
    setup.prepare()

    # ── 트래커 시작 ─────────────────────────────────────────
    tracker = BoxTracker()
    tracker.start()

    if args.stream:
        StreamServer(tracker).start()

    # ── 캘리브레이션 모드 ───────────────────────────────────
    if args.calib:
        run_calib(tracker)
        tracker.stop()
        return

    # ── 픽앤플레이스 실행 ───────────────────────────────────
    pnp = PickAndPlace(
        mc      = setup.mc,
        tracker = tracker,
        flip_x  = args.flip_x,
    )

    try:
        pnp.run()
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        tracker.stop()
        print("종료")


if __name__ == "__main__":
    main()