import time
import numpy as np
from pymycobot.mycobot280 import MyCobot280

mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
mc.thread_lock = True
print("로봇이 연결되었습니다.")

# ─────────────────────────────────────────
# 원점 기준값
# ─────────────────────────────────────────
HOME_X  =  52.50
HOME_Y  = -64.50
HOME_Z  = 409.00
HOME_RX = -92.02
HOME_RY =   0.72
HOME_RZ = -89.93

# ─────────────────────────────────────────
# 확정된 보정값
# ─────────────────────────────────────────
NEG_Y_OFFSET = +4.15
POS_Y_OFFSET = +5.05
NEG_Z_OFFSET = +3.55
POS_Z_OFFSET = -4.90
Z_DEAD_ZONE  =  15.0

# Y 양수 이동을 위한 음수 기준점
Y_POS_BASELINE = -50.0  # 원점 기준 Y -50 위치에서 양수 측정


# ─────────────────────────────────────────
# 유틸
# ─────────────────────────────────────────
def wait_until_stopped(timeout=15, threshold=0.5):
    prev_angles = mc.get_angles()
    start = time.time()
    while time.time() - start < timeout:
        time.sleep(0.3)
        curr_angles = mc.get_angles()
        if curr_angles is None or prev_angles is None:
            prev_angles = curr_angles
            continue
        delta = max(abs(c - p) for c, p in zip(curr_angles, prev_angles))
        if delta < threshold:
            time.sleep(0.3)
            return True
        prev_angles = curr_angles
    print("  [경고] 타임아웃")
    return False

def go_home(speed=50, stabilize=1.5):
    mc.send_angles([0, 0, 0, 0, 0, 0], speed)
    wait_until_stopped()
    time.sleep(0.5)
    mc.send_coords([HOME_X, HOME_Y, HOME_Z, HOME_RX, HOME_RY, HOME_RZ], speed, 0)
    wait_until_stopped()
    time.sleep(stabilize)
    actual = mc.get_coords()
    print(f"  원점 도달: X={actual[0]:.2f} Y={actual[1]:.2f} Z={actual[2]:.2f}")
    return actual

def send_coords_raw(target_coords, speed, stabilize=1.5):
    mc.send_coords(list(target_coords), speed, 0)
    wait_until_stopped()
    time.sleep(stabilize)


# ─────────────────────────────────────────
# 보정 함수
# ─────────────────────────────────────────
def correct_y(delta):
    if delta < 0:
        return delta + NEG_Y_OFFSET
    elif delta > 0:
        return delta + POS_Y_OFFSET
    return 0.0

def correct_z(delta):
    if delta < -Z_DEAD_ZONE:
        return delta + NEG_Z_OFFSET
    elif delta > Z_DEAD_ZONE:
        return delta + POS_Z_OFFSET
    return delta


# ─────────────────────────────────────────
# 이동 함수: 음수/양수 방향 분기
# ─────────────────────────────────────────
def move_to(dy=0, dz=0, speed=30, stabilize=1.5):
    """
    원점 기준 절대 좌표로 이동.
    Y 양수: Y_POS_BASELINE(-50)으로 먼저 이동 후 양수 이동.
    """
    cy = correct_y(dy)
    cz = correct_z(dz)

    abs_target_y = HOME_Y + cy
    abs_target_z = HOME_Z + cz

    print(f"  Y: {dy:+.1f}mm → 보정: {cy:+.2f}mm (목표Y: {abs_target_y:.2f}mm)")
    print(f"  Z: {dz:+.1f}mm → 보정: {cz:+.2f}mm (목표Z: {abs_target_z:.2f}mm)")

    if dy > 0:
        # Y 양수: Z 이동 → Y 음수 기준점 경유 → 목표
        # 1) Z 이동
        z_only = [HOME_X, HOME_Y, abs_target_z, HOME_RX, HOME_RY, HOME_RZ]
        send_coords_raw(z_only, speed, stabilize=1.0)
        print(f"  → Z 이동 완료: Z={mc.get_coords()[2]:.2f}mm")

        # 2) Y 음수 기준점으로 이동 (Y -50 보정 적용)
        y_base_corrected = correct_y(Y_POS_BASELINE)
        y_base = [HOME_X, HOME_Y + y_base_corrected, abs_target_z, HOME_RX, HOME_RY, HOME_RZ]
        send_coords_raw(y_base, speed, stabilize=1.0)
        base_actual = mc.get_coords()
        print(f"  → Y 기준점 도달: Y={base_actual[1]:.2f}mm")

        # 3) 기준점에서 실제 도달 좌표 기준으로 양수 이동
        cy_pos = correct_y(dy)
        target = [HOME_X, HOME_Y + cy_pos, abs_target_z, HOME_RX, HOME_RY, HOME_RZ]
        send_coords_raw(target, speed, stabilize=stabilize)

    else:
        # Y 음수 또는 Z만: 바로 절대 좌표로 이동
        target = [HOME_X, abs_target_y, abs_target_z, HOME_RX, HOME_RY, HOME_RZ]
        send_coords_raw(target, speed, stabilize=stabilize)

    actual = mc.get_coords()
    dy_actual = actual[1] - HOME_Y
    dz_actual = actual[2] - HOME_Z
    dy_error  = dy_actual - dy
    dz_error  = dz_actual - dz

    print(f"  Y 실제: {dy_actual:+.2f}mm | 오차: {dy_error:+.2f}mm")
    print(f"  Z 실제: {dz_actual:+.2f}mm | 오차: {dz_error:+.2f}mm")
    return actual


# ─────────────────────────────────────────
# 메인
# ─────────────────────────────────────────
if __name__ == "__main__":

    print("\n로봇을 초기 위치로 리셋합니다.")
    go_home(stabilize=2.0)
    mc.set_gripper_value(100, 50)
    time.sleep(1.5)
    print("리셋 완료\n")

    test_cases = [
        (-50,   0),
        (-50, -30),
        (-50, +30),
        (-70, -40),
        (-70, +40),
        (-50, -50),
        (-50, +50),
    ]

    results = []
    print("=== Y/Z 보정 통합 검증 ===\n")
    for dz, dy in test_cases:
        print(f"── 테스트: Z{dz:+.0f}mm, Y{dy:+.0f}mm ──")
        go_home(stabilize=1.5)
        actual = move_to(dy=dy, dz=dz, speed=30, stabilize=1.5)
        results.append((dz, dy, actual))
        print()

    go_home(stabilize=1.0)

    print("=== 검증 결과 요약 ===")
    print(f"{'Z목표':>6} {'Y목표':>6} | {'Y오차':>8} {'Z오차':>8} {'판정':>6}")
    print("-" * 46)
    for dz, dy, actual in results:
        if actual:
            dy_err = (actual[1] - HOME_Y) - dy
            dz_err = (actual[2] - HOME_Z) - dz
            judge = "OK" if abs(dy_err) <= 2.0 and abs(dz_err) <= 2.0 else "확인필요"
            print(f"  {dz:+5.0f} {dy:+5.0f} | "
                  f"Y={dy_err:+6.2f}mm "
                  f"Z={dz_err:+6.2f}mm  {judge}")