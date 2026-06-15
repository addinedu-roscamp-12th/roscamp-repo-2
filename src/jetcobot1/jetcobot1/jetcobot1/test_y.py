# import os
# import time
# import threading
# import numpy as np
# from pymycobot.mycobot280 import MyCobot280
# from pymycobot.genre import Angle, Coord

# mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
# mc.thread_lock = True
# print("로봇이 연결되었습니다.")

# def move_and_measure(mc, axis, delta, base_coords, speed=30, wait=5):
#     mc.send_angles([0, 0, 0, 0, 0, 0], 50)
#     time.sleep(5)

#     z_base = base_coords.copy()
#     z_base[2] -= 50
#     mc.send_coords(z_base, speed, 1)
#     time.sleep(wait)

#     current = mc.get_coords()
#     target = current.copy()
#     target[axis] += delta
#     mc.send_coords(target, speed, 1)
#     time.sleep(wait)

#     actual = mc.get_coords()
#     actual_delta = actual[axis] - current[axis]
#     error = actual_delta - delta
#     print(f"목표: {delta:+.1f}mm | 실제: {actual_delta:+.2f}mm | 오차: {error:+.2f}mm | 오차율: {error/delta*100:+.1f}%")
#     return actual_delta, error

# mc.send_angles([0, 0, 0, 0, 0, 0], 50)
# time.sleep(5)
# base_coords = mc.get_coords()
# print(f"기준 좌표: {base_coords}")

# print("=== Y축 선형성 실험 (Z -50 고정, Y: +10 ~ +100) ===")
# test_deltas = [-10, -20, -30, -40, -50, -60, -70, -80, -90, -100]

# results = []
# for delta in test_deltas:
#     actual_delta, error = move_and_measure(mc, axis=1, delta=delta, base_coords=base_coords)
#     results.append((delta, actual_delta, error))

# targets = np.array([r[0] for r in results])
# actuals = np.array([r[1] for r in results])
# errors  = np.array([r[2] for r in results])

# print(f"\n=== 결과 요약 ===")
# print(f"절대 오차 평균: {np.mean(errors):.2f}mm | 표준편차: {np.std(errors):.2f}mm")
# print(f"비율 오차 평균: {np.mean(errors/targets)*100:.1f}% | 표준편차: {np.std(errors/targets)*100:.1f}%")

# A, B = np.polyfit(targets, actuals, 1)
# print(f"\n=== 선형회귀 결과 ===")
# print(f"기울기 a = {A:.4f}, 절편 b = {B:.4f}")
# print(f"보정 공식: command = (target - {B:.4f}) / {A:.4f}")

# print("\n=== 그래프용 데이터 ===")
# print("targets =", targets.tolist())
# print("actuals =", actuals.tolist())
# print("errors =", errors.tolist())

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

Z_OFFSET_FOR_Y = -50.0  # Y축 측정용 Z 내림값
NEG_Z_OFFSET   = +3.55
POS_Z_OFFSET   = -4.90


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
    print(f"  원점 실제 도달: Y={actual[1]:.2f}mm, Z={actual[2]:.2f}mm")
    return actual

def send_coords_fixed_x(target_coords, speed, stabilize=1.5):
    fixed = list(target_coords)
    fixed[0] = HOME_X
    mc.send_coords(fixed, speed, 0)
    wait_until_stopped()
    time.sleep(stabilize)


# ─────────────────────────────────────────
# Y축 음수 측정용 기준점 (Z -50)
# ─────────────────────────────────────────
def go_y_neg_baseline(speed=30, stabilize=1.5):
    """원점 → Z -50 (Z보정 적용)"""
    home = go_home(speed=50, stabilize=1.0)
    target = list(home)
    target[0] = HOME_X
    target[2] += Z_OFFSET_FOR_Y + NEG_Z_OFFSET
    send_coords_fixed_x(target, speed, stabilize=stabilize)
    actual = mc.get_coords()
    print(f"  음수 기준점 도달: Y={actual[1]:.2f}mm, Z={actual[2]:.2f}mm")
    return actual


# ─────────────────────────────────────────
# Y축 양수 측정용 기준점 (Z -50 후 Y -90)
# ─────────────────────────────────────────
def go_y_pos_baseline(speed=30, stabilize=1.5):
    """원점 → Z -50 → Y -90 (공간 확보 후 양수 측정)"""
    neg_base = go_y_neg_baseline(speed=speed, stabilize=1.0)
    target = list(neg_base)
    target[0] = HOME_X
    target[1] += -90  # Y -90으로 내려서 양수 공간 확보
    send_coords_fixed_x(target, speed, stabilize=stabilize)
    actual = mc.get_coords()
    print(f"  양수 기준점 도달: Y={actual[1]:.2f}mm, Z={actual[2]:.2f}mm")
    return actual


# ─────────────────────────────────────────
# Y축 오차 측정
# ─────────────────────────────────────────
def measure_y_raw(desired_delta, baseline_coords, speed=30):
    start = mc.get_coords()  # 기준점에서 바로 읽기
    if start is None:
        return None

    # 기준점으로 복귀
    send_coords_fixed_x(list(baseline_coords), speed, stabilize=1.5)
    start = mc.get_coords()

    target = list(start)
    target[0] = HOME_X
    target[1] += desired_delta
    send_coords_fixed_x(target, speed, stabilize=1.5)

    actual = mc.get_coords()
    if actual is None:
        return None

    actual_delta = actual[1] - start[1]
    error = actual_delta - desired_delta
    print(f"  목표: {desired_delta:+6.1f}mm | 실제: {actual_delta:+6.2f}mm | 오차: {error:+5.2f}mm")
    return error


# ─────────────────────────────────────────
# 메인
# ─────────────────────────────────────────
if __name__ == "__main__":

    print("\n로봇을 초기 위치로 리셋합니다.")
    go_home(stabilize=2.0)
    mc.set_gripper_value(100, 50)
    time.sleep(1.5)
    print("리셋 완료\n")

    neg_deltas = [-10, -30, -50, -70, -90]
    pos_deltas = [ 10,  30,  50,  70,  90]

    neg_errors, pos_errors = [], []

    # ── 음수 방향: Z -50 기준점 → Y 음수 ──
    print("=== Y축 음수 방향 측정 ===")
    neg_baseline = go_y_neg_baseline(stabilize=1.5)
    for delta in neg_deltas:
        err = measure_y_raw(delta, neg_baseline)
        if err is not None:
            neg_errors.append(err)

    # ── 양수 방향: Z -50 → Y -90 기준점 → Y 양수 ──
    print("\n=== Y축 양수 방향 측정 ===")
    pos_baseline = go_y_pos_baseline(stabilize=1.5)
    for delta in pos_deltas:
        err = measure_y_raw(delta, pos_baseline)
        if err is not None:
            pos_errors.append(err)

    go_home(stabilize=1.0)

    print(f"\n=== Y축 오차 요약 ===")
    print(f"  음수 방향 오차 평균: {np.mean(neg_errors):+.2f}mm")
    print(f"  양수 방향 오차 평균: {np.mean(pos_errors):+.2f}mm")
    print(f"  음수 방향 오차 std : {np.std(neg_errors):.3f}mm")
    print(f"  양수 방향 오차 std : {np.std(pos_errors):.3f}mm")
    print(f"\n  → 권장 NEG_Y_OFFSET: {-np.mean(neg_errors):+.2f}mm")
    print(f"  → 권장 POS_Y_OFFSET: {-np.mean(pos_errors):+.2f}mm")

    print("\n=== 데이터 (Claude에게 붙여넣기) ===")
    print("neg_deltas =", neg_deltas)
    print("neg_errors =", [round(e, 2) for e in neg_errors])
    print("pos_deltas =", pos_deltas)
    print("pos_errors =", [round(e, 2) for e in pos_errors])