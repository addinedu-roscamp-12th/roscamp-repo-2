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

Z_OFFSET_FOR_X = -50.0
NEG_Z_OFFSET   = +3.55

# ─────────────────────────────────────────
# X축 음수 방향 오프셋 (확정)
# 양수 방향은 이번에 재측정
# ─────────────────────────────────────────
NEG_X_OFFSET = -5.57


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
    print(f"  원점 실제 도달: X={actual[0]:.2f}mm, Z={actual[2]:.2f}mm")
    return actual

def send_coords_raw(target_coords, speed, stabilize=1.5):
    mc.send_coords(list(target_coords), speed, 0)
    wait_until_stopped()
    time.sleep(stabilize)

def go_x_baseline(speed=30, stabilize=1.5):
    home = go_home(speed=50, stabilize=1.0)
    target = list(home)
    target[2] += Z_OFFSET_FOR_X + NEG_Z_OFFSET
    send_coords_raw(target, speed, stabilize=stabilize)
    actual = mc.get_coords()
    print(f"  X기준점 도달: X={actual[0]:.2f}mm, Z={actual[2]:.2f}mm")
    return actual

def measure_x_raw(desired_delta, baseline_coords, speed=30):
    send_coords_raw(list(baseline_coords), speed, stabilize=1.5)
    start = mc.get_coords()
    if start is None:
        return None

    target = list(start)
    target[0] += desired_delta
    send_coords_raw(target, speed, stabilize=1.5)

    actual = mc.get_coords()
    if actual is None:
        return None

    actual_delta = actual[0] - start[0]
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

    # ── 음수 방향 검증 (오프셋 적용) ──
    print("=== X축 음수 방향 검증 (오프셋 적용) ===")
    print(f"  NEG_X_OFFSET={NEG_X_OFFSET:+.2f}mm\n")
    baseline = go_x_baseline(stabilize=1.5)

    neg_verify = [-25, -35, -45]
    neg_results = []
    for delta in neg_verify:
        corrected = delta + NEG_X_OFFSET
        send_coords_raw(list(baseline), 30, stabilize=1.5)
        start = mc.get_coords()
        target = list(start)
        target[0] += corrected
        send_coords_raw(target, 30, stabilize=1.5)
        actual = mc.get_coords()
        actual_delta = actual[0] - start[0]
        error = actual_delta - delta
        print(f"  목표: {delta:+6.1f}mm | 보정명령: {corrected:+6.2f}mm | 실제: {actual_delta:+6.2f}mm | 오차: {error:+5.2f}mm")
        neg_results.append(error)

    # ── 양수 방향 재측정 (+20~+40만) ──
    print("\n=== X축 양수 방향 재측정 (+20~+40mm) ===")
    send_coords_raw(list(baseline), 30, stabilize=1.0)
    start = mc.get_coords()
    target = list(start)
    target[0] += -50
    send_coords_raw(target, 30, stabilize=1.5)
    pos_baseline = mc.get_coords()
    print(f"  양수 기준점 도달: X={pos_baseline[0]:.2f}mm")

    pos_deltas = [20, 25, 30, 35, 40]
    pos_errors = []
    for delta in pos_deltas:
        err = measure_x_raw(delta, pos_baseline)
        if err is not None:
            pos_errors.append(err)

    go_home(stabilize=1.0)

    print(f"\n=== X축 음수 검증 결과 ===")
    print(f"  평균 절대 오차: {np.mean([abs(e) for e in neg_results]):.2f}mm")

    print(f"\n=== X축 양수 재측정 결과 ===")
    print(f"  양수 방향 오차 평균: {np.mean(pos_errors):+.2f}mm")
    print(f"  양수 방향 오차 std : {np.std(pos_errors):.3f}mm")
    print(f"  → 권장 POS_X_OFFSET: {-np.mean(pos_errors):+.2f}mm")

    print("\n=== 데이터 (Claude에게 붙여넣기) ===")
    print("neg_verify  =", neg_verify)
    print("neg_results =", [round(e, 2) for e in neg_results])
    print("pos_deltas  =", pos_deltas)
    print("pos_errors  =", [round(e, 2) for e in pos_errors])