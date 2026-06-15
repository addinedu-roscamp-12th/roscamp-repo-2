# import os
# import time
# import threading
# import numpy as np
# from pymycobot.mycobot280 import MyCobot280
# from pymycobot.genre import Angle, Coord

# mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
# mc.thread_lock = True
# print("로봇이 연결되었습니다.")

# # 선형회귀 보정 계수 (Z축)
# A = 0.9946
# B = 2.3933

# def correct_target(target_delta):
#     return (target_delta - B) / A

# def corrected_send_coords(mc, base_coords, delta_z, speed=30, mode=0):
#     corrected = base_coords.copy()
#     corrected[2] += correct_target(delta_z)
#     mc.send_coords(corrected, speed, mode)

# # 검증
# mc.send_angles([0, 0, 0, 0, 0, 0], 50)
# time.sleep(4)

# base = mc.get_coords()
# print("초기 좌표:", base)

# corrected_send_coords(mc, base, delta_z=-50)
# time.sleep(5)

# final = mc.get_coords()
# print(f"목표 Z: {base[2]-50:.1f} | 실제 Z: {final[2]:.1f} | 오차: {final[2]-(base[2]-50):.2f}mm")

#--------------------------------------------------------------------------------------------------------------

# import time
# import numpy as np
# from scipy import stats
# from pymycobot.mycobot280 import MyCobot280

# mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
# mc.thread_lock = True
# print("로봇이 연결되었습니다.")

# # ─────────────────────────────────────────
# # 확정된 원점 기준값
# # ─────────────────────────────────────────
# HOME_X =  52.50
# HOME_Y = -64.50
# HOME_Z = 409.00
# HOME_COORDS = [HOME_X, HOME_Y, HOME_Z, -92.02, 0.72, -89.93]


# # ─────────────────────────────────────────
# # 유틸: 이동 완료까지 대기
# # ─────────────────────────────────────────
# def wait_until_stopped(timeout=15, threshold=0.5):
#     prev_angles = mc.get_angles()
#     start = time.time()
#     while time.time() - start < timeout:
#         time.sleep(0.3)
#         curr_angles = mc.get_angles()
#         if curr_angles is None or prev_angles is None:
#             prev_angles = curr_angles
#             continue
#         delta = max(abs(c - p) for c, p in zip(curr_angles, prev_angles))
#         if delta < threshold:
#             time.sleep(0.3)
#             return True
#         prev_angles = curr_angles
#     print("  [경고] 타임아웃: 이동 미완료 가능성")
#     return False


# # ─────────────────────────────────────────
# # 유틸: 원점 복귀 (각도 기준)
# # ─────────────────────────────────────────
# def go_home(speed=50):
#     mc.send_angles([0, 0, 0, 0, 0, 0], speed)
#     wait_until_stopped()
#     time.sleep(0.3)


# # ─────────────────────────────────────────
# # 유틸: X 고정 좌표 이동
# # ─────────────────────────────────────────
# def send_coords_fixed_x(target_coords, speed, mode=0):
#     fixed = target_coords.copy()
#     fixed[0] = HOME_X
#     mc.send_coords(fixed, speed, mode)
#     wait_until_stopped()


# # ─────────────────────────────────────────
# # 측정: 지정 좌표에서 delta 이동 후 실제 변위 측정
# # ─────────────────────────────────────────
# def measure_from(axis, delta, from_coords, speed=30, repeat=3):
#     actuals = []

#     for i in range(repeat):
#         # 시작 좌표로 복귀
#         send_coords_fixed_x(list(from_coords), speed)
#         time.sleep(0.2)

#         start = mc.get_coords()
#         if start is None:
#             print(f"  [경고] 반복 {i+1}: 시작 좌표 읽기 실패, 스킵")
#             continue

#         target = list(start)
#         target[0] = HOME_X   # X 항상 고정
#         target[axis] += delta
#         send_coords_fixed_x(target, speed)

#         actual = mc.get_coords()
#         if actual is None:
#             print(f"  [경고] 반복 {i+1}: 도달 좌표 읽기 실패, 스킵")
#             continue

#         actuals.append(actual[axis] - start[axis])

#     if len(actuals) == 0:
#         raise RuntimeError(f"delta={delta}mm 측정 완전 실패")

#     mean  = np.mean(actuals)
#     std   = np.std(actuals)
#     error = mean - delta

#     print(f"  목표: {delta:+6.1f}mm | "
#           f"실제(평균): {mean:+6.2f}mm | "
#           f"오차: {error:+5.2f}mm | "
#           f"std: {std:.3f}mm | "
#           f"오차율: {error/delta*100:+.1f}%")

#     return mean, std, error


# # ─────────────────────────────────────────
# # 선형회귀 보정 함수 생성 (방향별)
# # ─────────────────────────────────────────
# def build_correction(targets, actuals, label=""):
#     slope, intercept, r, p, se = stats.linregress(targets, actuals)

#     print(f"\n[선형회귀 결과 - {label}]")
#     print(f"  slope     = {slope:.6f}")
#     print(f"  intercept = {intercept:.6f}")
#     print(f"  R²        = {r**2:.6f}  {'(양호)' if r**2 > 0.99 else '(주의: 비선형 가능성)'}")

#     def correct_fn(desired_delta):
#         return (desired_delta - intercept) / slope

#     return correct_fn, slope, intercept, r**2


# def validate_correction(targets, actuals, correct_fn, label=""):
#     print(f"\n[보정 함수 검증 - {label}]")
#     print(f"{'목표':>8} | {'기존오차':>8} | {'보정명령':>8}")
#     print("-" * 35)
#     for t, a in zip(targets, actuals):
#         print(f"  {t:+6.1f}mm | {a-t:+6.2f}mm | {correct_fn(t):+6.2f}mm")


# # ─────────────────────────────────────────
# # 메인 실험
# # ─────────────────────────────────────────
# if __name__ == "__main__":

#     # 초기 리셋
#     print("\n로봇을 초기 위치로 리셋합니다.")
#     go_home()
#     mc.set_gripper_value(100, 50)
#     time.sleep(1.5)
#     print("리셋 완료\n")

#     neg_targets, neg_actuals, neg_errors, neg_stds = [], [], [], []
#     pos_targets, pos_actuals, pos_errors, pos_stds = [], [], [], []

#     # ── 음수 방향: 원점 → -10, -20, ..., -100 ──
#     print("=== 음수 방향 측정 (원점 → -Z) ===")
#     neg_deltas = [-10, -20, -30, -40, -50, -60, -70, -80, -90, -100]

#     for delta in neg_deltas:
#         mean, std, error = measure_from(
#             axis=2,
#             delta=delta,
#             from_coords=HOME_COORDS,
#             speed=30,
#             repeat=3
#         )
#         neg_targets.append(delta)
#         neg_actuals.append(mean)
#         neg_errors.append(error)
#         neg_stds.append(std)

#     # ── 양수 방향: 원점 → -100 기준점 → +10, +20, ..., +90 ──
#     print("\n=== 양수 방향 측정 (Z -100 기준점 → +Z) ===")
#     pos_deltas = [10, 20, 30, 40, 50, 60, 70, 80, 90]  # +100은 관절한계로 제외

#     # -100 기준점으로 이동
#     baseline_coords = list(HOME_COORDS)
#     baseline_coords[2] = HOME_Z - 100
#     send_coords_fixed_x(baseline_coords, speed=30)
#     actual_baseline = mc.get_coords()
#     print(f"기준점 좌표: {actual_baseline}")

#     for delta in pos_deltas:
#         mean, std, error = measure_from(
#             axis=2,
#             delta=delta,
#             from_coords=actual_baseline,
#             speed=30,
#             repeat=3
#         )
#         pos_targets.append(delta)
#         pos_actuals.append(mean)
#         pos_errors.append(error)
#         pos_stds.append(std)

#     # 원점 복귀
#     go_home()

#     # ── 통계 요약 ──
#     print(f"\n=== 음수 방향 통계 ===")
#     print(f"절대 오차 평균    : {np.mean(neg_errors):+.2f}mm")
#     print(f"절대 오차 표준편차 : {np.std(neg_errors):.3f}mm")
#     print(f"반복 측정 std 평균 : {np.mean(neg_stds):.3f}mm")

#     print(f"\n=== 양수 방향 통계 ===")
#     print(f"절대 오차 평균    : {np.mean(pos_errors):+.2f}mm")
#     print(f"절대 오차 표준편차 : {np.std(pos_errors):.3f}mm")
#     print(f"반복 측정 std 평균 : {np.mean(pos_stds):.3f}mm")

#     # ── 방향별 선형회귀 보정 ──
#     correct_neg, neg_slope, neg_intercept, neg_r2 = build_correction(
#         neg_targets, neg_actuals, label="음수 방향"
#     )
#     correct_pos, pos_slope, pos_intercept, pos_r2 = build_correction(
#         pos_targets, pos_actuals, label="양수 방향"
#     )

#     validate_correction(neg_targets, neg_actuals, correct_neg, label="음수 방향")
#     validate_correction(pos_targets, pos_actuals, correct_pos, label="양수 방향")

#     # ── 최종 보정 함수 ──
#     def correct_z(desired_delta):
#         """방향에 따라 자동으로 보정값 반환"""
#         if desired_delta < 0:
#             return correct_neg(desired_delta)
#         elif desired_delta > 0:
#             return correct_pos(desired_delta)
#         return 0.0

#     # ── 데이터 출력 ──
#     print(f"\n=== 그래프용 데이터 (Claude에게 붙여넣기) ===")
#     print("neg_targets =", neg_targets)
#     print("neg_actuals =", neg_actuals)
#     print("neg_errors  =", neg_errors)
#     print("pos_targets =", pos_targets)
#     print("pos_actuals =", pos_actuals)
#     print("pos_errors  =", pos_errors)
#     print(f"neg: slope={neg_slope:.6f}, intercept={neg_intercept:.6f}, R²={neg_r2:.6f}")
#     print(f"pos: slope={pos_slope:.6f}, intercept={pos_intercept:.6f}, R²={pos_r2:.6f}")

# import time
# import numpy as np
# from pymycobot.mycobot280 import MyCobot280

# mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
# mc.thread_lock = True
# print("로봇이 연결되었습니다.")

# # ─────────────────────────────────────────
# # 확정된 원점 기준값
# # ─────────────────────────────────────────
# HOME_X =  52.50
# HOME_Y = -64.50
# HOME_Z = 409.00
# HOME_COORDS     = [HOME_X, HOME_Y, HOME_Z,       -92.02, 0.72, -89.93]
# BASELINE_COORDS = [HOME_X, HOME_Y, HOME_Z - 100, -92.02, 0.72, -89.93]

# # ─────────────────────────────────────────
# # 오프셋 보정값 (원복)
# # ─────────────────────────────────────────
# NEG_OFFSET = -6.12
# POS_OFFSET =  4.44


# # ─────────────────────────────────────────
# # 유틸
# # ─────────────────────────────────────────
# def wait_until_stopped(timeout=15, threshold=0.5):
#     prev_angles = mc.get_angles()
#     start = time.time()
#     while time.time() - start < timeout:
#         time.sleep(0.3)
#         curr_angles = mc.get_angles()
#         if curr_angles is None or prev_angles is None:
#             prev_angles = curr_angles
#             continue
#         delta = max(abs(c - p) for c, p in zip(curr_angles, prev_angles))
#         if delta < threshold:
#             time.sleep(0.3)
#             return True
#         prev_angles = curr_angles
#     print("  [경고] 타임아웃")
#     return False

# def go_home(speed=50, stabilize=1.5):
#     mc.send_angles([0, 0, 0, 0, 0, 0], speed)
#     wait_until_stopped()
#     time.sleep(0.5)
#     mc.send_coords(list(HOME_COORDS), speed, 0)
#     wait_until_stopped()
#     time.sleep(stabilize)

# def go_baseline(speed=30, stabilize=1.5):
#     """기준점 이동 후 실제 도달 좌표 반환"""
#     go_home(speed=50, stabilize=1.0)
#     mc.send_coords(list(BASELINE_COORDS), speed, 0)
#     wait_until_stopped()
#     time.sleep(stabilize)
#     # 실제 도달한 좌표를 읽어서 반환 (고정값 사용 X)
#     actual = mc.get_coords()
#     print(f"  기준점 실제 도달: Z={actual[2]:.2f}mm")
#     return actual

# def send_coords_fixed_x(target_coords, speed, mode=0, stabilize=1.5):
#     fixed = list(target_coords)
#     fixed[0] = HOME_X
#     mc.send_coords(fixed, speed, mode)
#     wait_until_stopped()
#     time.sleep(stabilize)


# # ─────────────────────────────────────────
# # 오프셋 보정 함수
# # ─────────────────────────────────────────
# def correct_offset(desired_delta):
#     if desired_delta < 0:
#         return desired_delta + NEG_OFFSET
#     elif desired_delta > 0:
#         return desired_delta + POS_OFFSET
#     return 0.0


# # ─────────────────────────────────────────
# # 검증: 1회 측정 (양수/음수 분기)
# # ─────────────────────────────────────────
# def verify_once(desired_delta, speed=30):
#     print(f"\n── delta={desired_delta:+.1f}mm ──")
#     results = {}

#     for label, correct_fn in [
#         ("보정 없음", lambda d: d),
#         ("오프셋",    correct_offset),
#     ]:
#         if desired_delta < 0:
#             # 음수: HOME_COORDS 출발
#             go_home(stabilize=1.5)
#             start = mc.get_coords()
#         else:
#             # 양수: 실제 기준점 도달 좌표를 출발점으로 사용
#             start = go_baseline(stabilize=1.5)

#         if start is None:
#             print(f"  [{label}] 시작 좌표 읽기 실패, 스킵")
#             continue

#         corrected = correct_fn(desired_delta)
#         target = list(start)
#         target[0] = HOME_X
#         target[2] += corrected  # 실제 출발점 기준으로 delta 적용
#         send_coords_fixed_x(target, speed, stabilize=1.5)

#         actual = mc.get_coords()
#         if actual is None:
#             print(f"  [{label}] 도달 좌표 읽기 실패, 스킵")
#             continue

#         actual_delta = actual[2] - start[2]
#         error = actual_delta - desired_delta
#         print(f"  [{label:5s}] 명령: {corrected:+6.2f}mm | 실제: {actual_delta:+6.2f}mm | 오차: {error:+5.2f}mm")
#         results[label] = error

#     return results


# # ─────────────────────────────────────────
# # 메인
# # ─────────────────────────────────────────
# if __name__ == "__main__":

#     print("\n로봇을 초기 위치로 리셋합니다.")
#     go_home(stabilize=2.0)
#     mc.set_gripper_value(100, 50)
#     time.sleep(1.5)
#     print("리셋 완료\n")

#     test_deltas = [-15, -35, -55, -75, -95,
#                     15,  35,  55,  75,  85]

#     all_results = []
#     print("=== 검증 시작 ===")
#     print(f"  NEG_OFFSET={NEG_OFFSET}, POS_OFFSET={POS_OFFSET}")

#     for delta in test_deltas:
#         res = verify_once(delta, speed=30)
#         all_results.append((delta, res))

#     go_home(stabilize=1.0)

#     # ── 결과 요약 ──
#     print("\n=== 검증 결과 요약 ===")
#     print(f"{'목표':>8} | {'보정없음':>8} | {'오프셋':>8}")
#     print("-" * 34)
#     for delta, res in all_results:
#         print(f"  {delta:+6.1f}mm "
#               f"| {res.get('보정 없음', 0):+7.2f}mm "
#               f"| {res.get('오프셋',    0):+7.2f}mm")

#     neg_errs = [abs(r[1].get('오프셋', 0)) for r in all_results if r[0] < 0]
#     pos_errs = [abs(r[1].get('오프셋', 0)) for r in all_results if r[0] > 0]
#     print(f"\n  오프셋 평균 절대 오차 (음수): {np.mean(neg_errs):.2f}mm")
#     print(f"  오프셋 평균 절대 오차 (양수): {np.mean(pos_errs):.2f}mm")

#     print("\n=== 그래프용 데이터 (Claude에게 붙여넣기) ===")
#     print("deltas      =", [r[0] for r in all_results])
#     print("no_correct  =", [round(r[1].get('보정 없음', 0), 3) for r in all_results])
#     print("offset_corr =", [round(r[1].get('오프셋',    0), 3) for r in all_results])

# import time
# import numpy as np
# from pymycobot.mycobot280 import MyCobot280

# mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
# mc.thread_lock = True
# print("로봇이 연결되었습니다.")

# # ─────────────────────────────────────────
# # 원점 기준값
# # ─────────────────────────────────────────
# HOME_X  =  52.50
# HOME_Y  = -64.50
# HOME_Z  = 409.00
# HOME_RX = -92.02
# HOME_RY =   0.72
# HOME_RZ = -89.93

# Z_OFFSET_FOR_Y = -50.0
# NEG_Z_OFFSET   = +3.55


# # ─────────────────────────────────────────
# # 유틸
# # ─────────────────────────────────────────
# def wait_until_stopped(timeout=15, threshold=0.5):
#     prev_angles = mc.get_angles()
#     start = time.time()
#     while time.time() - start < timeout:
#         time.sleep(0.3)
#         curr_angles = mc.get_angles()
#         if curr_angles is None or prev_angles is None:
#             prev_angles = curr_angles
#             continue
#         delta = max(abs(c - p) for c, p in zip(curr_angles, prev_angles))
#         if delta < threshold:
#             time.sleep(0.3)
#             return True
#         prev_angles = curr_angles
#     print("  [경고] 타임아웃")
#     return False

# def go_home(speed=50, stabilize=1.5):
#     mc.send_angles([0, 0, 0, 0, 0, 0], speed)
#     wait_until_stopped()
#     time.sleep(0.5)
#     mc.send_coords([HOME_X, HOME_Y, HOME_Z, HOME_RX, HOME_RY, HOME_RZ], speed, 0)
#     wait_until_stopped()
#     time.sleep(stabilize)
#     actual = mc.get_coords()
#     print(f"  원점 실제 도달: Y={actual[1]:.2f}mm, Z={actual[2]:.2f}mm")
#     return actual

# def send_coords_fixed_x(target_coords, speed, stabilize=1.5):
#     fixed = list(target_coords)
#     fixed[0] = HOME_X
#     mc.send_coords(fixed, speed, 0)
#     wait_until_stopped()
#     time.sleep(stabilize)


# # ─────────────────────────────────────────
# # Y축 측정용 기준점 (Z -50, Y중립)
# # ─────────────────────────────────────────
# def go_y_baseline(speed=30, stabilize=1.5):
#     """원점 → Z -50 → 기준점"""
#     home = go_home(speed=50, stabilize=1.0)
#     target = list(home)
#     target[0] = HOME_X
#     target[2] += Z_OFFSET_FOR_Y + NEG_Z_OFFSET
#     send_coords_fixed_x(target, speed, stabilize=stabilize)
#     actual = mc.get_coords()
#     print(f"  Y기준점 도달: Y={actual[1]:.2f}mm, Z={actual[2]:.2f}mm")
#     return actual


# # ─────────────────────────────────────────
# # Y축 오차 측정 (기준점 복귀 후 1회 측정)
# # ─────────────────────────────────────────
# def measure_y_raw(desired_delta, baseline_coords, speed=30):
#     # 기준점으로 복귀
#     send_coords_fixed_x(list(baseline_coords), speed, stabilize=1.5)
#     start = mc.get_coords()
#     if start is None:
#         return None

#     target = list(start)
#     target[0] = HOME_X
#     target[1] += desired_delta
#     send_coords_fixed_x(target, speed, stabilize=1.5)

#     actual = mc.get_coords()
#     if actual is None:
#         return None

#     actual_delta = actual[1] - start[1]
#     error = actual_delta - desired_delta
#     print(f"  목표: {desired_delta:+6.1f}mm | 실제: {actual_delta:+6.2f}mm | 오차: {error:+5.2f}mm")
#     return error


# # ─────────────────────────────────────────
# # 메인: 20~50mm 범위만 측정
# # ─────────────────────────────────────────
# if __name__ == "__main__":

#     print("\n로봇을 초기 위치로 리셋합니다.")
#     go_home(stabilize=2.0)
#     mc.set_gripper_value(100, 50)
#     time.sleep(1.5)
#     print("리셋 완료\n")

#     # 실제 작업 범위 20~50mm에 집중
#     neg_deltas = [-20, -30, -40, -50]
#     pos_deltas = [ 20,  30,  40,  50]

#     neg_errors, pos_errors = [], []

#     # ── 음수 방향 ──
#     print("=== Y축 음수 방향 측정 (20~50mm) ===")
#     baseline = go_y_baseline(stabilize=1.5)
#     for delta in neg_deltas:
#         err = measure_y_raw(delta, baseline)
#         if err is not None:
#             neg_errors.append(err)

#     # ── 양수 방향: 기준점을 Y -50으로 설정 후 측정 ──
#     print("\n=== Y축 양수 방향 측정 (기준점 Y-50 → +20~+50mm) ===")
#     # 음수 기준점에서 -50mm 더 이동해서 양수 측정 기준점 확보
#     send_coords_fixed_x(list(baseline), 30, stabilize=1.0)
#     start = mc.get_coords()
#     target = list(start)
#     target[0] = HOME_X
#     target[1] += -50  # Y -50 기준점
#     send_coords_fixed_x(target, 30, stabilize=1.5)
#     pos_baseline = mc.get_coords()
#     print(f"  양수 기준점 도달: Y={pos_baseline[1]:.2f}mm")

#     for delta in pos_deltas:
#         err = measure_y_raw(delta, pos_baseline)
#         if err is not None:
#             pos_errors.append(err)

#     go_home(stabilize=1.0)

#     # ── 결과 요약 ──
#     print(f"\n=== Y축 오차 요약 (20~50mm 범위) ===")
#     print(f"  음수 방향 오차 평균: {np.mean(neg_errors):+.2f}mm")
#     print(f"  양수 방향 오차 평균: {np.mean(pos_errors):+.2f}mm")
#     print(f"  음수 방향 오차 std : {np.std(neg_errors):.3f}mm")
#     print(f"  양수 방향 오차 std : {np.std(pos_errors):.3f}mm")
#     print(f"\n  → 권장 NEG_Y_OFFSET: {-np.mean(neg_errors):+.2f}mm")
#     print(f"  → 권장 POS_Y_OFFSET: {-np.mean(pos_errors):+.2f}mm")

#     print("\n=== 데이터 (Claude에게 붙여넣기) ===")
#     print("neg_deltas =", neg_deltas)
#     print("neg_errors =", [round(e, 2) for e in neg_errors])
#     print("pos_deltas =", pos_deltas)
#     print("pos_errors =", [round(e, 2) for e in pos_errors])

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

Z_OFFSET_FOR_Y = -50.0
NEG_Z_OFFSET   = +3.55

# ─────────────────────────────────────────
# Y축 오프셋 확정값
# ─────────────────────────────────────────
NEG_Y_OFFSET = +4.15
POS_Y_OFFSET = +5.05


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

def go_y_baseline(speed=30, stabilize=1.5):
    home = go_home(speed=50, stabilize=1.0)
    target = list(home)
    target[0] = HOME_X
    target[2] += Z_OFFSET_FOR_Y + NEG_Z_OFFSET
    send_coords_fixed_x(target, speed, stabilize=stabilize)
    actual = mc.get_coords()
    print(f"  Y기준점 도달: Y={actual[1]:.2f}mm, Z={actual[2]:.2f}mm")
    return actual


# ─────────────────────────────────────────
# Y축 보정 함수
# ─────────────────────────────────────────
def correct_y(desired_delta):
    if desired_delta < 0:
        return desired_delta + NEG_Y_OFFSET
    elif desired_delta > 0:
        return desired_delta + POS_Y_OFFSET
    return 0.0


# ─────────────────────────────────────────
# 검증: 보정 전/후 비교
# ─────────────────────────────────────────
def verify_y(desired_delta, baseline_coords, speed=30):
    results = {}

    for label, correct_fn in [
        ("보정 없음", lambda d: d),
        ("오프셋",    correct_y),
    ]:
        send_coords_fixed_x(list(baseline_coords), speed, stabilize=1.5)
        start = mc.get_coords()
        if start is None:
            continue

        corrected = correct_fn(desired_delta)
        target = list(start)
        target[0] = HOME_X
        target[1] += corrected
        send_coords_fixed_x(target, speed, stabilize=1.5)

        actual = mc.get_coords()
        if actual is None:
            continue

        actual_delta = actual[1] - start[1]
        error = actual_delta - desired_delta
        print(f"  [{label:5s}] 명령: {corrected:+6.2f}mm | 실제: {actual_delta:+6.2f}mm | 오차: {error:+5.2f}mm")
        results[label] = error

    return results


# ─────────────────────────────────────────
# 메인
# ─────────────────────────────────────────
if __name__ == "__main__":

    print("\n로봇을 초기 위치로 리셋합니다.")
    go_home(stabilize=2.0)
    mc.set_gripper_value(100, 50)
    time.sleep(1.5)
    print("리셋 완료\n")

    print("=== Y축 보정 검증 ===")
    print(f"  NEG_Y_OFFSET={NEG_Y_OFFSET:+.2f}mm")
    print(f"  POS_Y_OFFSET={POS_Y_OFFSET:+.2f}mm\n")

    # 음수 방향 검증
    print("── 음수 방향 ──")
    neg_baseline = go_y_baseline(stabilize=1.5)
    neg_results = []
    for delta in [-25, -35, -45]:
        print(f"\n  delta={delta:+.1f}mm")
        res = verify_y(delta, neg_baseline)
        neg_results.append((delta, res))

    # 양수 방향 기준점 설정
    print("\n── 양수 방향 ──")
    send_coords_fixed_x(list(neg_baseline), 30, stabilize=1.0)
    start = mc.get_coords()
    target = list(start)
    target[0] = HOME_X
    target[1] += -50
    send_coords_fixed_x(target, 30, stabilize=1.5)
    pos_baseline = mc.get_coords()
    print(f"  양수 기준점 도달: Y={pos_baseline[1]:.2f}mm")

    pos_results = []
    for delta in [25, 35, 45]:
        print(f"\n  delta={delta:+.1f}mm")
        res = verify_y(delta, pos_baseline)
        pos_results.append((delta, res))

    go_home(stabilize=1.0)

    # ── 결과 요약 ──
    print("\n=== 검증 결과 요약 ===")
    print(f"{'목표':>8} | {'보정없음':>8} | {'오프셋':>8}")
    print("-" * 34)
    for delta, res in neg_results + pos_results:
        print(f"  {delta:+6.1f}mm "
              f"| {res.get('보정 없음', 0):+7.2f}mm "
              f"| {res.get('오프셋',    0):+7.2f}mm")

    all_results = neg_results + pos_results
    offset_errs = [abs(r[1].get('오프셋', 0)) for r in all_results]
    print(f"\n  오프셋 평균 절대 오차: {np.mean(offset_errs):.2f}mm")