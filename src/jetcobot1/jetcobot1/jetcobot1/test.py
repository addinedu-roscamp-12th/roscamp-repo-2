# # import os
# # import time
# # import threading
# # from pymycobot.mycobot280 import MyCobot280
# # from pymycobot.genre import Angle, Coord

# # mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
# # mc.thread_lock = True
# # print("로봇이 연결되었습니다.")

# # # 현재 각도 읽기
# # angles = mc.get_angles()
# # print("현재 각도:", angles)
# # # 현재 좌표 읽기
# # coords = mc.get_coords()
# # print("현재 좌표:", coords)
# # # 인코더 값 읽기
# # encoders = mc.get_encoders()
# # print("인코더:", encoders)
# # # 라디안 값 읽기
# # radians = mc.get_radians()
# # print("라디안:", radians)

# # ANGLE_MIN = [-168, -135, -150, -145, -165, -180, 0]
# # ANGLE_MAX = [168, 135, 150, 145, 165, 180, 100]
# # for i in range(7):
# #     print(f"관절 {i+1}: {ANGLE_MIN[i]} ~ {ANGLE_MAX[i]}도")

# # # 로봇을 초기 위치로 리셋
# # initial_angles = [0, 0, 0, 0, 0, 0]
# # speed = 50
# # print("로봇을 초기 위치로 리셋합니다.")
# # mc.send_angles(initial_angles, speed)
# # mc.set_gripper_value(100, speed) # 그리퍼 열기
# # time.sleep(3) # 움직임이 완료될 때까지 대기

# # print("리셋 완료")

# # # 현재 좌표 확인
# # current_coords = mc.get_coords()
# # print("현재 좌표:", current_coords)


# # # # 1. 먼저 Z축을 낮추기
# # # work_coords = current_coords.copy()
# # # work_coords[2] -= 50 # Z를 50mm 내리기
# # # print(f"Z축을 {work_coords[2]}로 내립니다.")
# # # mc.send_coords(work_coords, 30, 0)
# # # time.sleep(2)
# # # final_coords = mc.get_coords()
# # # print("Z축 좌표:", final_coords)

# # # # # 2. X 좌표 이동
# # # # x_coords = work_coords.copy()
# # # # x_coords[0] += 20 # X + 20mm
# # # # print(f"X 좌표를 {x_coords[0]}로 이동합니다.")
# # # # mc.send_coords(x_coords, 30, 0)
# # # # time.sleep(2)
# # # # final_coords = mc.get_coords()
# # # # print("X축 좌표:", final_coords)

# # # # 3. Y 좌표 이동
# # # y_coords = work_coords.copy()
# # # y_coords[1] -= 20 # Y - 20mm
# # # print(f"Y 좌표를 {y_coords[1]}로 이동합니다.")
# # # mc.send_coords(y_coords, 30, 0)
# # # time.sleep(2)
# # # final_coords = mc.get_coords()
# # # print("Y축 좌표:", final_coords)

# # # # 4. 최종 좌표 확인
# # # final_coords = mc.get_coords()
# # # print("최종 좌표:", final_coords)

# # # # 5. 초기 위치로 복귀
# # # print("초기 위치로 복귀합니다.")
# # # mc.send_angles([0, 0, 0, 0, 0, 0], 50)
# # # time.sleep(3)
# # # print("초기 위치 복귀 완료")

# import time
# import numpy as np
# from pymycobot.mycobot280 import MyCobot280

# mc = MyCobot280('/dev/ttyJETCOBOT', 1000000)
# mc.thread_lock = True
# print("로봇이 연결되었습니다.")

# # 현재 상태 출력
# angles = mc.get_angles()
# print("현재 각도:", angles)
# coords = mc.get_coords()
# print("현재 좌표:", coords)
# encoders = mc.get_encoders()
# print("인코더:", encoders)
# radians = mc.get_radians()
# print("라디안:", radians)

# ANGLE_MIN = [-168, -135, -150, -145, -165, -180, 0]
# ANGLE_MAX  = [168, 135, 150, 145, 165, 180, 100]
# for i in range(7):
#     print(f"관절 {i+1}: {ANGLE_MIN[i]} ~ {ANGLE_MAX[i]}도")


# # ─────────────────────────────────────────
# # 유틸: 이동 완료까지 대기
# # ─────────────────────────────────────────
# def wait_until_stopped(timeout=10, threshold=0.5):
#     prev_angles = mc.get_angles()
#     start = time.time()
#     while time.time() - start < timeout:
#         time.sleep(0.3)
#         curr_angles = mc.get_angles()
#         if curr_angles is None or prev_angles is None:
#             continue
#         delta = max(abs(c - p) for c, p in zip(curr_angles, prev_angles))
#         if delta < threshold:
#             print(f"  → 이동 완료 (최대 변화량: {delta:.3f}도)")
#             return True
#         prev_angles = curr_angles
#     print("  → 타임아웃: 이동이 완료되지 않았을 수 있음")
#     return False


# # ─────────────────────────────────────────
# # 원점 복귀
# # ─────────────────────────────────────────
# def go_home(speed=50):
#     mc.send_angles([0, 0, 0, 0, 0, 0], speed)
#     wait_until_stopped()
#     time.sleep(0.5)

# def go_angle(speed=50):
#     mc.send_angles([+90, 0, 0, 0, 0, 0], speed)
#     wait_until_stopped()
#     time.sleep(0.5)


# # # ─────────────────────────────────────────
# # # HOME_X 자동 캘리브레이션
# # # ─────────────────────────────────────────
# # def calibrate_home_x(n=5, speed=50):
# #     """원점에서 N회 X좌표 측정 평균으로 HOME_X 결정"""
# #     print(f"\nHOME_X 캘리브레이션 시작 ({n}회 측정)...")
# #     xs = []
# #     for i in range(n):
# #         go_home(speed)
# #         coords = mc.get_coords()
# #         if coords is None:
# #             print(f"  측정 {i+1}: 좌표 읽기 실패, 스킵")
# #             continue
# #         xs.append(coords[0])
# #         print(f"  측정 {i+1}: X = {coords[0]:.2f}mm")

# #     if len(xs) == 0:
# #         raise RuntimeError("HOME_X 캘리브레이션 실패: 유효한 측정값 없음")

# #     home_x = np.mean(xs)
# #     print(f"→ HOME_X 확정: {home_x:.2f}mm (std: {np.std(xs):.3f}mm)")
# #     return home_x


# # # ─────────────────────────────────────────
# # # X 고정 좌표 이동
# # # ─────────────────────────────────────────
# # def send_coords_fixed_x(target_coords, speed, mode=0):
# #     """X축은 항상 HOME_X로 고정하여 이동"""
# #     fixed = target_coords.copy()
# #     fixed[0] = HOME_X
# #     mc.send_coords(fixed, speed, mode)


# # ─────────────────────────────────────────
# # 메인
# # ─────────────────────────────────────────
# speed = 50
# print("\n로봇을 초기 위치로 리셋합니다.")
# go_home(speed)
# go_angle(speed)
# print("현재 좌표:", mc.get_coords())
# print("현재 각도:", mc.get_angles())

# # 그리퍼 열기
# print("그리퍼를 엽니다.")
# mc.set_gripper_value(100, speed)
# time.sleep(1.5)

# # # HOME_X 캘리브레이션
# # HOME_X = calibrate_home_x(n=5, speed=speed)

# print("\n리셋 완료")
# # current_coords = mc.get_coords()
# # print("현재 좌표:", current_coords)
# # print(f"이후 X축 고정값: {HOME_X:.2f}mm")

# check_place_coords.py
from pymycobot.mycobot280 import MyCobot280
import time

mc = MyCobot280("/dev/ttyJETCOBOT", 1000000)


angles = mc.get_angles()
print(angles)
# J1=90 유지하면서 팔을 테이블 방향으로 뻗기
# mc.send_angles([0,0,0,0,0,0], 20)
# time.sleep(4)
# mc.send_angles([130.69, -9.49, -69.43, -9.93, -2.37, -4.48], 20)  # 팔 앞으로 뻗기
# time.sleep(4)
# mc.send_angles([0,0,0,0,0,0], 20)
# time.sleep(4)
# mc.send_angles([105.38, -11.77, -75.67, 1.23, 0.17, -24.16], 20)
# time.sleep(4)
# mc.send_angles([0,0,0,0,0,0], 20)
# time.sleep(4)
# mc.send_angles([76.46, -13.88, -74.79, 5.18, -2.63, -56.42], 20)
# time.sleep(4)



