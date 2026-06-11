"""
cv_lib.py ─ OpenCV 블럭 검출 라이브러리

역할:
  - process_frame() : 한 프레임에서 흰 블럭 검출
  - get_last_detection() : 최신 검출 결과 반환

분리 이유:
  - ROS2 노드(cv_node.py)와 영상처리 로직을 독립적으로 관리
  - Place 구현 시 이 파일만 교체하면 됨
  - ROS2 없이 단독으로 테스트 가능
"""

import threading
import cv2
import numpy as np


# ──────────────────────────────────────────────
# HSV 흰색 범위 상수
# H: 0~180 (흰색은 무채색이라 전 범위 허용)
# S: 0~50  (채도 낮을수록 흰색에 가까움)
# V: 200~255 (명도 높을수록 밝음)
# ──────────────────────────────────────────────
WHITE_LOWER = np.array([0,   0,   200], dtype=np.uint8)
WHITE_UPPER = np.array([180, 50,  255], dtype=np.uint8)

MIN_AREA            = 3000   # 블럭으로 인정할 최소 컨투어 면적 (픽셀²)
APPROX_EPSILON_RATIO = 0.04  # 사각형 판별 근사 정확도 (컨투어 둘레의 4%)

# 최신 검출 결과 전역 저장
_last_detection: tuple = (False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
_last_frame     = None   # rack_detector용 최신 프레임
_detection_lock = threading.Lock()


def process_frame(img: np.ndarray) -> tuple:
    """
    한 프레임에서 흰 블럭을 검출합니다.

    Args:
        img: BGR 이미지 (OpenCV 기본 포맷)

    Returns:
        (detected, cx, cy, area, hor, ver, ar, output)
            detected : 블럭 발견 여부
            cx, cy   : 블럭 중심 픽셀 좌표
            area     : 컨투어 면적 (픽셀²)
            hor, ver : 가로/세로 픽셀 길이
            ar       : 가로/세로 비율
            output   : 시각화된 이미지
    """
    global _last_detection

    output = img.copy()

    # 가우시안 블러 → BGR→HSV → 흰색 마스크
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)

    # 모폴로지 연산 (노이즈 제거 + 구멍 메움)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 컨투어 검출
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected = False
    result_cx = result_cy = result_area = 0.0
    result_hor = result_ver = result_ar  = 0.0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_AREA:
            continue

        # 다각형 근사
        peri   = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, APPROX_EPSILON_RATIO * peri, True)

        # 꼭짓점 4개 + 볼록 도형 → 사각형 판단
        if len(approx) != 4 or not cv2.isContourConvex(approx):
            continue

        # 중심 좌표 계산
        M = cv2.moments(cnt)
        if M['m00'] == 0:
            continue
        cx = float(M['m10'] / M['m00'])
        cy = float(M['m01'] / M['m00'])

        # 4변 길이 계산
        sides = [
            int(np.linalg.norm(approx[(i + 1) % 4][0] - approx[i][0]))
            for i in range(4)
        ]
        aspect_ratio = sides[0] / sides[1] if sides[1] != 0 else 0

        # 크기 조건 + 정사각형 비율 조건
        if not all(s > 150 for s in sides):
            continue
        if not (0.6 < aspect_ratio < 1.4):
            continue

        # 시각화
        x, y, w, h = cv2.boundingRect(approx)
        perimeter   = cv2.arcLength(approx, True)
        cv2.drawContours(output, [approx], 0, (0, 255, 0), 2)
        cv2.rectangle(output, (x, y), (x + w, y + h), (255, 165, 0), 1)
        cv2.circle(output, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        cv2.putText(output, f"Perimeter: {int(perimeter)}px, AR: {aspect_ratio:.2f}",
                    (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(output, f"Sides: {sides[0]} {sides[1]} {sides[2]} {sides[3]}",
                    (x, y - 5),  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(output, f"({cx:.0f}, {cy:.0f})",
                    (int(cx), int(cy) + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 0, 0), 1)

        # 첫 번째 사각형만 반환값으로 사용
        if not detected:
            result_cx, result_cy, result_area = cx, cy, float(area)
            result_hor, result_ver, result_ar = sides[0], sides[1], aspect_ratio
            detected = True

    if detected:
        cv2.putText(output, "BLOCK DETECTED",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

    with _detection_lock:
        _last_detection = (detected, result_cx, result_cy, result_area,
                           result_hor, result_ver, result_ar)
        globals()['_last_frame'] = output  # rack_detector용 최신 프레임 저장

    return detected, result_cx, result_cy, result_area, result_hor, result_ver, result_ar, output


def get_last_frame():
    """
    가장 최근 프레임 반환 (rack_detector 용)
    Place 시 랙 빈 공간 감지에 사용
    """
    with _detection_lock:
        return _last_frame


def get_last_detection() -> tuple:
    """가장 최근 process_frame() 결과 반환"""
    with _detection_lock:
        return _last_detection