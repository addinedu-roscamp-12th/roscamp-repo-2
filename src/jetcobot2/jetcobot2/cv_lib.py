"""

cv_lib.py ─ OpenCV 블럭 검출 라이브러리

QR 코드(흰색 바탕)를 ROI 안에서 감지하여 박스 유무 판단

"""

import threading

import time

import cv2

import numpy as np



# ──────────────────────────────────────────────

# HSV 흰색 범위 (QR 코드 흰색 바탕 감지용)

# S 상한을 80으로 제한 → 채도가 있는 색(빨강/파랑 등)이

# 밝기(V)만 높다는 이유로 흰색에 함께 잡히는 것을 방지

# ──────────────────────────────────────────────

WHITE_LOWER = np.array([0,   0,   180], dtype=np.uint8)

WHITE_UPPER = np.array([180, 80,  255], dtype=np.uint8)



# ──────────────────────────────────────────────

# ROI 설정 (운반대 영역 - 실측 후 교체 필요)

# Flask 스트림에서 운반대가 보이는 영역

# ──────────────────────────────────────────────

ROI_SIZE = 400  # ← 조정 가능



CENTER_X = 640 // 2  # = 320

CENTER_Y = 480 // 2  # = 240



ROI_X1 = CENTER_X - ROI_SIZE // 2  # = 320 - 200 = 120

ROI_Y1 = CENTER_Y - ROI_SIZE // 2  # = 240 - 200 = 40

ROI_X2 = CENTER_X + ROI_SIZE // 2  # = 320 + 200 = 520

ROI_Y2 = CENTER_Y + ROI_SIZE // 2  # = 240 + 200 = 440



MIN_AREA             = 500   # QR 코드 흰색 영역 최소 면적

APPROX_EPSILON_RATIO = 0.04



_last_detection: tuple = (False, 0.0, 0.0)

_last_frame     = None

_detection_lock = threading.Lock()

_last_proc_time_ms: float = 0.0   # process_frame() 마지막 처리 시간(ms), 성능 측정용





def process_frame(img: np.ndarray) -> tuple:

    global _last_detection, _last_proc_time_ms



    _t0 = time.time()   # 처리 시간 측정 시작



    output = img.copy()



    # ROI 영역 시각화

    cv2.rectangle(output,

                  (ROI_X1, ROI_Y1), (ROI_X2, ROI_Y2),

                  (255, 0, 0), 2)

    cv2.putText(output, "ROI",

                (ROI_X1, ROI_Y1 - 5),

                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)



    # ROI 영역만 추출

    roi = img[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]



    # 흰색 마스크

    # 커널 55x55 → 21x21로 축소 (연산량 약 6.9배 감소, 인식 정확도 영향 없음 실측 확인됨)

    blur = cv2.GaussianBlur(roi, (13, 13), 0)

    hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)



    # 모폴로지 연산

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)



    detected   = False

    result_cx  = result_cy = 0.0



    # 사각형 판별 대신 흰색 픽셀 면적으로 1차 판별

    white_area     = cv2.countNonZero(mask)

    MIN_WHITE_AREA = 5000  # ← 실측 후 조정



    if white_area > MIN_WHITE_AREA:

        # 2차 판별: 사각형 컨투어 검출

        contours, _ = cv2.findContours(

            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE

        )



        for cnt in contours:

            area = cv2.contourArea(cnt)



            if area < MIN_AREA:

                continue



            peri   = cv2.arcLength(cnt, True)

            approx = cv2.approxPolyDP(cnt, APPROX_EPSILON_RATIO * peri, True)

            n_pts  = len(approx)



            # 4~8개 사이로 허용 (기울어짐, 왜곡 허용)

            if not (4 <= n_pts <= 8):

                continue



            M = cv2.moments(cnt)

            if M['m00'] == 0:

                continue



            # ROI 좌표 → 전체 프레임 좌표 변환

            cx = float(M['m10'] / M['m00']) + ROI_X1

            cy = float(M['m01'] / M['m00']) + ROI_Y1



            # 수정: approx의 실제 점 개수(n_pts)만큼 변 길이를 계산

            # (이전에는 항상 첫 4개 점만 사용해서, 5~8개 점으로

            #  검출된 도형에서는 변 길이/비율이 실제와 다르게 계산됐음)

            sides = [

                int(np.linalg.norm(approx[(i+1) % n_pts][0] - approx[i][0]))

                for i in range(n_pts)

            ]



            # 크기 조건 - 모든 변이 일정 길이 이상이어야 함

            if not all(s > 30 for s in sides):

                continue



            # 비율 조건 - 가장 긴 변과 가장 짧은 변의 비율로 판단

            # (4개일 때의 sides[0]/sides[1] 방식은 점 순서에 따라

            #  결과가 들쑥날쑥할 수 있어, min/max 비율로 일관성 있게 계산)

            aspect_ratio = min(sides) / max(sides) if max(sides) != 0 else 0



            # 비율 조건 (기울어짐 고려해서 넓게) - 정사각형에 가까운지 확인

            if not (0.5 < aspect_ratio <= 1.0):

                continue



            # ROI 좌표 → 전체 프레임 좌표 변환 후 시각화

            approx_global = approx.copy()

            approx_global[:, :, 0] += ROI_X1

            approx_global[:, :, 1] += ROI_Y1



            x, y, w, h = cv2.boundingRect(approx_global)



            cv2.drawContours(output, [approx_global], 0, (0, 255, 0), 2)

            cv2.rectangle(output, (x, y), (x + w, y + h), (255, 165, 0), 1)

            cv2.circle(output, (int(cx), int(cy)), 5, (0, 0, 255), -1)

            cv2.putText(output,

                        f"({cx:.0f}, {cy:.0f})",

                        (int(cx), int(cy) + 12),

                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 0, 0), 1)



            if not detected:

                result_cx, result_cy = cx, cy

                detected = True



    if detected:

        cv2.putText(output, "BLOCK DETECTED",

                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)



    with _detection_lock:

        _last_detection = (detected, result_cx, result_cy)

        globals()['_last_frame'] = output



    _last_proc_time_ms = (time.time() - _t0) * 1000.0   # 처리 시간(ms) 기록



    return detected, result_cx, result_cy, output





def get_last_processing_time_ms() -> float:

    """가장 최근 process_frame() 호출의 처리 시간(ms)을 반환 (성능 측정용)"""

    return _last_proc_time_ms





def get_filtered_cx_cy(frame_getter, n_frames=5, feedback_cb=None):

    """

    N프레임 cx, cy 중앙값 반환 (이상값 제거)



    Args:

        frame_getter: 최신 프레임 반환 함수

        n_frames:     수집 프레임 수

        feedback_cb:  제공되면 검출 성공률, cx/cy 표준편차(흔들림 정도),

                      process_frame() 처리 시간(평균/최대 ms)을 보고함

                      (do_load의 ROS2 Feedback으로 전달 가능)



    Returns:

        (cx, cy) 또는 (None, None)

    """

    cx_list, cy_list   = [], []

    proc_times_ms      = []



    for _ in range(n_frames):

        time.sleep(0.1)

        frame = frame_getter()

        if frame is None:

            continue

        detected, cx, cy, _ = process_frame(frame)

        proc_times_ms.append(get_last_processing_time_ms())

        if detected:

            cx_list.append(cx)

            cy_list.append(cy)



    success_rate = len(cx_list) / n_frames if n_frames > 0 else 0.0



    avg_proc_ms = float(np.mean(proc_times_ms)) if proc_times_ms else 0.0

    max_proc_ms = float(np.max(proc_times_ms))  if proc_times_ms else 0.0



    if not cx_list:

        if feedback_cb:

            feedback_cb(

                f"[정확도] 검출 성공률: 0/{n_frames} (0.0%) → 박스 미감지 | "

                f"[속도] process_frame 평균 {avg_proc_ms:.1f}ms, 최대 {max_proc_ms:.1f}ms"

            )

        return None, None



    cx_arr = np.array(cx_list)

    cy_arr = np.array(cy_list)

    cx_std = float(np.std(cx_arr))

    cy_std = float(np.std(cy_arr))



    if feedback_cb:

        feedback_cb(

            f"[정확도] 검출 성공률: {len(cx_list)}/{n_frames} ({success_rate*100:.1f}%), "

            f"cx 표준편차: {cx_std:.2f}px, cy 표준편차: {cy_std:.2f}px "

            f"(값이 작을수록 흔들림이 적고 안정적) | "

            f"[속도] process_frame 평균 {avg_proc_ms:.1f}ms, 최대 {max_proc_ms:.1f}ms"

        )



    return float(np.median(cx_list)), float(np.median(cy_list))