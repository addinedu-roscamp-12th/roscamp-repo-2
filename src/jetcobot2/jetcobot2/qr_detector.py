import cv2
import json
import math
import numpy as np
from pyzbar import pyzbar

# ──────────────────────────────────────────────
# QR 코드 물리 파라미터 (실측값)
# ──────────────────────────────────────────────
QR_REAL_SIZE_CM   = 3.0   # QR 코드 실제 크기 (cm)
FOCAL_LENGTH      = 224   # 카메라 초점 거리 (픽셀)
OPTIMAL_DIST_CM   = 9.0   # 최적 인식 거리 (cm)
DIST_TOLERANCE_CM = 1.0   # 허용 거리 오차 (cm)


def estimate_distance(qr_pixel_size: float) -> float:
    """QR 코드 픽셀 크기로 카메라-QR 거리 추정"""
    if qr_pixel_size <= 0:
        return -1.0
    return round((QR_REAL_SIZE_CM * FOCAL_LENGTH) / qr_pixel_size, 1)


def estimate_tilt_angle(code) -> float:
    """
    QR 코드 꼭짓점으로 기울기 각도 추정
    polygon이 4개 점이 아니면(인식이 불완전한 경우)
    매우 큰 각도를 반환하여 in_angle 조건에서 걸러지도록 함

    Returns:
        추정 각도 (도)
    """
    points = code.polygon
    if len(points) != 4:
        return 999.0   # 각도 측정 불가 → 신뢰할 수 없는 인식으로 간주

    pts = np.array([[p.x, p.y] for p in points], dtype=np.float32)

    top    = np.linalg.norm(pts[1] - pts[0])
    bottom = np.linalg.norm(pts[2] - pts[3])
    left   = np.linalg.norm(pts[3] - pts[0])
    right  = np.linalg.norm(pts[2] - pts[1])

    h_ratio = min(top, bottom)   / max(top, bottom)   if max(top, bottom)   > 0 else 1.0
    v_ratio = min(left, right)   / max(left, right)   if max(left, right)   > 0 else 1.0

    h_angle = math.degrees(math.acos(max(0.0, min(1.0, h_ratio))))
    v_angle = math.degrees(math.acos(max(0.0, min(1.0, v_ratio))))

    return round(max(h_angle, v_angle), 1)


def reduce_glare(frame):
    """
    빛 번짐(과다 노출) 완화
    - 밝은 영역을 눌러주고 어두운 영역을 살려서
      흑백 대비를 다시 살림
    """
    gamma = 1.8
    inv_gamma = 1.0 / gamma
    table = np.array([
        ((i / 255.0) ** inv_gamma) * 255
        for i in range(256)
    ]).astype("uint8")
    corrected = cv2.LUT(frame, table)
    return corrected


def scan_qr(frame, target_item: str = None,
            max_angle_deg: float = 45.0,
            debug: bool = False) -> dict | None:
    """
    프레임에서 QR 코드 스캔 (각도 조건 포함)

    Args:
        frame:         BGR 이미지
        target_item:   찾을 품목명 (None이면 전체)
        max_angle_deg: 허용 최대 기울기 각도 (기본 45도)
        debug:         True면 단계별 print 로그 출력

    Returns:
        {
            'box_id':         'BOX-001',
            'item':           '물병',
            'position':       'left' or 'right',
            'cx':             320.0,
            'cy':             240.0,
            'distance_cm':    9.5,
            'distance_error': 0.5,
            'in_range':       True,
            'qr_pixel_size':  150.0,
            'tilt_angle':     15.0,
            'in_angle':       True,
        } 또는 None
    """
    h, w  = frame.shape[:2]
    mid_x = w // 2

    # 전처리 - 인식률 향상
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 감마 보정으로 과다노출(빛 번짐) 완화
    gamma_corrected = reduce_glare(gray)

    # 적응형 이진화 (그림자/번짐에 강함)
    adaptive = cv2.adaptiveThreshold(
        gamma_corrected, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY, 15, 5
    )

    # ✅ 인식 안정화 확인 완료 → 핵심 후보만 사용 (성능 개선)
    candidates = [frame, adaptive]

    if debug:
        raw_count = sum(len(pyzbar.decode(img)) for img in candidates)
        print(f"    [scan_qr] pyzbar 원시 감지 수(후보 {len(candidates)}종 합): {raw_count}")

    for img in candidates:
        codes = pyzbar.decode(img)
        for code in codes:
            try:
                data   = code.data.decode('utf-8')
                parsed = json.loads(data)
            except Exception:
                continue

            if 'box_id' not in parsed or 'item' not in parsed:
                continue

            # 각도 추정 및 조건 체크
            tilt_angle = estimate_tilt_angle(code)
            in_angle   = tilt_angle <= max_angle_deg

            if not in_angle:
                if debug:
                    print(f"    [scan_qr] 각도 초과로 스킵 → "
                          f"box_id={parsed.get('box_id')}, tilt_angle={tilt_angle}")
                continue  # 각도 초과 → 스킵

            cx       = code.rect.left + code.rect.width  / 2
            cy       = code.rect.top  + code.rect.height / 2
            position = 'left' if cx < mid_x else 'right'

            qr_pixel_size  = (code.rect.width + code.rect.height) / 2
            distance_cm    = estimate_distance(qr_pixel_size)
            distance_error = round(distance_cm - OPTIMAL_DIST_CM, 1)
            in_range       = abs(distance_error) <= DIST_TOLERANCE_CM

            parsed['cx']             = cx
            parsed['cy']             = cy
            parsed['position']       = position
            parsed['distance_cm']    = distance_cm
            parsed['distance_error'] = distance_error
            parsed['in_range']       = in_range
            parsed['qr_pixel_size']  = round(qr_pixel_size, 1)
            parsed['tilt_angle']     = tilt_angle
            parsed['in_angle']       = in_angle

            if target_item and parsed.get('item') != target_item:
                if debug:
                    print(f"    [scan_qr] item 불일치 → "
                          f"box_id={parsed.get('box_id')}, item={parsed.get('item')}")
                continue

            if debug:
                print(f"    [scan_qr] 매칭 성공 → box_id={parsed.get('box_id')}, item={parsed.get('item')}")
            return parsed

    return None


def draw_qr_debug(frame, target_item: str = None,
                  max_angle_deg: float = 45.0):
    """
    QR 코드 감지 결과 시각화
    Returns:
        (output, found_data)
    """
    output = frame.copy()
    h, w   = frame.shape[:2]
    mid_x  = w // 2
    codes  = pyzbar.decode(frame)
    found  = None

    cv2.putText(output,
                f"Optimal dist: {OPTIMAL_DIST_CM}cm (±{DIST_TOLERANCE_CM}cm)",
                (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    for code in codes:
        points = code.polygon
        if len(points) == 4:
            pts = np.array([[p.x, p.y] for p in points], dtype=np.int32)
            cv2.polylines(output, [pts], True, (0, 255, 0), 2)

        try:
            data   = code.data.decode('utf-8')
            parsed = json.loads(data)
            item   = parsed.get('item',   '')
            box_id = parsed.get('box_id', '')

            cx             = code.rect.left + code.rect.width  / 2
            cy             = code.rect.top  + code.rect.height / 2
            qr_pixel_size  = (code.rect.width + code.rect.height) / 2
            distance_cm    = estimate_distance(qr_pixel_size)
            distance_error = round(distance_cm - OPTIMAL_DIST_CM, 1)
            in_range       = abs(distance_error) <= DIST_TOLERANCE_CM
            position       = 'left' if cx < mid_x else 'right'
            tilt_angle     = estimate_tilt_angle(code)
            in_angle       = tilt_angle <= max_angle_deg

            if target_item and item == target_item:
                color = (0, 0, 255)
                found = {
                    **parsed,
                    'cx': cx, 'cy': cy,
                    'position':       position,
                    'distance_cm':    distance_cm,
                    'distance_error': distance_error,
                    'in_range':       in_range,
                    'qr_pixel_size':  round(qr_pixel_size, 1),
                    'tilt_angle':     tilt_angle,
                    'in_angle':       in_angle,
                }
            else:
                color = (0, 255, 0)

            dist_text   = f"{distance_cm}cm ({distance_error:+.1f})"
            range_color = (0, 255, 0) if in_range else (0, 0, 255)
            angle_color = (0, 255, 0) if in_angle else (0, 0, 255)

            cv2.putText(output,
                        f"{box_id} / {position}",
                        (code.rect.left, code.rect.top - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(output,
                        f"dist: {dist_text}",
                        (code.rect.left, code.rect.top - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_color, 1)
            cv2.putText(output,
                        f"angle: {tilt_angle}deg ({'OK' if in_angle else 'NG'})",
                        (code.rect.left, code.rect.top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, angle_color, 1)
            cv2.circle(output, (int(cx), int(cy)), 5, color, -1)

        except:
            continue

    return output, found