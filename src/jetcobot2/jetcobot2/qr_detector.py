import cv2
import json
import numpy as np
from pyzbar import pyzbar


def scan_qr(frame, target_item: str = None):
    h, w  = frame.shape[:2]
    mid_x = w // 2

    # ✅ 전처리 추가 - 인식률 향상
    gray      = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # 명암 대비 강화
    clahe     = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    enhanced  = clahe.apply(gray)
    # 이진화
    _, binary = cv2.threshold(enhanced, 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 원본과 전처리 둘 다 시도
    for img in [frame, binary]:
        codes = pyzbar.decode(img)
        for code in codes:
            try:
                data   = code.data.decode('utf-8')
                parsed = json.loads(data)

                if 'box_id' not in parsed or 'item' not in parsed:
                    continue

                cx       = code.rect.left + code.rect.width  / 2
                cy       = code.rect.top  + code.rect.height / 2
                position = 'left' if cx < mid_x else 'right'

                parsed['cx']       = cx
                parsed['cy']       = cy
                parsed['position'] = position

                if target_item:
                    if parsed.get('item') == target_item:
                        return parsed
                else:
                    return parsed
            except:
                continue

    return None


def draw_qr_debug(frame, target_item: str = None):
    """QR 코드 감지 결과 시각화"""
    output = frame.copy()
    h, w   = frame.shape[:2]
    mid_x  = w // 2
    codes  = pyzbar.decode(frame)
    found  = None

    # 화면 중앙선 표시
    cv2.line(output, (mid_x, 0), (mid_x, h), (255, 255, 0), 2)

    for code in codes:
        points = code.polygon
        if len(points) == 4:
            pts = np.array([[p.x, p.y] for p in points], dtype=np.int32)
            cv2.polylines(output, [pts], True, (0, 255, 0), 2)

        try:
            data   = code.data.decode('utf-8')
            parsed = json.loads(data)
            item   = parsed.get('item', '')
            box_id = parsed.get('box_id', '')

            cx       = code.rect.left + code.rect.width  / 2
            cy       = code.rect.top  + code.rect.height / 2
            position = 'left' if cx < mid_x else 'right'

            if target_item and item == target_item:
                color = (0, 0, 255)  # 빨간색 (일치)
                found = {**parsed, 'cx': cx, 'cy': cy, 'position': position}
            else:
                color = (0, 255, 0)  # 초록색 (불일치)

            cv2.putText(output,
                        f"{box_id} / {item} / {position}",
                        (code.rect.left, code.rect.top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.circle(output, (int(cx), int(cy)), 5, color, -1)

        except:
            continue

    return output, found