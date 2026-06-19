import cv2
import numpy as np


class RackSpaceDetector:
    """
    랙 빈 공간 감지 클래스
    - 흰색 테이프로 감긴 박스 감지 방식
    - ROI 안에서 가운데 선 기준 좌/우 "주변 대비 밝은 영역" 비율로 박스 유무 판단
    - adaptiveThreshold 사용 → 절대 밝기가 아니라 상대 밝기로 판단하므로
      낮/밤 조명 변화에 강함 (기존 HSV V채널 절대값 방식의 문제를 해결)
    - 밝은 영역 있으면 → 박스있음
    - 밝은 영역 없으면 → 빈공간
    """
    def __init__(self,
                 white_ratio_threshold = 0.10,  # 흰색(밝은 영역) 비율 임계값 (10%)
                 roi_x1 = 120,
                 roi_y1 = 40,
                 roi_x2 = 520,
                 roi_y2 = 440,
                 block_size = 51,
                 c_value = -10):
        """
        Args:
            white_ratio_threshold: 밝은 영역 비율 임계값 (0~1)
                                   이 값 이상이면 박스있음으로 판단
            roi_x1~y2:             ROI 영역 좌표
            block_size:            adaptiveThreshold 블록 크기 (홀수, 클수록 더 넓은 영역 기준 비교)
            c_value:                adaptiveThreshold 보정값 (음수일수록 더 엄격하게 밝은 픽셀만 선택)
        """
        self.white_ratio_threshold = white_ratio_threshold
        self.roi_x1 = roi_x1
        self.roi_y1 = roi_y1
        self.roi_x2 = roi_x2
        self.roi_y2 = roi_y2
        self.block_size = block_size
        self.c_value     = c_value

        print(
            f"[RackSpaceDetector] 초기화 완료\n"
            f"  흰색 비율 임계값: {white_ratio_threshold * 100:.1f}%\n"
            f"  ROI: ({roi_x1},{roi_y1}) ~ ({roi_x2},{roi_y2})\n"
            f"  adaptiveThreshold block_size={block_size}, C={c_value}"
        )

    def detect(self, frame) -> dict:
        """
        단일 프레임 랙 빈 공간 감지

        Returns:
            dict: {
                'left_empty':     bool,
                'right_empty':    bool,
                'left_ratio':     float,
                'right_ratio':    float,
                'debug_frame':    ndarray
            }
        """
        roi   = frame[self.roi_y1:self.roi_y2, self.roi_x1:self.roi_x2]
        h, w  = roi.shape[:2]
        mid_x = w // 2

        # adaptiveThreshold 방식 - 절대 밝기가 아니라
        # "주변 영역 대비 상대적으로 밝은지"로 판단 → 낮/밤 조명 변화에 강함
        blur = cv2.GaussianBlur(roi, (7, 7), 0)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)

        full_mask = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, self.block_size, self.c_value
        )

        # 노이즈 제거
        kernel    = np.ones((3, 3), np.uint8)
        full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_OPEN,  kernel)
        full_mask = cv2.morphologyEx(full_mask, cv2.MORPH_CLOSE, kernel)

        # 좌/우 분할
        left_mask  = full_mask[:, :mid_x]
        right_mask = full_mask[:, mid_x:]

        # 흰색(밝은 영역) 비율 계산
        left_total  = left_mask.shape[0]  * left_mask.shape[1]
        right_total = right_mask.shape[0] * right_mask.shape[1]

        left_white  = cv2.countNonZero(left_mask)
        right_white = cv2.countNonZero(right_mask)

        left_ratio  = left_white  / left_total  if left_total  > 0 else 0.0
        right_ratio = right_white / right_total if right_total > 0 else 0.0

        # 흰색 비율 임계값으로 박스 유무 판단
        left_empty  = left_ratio  < self.white_ratio_threshold
        right_empty = right_ratio < self.white_ratio_threshold

        # 컨투어 (디버그 시각화용)
        left_contours,  _ = cv2.findContours(
            left_mask,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        right_contours, _ = cv2.findContours(
            right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        debug_frame = self._draw_debug(
            frame, mid_x,
            left_empty, right_empty,
            left_ratio, right_ratio,
            left_contours, right_contours
        )

        return {
            'left_empty':     left_empty,
            'right_empty':    right_empty,
            'left_ratio':     left_ratio,
            'right_ratio':    right_ratio,
            'debug_frame':    debug_frame
        }

    def _draw_debug(self, frame, mid_x,
                    left_empty, right_empty,
                    left_ratio, right_ratio,
                    left_contours, right_contours):
        debug     = frame.copy()
        roi_mid_x = self.roi_x1 + (self.roi_x2 - self.roi_x1) // 2

        # ROI 영역 표시
        cv2.rectangle(debug,
                      (self.roi_x1, self.roi_y1),
                      (self.roi_x2, self.roi_y2),
                      (255, 255, 0), 2)

        # ROI 내 가운데 선
        cv2.line(debug,
                 (roi_mid_x, self.roi_y1),
                 (roi_mid_x, self.roi_y2),
                 (255, 255, 0), 2)

        # 좌측 컨투어
        for c in left_contours:
            s = c.copy()
            s[:, :, 0] += self.roi_x1
            s[:, :, 1] += self.roi_y1
            cv2.drawContours(debug, [s], -1, (0, 255, 0), 2)

        # 우측 컨투어
        for c in right_contours:
            s = c.copy()
            s[:, :, 0] += roi_mid_x
            s[:, :, 1] += self.roi_y1
            cv2.drawContours(debug, [s], -1, (0, 255, 0), 2)

        # 상태 텍스트
        left_color  = (0, 255, 0) if left_empty  else (0, 0, 255)
        right_color = (0, 255, 0) if right_empty else (0, 0, 255)

        cv2.putText(debug,
                    f"{'EMPTY' if left_empty  else 'BOX'} ({left_ratio*100:.1f}%)",
                    (self.roi_x1 + 5, self.roi_y1 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, left_color, 2)
        cv2.putText(debug,
                    f"{'EMPTY' if right_empty else 'BOX'} ({right_ratio*100:.1f}%)",
                    (roi_mid_x + 5, self.roi_y1 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, right_color, 2)
        cv2.putText(debug,
                    f"threshold: {self.white_ratio_threshold*100:.1f}%",
                    (self.roi_x1, self.roi_y2 + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        return debug