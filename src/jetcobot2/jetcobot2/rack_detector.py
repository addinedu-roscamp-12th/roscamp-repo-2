import cv2
import numpy as np


class RackSpaceDetector:
    """
    랙 빈 공간 감지 클래스
    가운데 선 기준으로 좌/우 영역에 박스가 있는지 감지
    """

    def __init__(self, min_contour_area=3000, canny_low=50, canny_high=150):
        """
        Args:
            min_contour_area: 박스로 인식할 최소 윤곽선 면적 (픽셀)
            canny_low: Canny 엣지 감지 하한 임계값
            canny_high: Canny 엣지 감지 상한 임계값
        """
        self.min_contour_area = min_contour_area
        self.canny_low        = canny_low
        self.canny_high       = canny_high

    def detect(self, frame):
        """
        랙 빈 공간 감지

        Args:
            frame: OpenCV BGR 이미지

        Returns:
            dict: {
                'left_empty': bool,    # 좌측 비어있음 여부
                'right_empty': bool,   # 우측 비어있음 여부
                'left_contours': int,  # 좌측 감지된 윤곽선 수
                'right_contours': int, # 우측 감지된 윤곽선 수
                'debug_frame': ndarray # 디버그 이미지
            }
        """
        h, w  = frame.shape[:2]
        mid_x = w // 2  # 가운데 선 x좌표

        # 1. 그레이스케일 변환
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 2. 가우시안 블러 (노이즈 제거)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # 3. Canny 엣지 감지
        edges = cv2.Canny(blurred, self.canny_low, self.canny_high)

        # 4. 모폴로지 연산 (엣지 강화)
        kernel = np.ones((3, 3), np.uint8)
        edges  = cv2.dilate(edges, kernel, iterations=1)

        # 5. 좌/우 영역 분할
        left_region  = edges[:, :mid_x]
        right_region = edges[:, mid_x:]

        # 6. 각 영역에서 윤곽선 감지
        left_contours, _  = cv2.findContours(
            left_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        right_contours, _ = cv2.findContours(
            right_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 7. 의미있는 윤곽선 필터링 (노이즈 제거)
        left_valid  = [c for c in left_contours  if cv2.contourArea(c) > self.min_contour_area]
        right_valid = [c for c in right_contours if cv2.contourArea(c) > self.min_contour_area]

        left_empty  = len(left_valid)  == 0
        right_empty = len(right_valid) == 0

        # 8. 디버그 이미지 생성
        debug_frame = self._draw_debug(
            frame, mid_x, left_valid, right_valid,
            left_empty, right_empty
        )

        return {
            'left_empty':     left_empty,
            'right_empty':    right_empty,
            'left_contours':  len(left_valid),
            'right_contours': len(right_valid),
            'debug_frame':    debug_frame
        }

    def _draw_debug(self, frame, mid_x, left_contours,
                    right_contours, left_empty, right_empty):
        """디버그용 이미지 생성"""
        debug = frame.copy()
        h, w  = debug.shape[:2]

        # 가운데 선
        cv2.line(debug, (mid_x, 0), (mid_x, h), (255, 255, 0), 2)

        # 좌측 윤곽선
        cv2.drawContours(debug, left_contours, -1, (0, 255, 0), 2)

        # 우측 윤곽선 (좌표 보정)
        right_shifted = []
        for c in right_contours:
            shifted = c.copy()
            shifted[:, :, 0] += mid_x
            right_shifted.append(shifted)
        cv2.drawContours(debug, right_shifted, -1, (0, 255, 0), 2)

        # 좌/우 상태 텍스트
        left_text   = "EMPTY" if left_empty  else "BOX"
        right_text  = "EMPTY" if right_empty else "BOX"
        left_color  = (0, 255, 0) if left_empty  else (0, 0, 255)
        right_color = (0, 255, 0) if right_empty else (0, 0, 255)

        cv2.putText(debug, left_text,
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, left_color, 2)
        cv2.putText(debug, right_text,
                    (mid_x + 10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, right_color, 2)

        return debug