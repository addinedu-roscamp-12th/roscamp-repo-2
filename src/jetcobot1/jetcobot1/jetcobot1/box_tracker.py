"""
흰색 박스 트래킹 모듈
카메라: /dev/jetcocam0 (Jetson 직접 연결)

감지 방식: HSV 흰색 임계값 → 윤곽선 → 중심점 계산
출력: 이미지 중앙 대비 픽셀 오차 (error_x, error_y)

실행 순서:
  1. 로봇팔 초기화 [0,0,0,0,0,0]
  2. 카메라가 박스를 내려다보는 자세로 이동
  3. 박스 트래킹 시작

action_server.py와 독립적으로 동작하며 import 또는 단독 실행 모두 가능.
"""

import cv2
import numpy as np
import threading
import time
import logging

from pymycobot.mycobot280 import MyCobot280

logger = logging.getLogger("jetcobot1.box_tracker")

# ──────────────────────────────────────────────────────────────
# 로봇팔 각도 상수
# ──────────────────────────────────────────────────────────────
ANGLES_ZERO        = [0,   0,   0,   0,   0,   0  ]  # 완전 초기화
ANGLES_CAMERA_DOWN = [0,  -35,  0,  -55,  0,  -47 ]  # 카메라가 박스를 내려다보는 자세
ROBOT_PORT         = "/dev/ttyJETCOBOT"
ROBOT_BAUD         = 1_000_000
ROBOT_SPEED        = 80

# HSV 흰색 범위
WHITE_HSV_LOWER = np.array([0,   0,   230])
WHITE_HSV_UPPER = np.array([180, 50,  255])

# 박스 크기 분류 기준 (픽셀 면적)
SIZE_SMALL_MAX  = 5_000
SIZE_MEDIUM_MAX = 20_000

# 감지 파라미터
MIN_BOX_AREA        = 1_000   # 이 면적 미만은 노이즈로 무시
BOX_SIDE_MIN        = 40      # 박스 단변 최소 픽셀 길이
BOX_SIDE_MAX        = 400     # 박스 장변 최대 픽셀 길이
BOX_ASPECT_MIN      = 0.6     # 장변/단변 비율 최솟값 (정사각형 기준)
BOX_ASPECT_MAX      = 1.8     # 장변/단변 비율 최댓값
CENTER_TOLERANCE_PX = 15      # 중심 오차 허용 범위 (px) — 10→15: 관성 오버슈트 방지


def classify_size(area: float) -> str:
    if area < SIZE_SMALL_MAX:
        return "small"
    if area < SIZE_MEDIUM_MAX:
        return "medium"
    return "large"


class BoxInfo:
    """단일 프레임에서 감지된 박스 정보."""

    def __init__(self, cx: int, cy: int, area: float, bbox: tuple, approx=None, width=None, angle=None):
        self.cx     = cx       # 박스 중심 x (픽셀)
        self.cy     = cy       # 박스 중심 y (픽셀)
        self.area   = area     # 윤곽선 면적
        self.bbox   = bbox     # (x, y, w, h)
        self.approx = approx   # 4각형 꼭짓점 배열
        self.width  = width    # minAreaRect short_side (실제 박스 폭, px) — _detect_blob은 None
        self.angle  = angle
        self.size   = classify_size(area)


    def error(self, frame_w: int, frame_h: int) -> tuple[int, int]:
        """이미지 중앙 대비 픽셀 오차 (error_x, error_y)."""
        return self.cx - frame_w // 2, self.cy - frame_h // 2

    def is_centered(self, frame_w: int, frame_h: int) -> bool:
        ex, ey = self.error(frame_w, frame_h)
        return abs(ex) <= CENTER_TOLERANCE_PX and abs(ey) <= CENTER_TOLERANCE_PX

    def __repr__(self):
        # ★ width 추가: pick_and_place.py 캘리브 시 픽셀 폭 확인용
        w_str = f"{self.width:.1f}px" if self.width is not None else "N/A"
        return (f"BoxInfo(cx={self.cx}, cy={self.cy}, "
                f"area={self.area:.0f}, width={w_str}, size={self.size})")


class CameraSetup:
    """
    박스 트래킹 전 로봇팔을 카메라가 아래를 향하는 자세로 준비한다.

    단계:
      1. 완전 초기화 [0,0,0,0,0,0]
      2. 그리퍼 열기
      3. 카메라 하향 자세 [0,-35,0,-55,0,-47]
    """

    def __init__(self, port: str = ROBOT_PORT, baud: int = ROBOT_BAUD):
        self.mc = MyCobot280(port, baud)
        self.mc.thread_lock = True
        logger.info(f"로봇 연결: {port}")

    def prepare(self, speed: int = ROBOT_SPEED):
        """로봇팔을 카메라 하향 자세로 이동한다."""
        logger.info("Step 1: 완전 초기화 [0,0,0,0,0,0]")
        self.mc.send_angles(ANGLES_ZERO, speed)
        time.sleep(2)

        logger.info("Step 2: 그리퍼 열기")
        self.mc.set_gripper_value(100, speed)
        time.sleep(1)

        logger.info(f"Step 3: 카메라 하향 자세 {ANGLES_CAMERA_DOWN}")
        self.mc.send_angles(ANGLES_CAMERA_DOWN, speed)
        time.sleep(2)

        logger.info("카메라 준비 완료 — 트래킹 시작 가능")


class BoxTracker:
    """
    카메라에서 흰색 박스를 실시간으로 트래킹한다.

    사용 예:
        tracker = BoxTracker()
        tracker.start()
        box = tracker.get_best_box()   # 최신 감지 결과
        tracker.stop()
    """

    def __init__(self, device: str = "/dev/jetcocam0",
                 width: int = 640, height: int = 480, fps: int = 30):
        self.device = device
        self.width  = width
        self.height = height
        self.fps    = fps

        self._cap: cv2.VideoCapture | None = None
        self._lock   = threading.Lock()
        self._thread: threading.Thread | None = None
        self._running = False

        self._latest_boxes: list[BoxInfo] = []
        self._latest_blob:  BoxInfo | None  = None   # 형태 무관 최대 흰색 영역
        self._latest_frame: np.ndarray | None = None

    # ──────────────────────────────────────────
    # 공개 API
    # ──────────────────────────────────────────

    def start(self):
        """백그라운드 스레드에서 카메라 루프를 시작한다."""
        if self._running:
            return
        self._cap = cv2.VideoCapture(self.device)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_FPS,          self.fps)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self._cap.isOpened():
            raise RuntimeError(f"카메라를 열 수 없습니다: {self.device}")
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info(f"BoxTracker 시작: {self.device} ({self.width}x{self.height})")

    def stop(self):
        """트래킹 루프를 종료한다."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        logger.info("BoxTracker 종료")

    def get_best_box(self) -> BoxInfo | None:
        """가장 큰 흰색 박스를 반환한다. 감지 없으면 None."""
        with self._lock:
            if not self._latest_boxes:
                return None
            return max(self._latest_boxes, key=lambda b: b.area)

    def get_all_boxes(self) -> list[BoxInfo]:
        """현재 프레임에서 감지된 모든 박스를 반환한다."""
        with self._lock:
            return list(self._latest_boxes)

    def get_best_blob(self) -> BoxInfo | None:
        """형태 무관 최대 흰색 영역을 반환한다. 박스 일부만 보일 때 방향 탐색용."""
        with self._lock:
            return self._latest_blob

    def get_latest_frame(self) -> np.ndarray | None:
        """감지 결과가 그려진 최신 프레임을 반환한다."""
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    # ──────────────────────────────────────────
    # 내부 처리
    # ──────────────────────────────────────────

    def _loop(self):
        while self._running:
            # ── [④ 개선] 버퍼 플러시: 3→5프레임으로 늘려 오래된 프레임 제거 ──
            for _ in range(5):
                self._cap.grab()
            ret, frame = self._cap.read()
            if not ret:
                logger.warning("프레임 읽기 실패, 재시도...")
                time.sleep(0.05)
                continue

            boxes     = self._detect(frame)
            blob      = self._detect_blob(frame)
            annotated = self._annotate(frame.copy(), boxes, blob)

            with self._lock:
                self._latest_boxes = boxes
                self._latest_blob  = blob
                self._latest_frame = annotated

    def _detect(self, frame: np.ndarray) -> list[BoxInfo]:
        """GaussianBlur → HSV → minAreaRect로 박스를 감지한다.
        기울어진 박스도 안정적으로 인식하며, 로봇 팔처럼 얇고 긴 형태는 비율 필터로 제거."""
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, WHITE_HSV_LOWER, WHITE_HSV_UPPER)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_BOX_AREA:
                continue

            # minAreaRect: 기울어진 박스도 안정적으로 외접 사각형 추출
            rect = cv2.minAreaRect(cnt)
            # (rcx, rcy), (rw, rh), _ = rect
            # _detect() 내부 minAreaRect 부분
            (rcx, rcy), (rw, rh), rect_angle = rect # angle 추출

            # minAreaRect 각도 → 그리퍼 회전각으로 변환
            # rw > rh 이면 장변이 가로 방향 → 90도 보정 필요
            if rw < rh:
                grip_angle = rect_angle
            else:
                grip_angle = rect_angle + 90.0

            # 장변/단변 구분
            long_side  = max(rw, rh)
            short_side = min(rw, rh)

            if short_side < 1:
                continue

            # 크기 필터
            if not (BOX_SIDE_MIN < short_side and long_side < BOX_SIDE_MAX):
                continue

            # 정사각형 비율 필터 (장변/단변)
            aspect = long_side / short_side
            if not (BOX_ASPECT_MIN <= aspect <= BOX_ASPECT_MAX):
                continue

            # 시각화용 4각형 꼭짓점
            approx = np.int32(cv2.boxPoints(rect))

            cx, cy = int(rcx), int(rcy)
            x, y, w, h = cv2.boundingRect(approx)
            # ★ width=short_side: minAreaRect 단변을 실제 박스 폭으로 저장
            boxes.append(BoxInfo(cx, cy, area, (x, y, w, h), approx, width=short_side,angle=grip_angle))

        return boxes

    def _detect_blob(self, frame: np.ndarray) -> BoxInfo | None:
        """형태 조건 없이 가장 큰 흰색 영역의 중심을 반환한다. 박스 일부만 보일 때 탐색용."""
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv  = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, WHITE_HSV_LOWER, WHITE_HSV_UPPER)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best, best_area = None, MIN_BOX_AREA
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > best_area:
                best_area = area
                best = cnt

        if best is None:
            return None

        x, y, w, h = cv2.boundingRect(best)
        cx, cy = x + w // 2, y + h // 2
        # blob은 형태 무관 탐색용 — width는 None
        return BoxInfo(cx, cy, best_area, (x, y, w, h))

    def _annotate(self, frame: np.ndarray, boxes: list[BoxInfo],
                  blob: BoxInfo | None = None) -> np.ndarray:
        """감지된 박스와 중심 오차를 프레임에 그린다."""
        fh, fw = frame.shape[:2]
        # 이미지 중심 십자선
        cv2.line(frame, (fw // 2 - 20, fh // 2), (fw // 2 + 20, fh // 2), (0, 255, 0), 1)
        cv2.line(frame, (fw // 2, fh // 2 - 20), (fw // 2, fh // 2 + 20), (0, 255, 0), 1)

        # blob 탐색 표시 (시안색) — 박스 미감지 시에만 표시
        if blob is not None and not boxes:
            bx, by, bw, bh = blob.bbox
            cv2.rectangle(frame, (bx, by), (bx + bw, by + bh), (255, 255, 0), 1)
            cv2.circle(frame, (blob.cx, blob.cy), 5, (255, 255, 0), -1)
            cv2.putText(frame, "SEARCH",
                        (bx, by - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

        for box in boxes:
            x, y, w, h = box.bbox
            ex, ey = box.error(fw, fh)
            color = (0, 255, 0) if box.is_centered(fw, fh) else (0, 165, 255)

            # 실제 4각형 윤곽선
            if box.approx is not None:
                cv2.drawContours(frame, [box.approx], 0, (0, 255, 0), 2)
            # 바운딩 박스
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 1)
            # 중심점
            cv2.circle(frame, (box.cx, box.cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"({box.cx},{box.cy})",
                        (box.cx + 6, box.cy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 0, 0), 1)
            # ★ width 추가 출력
            w_str = f"{box.width:.0f}px" if box.width is not None else "?"
            cv2.putText(frame,
                        f"{box.size} w={w_str} ex={ex:+d} ey={ey:+d}",
                        (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return frame


class GripperAligner:
    # ── 게인 / 스텝 한계 ───────────────────────────────────────
    KP_J1:        float = 0.08   # 0.05 → 0.08 (전반적 게인 상향)
    KP_J2:        float = 0.08   # 0.05 → 0.08
    MAX_STEP_DEG: float = 6.0    # 5.0 → 6.0 (스텝 상한 확대)

    # ── 연속 속도 범위 (구간 경계 제거) ────────────────────────
    SPEED_MIN:  int   = 25    # 15 → 25 (미세 구간 최소 속도 상향)
    SPEED_MAX:  int   = 70    # 오차 ERR_REF 이상일 때 최대 속도
    ERR_REF:    float = 60.0  # 속도 포화 기준 오차 (px)

    # ── EMA 필터 (카메라 노이즈 제거) ──────────────────────────
    EMA_ALPHA:  float = 0.6   # 0.4 → 0.6 (최신 오차 반영 비율 높임)

    # ── 수렴 판정 (데드밴드 + 연속 카운터) ─────────────────────
    STABLE_COUNT_TARGET: int = 1   # 5 → 1 (수렴 판정 완화)

    def _smooth_speed(self, abs_err: float) -> int:
        """
        오차 크기를 [SPEED_MIN, SPEED_MAX] 로 연속 선형 매핑한다.
        구간 경계가 없으므로 속도가 부드럽게 변한다.

        abs_err=0       → SPEED_MIN
        abs_err=ERR_REF → SPEED_MAX
        abs_err>ERR_REF → SPEED_MAX (포화)
        """
        ratio = min(abs_err / self.ERR_REF, 1.0)
        return int(self.SPEED_MIN + (self.SPEED_MAX - self.SPEED_MIN) * ratio)

    def _smooth_step(self, error_px: float, kp: float) -> tuple[float, int]:
        """
        구간별 차등 승수로 스텝과 속도를 계산한다.

        구간별 동작:
          오차 >60px  → 1.5승: 빠르게 접근
          오차 >20px  → 1.1승: 선형보다 약간 공격적
          오차 <=20px → 1.0승: 선형 (0.7 → 1.0, 미세 구간 스텝 대폭 확대)
        """
        abs_err = abs(error_px)
        sign    = 1 if error_px > 0 else -1

        if abs_err > 60:
            step = kp * (abs_err ** 1.5)   # 빠른 접근
        elif abs_err > 20:
            step = kp * (abs_err ** 1.1)   # 선형보다 약간 공격적
        else:
            step = kp * (abs_err ** 1.0)   # 미세 구간: 선형으로 확대

        speed = self._smooth_speed(abs_err)
        step  = max(-self.MAX_STEP_DEG, min(self.MAX_STEP_DEG, step * sign))
        return step, speed

    def __init__(self, mc: MyCobot280, tracker: BoxTracker, flip_x: bool = False):
        self.mc      = mc
        self.tracker = tracker
        self._sx     = 1 if flip_x else -1

        # [② EMA 필터] 초기값 None → 첫 프레임에서 raw 값으로 초기화
        self._ema_ex: float | None = None
        self._ema_ey: float | None = None

    def _update_ema(self, ex: float, ey: float) -> tuple[float, float]:
        """
        [② EMA 필터] 지수 이동평균으로 오차를 평활화한다.
        카메라 노이즈로 인한 한 프레임 튀김을 억제한다.
        """
        if self._ema_ex is None:
            self._ema_ex, self._ema_ey = float(ex), float(ey)
        else:
            a = self.EMA_ALPHA
            self._ema_ex = a * ex + (1 - a) * self._ema_ex
            self._ema_ey = a * ey + (1 - a) * self._ema_ey
        return self._ema_ex, self._ema_ey

    def _reset_ema(self):
        """박스를 잃었다가 다시 찾을 때 EMA 상태를 초기화한다."""
        self._ema_ex = None
        self._ema_ey = None

    def align(self, timeout: float = 15.0) -> bool:
        """
        J1·J2 동시 이동 + EMA 필터 + 연속 수렴 카운터로
        부드럽고 안정적인 중앙 정렬을 수행한다.

        개선 사항:
          ① 연속 게인 (smooth gain): 구간 경계 없이 속도·스텝이 연속 변화
          ② EMA 필터: 카메라 노이즈를 평활화해 떨림 억제
          ③ 수렴 카운터: N프레임 연속 허용 범위 진입 시에만 완료 판정

        성공 시 True, 타임아웃 시 False.
        """
        logger.info("GripperAligner: 부드러운 정렬 시작 (smooth + EMA + stable counter)")

        init_angles = self.mc.get_angles()
        if not init_angles or len(init_angles) < 2:
            logger.error("초기 각도 읽기 실패")
            return False

        current_angles = list(init_angles)
        stable_count   = 0          # [③ 수렴 카운터]
        prev_box_found = False       # 박스 재감지 시 EMA 리셋용

        # ── 시간 측정 ────────────────────────────────────────────
        t_start     = time.perf_counter()   # 정렬 시작 시각
        t_box_first = None                  # 박스 최초 감지 시각
        frame_count = 0                     # 총 처리 프레임 수
        move_count  = 0                     # 실제 관절 명령 횟수

        while True:
            box  = self.tracker.get_best_box()
            blob = self.tracker.get_best_blob()
            frame_count += 1

            # ── 박스 최초 감지 시각 기록 ─────────────────────────────
            if box is not None and t_box_first is None:
                t_box_first = time.perf_counter()

            # ── 박스 재감지 시 EMA 리셋 ─────────────────────────────
            if box is not None and not prev_box_found:
                self._reset_ema()
            prev_box_found = box is not None

            # ── 1단계: 박스 전체 감지 → 정밀 정렬 ──────────────────
            if box is not None:
                raw_ex, raw_ey = box.error(self.tracker.width, self.tracker.height)

                # [② EMA] 노이즈 평활화된 오차 사용
                ex, ey = self._update_ema(raw_ex, raw_ey)

                # [③ 수렴 카운터] EMA 오차 기준으로 판정
                # → raw 오차 기준보다 노이즈에 덜 민감해 stable_count 리셋 빈도 감소
                if abs(ex) <= CENTER_TOLERANCE_PX and abs(ey) <= CENTER_TOLERANCE_PX:
                    stable_count += 1
                    print(
                        f"\r  [수렴 {stable_count}/{self.STABLE_COUNT_TARGET}]"
                        f"  raw=({raw_ex:+4d},{raw_ey:+4d})px"
                        f"  ema=({ex:+5.1f},{ey:+5.1f})px",
                        end="", flush=True,
                    )
                    if stable_count >= self.STABLE_COUNT_TARGET:
                        t_end      = time.perf_counter()
                        elapsed    = t_end - t_start
                        t_to_box   = (t_box_first - t_start) if t_box_first else 0.0
                        t_to_align = (t_end - t_box_first)   if t_box_first else elapsed

                        # ★ 정렬 완료 시점의 박스 크기 정보
                        w_str = f"{box.width:.1f}px" if box.width is not None else "N/A"

                        # \r 잔류 문자를 공백으로 지우고 새 줄에서 리포트 출력
                        print("\r" + " " * 80)   # 현재 줄 클리어
                        print(
                            f"  [완료] {self.STABLE_COUNT_TARGET}프레임 연속 안정 — 중앙 정렬됨\n"
                            f"  ┌─ 정렬 완료 시간 리포트 ──────────────────────\n"
                            f"  │  총 소요 시간       : {elapsed:.2f} s\n"
                            f"  │  박스 최초 감지까지 : {t_to_box:.2f} s\n"
                            f"  │  감지 후 정렬까지   : {t_to_align:.2f} s\n"
                            f"  │  총 처리 프레임     : {frame_count} frames\n"
                            f"  │  관절 명령 횟수     : {move_count} 회\n"
                            f"  │  박스 픽셀 폭       : {w_str}\n"
                            f"  │  박스 면적          : {box.area:.0f}px²\n"
                            f"  └───────────────────────────────────────────────",
                            flush=True,
                        )
                        logger.info(
                            f"정렬 완료 — 총={elapsed:.2f}s  감지까지={t_to_box:.2f}s"
                            f"  정렬까지={t_to_align:.2f}s  프레임={frame_count}  명령={move_count}"
                            f"  박스폭={w_str}  면적={box.area:.0f}px²"
                        )
                        return True
                    # time.sleep(0.005)   # 수렴 대기: 0.02 → 0.005
                    continue
                else:
                    stable_count = 0   # 허용 범위 벗어나면 카운터 리셋

                # [① 연속 게인] EMA 평활화된 오차로 스텝·속도 계산
                need_j1 = abs(ey) > CENTER_TOLERANCE_PX
                need_j2 = abs(ex) > CENTER_TOLERANCE_PX

                spd_j1 = spd_j2 = self.SPEED_MIN

                if need_j1:
                    delta_j1, spd_j1 = self._smooth_step(self._sx * ey, self.KP_J1)
                    current_angles[0] += delta_j1

                if need_j2:
                    delta_j2, spd_j2 = self._smooth_step(-self._sx * ex, self.KP_J2)
                    current_angles[1] += delta_j2

                speed = max(spd_j1, spd_j2)
                self.mc.send_angles(current_angles, speed)
                move_count += 1

                print(
                    f"\r  [정렬] raw=({raw_ex:+4d},{raw_ey:+4d})px"
                    f"  ema=({ex:+5.1f},{ey:+5.1f})px"
                    f"  J1={current_angles[0]:+6.1f}°  J2={current_angles[1]:+6.1f}°"
                    f"  spd={speed}",
                    end="", flush=True,
                )

            # ── 2단계: 일부만 보임 → blob 방향으로 탐색 ────────────
            elif blob is not None:
                ex, ey = blob.error(self.tracker.width, self.tracker.height)
                stable_count = 0

                need_j1 = abs(ey) > CENTER_TOLERANCE_PX * 2
                need_j2 = abs(ex) > CENTER_TOLERANCE_PX * 2

                if need_j1:
                    delta_j1 = self._sx * ey * self.KP_J1 * 0.5
                    delta_j1 = max(-1.5, min(1.5, delta_j1))
                    current_angles[0] += delta_j1

                if need_j2:
                    delta_j2 = -self._sx * ex * self.KP_J2 * 0.5
                    delta_j2 = max(-1.5, min(1.5, delta_j2))
                    current_angles[1] += delta_j2

                if need_j1 or need_j2:
                    self.mc.send_angles(current_angles, max(10, self.SPEED_MIN - 5))
                    print(
                        f"\r  [탐색] blob=({ex:+4d},{ey:+4d})px"
                        f"  J1={current_angles[0]:+6.1f}°  J2={current_angles[1]:+6.1f}°",
                        end="", flush=True,
                    )

            # ── 흰색 영역 없음 → 대기 ────────────────────────────────
            else:
                stable_count = 0
                print(f"\r  [대기] 흰색 영역 없음                              ",
                      end="", flush=True)
                time.sleep(0.1)
                continue

            # 오차가 클 때는 대기 없이 즉시 다음 명령
            # 오차가 작을 때만 짧게 대기해 미세 진동 방지
            total_err = abs(ex) + abs(ey)
            time.sleep(0.0 if total_err > 20 else 0.005)  # 0.02 → 0.005


class StreamServer:
    """
    BoxTracker의 프레임을 MJPEG로 스트리밍하는 Flask 서버.
    브라우저에서 http://<로봇IP>:5000 으로 접속하면 실시간 화면을 볼 수 있다.
    """

    def __init__(self, tracker: BoxTracker, host: str = "0.0.0.0", port: int = 5000):
        from flask import Flask
        self.tracker = tracker
        self.host    = host
        self.port    = port
        self.app     = Flask(__name__)
        self.app.add_url_rule("/",       "index",  self._index)
        self.app.add_url_rule("/stream", "stream", self._stream)
        self._thread: threading.Thread | None = None

    def start(self):
        self._thread = threading.Thread(
            target=lambda: self.app.run(host=self.host, port=self.port,
                                        debug=False, use_reloader=False),
            daemon=True,
        )
        self._thread.start()
        logger.info(f"스트림 서버 시작 → http://localhost:{self.port}")
        print(f"\n[STREAM] http://localhost:{self.port}")

    def _index(self):
        return (
            "<html><body style='background:#000;margin:0'>"
            f"<img src='/stream' style='width:100%'>"
            "</body></html>"
        )

    def _stream(self):
        from flask import Response
        return Response(self._generate(), mimetype="multipart/x-mixed-replace; boundary=frame")

    def _generate(self):
        while True:
            frame = self.tracker.get_latest_frame()
            if frame is None:
                time.sleep(0.01)
                continue
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                   + buf.tobytes() + b"\r\n")
            time.sleep(0.05)


# ──────────────────────────────────────────────────────────────
# 단독 실행 시 카메라 미리보기
# ──────────────────────────────────────────────────────────────

def main():
    import argparse
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(description="박스 트래킹 + 그리퍼 자동 정렬")
    parser.add_argument("--no-robot",  action="store_true",
                        help="로봇 연결 없이 카메라만 실행 (테스트용)")
    parser.add_argument("--no-align",  action="store_true",
                        help="그리퍼 자동 정렬 생략, 트래킹 모니터만 표시")
    parser.add_argument("--flip-x",    action="store_true",
                        help="J1 회전 방향 반전 (박스가 반대로 움직이면 사용)")
    parser.add_argument("--stream",    action="store_true",
                        help="브라우저 MJPEG 스트리밍 서버 시작 (http://<IP>:5000)")
    parser.add_argument("--show",      action="store_true",
                        help="cv2.imshow GUI 창 표시 (X 디스플레이 필요)")
    args = parser.parse_args()

    # ── 1. 로봇팔 준비 ──────────────────────────────
    setup = None
    if not args.no_robot:
        print("로봇팔 초기화 중...")
        setup = CameraSetup()
        setup.prepare()
    else:
        print("--no-robot 모드: 로봇 연결 생략")

    # ── 2. 박스 트래킹 시작 ─────────────────────────
    tracker = BoxTracker()
    tracker.start()

    # ── 3. 스트리밍 서버 (선택) ──────────────────────
    if args.stream:
        StreamServer(tracker).start()

    # ── 4. 그리퍼 자동 정렬 ─────────────────────────
    if not args.no_robot and not args.no_align and setup is not None:
        print("\n그리퍼 자동 정렬 시작 (Ctrl+C로 중단)")
        aligner = GripperAligner(setup.mc, tracker, flip_x=args.flip_x)
        try:
            success = aligner.align(timeout=20.0)
            print()
            if success:
                print("정렬 완료 — 그리퍼가 박스 중앙 위에 있습니다.")
            else:
                print("타임아웃: 박스를 카메라 시야 내에 놓아주세요.")
            if args.stream:
                print("스트리밍 유지 중 - Ctrl+C로 종료")
                while True:
                    time.sleep(1)

        except KeyboardInterrupt:
            print("\n정렬 중단됨")
        finally:
            tracker.stop()
            return

    # ── 5. 모니터 모드 (--no-align 또는 --no-robot) ──
    quit_flag = "q 키" if args.show else "Ctrl+C"
    print(f"트래킹 모니터 — {quit_flag}로 종료")
    try:
        while True:
            box = tracker.get_best_box()
            if box:
                ex, ey = box.error(640, 480)
                # ★ width 추가 출력
                w_str = f"{box.width:.1f}px" if box.width is not None else "N/A"
                print(f"\r감지: {box}  error_x={ex:+4d}  error_y={ey:+4d}  "
                      f"centered={box.is_centered(640, 480)}  width={w_str}",
                      end="", flush=True)
            else:
                print("\r박스 없음                                              ",
                      end="", flush=True)

            if args.show:
                frame = tracker.get_latest_frame()
                if frame is not None:
                    cv2.imshow("BoxTracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                time.sleep(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.stop()
        if args.show:
            cv2.destroyAllWindows()
        print()


if __name__ == "__main__":
    main()