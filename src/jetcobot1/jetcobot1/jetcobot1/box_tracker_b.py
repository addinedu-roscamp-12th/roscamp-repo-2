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
CENTER_TOLERANCE_PX = 10      # 중심 오차 허용 범위 (px)


def classify_size(area: float) -> str:
    if area < SIZE_SMALL_MAX:
        return "small"
    if area < SIZE_MEDIUM_MAX:
        return "medium"
    return "large"


class BoxInfo:
    """단일 프레임에서 감지된 박스 정보."""

    def __init__(self, cx: int, cy: int, area: float, bbox: tuple, approx=None):
        self.cx      = cx
        self.cy      = cy
        self.area    = area
        self.bbox    = bbox
        self.approx  = approx
        self.size    = classify_size(area)

    def error(self, frame_w: int, frame_h: int) -> tuple[int, int]:
        return self.cx - frame_w // 2, self.cy - frame_h // 2

    def is_centered(self, frame_w: int, frame_h: int) -> bool:
        ex, ey = self.error(frame_w, frame_h)
        return abs(ex) <= CENTER_TOLERANCE_PX and abs(ey) <= CENTER_TOLERANCE_PX

    def __repr__(self):
        return (f"BoxInfo(cx={self.cx}, cy={self.cy}, "
                f"area={self.area:.0f}, size={self.size})")


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
        self._latest_blob:  BoxInfo | None  = None
        self._latest_frame: np.ndarray | None = None

    def start(self):
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
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        logger.info("BoxTracker 종료")

    def get_best_box(self) -> BoxInfo | None:
        with self._lock:
            if not self._latest_boxes:
                return None
            return max(self._latest_boxes, key=lambda b: b.area)

    def get_all_boxes(self) -> list[BoxInfo]:
        with self._lock:
            return list(self._latest_boxes)

    def get_best_blob(self) -> BoxInfo | None:
        with self._lock:
            return self._latest_blob

    def get_latest_frame(self) -> np.ndarray | None:
        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def _loop(self):
        while self._running:
            for _ in range(3):
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

            rect = cv2.minAreaRect(cnt)
            (rcx, rcy), (rw, rh), _ = rect

            long_side  = max(rw, rh)
            short_side = min(rw, rh)

            if short_side < 1:
                continue
            if not (BOX_SIDE_MIN < short_side and long_side < BOX_SIDE_MAX):
                continue

            aspect = long_side / short_side
            if not (BOX_ASPECT_MIN <= aspect <= BOX_ASPECT_MAX):
                continue

            approx = np.int32(cv2.boxPoints(rect))
            cx, cy = int(rcx), int(rcy)
            x, y, w, h = cv2.boundingRect(approx)
            boxes.append(BoxInfo(cx, cy, area, (x, y, w, h), approx))

        return boxes

    def _detect_blob(self, frame: np.ndarray) -> BoxInfo | None:
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
        return BoxInfo(cx, cy, best_area, (x, y, w, h))

    def _annotate(self, frame: np.ndarray, boxes: list[BoxInfo],
                  blob: BoxInfo | None = None) -> np.ndarray:
        fh, fw = frame.shape[:2]
        cv2.line(frame, (fw // 2 - 20, fh // 2), (fw // 2 + 20, fh // 2), (0, 255, 0), 1)
        cv2.line(frame, (fw // 2, fh // 2 - 20), (fw // 2, fh // 2 + 20), (0, 255, 0), 1)

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

            if box.approx is not None:
                cv2.drawContours(frame, [box.approx], 0, (0, 255, 0), 2)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 1)
            cv2.circle(frame, (box.cx, box.cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"({box.cx},{box.cy})",
                        (box.cx + 6, box.cy + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 0, 0), 1)
            cv2.putText(frame,
                        f"{box.size} ex={ex:+d} ey={ey:+d}",
                        (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return frame


class GripperAligner:
    # ── 기존 상수 ──────────────────────────────────────
    KP_J1:          float = 0.03
    KP_J2:          float = 0.03
    MAX_STEP_DEG:   float = 3.0
    STEP_INTERVAL:  float = 0.03
    ALIGN_SPEED:    int   = 20

    # ── 적응형 속도 상수 ────────────────────────────────
    ERR_FAST  = 60
    ERR_MID   = 20

    SPEED_FAST = 60
    SPEED_MID  = 35
    SPEED_FINE = 15

    def _adaptive_step(self, error_px: float, kp: float) -> tuple[float, int]:
        abs_err = abs(error_px)
        sign    = 1 if error_px > 0 else -1

        if abs_err >= self.ERR_FAST:
            step  = kp * abs_err * (abs_err / self.ERR_FAST) ** 0.5
            speed = self.SPEED_FAST
        elif abs_err >= self.ERR_MID:
            step  = kp * abs_err
            speed = self.SPEED_MID
        else:
            step  = kp * abs_err * 0.6
            speed = self.SPEED_FINE

        step = max(-self.MAX_STEP_DEG, min(self.MAX_STEP_DEG, step * sign))
        return step, speed

    def __init__(self, mc: MyCobot280, tracker: BoxTracker, flip_x: bool = False):
        self.mc      = mc
        self.tracker = tracker
        self._sx     = 1 if flip_x else -1

    def align(self, timeout: float = 15.0) -> bool:
        """
        J1으로 error_y, J2로 error_x를 순차적으로 맞춘다.
        성공 시 True, 타임아웃 시 False.
        """
        logger.info("GripperAligner: J1(y) → J2(x) 순차 정렬 시작")

        # ── 시간 측정 변수 ──────────────────────────────
        t_start     = time.perf_counter()   # 정렬 시작 시각
        t_box_first = None                  # 박스 최초 감지 시각

        init_angles = self.mc.get_angles()
        if not init_angles or len(init_angles) < 2:
            logger.error("초기 각도 읽기 실패")
            return False

        current_j1 = init_angles[0]
        current_j2 = init_angles[1]

        while True:
            box  = self.tracker.get_best_box()
            blob = self.tracker.get_best_blob()

            # ── 박스 최초 감지 시각 기록 ─────────────────
            if box is not None and t_box_first is None:
                t_box_first = time.perf_counter()

            # ── 1단계: 박스 전체 감지 → 정밀 정렬 ──────────────────
            if box is not None:
                ex, ey = box.error(self.tracker.width, self.tracker.height)

                # x, y 둘 다 맞으면 완료
                if abs(ex) <= CENTER_TOLERANCE_PX and abs(ey) <= CENTER_TOLERANCE_PX:
                    t_end      = time.perf_counter()
                    elapsed    = t_end - t_start
                    t_to_box   = (t_box_first - t_start) if t_box_first else 0.0
                    t_to_align = (t_end - t_box_first)   if t_box_first else elapsed

                    print("\r" + " " * 80)   # 현재 줄 클리어
                    print(
                        f"  [완료] 중앙 정렬됨\n"
                        f"  ┌─ 정렬 완료 시간 리포트 ──────────────────────\n"
                        f"  │  총 소요 시간       : {elapsed:.2f} s\n"
                        f"  │  박스 최초 감지까지 : {t_to_box:.2f} s\n"
                        f"  │  감지 후 정렬까지   : {t_to_align:.2f} s\n"
                        f"  └───────────────────────────────────────────────",
                        flush=True,
                    )
                    logger.info(
                        f"정렬 완료 — 총={elapsed:.2f}s  감지까지={t_to_box:.2f}s"
                        f"  정렬까지={t_to_align:.2f}s"
                    )
                    return True  # ← 루프 종료

                # 1단계: J1으로 y 먼저 맞추기
                if abs(ey) > CENTER_TOLERANCE_PX:
                    delta_j1, spd = self._adaptive_step(self._sx * ey, self.KP_J1)
                    current_j1 += delta_j1
                    self.mc.send_angle(1, current_j1, spd)
                    print(f"\r  [J1-y] error_x={ex:+4d}px  error_y={ey:+4d}px  J1→{current_j1:.1f}°",
                          end="", flush=True)

                # 2단계: y 맞으면 J2로 x 맞추기
                elif abs(ex) > CENTER_TOLERANCE_PX:
                    delta_j2, spd = self._adaptive_step(-self._sx * ex, self.KP_J2)
                    current_j2 += delta_j2
                    self.mc.send_angle(2, current_j2, spd)
                    self.mc.send_angle(2, current_j2, self.ALIGN_SPEED)
                    print(f"\r  [J2-x] error_x={ex:+4d}px  error_y={ey:+4d}px  J2→{current_j2:.1f}°",
                          end="", flush=True)

            # ── 2단계: 일부만 보임 → 박스 방향으로 천천히 탐색 ──────
            elif blob is not None:
                ex, ey = blob.error(self.tracker.width, self.tracker.height)

                if abs(ey) > CENTER_TOLERANCE_PX * 2:
                    delta_j1   = self._sx * ey * self.KP_J1 * 0.5
                    delta_j1   = max(-1.5, min(1.5, delta_j1))
                    current_j1 += delta_j1
                    self.mc.send_angle(1, current_j1, max(10, self.ALIGN_SPEED - 5))
                    print(f"\r  [탐색-y] blob_y={ey:+4d}px  J1→{current_j1:.1f}°",
                          end="", flush=True)

                elif abs(ex) > CENTER_TOLERANCE_PX * 2:
                    delta_j2   = -self._sx * ex * self.KP_J2 * 0.5
                    delta_j2   = max(-1.5, min(1.5, delta_j2))
                    current_j2 += delta_j2
                    self.mc.send_angle(2, current_j2, max(10, self.ALIGN_SPEED - 5))
                    print(f"\r  [탐색-x] blob_x={ex:+4d}px  J2→{current_j2:.1f}°",
                          end="", flush=True)

            # ── 흰색 영역 없음 → 대기 ────────────────────────────────
            else:
                print(f"\r  [대기] 흰색 영역 없음                    ",
                      end="", flush=True)
                time.sleep(0.1)
                continue

            time.sleep(self.STEP_INTERVAL)


class StreamServer:
    """
    BoxTracker의 프레임을 MJPEG로 스트리밍하는 Flask 서버.
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
            if success:
                print("그리퍼가 박스 중앙 위에 있습니다.")
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
                print(f"\r감지: {box}  error_x={ex:+4d}  error_y={ey:+4d}  "
                      f"centered={box.is_centered(640, 480)}",
                      end="", flush=True)
            else:
                print("\r박스 없음                                    ",
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