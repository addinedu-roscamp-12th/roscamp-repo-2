"""
흰색 박스 트래킹 모듈 (좌표 제어 버전)
카메라: /dev/jetcocam0
감지 방식: HSV 흰색 임계값 → 윤곽선 → 중심점 계산
"""

import cv2
import numpy as np
import threading
import time
import logging

from pymycobot.mycobot280 import MyCobot280

logger = logging.getLogger("jetcobot1.box_tracker")

# ──────────────────────────────────────────────────────────────
# 로봇 상수
# ──────────────────────────────────────────────────────────────
ROBOT_PORT  = "/dev/ttyJETCOBOT"
ROBOT_BAUD  = 1_000_000
ROBOT_SPEED = 50

ANGLES_ZERO        = [0,   0,   0,   0,   0,   0  ]
ANGLES_CAMERA_DOWN = [0, -35,   0, -55,   0, -47  ]  # 카메라 하향 자세

# ──────────────────────────────────────────────────────────────
# 확정된 원점 기준값 및 보정값
# ──────────────────────────────────────────────────────────────
HOME_X  =  52.50
HOME_Y  = -64.50
HOME_Z  = 409.00
HOME_RX = -92.02
HOME_RY =   0.72
HOME_RZ = -89.93

NEG_Y_OFFSET = +4.15
POS_Y_OFFSET = +5.05
NEG_Z_OFFSET = +3.55
POS_Z_OFFSET = -4.90
Z_DEAD_ZONE  =  15.0

# ──────────────────────────────────────────────────────────────
# HSV 흰색 범위
# ──────────────────────────────────────────────────────────────
WHITE_HSV_LOWER = np.array([0,   0,   230])
WHITE_HSV_UPPER = np.array([180, 50,  255])

# 박스 크기 분류 기준
SIZE_SMALL_MAX  = 5_000
SIZE_MEDIUM_MAX = 20_000

# 감지 파라미터
MIN_BOX_AREA        = 1_000
BOX_SIDE_MIN        = 40
BOX_SIDE_MAX        = 400
BOX_ASPECT_MIN      = 0.6
BOX_ASPECT_MAX      = 1.8
CENTER_TOLERANCE_PX = 10


def classify_size(area: float) -> str:
    if area < SIZE_SMALL_MAX:
        return "small"
    if area < SIZE_MEDIUM_MAX:
        return "medium"
    return "large"


# ──────────────────────────────────────────────────────────────
# BoxInfo
# ──────────────────────────────────────────────────────────────
class BoxInfo:
    def __init__(self, cx, cy, area, bbox, approx=None):
        self.cx     = cx
        self.cy     = cy
        self.area   = area
        self.bbox   = bbox
        self.approx = approx
        self.size   = classify_size(area)

    def error(self, frame_w, frame_h):
        return self.cx - frame_w // 2, self.cy - frame_h // 2

    def is_centered(self, frame_w, frame_h):
        ex, ey = self.error(frame_w, frame_h)
        return abs(ex) <= CENTER_TOLERANCE_PX and abs(ey) <= CENTER_TOLERANCE_PX

    def __repr__(self):
        return f"BoxInfo(cx={self.cx}, cy={self.cy}, area={self.area:.0f}, size={self.size})"


# ──────────────────────────────────────────────────────────────
# BoxTracker (기존과 동일)
# ──────────────────────────────────────────────────────────────
class BoxTracker:
    def __init__(self, device="/dev/jetcocam0", width=640, height=480, fps=30):
        self.device  = device
        self.width   = width
        self.height  = height
        self.fps     = fps
        self._cap    = None
        self._lock   = threading.Lock()
        self._thread = None
        self._running         = False
        self._latest_boxes    = []
        self._latest_blob     = None
        self._latest_frame    = None

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
        self._thread  = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info(f"BoxTracker 시작: {self.device} ({self.width}x{self.height})")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        logger.info("BoxTracker 종료")

    def get_best_box(self):
        with self._lock:
            if not self._latest_boxes:
                return None
            return max(self._latest_boxes, key=lambda b: b.area)

    def get_all_boxes(self):
        with self._lock:
            return list(self._latest_boxes)

    def get_best_blob(self):
        with self._lock:
            return self._latest_blob

    def get_latest_frame(self):
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
            boxes    = self._detect(frame)
            blob     = self._detect_blob(frame)
            annotated = self._annotate(frame.copy(), boxes, blob)
            with self._lock:
                self._latest_boxes = boxes
                self._latest_blob  = blob
                self._latest_frame = annotated

    def _detect(self, frame):
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

    def _detect_blob(self, frame):
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

    def _annotate(self, frame, boxes, blob=None):
        fh, fw = frame.shape[:2]
        cv2.line(frame, (fw//2 - 20, fh//2), (fw//2 + 20, fh//2), (0, 255, 0), 1)
        cv2.line(frame, (fw//2, fh//2 - 20), (fw//2, fh//2 + 20), (0, 255, 0), 1)
        if blob is not None and not boxes:
            bx, by, bw, bh = blob.bbox
            cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), (255, 255, 0), 1)
            cv2.circle(frame, (blob.cx, blob.cy), 5, (255, 255, 0), -1)
            cv2.putText(frame, "SEARCH", (bx, by-6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
        for box in boxes:
            x, y, w, h = box.bbox
            ex, ey = box.error(fw, fh)
            color = (0, 255, 0) if box.is_centered(fw, fh) else (0, 165, 255)
            if box.approx is not None:
                cv2.drawContours(frame, [box.approx], 0, (0, 255, 0), 2)
            cv2.rectangle(frame, (x, y), (x+w, y+h), color, 1)
            cv2.circle(frame, (box.cx, box.cy), 5, (0, 0, 255), -1)
            cv2.putText(frame, f"({box.cx},{box.cy})",
                        (box.cx+6, box.cy+4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 0, 0), 1)
            cv2.putText(frame, f"{box.size} ex={ex:+d} ey={ey:+d}",
                        (x, y-8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return frame


# ──────────────────────────────────────────────────────────────
# CoordAligner
# ──────────────────────────────────────────────────────────────
class CoordAligner:
    """
    픽셀 오차 → mm 변환 → Y/Z 보정 적용 좌표 이동으로 박스 중앙 정렬.
    """
    PX_TO_MM_X          = 0.3
    PX_TO_MM_Y          = 0.3
    CENTER_TOLERANCE_PX = 10
    MOVE_SPEED          = 30
    STEP_INTERVAL       = 2.0

    def __init__(self, mc: MyCobot280, tracker: BoxTracker):
        self.mc      = mc
        self.tracker = tracker

    def _wait_until_stopped(self, timeout=10, threshold=0.5):
        prev = self.mc.get_angles()
        start = time.time()
        while time.time() - start < timeout:
            time.sleep(0.3)
            curr = self.mc.get_angles()
            if curr is None or prev is None:
                prev = curr
                continue
            if max(abs(c - p) for c, p in zip(curr, prev)) < threshold:
                time.sleep(0.3)
                return True
            prev = curr
        return False

    def correct_y(self, delta: float) -> float:
        if delta < 0:
            return delta + NEG_Y_OFFSET
        elif delta > 0:
            return delta + POS_Y_OFFSET
        return 0.0

    def correct_z(self, delta: float) -> float:
        if delta < -Z_DEAD_ZONE:
            return delta + NEG_Z_OFFSET
        elif delta > Z_DEAD_ZONE:
            return delta + POS_Z_OFFSET
        return delta

    def prepare_position(self, dz: float = -50.0):
        """
        1) 완전 초기화 [0,0,0,0,0,0]
        2) 그리퍼 열기
        3) 카메라 하향 자세 ANGLES_CAMERA_DOWN
        4) 좌표 제어로 Z 내리기
        """
        print("Step 1: 완전 초기화...")
        self.mc.send_angles(ANGLES_ZERO, ROBOT_SPEED)
        self._wait_until_stopped()
        time.sleep(1.0)

        print("Step 2: 그리퍼 열기...")
        self.mc.set_gripper_value(100, ROBOT_SPEED)
        time.sleep(1.0)

        print(f"Step 3: 카메라 하향 자세 {ANGLES_CAMERA_DOWN}...")
        self.mc.send_angles(ANGLES_CAMERA_DOWN, ROBOT_SPEED)
        self._wait_until_stopped()
        time.sleep(1.5)

        actual = self.mc.get_coords()
        print(f"  카메라 하향 자세 도달: "
              f"X={actual[0]:.1f} Y={actual[1]:.1f} Z={actual[2]:.1f}")

        # print(f"Step 4: Z {dz:+.0f}mm 이동 (보정 적용)...")
        # cz = self.correct_z(dz)
        # target = [HOME_X, HOME_Y, HOME_Z + cz, HOME_RX, HOME_RY, HOME_RZ]
        # self.mc.send_coords(target, self.MOVE_SPEED, 0)
        # self._wait_until_stopped()
        # time.sleep(1.0)

        actual = self.mc.get_coords()
        print(f"  준비 완료: X={actual[0]:.1f} Y={actual[1]:.1f} Z={actual[2]:.1f}")
        return actual

    def align(self, timeout: float = 30.0) -> bool:
        print("\nCoordAligner: 좌표 제어 정렬 시작")
        start_time = time.time()

        while time.time() - start_time < timeout:
            box = self.tracker.get_best_box()

            if box is None:
                blob = self.tracker.get_best_blob()
                if blob:
                    print(f"\r  [탐색] blob 감지 ({blob.cx},{blob.cy})",
                        end="", flush=True)
                else:
                    print("\r  [대기] 흰색 박스 없음", end="", flush=True)
                time.sleep(0.1)
                continue

            ex, ey = box.error(self.tracker.width, self.tracker.height)

            if abs(ex) <= self.CENTER_TOLERANCE_PX and abs(ey) <= self.CENTER_TOLERANCE_PX:
                print(f"\r  [완료] error_x={ex:+d}px error_y={ey:+d}px 중앙 정렬됨")
                return True

            # 현재 실제 좌표 읽기 (HOME 기준값 사용 X)
            curr = self.mc.get_coords()
            if curr is None:
                time.sleep(0.1)
                continue


            MAX_STEP_MM = 3.0

            dx_mm = +ey * self.PX_TO_MM_Y   # 픽셀 상하 → 로봇 X
            dy_mm = -ex * self.PX_TO_MM_X   # 픽셀 좌우 → 로봇 Y
            dx_mm = max(-MAX_STEP_MM, min(MAX_STEP_MM, dx_mm))
            dy_mm = max(-MAX_STEP_MM, min(MAX_STEP_MM, dy_mm))

            target = list(curr)
            target[0] += dx_mm
            target[1] += dy_mm
            
            # # 픽셀 → mm 변환
            # dx_mm =  ex * self.PX_TO_MM_X
            # dy_mm = -ey * self.PX_TO_MM_Y
            
            # # 현재 좌표 기준으로 target 계산
            # target = list(curr)
            # target[0] += dx_mm          # X: 보정 없음
            # target[1] += dy_mm          # Y: 일단 보정 없이 테스트
            # # target[2] 유지 (Z 고정)

            print(f"\r  [이동] ex={ex:+4d}px ey={ey:+4d}px "
                f"dx={dx_mm:+.1f}mm dy={dy_mm:+.1f}mm  "
                f"→ X={target[0]:.1f} Y={target[1]:.1f}",
                end="", flush=True)

            self.mc.send_coords(target, self.MOVE_SPEED, 1)
            self._wait_until_stopped()
            
            # self.mc.send_coords(target, self.MOVE_SPEED, 0)
            # time.sleep(self.STEP_INTERVAL)  # wait_until_stopped 대신 단순 sleep

            after = self.mc.get_coords()
            print(f"\n  curr:  X={curr[0]:.1f} Y={curr[1]:.1f}")
            print(f"  target: X={target[0]:.1f} Y={target[1]:.1f}")
            print(f"  after:  X={after[0]:.1f} Y={after[1]:.1f}")

        print("\n타임아웃: 정렬 실패")
        return False


# ──────────────────────────────────────────────────────────────
# StreamServer (기존과 동일)
# ──────────────────────────────────────────────────────────────
class StreamServer:
    def __init__(self, tracker: BoxTracker, host="0.0.0.0", port=5000):
        from flask import Flask
        self.tracker = tracker
        self.host    = host
        self.port    = port
        self.app     = Flask(__name__)
        self.app.add_url_rule("/",       "index",  self._index)
        self.app.add_url_rule("/stream", "stream", self._stream)
        self._thread = None

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
        return ("<html><body style='background:#000;margin:0'>"
                "<img src='/stream' style='width:100%'></body></html>")

    def _stream(self):
        from flask import Response
        return Response(self._generate(),
                        mimetype="multipart/x-mixed-replace; boundary=frame")

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
# main
# ──────────────────────────────────────────────────────────────
def main():
    import argparse
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-robot", action="store_true",
                        help="로봇 연결 없이 카메라만 실행")
    parser.add_argument("--no-align", action="store_true",
                        help="정렬 생략, 트래킹 모니터만")
    parser.add_argument("--stream",   action="store_true",
                        help="MJPEG 스트리밍 서버 시작")
    parser.add_argument("--show",     action="store_true",
                        help="cv2.imshow GUI 창 표시")
    args = parser.parse_args()

    mc      = None
    aligner = None

    if not args.no_robot:
        print("로봇 연결 중...")
        mc = MyCobot280(ROBOT_PORT, ROBOT_BAUD)
        mc.thread_lock = True
        time.sleep(0.5)

    tracker = BoxTracker()
    tracker.start()

    if args.stream:
        StreamServer(tracker).start()

    if mc is not None and not args.no_align:
        aligner = CoordAligner(mc, tracker)

        # 카메라 하향 자세 → Z 내리기
        aligner.prepare_position(dz=-50.0)

        print("\n좌표 제어 정렬 시작 (Ctrl+C로 중단)")
        try:
            success = aligner.align(timeout=30.0)
            if success:
                coords = mc.get_coords()
                print(f"\n정렬 완료!")
                print(f"  로봇 좌표: X={coords[0]:.1f} Y={coords[1]:.1f} Z={coords[2]:.1f}")
                print(f"  초기 대비: "
                      f"dX={coords[0]-HOME_X:+.1f}mm "
                      f"dY={coords[1]-HOME_Y:+.1f}mm "
                      f"dZ={coords[2]-HOME_Z:+.1f}mm")
            if args.stream:
                print("스트리밍 유지 중 - Ctrl+C로 종료")
                while True:
                    time.sleep(1)
        except KeyboardInterrupt:
            print("\n중단됨")
        finally:
            tracker.stop()
            return

    # 모니터 모드
    quit_flag = "q 키" if args.show else "Ctrl+C"
    print(f"트래킹 모니터 — {quit_flag}로 종료")
    try:
        while True:
            box = tracker.get_best_box()
            if box:
                ex, ey = box.error(640, 480)
                print(f"\r감지: {box} ex={ex:+d} ey={ey:+d} "
                      f"centered={box.is_centered(640, 480)}",
                      end="", flush=True)
            else:
                print("\r박스 없음", end="", flush=True)
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