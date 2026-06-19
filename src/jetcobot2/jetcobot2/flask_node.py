
#!/usr/bin/env python3
# flask_node.py — 카메라 점유 + Flask 스트리밍 전담 노드
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'lib', 'python3', 'dist-packages'))

# - /dev/video0 을 열어 프레임을 읽음
# - process_frame() 으로 검출 결과를 _last_detection 에 저장
# - camera/image 토픽을 발행 → cv_node 가 수신
# - http://<ip>:5000/stream 으로 MJPEG 스트리밍
# - /debug_stream 토픽을 구독 → action_server의 rack_detector/qr_detector
#   디버그 프레임을 받아 Flask 화면을 override
# - /debug_stream_clear 토픽을 구독 → override 해제, 원래 화면으로 복귀

import threading
import time
import cv2
from flask import Flask, Response

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from cv_bridge import CvBridge
from jetcobot2.cv_lib import process_frame

# Flask 앱
app  = Flask(__name__)

_stream_frame: bytes | None = None
_frame_lock   = threading.Lock()
_override_frame:  bytes | None = None  # 외부 주입 프레임 (rack_detector, qr_detector 디버그 화면용)
_override_lock    = threading.Lock()

def _generate_frames():
    while True:
        # 외부 프레임 우선 사용
        with _override_lock:
            override = _override_frame

        if override is not None:
            current = override
        else:
            with _frame_lock:
                current = _stream_frame

        if current is None:
            time.sleep(0.05)
            continue

        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            current + b'\r\n'
        )

@app.route('/stream')
def video_feed():
    return Response(
        _generate_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_node')
        self._bridge = CvBridge()
        self._pub    = self.create_publisher(RosImage, 'camera/image', 10)

        # ✅ action_server가 do_load Pick 단계에서만 켜고(_set_detect(True))
        # 그 외 구간(랙 스캔, QR 스캔, Place 등)에서는 꺼두는(_set_detect(False))
        # /detect_enable 신호를 그대로 활용.
        # process_frame()은 do_load Pick 단계의 박스 위치 검출에만 필요하므로
        # 꺼져있는 동안은 호출을 건너뛰어 CPU 부하를 줄인다.
        self._detect_enabled = True
        self.create_subscription(
            Bool, 'detect_enable', self._detect_enable_callback, 10
        )

        # action_server(별도 프로세스)가 보내는 디버그 프레임 구독
        # set_stream_frame/clear_stream_frame을 직접 import해서 호출하던 방식은
        # action_server와 flask_node가 서로 다른 프로세스라 전역변수를 공유하지
        # 못해 동작하지 않았음 → ROS2 토픽으로 전달받는 방식으로 변경
        self.create_subscription(
            RosImage, 'debug_stream', self._debug_stream_callback, 10
        )
        self.create_subscription(
            Empty, 'debug_stream_clear', self._debug_stream_clear_callback, 10
        )

        self._cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._cap.set(cv2.CAP_PROP_FPS, 30)

        if not self._cap.isOpened():
            self.get_logger().error("카메라 열기 실패 (/dev/video0)")
            raise RuntimeError("카메라 열기 실패")

        self.get_logger().info("카메라 열림 → camera/image 발행 + Flask 스트리밍 시작")
        self.create_timer(1.0 / 30.0, self._timer_callback)

    def _detect_enable_callback(self, msg):
        """/detect_enable 신호 수신 → process_frame() 호출 여부 결정"""
        self._detect_enabled = msg.data

    def _debug_stream_callback(self, msg):
        """/debug_stream 토픽 수신 → Flask 화면 override"""
        global _override_frame
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, buf = cv2.imencode('.jpg', frame)
        with _override_lock:
            _override_frame = buf.tobytes()

    def _debug_stream_clear_callback(self, msg):
        """/debug_stream_clear 토픽 수신 → override 해제"""
        global _override_frame
        with _override_lock:
            _override_frame = None

    def _timer_callback(self):
        global _stream_frame

        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn("프레임 읽기 실패")
            return

        if self._detect_enabled:
            # 검출 + 시각화 프레임 받기
            # output : 컨투어 박스, 변 길이, AR, 중심점이 그려진 프레임
            _, _, _, output = process_frame(frame)
        else:
            # ✅ do_load Pick 단계가 아닐 때는 검출이 필요 없으므로
            # process_frame()(GaussianBlur, HSV 변환, findContours 등) 자체를
            # 건너뛰고 원본 프레임을 그대로 사용 → CPU 부하 절감
            output = frame

        # Flask 스트리밍 버퍼 갱신 — 원본 대신 시각화된(또는 원본) output 사용
        with _override_lock:
            if _override_frame is None:
                _, buf = cv2.imencode('.jpg', output)
                with _frame_lock:
                    _stream_frame = buf.tobytes()

        # camera/image 토픽 발행 — 원본 frame (cv_node가 별도로 검출 로직 수행)
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self._pub.publish(msg)

    def destroy_node(self):
        self._cap.release()
        super().destroy_node()


def main(args=None):
    # Flask 백그라운드 스레드 시작
    flask_thread = threading.Thread(
        target=lambda: app.run(host='0.0.0.0', port=5000, threaded=True),
        daemon=True
    )
    flask_thread.start()
    print("Flask 스트리밍 → http://0.0.0.0:5000/stream")

    rclpy.init(args=args)
    node = FlaskNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()