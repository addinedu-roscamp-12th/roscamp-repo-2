
#!/usr/bin/env python3
# flask_node.py — 카메라 점유 + Flask 스트리밍 전담 노드
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'lib', 'python3', 'dist-packages'))

# - /dev/video0 을 열어 프레임을 읽음
# - process_frame() 으로 검출 결과를 _last_detection 에 저장
# - camera/image 토픽을 발행 → cv_node 가 수신
# - http://<ip>:5000/stream 으로 MJPEG 스트리밍

import threading
import time
import cv2
from flask import Flask, Response

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
from jetcobot2.cv_lib import process_frame

# Flask 앱
app  = Flask(__name__)
_stream_frame: bytes | None = None
_frame_lock   = threading.Lock()


def _generate_frames():
    while True:
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

        self._cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._cap.set(cv2.CAP_PROP_FPS, 30)

        if not self._cap.isOpened():
            self.get_logger().error("카메라 열기 실패 (/dev/video0)")
            raise RuntimeError("카메라 열기 실패")

        self.get_logger().info("카메라 열림 → camera/image 발행 + Flask 스트리밍 시작")
        self.create_timer(1.0 / 30.0, self._timer_callback)

    def _timer_callback(self):
        global _stream_frame

        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn("프레임 읽기 실패")
            return

        # 검출 + 시각화 프레임 받기
        # output : 컨투어 박스, 변 길이, AR, 중심점이 그려진 프레임
        detected, cx, cy, area, hor, ver, ar, output = process_frame(frame)

        # Flask 스트리밍 버퍼 갱신 — 원본 대신 시각화된 output 사용
        _, buf = cv2.imencode('.jpg', output)
        with _frame_lock:
            _stream_frame = buf.tobytes()

        # camera/image 토픽 발행 — 원본 frame (검출 로직용)
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