import socket
import struct
import numpy as np
import cv2
from PyQt5.QtCore import QThread, pyqtSignal

HEADER_FMT  = ">IHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

class CameraThread(QThread):
    frame_received = pyqtSignal(np.ndarray)

    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self.running = True

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(5.0)

        try:
            sock.sendto(b'\x00', (self.host, self.port))
            print(f"[연결] {self.host}:{self.port}")

            buf = {}

            while self.running:
                try:
                    packet, _ = sock.recvfrom(65536)
                except socket.timeout:
                    print("[경고] 수신 타임아웃 — 재연결 시도")
                    sock.sendto(b'\x00', (self.host, self.port))
                    continue

                if len(packet) < HEADER_SIZE:
                    continue

                frame_id, chunk_idx, total = struct.unpack(
                    HEADER_FMT, packet[:HEADER_SIZE]
                )
                chunk_data = packet[HEADER_SIZE:]

                if frame_id not in buf:
                    buf[frame_id] = {}
                buf[frame_id][chunk_idx] = chunk_data

                if len(buf[frame_id]) == total:
                    data = b"".join(buf[frame_id][i] for i in range(total))
                    frame = cv2.imdecode(
                        np.frombuffer(data, dtype=np.uint8),
                        cv2.IMREAD_COLOR
                    )
                    if frame is not None:
                        self.frame_received.emit(frame)
                    del buf[frame_id]

                for fid in [f for f in buf if f < frame_id - 10]:
                    del buf[fid]

        except Exception as e:
            print(f"[오류] {e}")
        finally:
            sock.close()

    def stop(self):
        self.running = False
        self.quit()
