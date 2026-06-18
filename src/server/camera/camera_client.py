import socket
import struct
import os
from datetime import datetime

import numpy as np
import cv2

SERVER_IP   = "192.168.1.65"
SERVER_PORT = 9000
SAVE_DIR    = os.path.expanduser("~/recordings")

# 패킷 헤더: frame_id(4B) + chunk_idx(2B) + total_chunks(2B) = 8B
HEADER_FMT  = ">IHH"
HEADER_SIZE = struct.calcsize(HEADER_FMT)


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(5.0)

    sock.sendto(b'\x00', (SERVER_IP, SERVER_PORT))
    print(f"[연결] {SERVER_IP}:{SERVER_PORT}")
    print("  r : 녹화 시작 / 정지")
    print("  q : 종료")

    buf       = {}
    writer    = None
    recording = False

    try:
        while True:
            try:
                packet, _ = sock.recvfrom(65536)
            except socket.timeout:
                print("[경고] 수신 타임아웃 — 재연결 시도")
                sock.sendto(b'\x00', (SERVER_IP, SERVER_PORT))
                continue

            if len(packet) < HEADER_SIZE:
                continue

            frame_id, chunk_idx, total = struct.unpack(HEADER_FMT, packet[:HEADER_SIZE])
            chunk_data = packet[HEADER_SIZE:]

            if frame_id not in buf:
                buf[frame_id] = {}
            buf[frame_id][chunk_idx] = chunk_data

            if len(buf[frame_id]) == total:
                data  = b"".join(buf[frame_id][i] for i in range(total))
                frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    if recording and writer is not None:
                        writer.write(frame)
                        # 녹화 중 빨간 점 표시
                        cv2.circle(frame, (20, 20), 8, (0, 0, 255), -1)
                        cv2.putText(frame, "REC", (35, 27),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                    cv2.imshow("Pinky Camera", frame)
                del buf[frame_id]

            for fid in [f for f in buf if f < frame_id - 30]:
                del buf[fid]

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("r"):
                if not recording:
                    filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".mp4"
                    filepath = os.path.join(SAVE_DIR, filename)
                    fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
                    writer   = cv2.VideoWriter(filepath, fourcc, 30.0, (frame.shape[1], frame.shape[0]))
                    recording = True
                    print(f"[녹화 시작] {filepath}")
                else:
                    recording = False
                    if writer:
                        writer.release()
                        writer = None
                    print("[녹화 정지]")

    finally:
        if writer:
            writer.release()
        sock.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
