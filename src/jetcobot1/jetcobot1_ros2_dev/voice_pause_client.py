#!/usr/bin/env python3
"""
voice_pause_client.py - 노트북에서 실행하는 음성 STT 클라이언트 (경량화 버전)

마이크로 음성을 받아 STT(Whisper)만 수행하고, 인식된 텍스트를
Task Manager(메인 서버)의 POST /voice/transcript 로 그대로 전송한다.

────────────────────────────────────────────────────────────────
변경 사항 (역할 재분배):
  - 기존: 노트북에서 키워드 매칭 + LLM(Claude Haiku) 분류까지 전부 수행
         → Jetson의 voice_command_server.py(HTTP)로 pause/resume 전송
  - 변경: 노트북은 STT만 수행 (분류는 Task Manager가 이미 보유한 LLM으로 처리)
         → Task Manager의 POST /voice/transcript 로 인식된 텍스트만 전송
         → Task Manager가 분류 후 JetCobot1에 SetPause 서비스 호출까지 처리

  이 노트북은 더 이상 ROS2/anthropic SDK에 의존하지 않는다.
  (rclpy, pinky_msgs, anthropic 패키지 불필요 — 순수 HTTP 클라이언트)
────────────────────────────────────────────────────────────────

⚠️ 이 기능은 "비상정지"가 아니라 관리자가 정리/청소 등을 위해
   다음 동작 시작 전에 로봇을 잠시 멈추는 운영성 일시정지입니다.

사전 설치 (노트북):
    pip install openai-whisper sounddevice numpy requests
    (Whisper는 ffmpeg 필요: brew install ffmpeg / apt install ffmpeg)

실행:
    python3 voice_pause_client.py --task-manager-ip 192.168.1.50

옵션:
    --task-manager-ip   Task Manager가 도는 머신 IP (기본 192.168.1.50)
    --port               Task Manager 포트 (기본 8090)
    --robot-id           대상 로봇 ID (기본 jetcobot1)
"""

import argparse
import time

import numpy as np
import requests
import sounddevice as sd
import whisper

# ── 설정 ──────────────────────────────────────────────────────────
SAMPLE_RATE = 16000
RECORD_SECONDS = 4          # 한 번에 듣는 길이 (초)
SILENCE_RMS_THRESHOLD = 0.01  # 이보다 조용하면 STT 호출 생략 (불필요한 처리 방지)

WHISPER_MODEL_NAME = "small"  # tiny/base/small/medium — 노트북 성능에 맞게 조정

DEFAULT_TASK_MANAGER_IP = "192.168.1.50"
DEFAULT_TASK_MANAGER_PORT = 8090
DEFAULT_ROBOT_ID = "jetcobot1"


def record_audio(seconds=RECORD_SECONDS, sample_rate=SAMPLE_RATE) -> np.ndarray:
    print(f"[voice] 녹음 중... ({seconds}s)")
    audio = sd.rec(int(seconds * sample_rate), samplerate=sample_rate, channels=1, dtype="float32")
    sd.wait()
    return audio.flatten()


def is_silent(audio: np.ndarray) -> bool:
    rms = np.sqrt(np.mean(audio ** 2))
    return rms < SILENCE_RMS_THRESHOLD


def transcribe(model, audio: np.ndarray) -> str:
    result = model.transcribe(audio, language="ko", fp16=False)
    return result.get("text", "").strip()


def send_transcript(base_url: str, text: str, robot_id: str):
    """
    인식된 텍스트를 그대로 Task Manager에 전송.
    pause/resume 여부 판단은 전적으로 Task Manager(LLM) 책임.
    """
    try:
        r = requests.post(
            f"{base_url}/voice/transcript",
            json={"text": text, "robot_id": robot_id},
            timeout=5,
        )
        r.raise_for_status()
        res = r.json()
        print(f"[voice] → Task Manager 응답: {res}")
    except requests.exceptions.RequestException as e:
        print(f"[voice] ⚠️ Task Manager 전송 실패: {e}")


def main():
    parser = argparse.ArgumentParser(description="JetCobot1 음성 STT 클라이언트 (경량화)")
    parser.add_argument("--task-manager-ip", default=DEFAULT_TASK_MANAGER_IP)
    parser.add_argument("--port", type=int, default=DEFAULT_TASK_MANAGER_PORT)
    parser.add_argument("--robot-id", default=DEFAULT_ROBOT_ID)
    args = parser.parse_args()

    base_url = f"http://{args.task_manager_ip}:{args.port}"
    print(f"[voice] Task Manager 대상: {base_url}")
    print(f"[voice] 대상 로봇: {args.robot_id}")
    print(f"[voice] Whisper 모델 로딩 중 ({WHISPER_MODEL_NAME})...")
    model = whisper.load_model(WHISPER_MODEL_NAME)
    print("[voice] 준비 완료. 말씀하시면 Task Manager로 전송합니다. (Ctrl+C로 종료)")

    try:
        while True:
            audio = record_audio()

            if is_silent(audio):
                continue

            text = transcribe(model, audio)
            if not text:
                continue

            print(f"[voice] 인식: \"{text}\"")
            send_transcript(base_url, text, args.robot_id)

    except KeyboardInterrupt:
        print("\n[voice] 종료됨")


if __name__ == "__main__":
    main()
