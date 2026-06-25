#!/usr/bin/env python3
"""
voice_pause_client.py - 노트북에서 실행하는 음성 일시정지 클라이언트

마이크로 음성을 받아 STT(Whisper) → 키워드 매칭(1차) → 필요시 LLM 분류(2차)
→ Jetson의 voice_command_server.py로 HTTP 전송.

⚠️ 이 기능은 "비상정지"가 아니라 관리자가 정리/청소 등을 위해
   다음 동작 시작 전에 로봇을 잠시 멈추는 운영성 일시정지입니다.

사전 설치 (노트북):
    pip install openai-whisper sounddevice numpy requests anthropic
    (Whisper는 ffmpeg 필요: brew install ffmpeg / apt install ffmpeg)

실행:
    export ANTHROPIC_API_KEY=sk-...      # LLM 보조 분류를 쓰려면 필요
    python3 voice_pause_client.py --jetson-ip 192.168.1.65

옵션:
    --jetson-ip   Jetson IP (기본 192.168.1.65)
    --port        Jetson voice_command_server 포트 (기본 8011)
    --no-llm      LLM 보조 분류 끄고 키워드만 사용 (네트워크/API 의존 제거)
"""

import argparse
import re
import sys
import time

import numpy as np
import requests
import sounddevice as sd
import whisper

# ── 설정 ──────────────────────────────────────────────────────────
SAMPLE_RATE = 16000
RECORD_SECONDS = 4          # 한 번에 듣는 길이 (초)
SILENCE_RMS_THRESHOLD = 0.01  # 이보다 조용하면 STT 호출 생략 (불필요한 처리 방지)

PAUSE_KEYWORDS = ["정지", "멈춰", "스톱", "그만", "중지"]
RESUME_KEYWORDS = ["재시작", "다시 시작", "다시시작", "시작해", "계속"]

WHISPER_MODEL_NAME = "medium"  # tiny/base/small/medium — 노트북 성능에 맞게 조정


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


def classify_keyword(text: str) -> str:
    """1차: 키워드 즉시 매칭. pause / resume / none 반환."""
    if any(k in text for k in PAUSE_KEYWORDS):
        return "pause"
    if any(k in text for k in RESUME_KEYWORDS):
        return "resume"
    return "none"


def classify_llm(text: str) -> str:
    """2차: 키워드로 애매할 때만 호출. 실패 시 항상 'none'으로 안전하게 처리."""
    try:
        import anthropic
        client = anthropic.Anthropic()
        resp = client.messages.create(
            model="claude-haiku-4-5-20251001",
            max_tokens=10,
            system=(
                "당신은 협동로봇 제어 명령 분류기입니다. "
                "입력된 한국어 발화가 로봇 동작을 일시정지하라는 의도면 'pause', "
                "다시 시작/재개하라는 의도면 'resume', 둘 다 아니면 'none'만 출력하세요. "
                "단어 하나만 출력하고 다른 설명은 절대 포함하지 마세요."
            ),
            messages=[{"role": "user", "content": text}],
        )
        result = resp.content[0].text.strip().lower()
        return result if result in ("pause", "resume", "none") else "none"
    except Exception as e:
        print(f"[voice] LLM 분류 실패 (안전하게 무시): {e}")
        return "none"


def send_command(base_url: str, action: str, reason: str = ""):
    try:
        if action == "pause":
            r = requests.post(f"{base_url}/voice/pause", json={"reason": reason}, timeout=3)
        elif action == "resume":
            r = requests.post(f"{base_url}/voice/resume", timeout=3)
        else:
            return
        r.raise_for_status()
        print(f"[voice] → Jetson 전송 성공: {action} ({r.json()})")
    except requests.exceptions.RequestException as e:
        print(f"[voice] ⚠️ Jetson 전송 실패: {e}")


def main():
    parser = argparse.ArgumentParser(description="JetCobot1 음성 일시정지 클라이언트")
    parser.add_argument("--jetson-ip", default="192.168.1.65")
    parser.add_argument("--port", type=int, default=8011)
    parser.add_argument("--no-llm", action="store_true", help="LLM 보조 분류 비활성화 (키워드만 사용)")
    args = parser.parse_args()

    base_url = f"http://{args.jetson_ip}:{args.port}"
    print(f"[voice] Jetson 대상: {base_url}")
    print(f"[voice] Whisper 모델 로딩 중 ({WHISPER_MODEL_NAME})...")
    model = whisper.load_model(WHISPER_MODEL_NAME)
    print("[voice] 준비 완료. 정지/재시작을 말해보세요. (Ctrl+C로 종료)")
    print(f"[voice] 정지 키워드: {PAUSE_KEYWORDS}")
    print(f"[voice] 재시작 키워드: {RESUME_KEYWORDS}")

    try:
        while True:
            audio = record_audio()

            if is_silent(audio):
                continue

            text = transcribe(model, audio)
            if not text:
                continue

            print(f"[voice] 인식: \"{text}\"")

            # 1차: 키워드 매칭 (저지연, 네트워크 불필요)
            action = classify_keyword(text)

            # 2차: 애매하면 LLM 보조 (옵션)
            if action == "none" and not args.no_llm:
                action = classify_llm(text)

            if action == "pause":
                print("[voice] ⏸ 정지 명령 감지")
                send_command(base_url, "pause", reason=text)
            elif action == "resume":
                print("[voice] ▶ 재시작 명령 감지")
                send_command(base_url, "resume")
            # action == "none" → 무시, 다음 녹음으로

    except KeyboardInterrupt:
        print("\n[voice] 종료됨")


if __name__ == "__main__":
    main()