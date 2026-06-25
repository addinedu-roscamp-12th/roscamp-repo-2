#!/usr/bin/env python3
"""
voice_command_server.py - Jetson에서 실행하는 음성 명령 수신 서버

노트북에서 STT + 키워드/LLM 분류 후 보낸 pause/resume 명령을
HTTP POST로 받아 pause_control.PAUSE_EVENT를 제어한다.

main.py와는 별도 프로세스로 실행 (포트 8011 고정).
main.py, pick_and_place.py, weight_aging.py가 같은
pause_control.PAUSE_EVENT를 보고 있으므로, 이 서버를 통해
들어온 pause()/resume() 호출이 즉시 전체 로봇 동작에 반영된다.

실행:
    python3 voice_command_server.py
    (또는 main.py와 같은 프로세스에서 백그라운드 스레드로 띄우려면
     main.py에서 start_voice_server_in_background() 호출)

엔드포인트:
    POST /voice/pause   { "reason": "관리자 정리 작업" }  (reason 생략 가능)
    POST /voice/resume
    GET  /voice/status   → 현재 일시정지 상태 확인
"""

import threading
import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel

from pause_control import pause, resume, is_paused, get_pause_reason

app = FastAPI(title="JetCobot1 Voice Command Server")


class PauseRequest(BaseModel):
    reason: str = ""


@app.post("/voice/pause")
def voice_pause(req: PauseRequest = PauseRequest()):
    pause(req.reason or "음성 명령")
    return {"status": "ok", "paused": True, "reason": get_pause_reason()}


@app.post("/voice/resume")
def voice_resume():
    resume()
    return {"status": "ok", "paused": False}


@app.get("/voice/status")
def voice_status():
    return {"paused": is_paused(), "reason": get_pause_reason()}


def run_server(host="0.0.0.0", port=8011):
    uvicorn.run(app, host=host, port=port, log_level="info")


def start_voice_server_in_background(host="0.0.0.0", port=8011):
    """main.py에서 별도 스레드로 띄우고 싶을 때 사용."""
    thread = threading.Thread(
        target=run_server, kwargs={"host": host, "port": port}, daemon=True
    )
    thread.start()
    print(f"[voice_server] 백그라운드 실행 중 (포트 {port})")
    return thread


if __name__ == "__main__":
    print("[voice_server] JetCobot1 음성 명령 서버 시작 (포트 8011)")
    run_server()