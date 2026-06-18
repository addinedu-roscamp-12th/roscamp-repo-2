import json
import os
import re
from pathlib import Path

import requests
import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, JSONResponse
from pydantic import BaseModel

OLLAMA_URL        = os.getenv("OLLAMA_URL", "http://localhost:11434")
TASK_MANAGER_URL  = os.getenv("TASK_MANAGER_URL", "http://localhost:8090")
MODEL             = os.getenv("MODEL", "qwen2.5:3b")  # 파인튜닝 후 pinky-warehouse로 변경

def _get_routing_mode() -> str:
    """task_manager에서 현재 라우팅 모드를 조회."""
    try:
        resp = requests.get(f"{TASK_MANAGER_URL}/mode", timeout=2)
        return resp.json().get("routing_mode", "zone")
    except Exception:
        return "zone"


def _build_system_prompt(mode: str) -> str:
    if mode == "waypoint":
        inbound_block = """\
[입고 작업] 렉 구역(1~3)을 지정해 작업 시작:
{"action": "inbound_task", "parameters": {"storage_zone": "rack_<N>"}}

렉 번호 변환 규칙 (반드시 따를 것):
  "N번 렉" / "N번 랙" / "N번 구역" / "N번에 넣어줘" → storage_zone: "rack_N"
  예) "1번 렉에 넣어줘" → {"action": "inbound_task", "parameters": {"storage_zone": "rack_1"}}
  예) "3번 랙으로"     → {"action": "inbound_task", "parameters": {"storage_zone": "rack_3"}}
  렉 번호 범위: 1 ~ 3"""
    else:
        inbound_block = """\
[입고 작업] 렉 번호(1~16)를 지정해 작업 시작:
{"action": "inbound_task", "parameters": {"storage_zone": "zone_<N>"}}

렉 번호 변환 규칙 (반드시 따를 것):
  "N번 렉" / "N번 랙" / "N번 구역" / "N번에 넣어줘" → storage_zone: "zone_N"
  예) "3번 렉에 넣어줘" → {"action": "inbound_task", "parameters": {"storage_zone": "zone_3"}}
  예) "7번 랙으로"     → {"action": "inbound_task", "parameters": {"storage_zone": "zone_7"}}
  렉 번호 범위: 1 ~ 16"""

    return f"""\
당신은 물류창고 로봇 제어 AI입니다.
사용자의 자연어 명령을 해석하여 반드시 JSON만 출력하세요.
설명 텍스트, 마크다운, 줄바꿈 없이 JSON 한 줄만 출력하세요.

{inbound_block}

[출고 작업] 렉에서 물건 꺼내기:
{{"action": "outbound_task", "parameters": {{"storage_zone": "zone_<N>"}}}}

출고 변환 규칙:
  "N번 렉 꺼내줘" / "N번 랙 출고" / "N번 출고해줘" → outbound_task, storage_zone: "zone_N"
  예) "1번 렉 꺼내줘" → {{"action": "outbound_task", "parameters": {{"storage_zone": "zone_1"}}}}
  렉 번호 범위: 1 ~ 3

[단순 이동] 로봇 직접 이동 명령:
{{"action": "navigate", "parameters": {{"location": "<장소키>"}}}}

[핑키 출발] 홈에서 앞쪽 핑키를 load_wait_1으로 출발:
{{"action": "dispatch_pinky", "parameters": {{}}}}

출발 변환 규칙:
  "앞에 있는 핑키 출발시켜" / "핑키 보내줘" / "핑키 출발" / "로봇 출발" → dispatch_pinky

[정지]: {{"action": "navigate", "parameters": {{"location": "stop"}}}}
[이해불가]: {{"action": "unknown", "parameters": {{"reason": "<이유>"}}}}

장소 키: home, load_wait_1, load_wait_2, outbound_zone,
         zone_1, zone_2, zone_3, stop

장소 키 변환 규칙:
  "홈" / "집" / "원위치"          → home
  "상차 대기" / "대기 구역 1"     → load_wait_1
  "상차 대기 2" / "대기 구역 2"   → load_wait_2
  "출고 구역" / "출고 대기"        → outbound_zone
  "N번 존" / "존 N" / "zone N"   → zone_N  (N: 1~3)

"멈춰" / "정지" → navigate, location은 stop

JSON만 출력하세요."""


def ask_llm(text: str, mode: str = "zone") -> str:
    resp = requests.post(
        f"{OLLAMA_URL}/api/chat",
        json={
            "model": MODEL,
            "messages": [
                {"role": "system", "content": _build_system_prompt(mode)},
                {"role": "user",   "content": text},
            ],
            "stream": False,
            "options": {"temperature": 0.1, "num_predict": 50},
        },
        timeout=60,
    )
    resp.raise_for_status()
    return resp.json()["message"]["content"]


def forward_navigate_to_task_manager(robot: str, cmd: dict) -> str:
    if cmd.get("action") == "unknown":
        return f"이해 불가: {cmd.get('parameters', {}).get('reason', '')}"

    location = cmd.get("parameters", {}).get("location", "home")
    try:
        resp = requests.post(
            f"{TASK_MANAGER_URL}/navigate",
            json={"robot_id": robot, "location": location},
            timeout=5,
        )
        resp.raise_for_status()
        return f"{robot} → navigate({location}) 명령 전송"
    except requests.exceptions.ConnectionError:
        return f"태스크매니저 연결 실패 ({TASK_MANAGER_URL})"
    except requests.exceptions.Timeout:
        return f"태스크매니저 응답 없음 (timeout)"
    except Exception as e:
        return f"태스크매니저 오류: {e}"


app = FastAPI()


class CommandReq(BaseModel):
    text:  str
    robot: str = "pinky1"  # "pinky1" | "pinky2"


def forward_outbound_to_task_manager(cmd: dict) -> str:
    zone = cmd.get("parameters", {}).get("storage_zone")
    if not zone:
        return "storage_zone 누락"
    try:
        resp = requests.post(
            f"{TASK_MANAGER_URL}/task/outbound",
            json={"storage_zone": zone},
            timeout=5,
        )
        resp.raise_for_status()
        data = resp.json()
        return f"출고 태스크 생성 완료 — task_id: {data.get('task_id')} / 상태: {data.get('state')}"
    except requests.exceptions.ConnectionError:
        return f"태스크매니저 연결 실패 ({TASK_MANAGER_URL})"
    except requests.exceptions.HTTPError as e:
        status = e.response.status_code
        if status == 409:
            return f"{zone} 출고 불가 — 물건이 없거나 이미 출고 중"
        if status == 503:
            return "로봇 사용 중 — 잠시 후 다시 시도해주세요"
        return f"태스크매니저 오류 ({status}): {e.response.text}"
    except Exception as e:
        return f"태스크매니저 오류: {e}"


def forward_dispatch_to_task_manager(cmd: dict) -> str:
    try:
        resp = requests.post(
            f"{TASK_MANAGER_URL}/test/stack/dispatch",
            json={"location": "load_wait_1"},
            timeout=5,
        )
        resp.raise_for_status()
        data = resp.json()
        picked = data.get("picked_pinky", "?")
        return f"{picked} → load_wait_1 출발 명령 전송 (task_id: {data.get('task_id', '?')})"
    except requests.exceptions.ConnectionError:
        return f"태스크매니저 연결 실패 ({TASK_MANAGER_URL})"
    except requests.exceptions.HTTPError as e:
        if e.response.status_code == 503:
            return "배차 가능한 핑키 없음 — home_stack이 비어있습니다"
        return f"태스크매니저 오류 ({e.response.status_code}): {e.response.text}"
    except Exception as e:
        return f"태스크매니저 오류: {e}"


def forward_to_task_manager(cmd: dict) -> str:
    zone = cmd.get("parameters", {}).get("storage_zone")
    if not zone:
        return "storage_zone 누락"
    try:
        resp = requests.post(
            f"{TASK_MANAGER_URL}/task",
            json={"storage_zone": zone},
            timeout=5,
        )
        resp.raise_for_status()
        data = resp.json()
        return f"태스크 생성 완료 — task_id: {data.get('task_id')} / 상태: {data.get('state')}"
    except requests.exceptions.ConnectionError:
        return f"태스크매니저 연결 실패 ({TASK_MANAGER_URL})"
    except requests.exceptions.HTTPError as e:
        status = e.response.status_code
        if status == 409:
            return f"{zone} 이미 점유 중 — 다른 렉을 선택해주세요"
        if status == 503:
            return "jetcobot1 사용 중 — 잠시 후 다시 시도해주세요"
        return f"태스크매니저 오류 ({status}): {e.response.text}"
    except Exception as e:
        return f"태스크매니저 오류: {e}"


@app.post("/command")
async def command(req: CommandReq):
    try:
        mode = _get_routing_mode()
        raw  = ask_llm(req.text, mode)

        match = re.search(r'\{.*\}', raw, re.DOTALL)
        if not match:
            raise json.JSONDecodeError("JSON 없음", raw, 0)
        cmd = json.loads(match.group())

        if cmd.get("action") == "inbound_task":
            result = forward_to_task_manager(cmd)
        elif cmd.get("action") == "outbound_task":
            result = forward_outbound_to_task_manager(cmd)
        elif cmd.get("action") == "dispatch_pinky":
            result = forward_dispatch_to_task_manager(cmd)
        else:
            result = forward_navigate_to_task_manager(req.robot, cmd)

        return {"ok": True, "mode": mode, "llm_json": cmd, "result": result}

    except json.JSONDecodeError:
        return JSONResponse(status_code=400,
                            content={"ok": False, "error": f"JSON 파싱 실패: {raw}",
                                     "tip": "LLM이 JSON 외 텍스트를 포함했습니다."})
    except Exception as e:
        return JSONResponse(status_code=500,
                            content={"ok": False, "error": str(e)})


@app.get("/status/{robot}")
async def status(robot: str):
    try:
        resp = requests.get(f"{TASK_MANAGER_URL}/robots", timeout=3)
        robots = resp.json()
        if robot not in robots:
            return JSONResponse(status_code=404,
                                content={"ok": False, "error": f"알 수 없는 로봇: {robot}"})
        return {"ok": True, "robot": robot, "status": robots[robot]}
    except Exception as e:
        return JSONResponse(status_code=503, content={"ok": False, "error": str(e)})


@app.get("/", response_class=HTMLResponse)
async def index():
    with open(Path(__file__).parent / "index.html", encoding="utf-8") as f:
        return f.read()


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
