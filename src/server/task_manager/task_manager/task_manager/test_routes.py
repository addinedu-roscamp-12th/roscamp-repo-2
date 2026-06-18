"""
테스트 전용 라우터 — 기존 시나리오 코드와 완전 분리
/test/* 경로에서만 동작하며 프로덕션 플로우에 영향 없음

Test 1 (/test/stack/*):
    home_stack 기반 배차 순서 검증
    - pinky를 수동으로 home_stack에 등록/해제
    - pick_pinky 호출 → 선택된 pinky에게 실제 navigate 명령 전송

Test 2 (/test/wait/*):
    load_wait 대기 로직 검증
    - pinky_A: load_wait 도착 후 적재 완료 → 출발 시도
    - pinky_B: home → load_wait 이동 중(incoming)이면 pinky_A 대기
    - pinky_B load_wait 도착 신호 → pinky_A 자동 출발
"""

import threading
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter(prefix="/test", tags=["test"])

# ── 공유 상태 참조 (task_manager 모듈에서 lazy import) ───────────
def _tm():
    import importlib, sys
    return sys.modules["task_manager.task_manager"]


# ════════════════════════════════════════════════════════════
# Test 2 전용 상태 (메인 tasks dict와 완전 분리)
# ════════════════════════════════════════════════════════════

class _WaitLogicState:
    def __init__(self):
        self.lock       = threading.Lock()
        self.reset()

    def reset(self):
        self.pinky_a: str | None = None   # load_wait에서 출발 대기 중인 pinky
        self.pinky_b: str | None = None   # load_wait로 이동 중인 pinky
        self.a_load_wait: str | None = None
        self.b_load_wait: str | None = None
        self.a_loaded: bool = False        # A가 적재 완료됐는지
        self.b_arrived: bool = False       # B가 load_wait에 도착했는지
        self.a_departed: bool = False      # A가 storage로 출발했는지
        self.log: list[str] = []

    def record(self, msg: str):
        from datetime import datetime
        self.log.append(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")
        print(f"[TEST-WAIT] {msg}")

_wait_state = _WaitLogicState()


# ════════════════════════════════════════════════════════════
# 공통 헬퍼
# ════════════════════════════════════════════════════════════

def _robots():
    return _tm().ROBOTS

def _stack():
    return _tm()._home_stack

def _lock():
    return _tm()._lock

def _send(robot_id, action, parameters, task_id):
    _tm()._send_command(robot_id, action, parameters, task_id)

def _pick_pinky():
    tm = _tm()
    return tm._dispatcher.pick_pinky(tm.ROBOTS, tm._home_stack)

def _is_incoming(exclude_id):
    return _tm()._is_any_pinky_incoming_to_load_wait(exclude_id)


# ════════════════════════════════════════════════════════════
# Test 1: home_stack 배차 순서 테스트
# ════════════════════════════════════════════════════════════

class DispatchReq(BaseModel):
    location: str = "load_wait_1"

@router.post("/stack/push/{pinky_id}",
             summary="[Test1] pinky를 home_stack에 수동 등록")
def t1_push(pinky_id: str):
    """
    pinky를 idle + home 상태로 설정하고 home_stack에 추가.
    stack 순서: 먼저 push된 것이 inner, 나중이 outer(먼저 배차).
    """
    robots = _robots()
    if pinky_id not in robots or robots[pinky_id].get("type") != "pinky":
        raise HTTPException(400, f"{pinky_id}는 pinky 타입이 아닙니다")
    with _lock():
        robots[pinky_id]["status"]       = "idle"
        robots[pinky_id]["task_id"]      = None
        robots[pinky_id]["current_zone"] = "home"
        robots[pinky_id]["target_zone"]  = None
        stack = _stack()
        if pinky_id not in stack:
            stack.append(pinky_id)
    stack = _stack()
    return {
        "ok":         True,
        "pushed":     pinky_id,
        "home_stack": list(stack),
        "positions":  {pid: ("outer" if stack[-1] == pid else "inner") for pid in stack},
        "tip":        "나중에 push된 pinky가 outer → 먼저 배차됩니다",
    }


@router.post("/stack/pop/{pinky_id}",
             summary="[Test1] pinky를 home_stack에서 제거")
def t1_pop(pinky_id: str):
    """home_stack에서 pinky 제거 (출발 시뮬레이션)."""
    with _lock():
        stack = _stack()
        if pinky_id in stack:
            stack.remove(pinky_id)
    return {"ok": True, "removed": pinky_id, "home_stack": list(_stack())}


@router.get("/stack/status",
            summary="[Test1] home_stack 현재 상태 조회")
def t1_status():
    """현재 home_stack 순서와 각 pinky 상태 반환."""
    robots = _robots()
    stack  = list(_stack())
    return {
        "home_stack": stack,
        "positions": {
            pid: ("outer(먼저배차)" if stack[-1] == pid else "inner")
            for pid in stack
        },
        "robots": {
            rid: {
                "status":       r.get("status"),
                "current_zone": r.get("current_zone"),
                "target_zone":  r.get("target_zone"),
            }
            for rid, r in robots.items()
            if r.get("type") == "pinky"
        },
    }


@router.post("/stack/dispatch",
             summary="[Test1] pick_pinky 실행 → 선택된 pinky에게 navigate 명령 전송")
def t1_dispatch(req: DispatchReq):
    """
    home_stack 기준으로 outer(바깥쪽) idle pinky를 선택하여
    실제 navigate 명령을 전송합니다.
    """
    import uuid
    with _lock():
        picked = _pick_pinky()
        if picked is None:
            raise HTTPException(503, "배차 가능한 idle pinky 없음 — /test/stack/push 로 먼저 등록하세요")
        robots = _robots()
        robots[picked]["status"]      = "busy"
        robots[picked]["target_zone"] = req.location
        robots[picked]["current_zone"] = None
        stack = _stack()
        if picked in stack:
            stack.remove(picked)

    task_id = f"test_dispatch_{uuid.uuid4()}"
    _send(picked, "navigate", {"location": req.location}, task_id)

    return {
        "ok":            True,
        "picked_pinky":  picked,
        "location":      req.location,
        "task_id":       task_id,
        "remaining_stack": list(_stack()),
        "tip":           f"{picked}에게 navigate({req.location}) 전송 완료",
    }


# ════════════════════════════════════════════════════════════
# Test 2: load_wait 대기 로직 테스트
# ════════════════════════════════════════════════════════════

class WaitInitReq(BaseModel):
    pinky_a: str = "pinky1"     # load_wait 먼저 도착할 pinky
    pinky_b: str = "pinky2"     # 뒤따라 오는 pinky
    load_wait_a: str = "load_wait_1"
    load_wait_b: str = "load_wait_2"
    storage_zone: str = "zone_1"

class StorageReq(BaseModel):
    storage_zone: str = "zone_1"


@router.post("/wait/init",
             summary="[Test2] 대기 로직 테스트 초기화 — 두 pinky를 load_wait으로 보냄")
def t2_init(req: WaitInitReq):
    """
    시나리오:
      pinky_A → load_wait_A 이동 (먼저 출발)
      pinky_B → load_wait_B 이동 (나중 출발, incoming 상태)
    두 로봇 모두 idle 상태로 설정 후 실제 navigate 명령 전송.
    """
    import uuid
    robots = _robots()
    for pid in (req.pinky_a, req.pinky_b):
        if pid not in robots or robots[pid].get("type") != "pinky":
            raise HTTPException(400, f"{pid}는 pinky 타입이 아닙니다")

    with _lock():
        _wait_state.reset()
        _wait_state.pinky_a    = req.pinky_a
        _wait_state.pinky_b    = req.pinky_b
        _wait_state.a_load_wait = req.load_wait_a
        _wait_state.b_load_wait = req.load_wait_b

        for pid, lw in ((req.pinky_a, req.load_wait_a), (req.pinky_b, req.load_wait_b)):
            robots[pid]["status"]       = "busy"
            robots[pid]["task_id"]      = f"test_wait_{uuid.uuid4()}"
            robots[pid]["current_zone"] = None
            robots[pid]["target_zone"]  = lw
            stack = _stack()
            if pid in stack:
                stack.remove(pid)

    _send(req.pinky_a, "navigate", {"location": req.load_wait_a}, f"test_wait_a_{uuid.uuid4()}")
    _send(req.pinky_b, "navigate", {"location": req.load_wait_b}, f"test_wait_b_{uuid.uuid4()}")
    _wait_state.record(f"{req.pinky_a} → {req.load_wait_a} 이동 명령 전송")
    _wait_state.record(f"{req.pinky_b} → {req.load_wait_b} 이동 명령 전송 (incoming 상태)")

    return {
        "ok":     True,
        "pinky_a": req.pinky_a,
        "pinky_b": req.pinky_b,
        "tip":    f"다음: /test/wait/arrived/{{pinky_id}} 로 도착 신호 보내세요",
    }


@router.post("/wait/arrived/{pinky_id}",
             summary="[Test2] pinky load_wait 도착 신호 (수동 주입)")
def t2_arrived(pinky_id: str, req: StorageReq):
    """
    pinky가 load_wait에 도착했음을 수동으로 신호.
    - pinky_A 도착: 적재 완료로 표시, 출발 시도 → pinky_B incoming이면 대기
    - pinky_B 도착: pinky_A 대기 해제 → 자동 출발
    """
    import uuid
    robots = _robots()
    ws     = _wait_state

    if pinky_id not in (ws.pinky_a, ws.pinky_b):
        raise HTTPException(400, f"{pinky_id}는 현재 테스트에 없음. /test/wait/init 먼저 실행")

    with _lock():
        is_a = (pinky_id == ws.pinky_a)
        lw   = ws.a_load_wait if is_a else ws.b_load_wait
        robots[pinky_id]["current_zone"] = lw
        robots[pinky_id]["target_zone"]  = None

        if is_a:
            ws.a_loaded = True
            ws.record(f"{pinky_id} load_wait({lw}) 도착 + 적재 완료 표시")

            # 출발 시도
            incoming = _is_incoming(pinky_id)
            if incoming:
                ws.record(f"{ws.pinky_b} 아직 incoming → {pinky_id} 대기")
                return {
                    "ok":     True,
                    "event":  "waiting",
                    "reason": f"{ws.pinky_b}가 load_wait로 이동 중 → 도착 후 자동 출발",
                    "log":    ws.log[-5:],
                    "tip":    f"/test/wait/arrived/{ws.pinky_b} 로 pinky_B 도착 신호 보내세요",
                }
            else:
                # 바로 출발
                robots[pinky_id]["current_zone"] = None
                robots[pinky_id]["target_zone"]  = req.storage_zone
                ws.a_departed = True
                ws.record(f"{pinky_id} → {req.storage_zone} 즉시 출발")
                _send(pinky_id, "navigate", {"location": req.storage_zone},
                      f"test_wait_depart_{uuid.uuid4()}")
                return {
                    "ok":    True,
                    "event": "departed",
                    "pinky": pinky_id,
                    "to":    req.storage_zone,
                    "log":   ws.log[-5:],
                }

        else:  # pinky_B 도착
            ws.b_arrived = True
            ws.record(f"{pinky_id} load_wait({lw}) 도착")

            # pinky_A가 대기 중이었으면 출발시킴
            if ws.a_loaded and not ws.a_departed:
                robots[ws.pinky_a]["current_zone"] = None
                robots[ws.pinky_a]["target_zone"]  = req.storage_zone
                ws.a_departed = True
                ws.record(f"→ {ws.pinky_a} 대기 해제 → {req.storage_zone} 출발")
                _send(ws.pinky_a, "navigate", {"location": req.storage_zone},
                      f"test_wait_depart_{uuid.uuid4()}")
                return {
                    "ok":    True,
                    "event": "pinky_a_released",
                    "pinky_b_arrived": pinky_id,
                    "pinky_a_departed_to": req.storage_zone,
                    "log":   ws.log[-5:],
                }
            return {
                "ok":    True,
                "event": "b_arrived",
                "pinky": pinky_id,
                "log":   ws.log[-5:],
            }


@router.get("/wait/status",
            summary="[Test2] 현재 대기 로직 테스트 상태 조회")
def t2_status():
    ws     = _wait_state
    robots = _robots()
    return {
        "pinky_a":       ws.pinky_a,
        "pinky_b":       ws.pinky_b,
        "a_load_wait":   ws.a_load_wait,
        "b_load_wait":   ws.b_load_wait,
        "a_loaded":      ws.a_loaded,
        "b_arrived":     ws.b_arrived,
        "a_departed":    ws.a_departed,
        "robots": {
            pid: {
                "status":       robots[pid].get("status"),
                "current_zone": robots[pid].get("current_zone"),
                "target_zone":  robots[pid].get("target_zone"),
            }
            for pid in (ws.pinky_a, ws.pinky_b)
            if pid and pid in robots
        },
        "log": ws.log,
    }


@router.post("/wait/reset",
             summary="[Test2] 테스트 상태 초기화")
def t2_reset():
    """테스트 상태를 초기화합니다. robots 상태도 idle로 리셋."""
    robots = _robots()
    ws     = _wait_state
    with _lock():
        for pid in (ws.pinky_a, ws.pinky_b):
            if pid and pid in robots:
                robots[pid]["status"]       = "idle"
                robots[pid]["task_id"]      = None
                robots[pid]["current_zone"] = None
                robots[pid]["target_zone"]  = None
        ws.reset()
    return {"ok": True, "msg": "테스트 상태 초기화 완료"}
