"""
Task Manager (Pinky Only Test) - 핑키 1, 2만으로 알고리즘 테스트
포트: 8091 (기존 8090과 충돌 방지)

제코봇 없이 핑키의 배차/존 관리/출발 대기 알고리즘만 검증.
제코봇 픽업·적재·하역은 타이머로 자동 시뮬레이션.

플로우:
  PENDING
    → PINKY_TO_LOAD_WAIT   (핑키 → 주차대기구역)
    → MOCK_LOADING         (적재 시뮬, MOCK_LOAD_DELAY_SEC초 후 자동 완료)
    → [PINKY_WAIT_INCOMING]  (다른 핑키 주차대기구역 도착 대기, 조건부)
    → PINKY_TO_STORAGE     (핑키 → 보관구역)
    → PINKY_TO_HOME        (핑키 → 홈)
    → DONE
"""

import json
import queue
import threading
import uuid
from datetime import datetime
from enum import Enum

import rclpy
import uvicorn
from fastapi import FastAPI, HTTPException
from msgs.action import RobotCommand
from pydantic import BaseModel
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from .dispatcher import Dispatcher
from .task_queue import TaskQueue
from .zone_manager import ZoneManager


# ════════════════════════════════════════════════════════════
# CONFIG
# ════════════════════════════════════════════════════════════

MOCK_LOAD_DELAY_SEC = 2.0  # 제코봇 적재 시뮬레이션 시간(초)

ROBOTS: dict[str, dict] = {
    "pinky1": {
        "type": "pinky", "status": "idle", "task_id": None,
        "current_zone": None, "target_zone": None, "last_home_arrival_ts": 0.0,
    },
    "pinky2": {
        "type": "pinky", "status": "idle", "task_id": None,
        "current_zone": None, "target_zone": None, "last_home_arrival_ts": 0.0,
    },
}

# 주차대기구역 (load_wait) + 보관구역 (zone_1~16)
ZONES: dict[str, str] = {
    "load_wait_1": "free",
    "load_wait_2": "free",
}
ZONES.update({f"zone_{i}": "free" for i in range(1, 17)})

LOAD_WAIT_ZONES   = {"load_wait_1", "load_wait_2"}
VALID_STORAGE_ZONES = {f"zone_{i}" for i in range(1, 17)}


# ════════════════════════════════════════════════════════════
# DOMAIN MODEL
# ════════════════════════════════════════════════════════════

class TaskState(str, Enum):
    PENDING             = "pending"
    QUEUED              = "queued"
    PINKY_TO_LOAD_WAIT  = "pinky_to_load_wait"
    MOCK_LOADING        = "mock_loading"        # 적재 시뮬레이션 (타이머 대기)
    PINKY_WAIT_INCOMING = "pinky_wait_incoming" # 다른 핑키 load_wait 도착 대기
    PINKY_TO_STORAGE    = "pinky_to_storage"
    PINKY_TO_HOME       = "pinky_to_home"
    DONE                = "done"
    FAILED              = "failed"


class Task:
    def __init__(self, task_id: str, storage_zone: str):
        self.task_id        = task_id
        self.storage_zone   = storage_zone
        self.load_wait_zone: str | None = None
        self.state          = TaskState.PENDING
        self.pinky_id: str | None = None
        self.created_at     = datetime.now().isoformat()
        self.updated_at     = datetime.now().isoformat()
        self.log: list[str] = []

    def record(self, msg: str):
        self.log.append(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")
        self.updated_at = datetime.now().isoformat()
        print(f"[Task {self.task_id[:8]}] {msg}")

    def to_dict(self) -> dict:
        return {
            "task_id":        self.task_id,
            "state":          self.state,
            "storage_zone":   self.storage_zone,
            "load_wait_zone": self.load_wait_zone,
            "pinky":          self.pinky_id,
            "created_at":     self.created_at,
            "updated_at":     self.updated_at,
            "log":            self.log,
        }


# ════════════════════════════════════════════════════════════
# GLOBAL STATE
# ════════════════════════════════════════════════════════════

tasks: dict[str, Task] = {}
_lock = threading.Lock()
ros_node: "TaskManagerNode" = None
_cmd_queue: queue.Queue = queue.Queue()

_zone_manager = ZoneManager(ZONES)
_dispatcher   = Dispatcher()

_pending_queue = TaskQueue()   # 전체 dispatch 대기 (storage + pinky + load_wait)
_wait_queue    = TaskQueue()   # PINKY_WAIT_INCOMING 대기


# ════════════════════════════════════════════════════════════
# ROS2 NODE
# ════════════════════════════════════════════════════════════

class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager_pinky_test")
        self._cb_group = ReentrantCallbackGroup()
        self._action_clients = {
            robot_id: ActionClient(self, RobotCommand, f"/{robot_id}/command",
                                   callback_group=self._cb_group)
            for robot_id in ROBOTS
        }
        self.create_timer(0.05, self._flush_cmd_queue, callback_group=self._cb_group)
        self.get_logger().info("Task Manager (Pinky Only Test) 시작 — 포트 8091")

    def _flush_cmd_queue(self):
        try:
            cmd = _cmd_queue.get_nowait()
        except queue.Empty:
            return

        robot_id   = cmd["robot_id"]
        action     = cmd["action"]
        parameters = cmd["parameters"]
        task_id    = cmd["task_id"]
        client     = self._action_clients[robot_id]

        if not client.server_is_ready():
            self._retry = getattr(self, "_retry", {})
            self._retry[robot_id] = self._retry.get(robot_id, 0) + 1
            if self._retry[robot_id] % 40 == 1:
                print(f"[WARN] {robot_id} 서버 미준비 — 대기 중")
            _cmd_queue.put(cmd)
            return
        self._retry = getattr(self, "_retry", {})
        self._retry[robot_id] = 0

        goal = RobotCommand.Goal()
        goal.action_type     = action
        goal.parameters_json = json.dumps(parameters)
        goal.task_id         = task_id

        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_goal_response(f, robot_id, task_id)
        )
        print(f"[ROS2 Action] → /{robot_id}/command  action={action}  task={task_id[:8]}")

    def _on_goal_response(self, future, robot_id: str, task_id: str):
        print(f"[GOAL_RESPONSE] {robot_id} 응답 수신 (task={task_id[:8]})")
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f"[WARN] {robot_id} 목표 거부됨")
            task = tasks.get(task_id)
            if task:
                with _lock:
                    _fail_task(task, f"{robot_id} 목표 거부됨")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_result(f, robot_id, task_id)
        )

    def _on_result(self, future, robot_id: str, task_id: str):
        result  = future.result().result
        event   = result.event
        message = result.message
        print(f"[RESULT] robot={robot_id}  event={event!r}  msg={message!r}  task={task_id[:8]}")
        with _lock:
            task = tasks.get(task_id)
            if task is None:
                return
            task.record(f"{robot_id} → {event}")
            _advance(task, robot_id, event)


# ════════════════════════════════════════════════════════════
# ORCHESTRATION
# ════════════════════════════════════════════════════════════

def _set_robot_busy(robot_id: str, task_id: str):
    ROBOTS[robot_id]["status"]  = "busy"
    ROBOTS[robot_id]["task_id"] = task_id

def _set_robot_idle(robot_id: str):
    ROBOTS[robot_id]["status"]  = "idle"
    ROBOTS[robot_id]["task_id"] = None

def _fail_task(task: Task, reason: str):
    task.state = TaskState.FAILED
    task.record(f"태스크 실패 — {reason}")
    if task.pinky_id and ROBOTS[task.pinky_id]["task_id"] == task.task_id:
        pinky = ROBOTS[task.pinky_id]
        for z in {pinky.get("current_zone"), pinky.get("target_zone"), task.load_wait_zone} - {None}:
            _zone_manager.release(z)
        pinky["current_zone"] = None
        pinky["target_zone"]  = None
        _set_robot_idle(task.pinky_id)
    _zone_manager.release(task.storage_zone)
    _pending_queue.drain(_try_dispatch)

def _send_command(robot_id: str, action: str, parameters: dict, task_id: str):
    _cmd_queue.put({"robot_id": robot_id, "action": action,
                    "parameters": parameters, "task_id": task_id})

def _is_any_pinky_incoming_to_load_wait(exclude_robot_id: str) -> bool:
    for robot_id, robot in ROBOTS.items():
        if robot_id == exclude_robot_id:
            continue
        if robot.get("type") == "pinky" and robot.get("target_zone") in LOAD_WAIT_ZONES:
            return True
    return False


# ── 디스패치 ──────────────────────────────────────────────────
def _try_dispatch(task: Task) -> bool:
    """
    storage_zone 잠금 + idle pinky + free load_wait 모두 확보되면 즉시 출발.
    하나라도 실패하면 모두 원복 후 False.
    """
    if task.state not in (TaskState.PENDING, TaskState.QUEUED):
        return False
    if not _zone_manager.try_acquire(task.storage_zone, task.task_id):
        return False

    pinky_id = _dispatcher.pick_pinky(ROBOTS)
    if pinky_id is None:
        _zone_manager.release(task.storage_zone)
        return False

    load_wait = None
    for zone in ("load_wait_1", "load_wait_2"):
        if _zone_manager.try_acquire(zone, pinky_id):
            load_wait = zone
            break
    if load_wait is None:
        _zone_manager.release(task.storage_zone)
        return False

    task.pinky_id       = pinky_id
    task.load_wait_zone = load_wait
    task.state          = TaskState.PINKY_TO_LOAD_WAIT
    _set_robot_busy(pinky_id, task.task_id)
    ROBOTS[pinky_id]["target_zone"] = load_wait

    task.record(f"{pinky_id} → {load_wait}(주차대기구역) 이동 명령")
    _send_command(pinky_id, "navigate", {"location": load_wait}, task.task_id)
    return True


def _try_depart_to_storage(task: Task) -> bool:
    """load_wait 출발 시도. 다른 핑키가 load_wait로 오는 중이면 대기."""
    if task.state not in (TaskState.MOCK_LOADING, TaskState.PINKY_WAIT_INCOMING):
        return False
    if _is_any_pinky_incoming_to_load_wait(task.pinky_id):
        return False

    _zone_manager.release(task.load_wait_zone)
    ROBOTS[task.pinky_id]["current_zone"] = None
    ROBOTS[task.pinky_id]["target_zone"]  = task.storage_zone

    task.state = TaskState.PINKY_TO_STORAGE
    task.record(f"{task.pinky_id} → {task.storage_zone}(보관구역) 이동 명령")
    _send_command(task.pinky_id, "navigate", {"location": task.storage_zone}, task.task_id)

    # load_wait 해제 → 대기 중인 새 태스크 배차 재시도
    _pending_queue.drain(_try_dispatch)
    return True


def _mock_load_complete(task_id: str):
    """타이머 콜백: 제코봇 적재 완료 시뮬레이션."""
    with _lock:
        task = tasks.get(task_id)
        if task is None or task.state != TaskState.MOCK_LOADING:
            return
        task.record(f"[시뮬] 적재 완료 (mock {MOCK_LOAD_DELAY_SEC}초)")
        if not _try_depart_to_storage(task):
            task.state = TaskState.PINKY_WAIT_INCOMING
            task.record(f"{task.pinky_id}: 다른 핑키 load_wait 도착 대기 중")
            _wait_queue.submit(task, _try_depart_to_storage)


# ── 상태 전이 ─────────────────────────────────────────────────
def _advance(task: Task, robot_id: str, event: str):
    s = task.state

    # Step 1: 핑키 load_wait(주차대기구역) 도착 → 적재 시뮬 타이머 시작
    if s == TaskState.PINKY_TO_LOAD_WAIT and robot_id == task.pinky_id and event == "arrived":
        ROBOTS[task.pinky_id]["current_zone"] = task.load_wait_zone
        ROBOTS[task.pinky_id]["target_zone"]  = None
        task.state = TaskState.MOCK_LOADING
        task.record(f"load_wait 도착 — 적재 시뮬 {MOCK_LOAD_DELAY_SEC}초 시작")
        # 이 핑키 도착으로 다른 태스크의 PINKY_WAIT_INCOMING 해제 가능
        _wait_queue.drain(_try_depart_to_storage)
        threading.Timer(MOCK_LOAD_DELAY_SEC, _mock_load_complete, args=[task.task_id]).start()

    # Step 2: 핑키 보관구역 도착 → 즉시 홈 복귀 (제코봇2 하역 생략)
    elif s == TaskState.PINKY_TO_STORAGE and robot_id == task.pinky_id and event == "arrived":
        ROBOTS[task.pinky_id]["current_zone"] = task.storage_zone
        ROBOTS[task.pinky_id]["target_zone"]  = "home"
        _zone_manager.mark_occupied(task.storage_zone)
        task.state = TaskState.PINKY_TO_HOME
        task.record(f"[시뮬] {task.storage_zone} 도착 — 홈 복귀 명령")
        _send_command(task.pinky_id, "navigate", {"location": "home"}, task.task_id)

    # Step 3: 핑키 홈 도착 → 완료, 대기 태스크 재시도
    elif s == TaskState.PINKY_TO_HOME and robot_id == task.pinky_id and event == "arrived":
        pinky_id = task.pinky_id
        ROBOTS[pinky_id]["current_zone"] = "home"
        ROBOTS[pinky_id]["target_zone"]  = None
        _set_robot_idle(pinky_id)
        _dispatcher.on_home_arrived(ROBOTS, pinky_id)
        task.state = TaskState.DONE
        task.record(f"태스크 완료 — {task.storage_zone} 점유, {pinky_id} 홈 복귀")
        _pending_queue.drain(_try_dispatch)

    elif event == "error":
        _fail_task(task, f"{robot_id} 오류 발생")
    else:
        task.record(f"[무시] {robot_id} / {event} (현재 상태: {s})")


# ════════════════════════════════════════════════════════════
# HTTP API
# ════════════════════════════════════════════════════════════

app = FastAPI(title="Task Manager (Pinky Only Test)")


class TaskRequest(BaseModel):
    storage_zone: str

class NavigateRequest(BaseModel):
    robot_id: str
    location: str


@app.post("/task")
def create_task(req: TaskRequest):
    if req.storage_zone not in VALID_STORAGE_ZONES:
        raise HTTPException(400, f"유효하지 않은 구역: {req.storage_zone}. 사용 가능: zone_1~zone_16")
    if ZONES.get(req.storage_zone) == "occupied":
        raise HTTPException(409, f"{req.storage_zone} 이미 점유 중")

    task_id = str(uuid.uuid4())
    task = Task(task_id, req.storage_zone)
    with _lock:
        tasks[task_id] = task
        dispatched = _pending_queue.submit(task, _try_dispatch)

    if not dispatched:
        task.state = TaskState.QUEUED
        task.record(f"태스크 대기열 추가 (목표: {req.storage_zone})")

    return {"task_id": task_id, "state": task.state, "queued": not dispatched}


@app.get("/task/{task_id}")
def get_task(task_id: str):
    task = tasks.get(task_id)
    if task is None:
        raise HTTPException(404)
    return task.to_dict()


@app.get("/tasks")
def list_tasks():
    return [t.to_dict() for t in tasks.values()]


@app.post("/navigate")
def navigate(req: NavigateRequest):
    """특정 로봇에게 직접 navigate 액션 전송 (태스크 플로우 외 단순 이동용)."""
    if req.robot_id not in ROBOTS:
        raise HTTPException(400, f"알 수 없는 로봇: {req.robot_id}. 사용 가능: {list(ROBOTS.keys())}")
    task_id = f"direct_{uuid.uuid4()}"
    _send_command(req.robot_id, "navigate", {"location": req.location}, task_id)
    return {"ok": True, "robot_id": req.robot_id, "location": req.location}


@app.get("/robots")
def get_robots():
    return ROBOTS


@app.get("/zones")
def get_zones():
    return ZONES


@app.post("/zones/reset/{zone}")
def reset_zone(zone: str):
    """occupied 존을 free로 초기화 (테스트용)."""
    if zone not in ZONES:
        raise HTTPException(404, f"존 없음: {zone}")
    _zone_manager.release(zone)
    return {"zone": zone, "state": ZONES[zone]}


@app.get("/debug")
def debug():
    return {
        "robots": ROBOTS,
        "zones":  {k: v for k, v in ZONES.items() if v != "free"},
        "tasks": [
            {
                "task_id":   t.task_id[:8],
                "state":     t.state,
                "pinky":     t.pinky_id,
                "load_wait": t.load_wait_zone,
                "storage":   t.storage_zone,
                "log":       t.log[-5:],
            }
            for t in tasks.values()
        ],
        "pending_count": _pending_queue.pending_count(),
        "wait_count":    _wait_queue.pending_count(),
    }


# ════════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════════

def ros_spin():
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(ros_node)
        print("[ROS2] executor 시작")
        executor.spin()
    except Exception as e:
        import traceback
        print(f"[ERROR] ros_spin 크래시: {e}")
        traceback.print_exc()


def main():
    global ros_node
    rclpy.init()
    ros_node = TaskManagerNode()
    threading.Thread(target=ros_spin, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8091)
