"""
Task Manager - ROS2 Action 기반 물류창고 로봇 오케스트레이터
포트: 8090 (태스크 생성 HTTP — LLM 웹서버용)

send_goal_async()는 반드시 executor 스레드 안에서 호출해야 응답 콜백이 동작함.
타이머 콜백(50ms) 안에서 큐를 꺼내 goal을 전송하는 방식으로 이를 보장함.

멀티 핑키 지원:
- ZoneManager: zone 점유/잠금
- Dispatcher:  최근 귀환 idle 핑키 우선 선택
- TaskQueue:   자원 부족 시 대기열
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

# ── 로봇 설정 ──────────────────────────────────────────────────
ROBOTS: dict[str, dict] = {
    "jetcobot1": {"type": "jetcobot", "status": "idle", "task_id": None},
    "jetcobot2": {"type": "jetcobot", "status": "idle", "task_id": None},
    "pinky1": {
        "type": "pinky", "status": "idle", "task_id": None,
        "current_zone": None, "target_zone": None, "last_home_arrival_ts": 0.0,
    },
    "pinky2": {
        "type": "pinky", "status": "idle", "task_id": None,
        "current_zone": None, "target_zone": None, "last_home_arrival_ts": 0.0,
    },
}

# zone 상태 dict — ZoneManager에서 직접 참조
ZONES: dict[str, str] = {"loading_zone": "free"}
ZONES.update({f"zone_{i}": "free" for i in range(1, 17)})

# ── 태스크 상태 머신 ───────────────────────────────────────────
class TaskState(str, Enum):
    PENDING             = "pending"
    QUEUED              = "queued"
    JETCOBOT1_PICKUP    = "jetcobot1_pickup"
    PINKY_TO_LOADING    = "pinky_to_loading"
    JETCOBOT1_LOADING   = "jetcobot1_loading"
    PINKY_TO_STORAGE    = "pinky_to_storage"
    JETCOBOT2_UNLOADING = "jetcobot2_unloading"
    PINKY_TO_HOME       = "pinky_to_home"
    DONE                = "done"
    FAILED              = "failed"

class Task:
    def __init__(self, task_id: str, storage_zone: str):
        self.task_id      = task_id
        self.storage_zone = storage_zone
        self.state        = TaskState.PENDING
        self.jetcobot1_id = "jetcobot1"
        self.jetcobot2_id = "jetcobot2"
        self.pinky_id: str | None = None
        self.created_at   = datetime.now().isoformat()
        self.updated_at   = datetime.now().isoformat()
        self.log: list[str] = []

    def record(self, msg: str):
        self.log.append(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")
        self.updated_at = datetime.now().isoformat()
        print(f"[Task {self.task_id[:8]}] {msg}")

    def to_dict(self) -> dict:
        return {
            "task_id":      self.task_id,
            "state":        self.state,
            "storage_zone": self.storage_zone,
            "pinky":        self.pinky_id,
            "created_at":   self.created_at,
            "updated_at":   self.updated_at,
            "log":          self.log,
        }

# ── 전역 저장소 ────────────────────────────────────────────────
tasks: dict[str, Task] = {}
_lock = threading.Lock()
ros_node: "TaskManagerNode" = None
_cmd_queue: queue.Queue = queue.Queue()

# 멀티 핑키 매니저
_zone_manager = ZoneManager(ZONES)
_dispatcher   = Dispatcher()
_task_queue   = TaskQueue()

# ── ROS2 액션 클라이언트 노드 ──────────────────────────────────
class TaskManagerNode(Node):
    def __init__(self):
        super().__init__("task_manager")

        self._cb_group = ReentrantCallbackGroup()
        self._action_clients = {
            robot_id: ActionClient(self, RobotCommand, f"/{robot_id}/command",
                                   callback_group=self._cb_group)
            for robot_id in ROBOTS
        }
        # executor 스레드 안에서 50ms마다 큐 확인 후 goal 전송
        self.create_timer(0.05, self._flush_cmd_queue, callback_group=self._cb_group)
        self.get_logger().info("Task Manager Action Node 시작")

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
            if self._retry[robot_id] % 40 == 1:  # 50ms × 40 = 2초마다
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
                _fail_task(task, f"{robot_id} 목표 거부됨")
            return

        print(f"[GOAL_RESPONSE] {robot_id} 목표 수락됨 — result 대기 중")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._on_result(f, robot_id, task_id)
        )

    def _on_result(self, future, robot_id: str, task_id: str):
        result  = future.result().result
        event   = result.event
        message = result.message
        print(f"[RESULT] robot={robot_id}  event={event!r}  message={message!r}  task={task_id[:8]}")
        with _lock:
            task = tasks.get(task_id)
            if task is None:
                return
            task.record(f"{robot_id} → {event}")
            _advance(task, robot_id, event)

# ── 헬퍼 ──────────────────────────────────────────────────────
def _set_robot_busy(robot_id: str, task_id: str):
    ROBOTS[robot_id]["status"]  = "busy"
    ROBOTS[robot_id]["task_id"] = task_id

def _set_robot_idle(robot_id: str):
    ROBOTS[robot_id]["status"]  = "idle"
    ROBOTS[robot_id]["task_id"] = None

def _fail_task(task: Task, reason: str):
    task.state = TaskState.FAILED
    task.record(f"태스크 실패 — {reason}")
    for robot_id in [task.jetcobot1_id, task.jetcobot2_id]:
        if ROBOTS[robot_id]["task_id"] == task.task_id:
            _set_robot_idle(robot_id)
    if task.pinky_id and ROBOTS[task.pinky_id]["task_id"] == task.task_id:
        # 실패 시 점유 중이던 zone 모두 해제
        pinky = ROBOTS[task.pinky_id]
        for z in (pinky.get("current_zone"), pinky.get("target_zone")):
            if z:
                _zone_manager.release(z)
        pinky["current_zone"] = None
        pinky["target_zone"]  = None
        _set_robot_idle(task.pinky_id)
    # 실패 후 대기 태스크 재시도
    _task_queue.drain(_try_dispatch)

def _send_command(robot_id: str, action: str, parameters: dict, task_id: str):
    _cmd_queue.put({"robot_id": robot_id, "action": action,
                    "parameters": parameters, "task_id": task_id})

# ── 태스크 디스패치 ────────────────────────────────────────────
def _try_dispatch(task: Task) -> bool:
    """
    태스크 즉시 실행 시도. jetcobot1 + storage_zone 잠금이 모두 성공해야 시작.
    실패 시 False 반환 → TaskQueue에 보관됨.
    pinky 배차는 jetcobot1 PICKUP 완료 후 _advance에서 다시 시도.
    """
    # jetcobot1 사용 가능?
    if ROBOTS["jetcobot1"]["status"] != "idle":
        return False
    # storage_zone 사용 가능? (이미 점유면 거절)
    if not _zone_manager.is_free(task.storage_zone):
        return False

    # 시작 시점에 storage_zone 미리 잠금 (다른 태스크가 가로채지 못하게)
    # 잠금은 잠정적으로 jetcobot1 명의로 예약 — 실제 핑키 배정 후 holder 교체됨
    if not _zone_manager.try_acquire(task.storage_zone, "jetcobot1"):
        return False

    task.state = TaskState.JETCOBOT1_PICKUP
    _set_robot_busy("jetcobot1", task.task_id)
    task.record(f"태스크 시작 → jetcobot1 물건 집기 (목표: {task.storage_zone})")
    _send_command("jetcobot1", "pickup", {}, task.task_id)
    return True

def _dispatch_pinky_to_loading(task: Task) -> bool:
    """jetcobot1 pickup 완료 후 핑키 배차 + loading_zone 잠금."""
    pinky_id = _dispatcher.pick_pinky(ROBOTS)
    if pinky_id is None:
        task.record("유휴 pinky 없음 — 대기 중")
        return False
    if not _zone_manager.try_acquire("loading_zone", pinky_id):
        task.record("loading_zone 점유 중 — 대기")
        return False

    task.pinky_id = pinky_id
    task.state    = TaskState.PINKY_TO_LOADING
    _set_robot_busy(pinky_id, task.task_id)
    ROBOTS[pinky_id]["target_zone"]  = "loading_zone"
    task.record(f"{pinky_id} → 입고구역 이동 명령")
    _send_command(pinky_id, "navigate", {"location": "loading_zone"}, task.task_id)
    return True

# ── 상태 전이 엔진 ────────────────────────────────────────────
def _advance(task: Task, robot_id: str, event: str):
    s = task.state

    if s == TaskState.JETCOBOT1_PICKUP and robot_id == task.jetcobot1_id and event == "done":
        _set_robot_idle(task.jetcobot1_id)
        # 핑키 배차 시도. 실패하면 대기열로 — 이후 핑키 귀환 시 drain에서 재시도
        if not _dispatch_pinky_to_loading(task):
            task.state = TaskState.QUEUED
            _task_queue.submit(task, _dispatch_pinky_to_loading)

    elif s == TaskState.PINKY_TO_LOADING and robot_id == task.pinky_id and event == "arrived":
        # loading_zone 도착 → current_zone 갱신, target 해제
        pinky = ROBOTS[task.pinky_id]
        pinky["current_zone"] = "loading_zone"
        pinky["target_zone"]  = None
        # loading_zone은 jetcobot1 작업 중에도 핑키가 점유 중이므로 유지
        task.state = TaskState.JETCOBOT1_LOADING
        _set_robot_busy(task.jetcobot1_id, task.task_id)
        task.record(f"jetcobot1 → {task.pinky_id}에 물건 적재 명령")
        _send_command(task.jetcobot1_id, "load", {"target_pinky": task.pinky_id}, task.task_id)

    elif s == TaskState.JETCOBOT1_LOADING and robot_id == task.jetcobot1_id and event == "done":
        _set_robot_idle(task.jetcobot1_id)
        # 핑키가 storage_zone으로 이동 시작 — loading_zone 해제
        _zone_manager.release("loading_zone")
        ROBOTS[task.pinky_id]["current_zone"] = None
        # storage_zone holder를 jetcobot1 → 실제 핑키로 갱신 (재acquire)
        # 이미 try_dispatch에서 잠금되어 있으므로 holder 변경 위해 강제 재설정
        _zone_manager.try_acquire(task.storage_zone, task.pinky_id)
        ROBOTS[task.pinky_id]["target_zone"] = task.storage_zone
        task.state = TaskState.PINKY_TO_STORAGE
        task.record(f"{task.pinky_id} → {task.storage_zone} 이동 명령")
        _send_command(task.pinky_id, "navigate", {"location": task.storage_zone}, task.task_id)
        # loading_zone 해제 → 대기 태스크 재시도 (다른 핑키가 loading 가능)
        _task_queue.drain(_dispatch_pinky_to_loading)

    elif s == TaskState.PINKY_TO_STORAGE and robot_id == task.pinky_id and event == "arrived":
        pinky = ROBOTS[task.pinky_id]
        pinky["current_zone"] = task.storage_zone
        pinky["target_zone"]  = None
        # storage_zone은 jetcobot2 작업 동안 잠금 유지
        task.state = TaskState.JETCOBOT2_UNLOADING
        _set_robot_busy(task.jetcobot2_id, task.task_id)
        task.record(f"jetcobot2 → {task.storage_zone} 하역 명령")
        _send_command(task.jetcobot2_id, "unload",
                      {"zone": task.storage_zone, "source_pinky": task.pinky_id}, task.task_id)

    elif s == TaskState.JETCOBOT2_UNLOADING and robot_id == task.jetcobot2_id and event == "done":
        _set_robot_idle(task.jetcobot2_id)
        # storage_zone → occupied 승격 (물건 적재 완료)
        _zone_manager.mark_occupied(task.storage_zone)
        ROBOTS[task.pinky_id]["current_zone"] = None
        ROBOTS[task.pinky_id]["target_zone"]  = "home"
        task.state = TaskState.PINKY_TO_HOME
        task.record(f"{task.pinky_id} → 홈으로 복귀 명령")
        _send_command(task.pinky_id, "navigate", {"location": "home"}, task.task_id)

    elif s == TaskState.PINKY_TO_HOME and robot_id == task.pinky_id and event == "arrived":
        pinky_id = task.pinky_id
        pinky = ROBOTS[pinky_id]
        pinky["current_zone"] = "home"
        pinky["target_zone"]  = None
        _set_robot_idle(pinky_id)
        # 귀환 시각 갱신 → 다음 dispatch 우선순위
        _dispatcher.on_home_arrived(ROBOTS, pinky_id)
        task.state = TaskState.DONE
        task.record(f"태스크 완료 — {task.storage_zone} 점유, {pinky_id} 홈 복귀")
        # 대기 태스크 재시도 — pinky 배차 가능해짐
        _task_queue.drain(_dispatch_pinky_to_loading)
        # 신규 태스크(JETCOBOT1_PICKUP 단계) 대기도 함께 시도
        _task_queue.drain(_try_dispatch)

    elif event == "error":
        _fail_task(task, f"{robot_id} 오류 발생")

    else:
        task.record(f"[무시] {robot_id} / {event} (현재 상태: {s})")

# ── FastAPI ────────────────────────────────────────────────────
app = FastAPI(title="Task Manager")

class TaskRequest(BaseModel):
    storage_zone: str

@app.post("/task")
def create_task(req: TaskRequest):
    if req.storage_zone not in ZONES:
        raise HTTPException(400, f"유효하지 않은 구역: {req.storage_zone}")
    if ZONES[req.storage_zone] == "occupied":
        raise HTTPException(409, f"{req.storage_zone} 이미 점유 중")

    task_id = str(uuid.uuid4())
    task = Task(task_id, req.storage_zone)
    with _lock:
        tasks[task_id] = task
        # 즉시 dispatch 시도, 실패하면 대기열에 추가
        dispatched = _task_queue.submit(task, _try_dispatch)

    if not dispatched:
        task.state = TaskState.QUEUED
        task.record(f"태스크 대기열 추가 (목표: {req.storage_zone})")

    return {
        "task_id": task_id,
        "state":   task.state,
        "queued":  not dispatched,
    }

@app.get("/task/{task_id}")
def get_task(task_id: str):
    task = tasks.get(task_id)
    if task is None:
        raise HTTPException(404)
    return task.to_dict()

@app.get("/tasks")
def list_tasks():
    return [t.to_dict() for t in tasks.values()]

@app.get("/robots")
def get_robots():
    return ROBOTS

@app.get("/zones")
def get_zones():
    return ZONES

@app.get("/debug")
def debug():
    return {
        "robots": ROBOTS,
        "zones":  {k: v for k, v in ZONES.items() if v != "free"},
        "tasks":  [
            {"task_id": t.task_id[:8], "state": t.state, "pinky": t.pinky_id, "log": t.log[-5:]}
            for t in tasks.values()
        ],
        "pending_tasks": _task_queue.pending_count(),
    }

# ── 진입점 ────────────────────────────────────────────────────
def ros_spin():
    try:
        executor = MultiThreadedExecutor()
        executor.add_node(ros_node)
        print("[ROS2] executor 시작")
        executor.spin()
        print("[ROS2] executor 종료됨")
    except Exception as e:
        import traceback
        print(f"[ERROR] ros_spin 크래시: {e}")
        traceback.print_exc()

def main():
    global ros_node
    rclpy.init()
    ros_node = TaskManagerNode()

    threading.Thread(target=ros_spin, daemon=True).start()

    uvicorn.run(app, host="0.0.0.0", port=8090)
