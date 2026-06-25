"""
Task Manager - ROS2 Action 기반 물류창고 로봇 오케스트레이터
포트: 8090

ROUTING_MODE:
  "zone"     : load_wait_1/2 + zone_1~16 직접 이동 (기존 방식 확장)
  "waypoint" : load_wait_1/2 + rack_1~3  웨이포인트 경유 이동

공통 규칙 (두 모드 모두):
  · load_wait_1, load_wait_2 동시 사용 가능 (2대 병렬 적재)
  · 적재 완료 후, 다른 핑키가 load_wait로 오는 중이면 출발 대기
  · rack/zone 잠금은 zone별 독립 (다른 zone이면 즉시 출발)
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

# ── 모드 설정 ──────────────────────────────────────────────────
# "zone"     : storage_zone = zone_1~16, 직접 이동
# "waypoint" : storage_zone = rack_1~3,  settings.py WAYPOINTS 경유
ROUTING_MODE = "zone"

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

# ── 존 상태 ────────────────────────────────────────────────────
ZONES: dict[str, str] = {
    "load_wait_1": "free",
    "load_wait_2": "free",
    "rack_1":      "free",
    "rack_2":      "free",
    "rack_3":      "free",
}
ZONES.update({f"zone_{i}": "free" for i in range(1, 17)})

LOAD_WAIT_ZONES = {"load_wait_1", "load_wait_2"}


# ════════════════════════════════════════════════════════════
# DOMAIN MODEL
# ════════════════════════════════════════════════════════════

# ── 태스크 상태 머신 ───────────────────────────────────────────
class TaskState(str, Enum):
    PENDING              = "pending"
    QUEUED               = "queued"
    JETCOBOT1_PICKUP     = "jetcobot1_pickup"
    PINKY_TO_LOAD_WAIT   = "pinky_to_load_wait"
    JETCOBOT1_LOADING    = "jetcobot1_loading"
    PINKY_WAIT_INCOMING  = "pinky_wait_incoming"  # 다른 핑키 load_wait 도착 대기
    PINKY_TO_STORAGE     = "pinky_to_storage"
    JETCOBOT2_UNLOADING  = "jetcobot2_unloading"
    PINKY_TO_HOME        = "pinky_to_home"
    DONE                 = "done"
    FAILED               = "failed"


class OutboundState(str, Enum):
    PENDING            = "out_pending"
    QUEUED             = "out_queued"
    PINKY_TO_ZONE      = "out_pinky_to_zone"       # 핑키 → zone (yaw=0 정렬 포함)
    JETCOBOT2_LOADING  = "out_jetcobot2_loading"   # jetcobot2 zone→핑키 상차
    PINKY_TO_OUTBOUND  = "out_pinky_to_outbound"   # 핑키 → outbound_zone
    PINKY_TO_HOME      = "out_pinky_to_home"       # 핑키 → home
    DONE               = "out_done"
    FAILED             = "out_failed"


class Task:
    def __init__(self, task_id: str, storage_zone: str):
        self.task_id        = task_id
        self.storage_zone   = storage_zone
        self.load_wait_zone: str | None = None
        self.state          = TaskState.PENDING
        self.jetcobot1_id   = "jetcobot1"
        self.jetcobot2_id   = "jetcobot2"
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
            "routing_mode":   ROUTING_MODE,
            "created_at":     self.created_at,
            "updated_at":     self.updated_at,
            "log":            self.log,
        }


class OutboundTask:
    def __init__(self, task_id: str, storage_zone: str):
        self.task_id      = task_id
        self.storage_zone = storage_zone
        self.state        = OutboundState.PENDING
        self.pinky_id: str | None = None
        self.jetcobot2_id = "jetcobot2"
        self.created_at   = datetime.now().isoformat()
        self.updated_at   = datetime.now().isoformat()
        self.log: list[str] = []

    def record(self, msg: str):
        self.log.append(f"[{datetime.now().strftime('%H:%M:%S')}] {msg}")
        self.updated_at = datetime.now().isoformat()
        print(f"[Outbound {self.task_id[:8]}] {msg}")

    def to_dict(self) -> dict:
        return {
            "task_id":      self.task_id,
            "task_type":    "outbound",
            "state":        self.state,
            "storage_zone": self.storage_zone,
            "pinky":        self.pinky_id,
            "created_at":   self.created_at,
            "updated_at":   self.updated_at,
            "log":          self.log,
        }


# ════════════════════════════════════════════════════════════
# GLOBAL STATE
# ════════════════════════════════════════════════════════════

# ── 전역 저장소 ────────────────────────────────────────────────
tasks: dict[str, Task] = {}
_direct_commands: dict[str, dict] = {}  # direct_ 명령 추적용
_home_stack: list[str] = []             # 홈 도착 순서 스택 [inner, ..., outer]
_lock = threading.Lock()
ros_node: "TaskManagerNode" = None
_cmd_queue: queue.Queue = queue.Queue()

_zone_manager = ZoneManager(ZONES)
_dispatcher   = Dispatcher()

# 큐 분리: 역할별로 독립 관리하여 잘못된 drain 방지
_dispatch_queue  = TaskQueue()   # jetcobot1 + storage_zone 대기
_pinky_queue     = TaskQueue()   # pinky + load_wait 대기
_wait_queue      = TaskQueue()   # PINKY_WAIT_INCOMING 대기
_outbound_queue  = TaskQueue()   # 출고 태스크 대기


# ════════════════════════════════════════════════════════════
# ROS2 NODE
# ════════════════════════════════════════════════════════════

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
        self.create_timer(0.05, self._flush_cmd_queue, callback_group=self._cb_group)
        self.get_logger().info("Task Manager 시작")

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
        print(f"[RESULT] robot={robot_id}  event={event!r}  message={message!r}  task={task_id[:8]}")
        with _lock:
            if task_id.startswith("direct_"):
                direct = _direct_commands.pop(task_id, None)
                if direct and event == "arrived":
                    ROBOTS[robot_id]["target_zone"] = None
                    ROBOTS[robot_id]["current_zone"] = direct["location"]
                    if direct["location"] == "home" and ROBOTS[robot_id].get("type") == "pinky":
                        _home_push(robot_id)
                return
            task = tasks.get(task_id)
            if task is None:
                return
            task.record(f"{robot_id} → {event}")
            if isinstance(task, OutboundTask):
                _advance_outbound(task, robot_id, event)
            else:
                _advance(task, robot_id, event)


# ════════════════════════════════════════════════════════════
# ORCHESTRATION
# ════════════════════════════════════════════════════════════

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
        pinky = ROBOTS[task.pinky_id]
        zones_to_release = {
            z for z in (
                pinky.get("current_zone"),
                pinky.get("target_zone"),
                task.load_wait_zone,
            ) if z
        }
        for z in zones_to_release:
            _zone_manager.release(z)
        pinky["current_zone"] = None
        pinky["target_zone"]  = None
        _set_robot_idle(task.pinky_id)
    # storage_zone 잠금 해제 (dispatch 시 예약한 것)
    _zone_manager.release(task.storage_zone)
    _dispatch_queue.drain(_try_dispatch)

def _send_command(robot_id: str, action: str, parameters: dict, task_id: str):
    _cmd_queue.put({"robot_id": robot_id, "action": action,
                    "parameters": parameters, "task_id": task_id})

def _home_push(pinky_id: str):
    """핑키 홈 도착 → 스택에 추가 (top = 바깥쪽)."""
    if pinky_id not in _home_stack:
        _home_stack.append(pinky_id)

def _home_pop(pinky_id: str):
    """핑키 홈 출발 → 스택에서 제거."""
    if pinky_id in _home_stack:
        _home_stack.remove(pinky_id)

def _home_position(pinky_id: str) -> str | None:
    """스택 위치로 안쪽/바깥쪽 반환."""
    if pinky_id not in _home_stack:
        return None
    return "outer" if _home_stack[-1] == pinky_id else "inner"

def _is_any_pinky_incoming_to_load_wait(exclude_robot_id: str) -> bool:
    """다른 핑키가 load_wait 존으로 이동 중인지 확인."""
    for robot_id, robot in ROBOTS.items():
        if robot_id == exclude_robot_id:
            continue
        if robot.get("type") == "pinky" and robot.get("target_zone") in LOAD_WAIT_ZONES:
            return True
    return False


# ── 디스패치 ──────────────────────────────────────────────────
def _try_dispatch(task: Task) -> bool:
    """
    태스크 즉시 실행 시도.
    jetcobot1 유휴 + storage_zone 잠금 성공 시에만 시작.
    """
    if task.state not in (TaskState.PENDING, TaskState.QUEUED):
        return False
    if ROBOTS["jetcobot1"]["status"] != "idle":
        return False
    if not _zone_manager.is_free(task.storage_zone):
        return False
    if not _zone_manager.try_acquire(task.storage_zone, "jetcobot1"):
        return False

    task.state = TaskState.JETCOBOT1_PICKUP
    _set_robot_busy("jetcobot1", task.task_id)
    task.record(f"태스크 시작 → jetcobot1 물건 집기 (목표: {task.storage_zone})")
    _send_command("jetcobot1", "pickup", {}, task.task_id)
    return True


def _dispatch_pinky_to_load_wait(task: Task) -> bool:
    """
    jetcobot1 pickup 완료 후 핑키 배차.
    load_wait_1, load_wait_2 중 빈 곳에 배차 (2대 동시 적재 지원).
    """
    # jetcobot1 pickup이 완료된 태스크만 처리
    if task.state not in (TaskState.JETCOBOT1_PICKUP, TaskState.QUEUED):
        return False
    if task.pinky_id is not None:  # 이미 핑키 배정됨
        return False

    pinky_id = _dispatcher.pick_pinky(ROBOTS, _home_stack)
    if pinky_id is None:
        return False

    load_wait = None
    for zone in ("load_wait_1", "load_wait_2"):
        if _zone_manager.try_acquire(zone, pinky_id):
            load_wait = zone
            break
    if load_wait is None:
        return False

    task.pinky_id       = pinky_id
    task.load_wait_zone = load_wait
    task.state          = TaskState.PINKY_TO_LOAD_WAIT
    _set_robot_busy(pinky_id, task.task_id)
    _home_pop(pinky_id)
    ROBOTS[pinky_id]["target_zone"] = load_wait

    task.record(f"{pinky_id} → {load_wait} 이동 명령")
    _send_command(pinky_id, "navigate", {"location": load_wait}, task.task_id)
    return True


def _try_depart_to_storage(task: Task) -> bool:
    """
    적재 완료 후 storage/rack으로 출발 시도.
    다른 핑키가 load_wait로 오는 중이면 그 핑키가 도착할 때까지 대기.
    """
    if task.state not in (TaskState.JETCOBOT1_LOADING, TaskState.PINKY_WAIT_INCOMING):
        return False

    # 다른 핑키가 load_wait로 이동 중이면 대기
    if _is_any_pinky_incoming_to_load_wait(task.pinky_id):
        return False

    # load_wait 해제
    _zone_manager.release(task.load_wait_zone)
    ROBOTS[task.pinky_id]["current_zone"] = None
    ROBOTS[task.pinky_id]["target_zone"]  = task.storage_zone

    task.state = TaskState.PINKY_TO_STORAGE
    task.record(f"{task.pinky_id} → {task.storage_zone} 이동 명령")
    _send_command(task.pinky_id, "navigate",
                  {"location": task.storage_zone}, task.task_id)

    # load_wait 해제 → 대기 중인 핑키 배차 재시도
    _pinky_queue.drain(_dispatch_pinky_to_load_wait)
    return True


# ── 출고 디스패치 ─────────────────────────────────────────────
def _dispatch_outbound(task: OutboundTask) -> bool:
    """유휴 핑키 + occupied zone 확인 후 출고 시작."""
    if task.state not in (OutboundState.PENDING, OutboundState.QUEUED):
        return False
    if ZONES.get(task.storage_zone) != "occupied":
        return False
    pinky_id = _dispatcher.pick_pinky(ROBOTS, _home_stack)
    if pinky_id is None:
        return False

    task.pinky_id = pinky_id
    task.state    = OutboundState.PINKY_TO_ZONE
    _set_robot_busy(pinky_id, task.task_id)
    _home_pop(pinky_id)
    ROBOTS[pinky_id]["target_zone"] = task.storage_zone

    task.record(f"{pinky_id} → {task.storage_zone} 출고 이동 (도착 후 yaw=0 정렬)")
    _send_command(pinky_id, "navigate",
                  {"location": task.storage_zone, "align_yaw": 0.0},
                  task.task_id)
    return True


# ── 출고 상태 전이 ─────────────────────────────────────────────
def _advance_outbound(task: OutboundTask, robot_id: str, event: str):
    s = task.state

    # Step 1: 핑키 zone 도착 (yaw=0 정렬 완료) → jetcobot2 상차 명령
    if s == OutboundState.PINKY_TO_ZONE and robot_id == task.pinky_id and event == "arrived":
        ROBOTS[task.pinky_id]["current_zone"] = task.storage_zone
        ROBOTS[task.pinky_id]["target_zone"]  = None
        task.state = OutboundState.JETCOBOT2_LOADING
        _set_robot_busy(task.jetcobot2_id, task.task_id)
        task.record(f"jetcobot2 → {task.storage_zone} 상차 명령")
        _send_command(task.jetcobot2_id, "load",
                      {"zone": task.storage_zone, "target_pinky": task.pinky_id},
                      task.task_id)

    # Step 2: jetcobot2 상차 완료 → 핑키 outbound_zone으로
    elif s == OutboundState.JETCOBOT2_LOADING and robot_id == task.jetcobot2_id and event == "done":
        _set_robot_idle(task.jetcobot2_id)
        _zone_manager.release(task.storage_zone)
        ROBOTS[task.pinky_id]["current_zone"] = None
        ROBOTS[task.pinky_id]["target_zone"]  = "outbound_zone"
        task.state = OutboundState.PINKY_TO_OUTBOUND
        task.record(f"{task.pinky_id} → outbound_zone 이동 명령")
        _send_command(task.pinky_id, "navigate",
                      {"location": "outbound_zone"}, task.task_id)

    # Step 3: 핑키 outbound_zone 도착 → 홈 복귀
    elif s == OutboundState.PINKY_TO_OUTBOUND and robot_id == task.pinky_id and event == "arrived":
        ROBOTS[task.pinky_id]["current_zone"] = None
        ROBOTS[task.pinky_id]["target_zone"]  = "home"
        task.state = OutboundState.PINKY_TO_HOME
        task.record(f"{task.pinky_id} → 홈 복귀 명령")
        _send_command(task.pinky_id, "navigate",
                      {"location": "home"}, task.task_id)

    # Step 4: 핑키 홈 도착 → 완료
    elif s == OutboundState.PINKY_TO_HOME and robot_id == task.pinky_id and event == "arrived":
        ROBOTS[task.pinky_id]["current_zone"] = "home"
        ROBOTS[task.pinky_id]["target_zone"]  = None
        _set_robot_idle(task.pinky_id)
        _home_push(task.pinky_id)
        task.state = OutboundState.DONE
        task.record("출고 태스크 완료")
        # 대기 중인 입고/출고 태스크 재시도
        _pinky_queue.drain(_dispatch_pinky_to_load_wait)
        _dispatch_queue.drain(_try_dispatch)
        _outbound_queue.drain(_dispatch_outbound)

    elif event == "error":
        task.state = OutboundState.FAILED
        task.record(f"출고 실패 — {robot_id} 오류")
        if task.pinky_id and ROBOTS[task.pinky_id]["task_id"] == task.task_id:
            ROBOTS[task.pinky_id]["current_zone"] = None
            ROBOTS[task.pinky_id]["target_zone"]  = None
            _set_robot_idle(task.pinky_id)
        if ROBOTS[task.jetcobot2_id]["task_id"] == task.task_id:
            _set_robot_idle(task.jetcobot2_id)
        _outbound_queue.drain(_dispatch_outbound)

    else:
        task.record(f"[무시] {robot_id} / {event} (현재 상태: {s})")


# ── 상태 전이 ─────────────────────────────────────────────────
def _advance(task: Task, robot_id: str, event: str):
    s = task.state

    # ── Step 1: jetcobot1 물건 집기 완료 → 핑키 배차
    if s == TaskState.JETCOBOT1_PICKUP and robot_id == task.jetcobot1_id and event == "done":
        _set_robot_idle(task.jetcobot1_id)
        if not _dispatch_pinky_to_load_wait(task):
            task.state = TaskState.QUEUED
            task.record("유휴 pinky / load_wait 없음 — 배차 대기")
            _pinky_queue.submit(task, _dispatch_pinky_to_load_wait)

    # ── Step 2: 핑키 load_wait 도착 → jetcobot1 적재 명령
    elif s == TaskState.PINKY_TO_LOAD_WAIT and robot_id == task.pinky_id and event == "arrived":
        pinky = ROBOTS[task.pinky_id]
        pinky["current_zone"] = task.load_wait_zone
        pinky["target_zone"]  = None
        task.state = TaskState.JETCOBOT1_LOADING
        _set_robot_busy(task.jetcobot1_id, task.task_id)
        task.record(f"jetcobot1 → {task.pinky_id}에 물건 적재 명령")
        _send_command(task.jetcobot1_id, "load",
                      {"target_pinky": task.pinky_id}, task.task_id)
        # 다른 핑키의 도착으로 PINKY_WAIT_INCOMING 해제 가능
        _wait_queue.drain(_try_depart_to_storage)

    # ── Step 3: jetcobot1 적재 완료 → storage/rack으로 출발 (대기 조건 확인)
    elif s == TaskState.JETCOBOT1_LOADING and robot_id == task.jetcobot1_id and event == "done":
        _set_robot_idle(task.jetcobot1_id)
        if not _try_depart_to_storage(task):
            task.state = TaskState.PINKY_WAIT_INCOMING
            task.record(f"{task.pinky_id}: 다른 핑키 load_wait 도착 대기 중")
            _wait_queue.submit(task, _try_depart_to_storage)

    # ── Step 4: 핑키 storage/rack 도착 → jetcobot2 하역 명령
    elif s == TaskState.PINKY_TO_STORAGE and robot_id == task.pinky_id and event == "arrived":
        pinky = ROBOTS[task.pinky_id]
        pinky["current_zone"] = task.storage_zone
        pinky["target_zone"]  = None
        task.state = TaskState.JETCOBOT2_UNLOADING
        _set_robot_busy(task.jetcobot2_id, task.task_id)
        task.record(f"jetcobot2 → {task.storage_zone} 하역 명령")
        _send_command(task.jetcobot2_id, "unload",
                      {"zone": task.storage_zone, "source_pinky": task.pinky_id},
                      task.task_id)

    # ── Step 5: jetcobot2 하역 완료 → 핑키 홈 복귀
    elif s == TaskState.JETCOBOT2_UNLOADING and robot_id == task.jetcobot2_id and event == "done":
        _set_robot_idle(task.jetcobot2_id)
        _zone_manager.mark_occupied(task.storage_zone)
        ROBOTS[task.pinky_id]["current_zone"] = None
        ROBOTS[task.pinky_id]["target_zone"]  = "home"
        task.state = TaskState.PINKY_TO_HOME
        task.record(f"{task.pinky_id} → 홈 복귀 명령")
        _send_command(task.pinky_id, "navigate",
                      {"location": "home"}, task.task_id)

    # ── Step 6: 핑키 홈 도착 → 태스크 완료, 대기 태스크 재시도
    elif s == TaskState.PINKY_TO_HOME and robot_id == task.pinky_id and event == "arrived":
        pinky_id = task.pinky_id
        pinky    = ROBOTS[pinky_id]
        pinky["current_zone"] = "home"
        pinky["target_zone"]  = None
        _set_robot_idle(pinky_id)
        _home_push(pinky_id)
        task.state = TaskState.DONE
        task.record(f"태스크 완료 — {task.storage_zone} 점유, {pinky_id} 홈 복귀")
        _pinky_queue.drain(_dispatch_pinky_to_load_wait)
        _dispatch_queue.drain(_try_dispatch)

    elif event == "error":
        _fail_task(task, f"{robot_id} 오류 발생")

    else:
        task.record(f"[무시] {robot_id} / {event} (현재 상태: {s})")


# ════════════════════════════════════════════════════════════
# HTTP API
# ════════════════════════════════════════════════════════════

app = FastAPI(title="Task Manager")


# ── 요청 모델 ─────────────────────────────────────────────────
class TaskRequest(BaseModel):
    storage_zone: str

class OutboundTaskRequest(BaseModel):
    storage_zone: str

class NavigateRequest(BaseModel):
    robot_id: str
    location: str


# ── 태스크 라우트 ─────────────────────────────────────────────
def _valid_storage_zones() -> set[str]:
    if ROUTING_MODE == "waypoint":
        return {"rack_1", "rack_2", "rack_3"}
    return {f"zone_{i}" for i in range(1, 17)}


@app.post("/task")
def create_task(req: TaskRequest):
    valid = _valid_storage_zones()
    if req.storage_zone not in valid:
        raise HTTPException(
            400,
            f"현재 모드({ROUTING_MODE})에서 유효하지 않은 구역: {req.storage_zone}. "
            f"사용 가능: {sorted(valid)}"
        )
    if ZONES.get(req.storage_zone) == "occupied":
        raise HTTPException(409, f"{req.storage_zone} 이미 점유 중")

    task_id = str(uuid.uuid4())
    task = Task(task_id, req.storage_zone)
    with _lock:
        tasks[task_id] = task
        dispatched = _dispatch_queue.submit(task, _try_dispatch)

    if not dispatched:
        task.state = TaskState.QUEUED
        task.record(f"태스크 대기열 추가 (목표: {req.storage_zone})")

    return {
        "task_id":      task_id,
        "state":        task.state,
        "routing_mode": ROUTING_MODE,
        "queued":       not dispatched,
    }


@app.post("/task/outbound")
def create_outbound_task(req: OutboundTaskRequest):
    """출고 태스크 생성. storage_zone은 occupied 상태여야 함."""
    valid = {f"zone_{i}" for i in range(1, 4)}
    if req.storage_zone not in valid:
        raise HTTPException(400,
            f"출고 가능 구역: {sorted(valid)}")
    if ZONES.get(req.storage_zone) != "occupied":
        raise HTTPException(409,
            f"{req.storage_zone} 에 물품 없음 (현재 상태: {ZONES.get(req.storage_zone)})")

    task_id = str(uuid.uuid4())
    task    = OutboundTask(task_id, req.storage_zone)
    with _lock:
        tasks[task_id] = task
        if not _dispatch_outbound(task):
            task.state = OutboundState.QUEUED
            task.record("유휴 핑키 없음 — 출고 대기열 추가")
            _outbound_queue.submit(task, _dispatch_outbound)

    return {"task_id": task_id, "state": task.state, "task_type": "outbound"}


@app.get("/task/{task_id}")
def get_task(task_id: str):
    task = tasks.get(task_id)
    if task is None:
        raise HTTPException(404)
    return task.to_dict()


@app.get("/tasks")
def list_tasks():
    return [t.to_dict() for t in tasks.values()]


# ── 로봇·존 라우트 ────────────────────────────────────────────
@app.post("/navigate")
def navigate(req: NavigateRequest):
    """특정 로봇에게 직접 navigate 액션 전송 (태스크 플로우 외 단순 이동용)."""
    if req.robot_id not in ROBOTS:
        raise HTTPException(400, f"알 수 없는 로봇: {req.robot_id}. 사용 가능: {list(ROBOTS.keys())}")
    task_id = f"direct_{uuid.uuid4()}"
    with _lock:
        ROBOTS[req.robot_id]["target_zone"] = req.location
        _direct_commands[task_id] = {"robot_id": req.robot_id, "location": req.location}
    _send_command(req.robot_id, "navigate", {"location": req.location}, task_id)
    return {"ok": True, "robot_id": req.robot_id, "location": req.location}


@app.get("/robots")
def get_robots():
    return ROBOTS


@app.get("/zones")
def get_zones():
    return ZONES


# ── 모드·디버그 라우트 ────────────────────────────────────────
@app.get("/mode")
def get_mode():
    return {"routing_mode": ROUTING_MODE}


@app.post("/mode/{mode}")
def set_mode(mode: str):
    global ROUTING_MODE
    if mode not in ("zone", "waypoint"):
        raise HTTPException(400, "mode는 'zone' 또는 'waypoint'")
    ROUTING_MODE = mode
    print(f"[MODE] 라우팅 모드 변경 → {ROUTING_MODE}")
    return {"routing_mode": ROUTING_MODE}


@app.post("/debug/home_stack")
def set_home_stack(order: list[str]):
    with _lock:
        _home_stack.clear()
        _home_stack.extend(order)
    return {"home_stack": _home_stack}


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
        "routing_mode":  ROUTING_MODE,
        "robots":        ROBOTS,
        "home_stack":    {
            "order":     _home_stack,  # [inner, ..., outer]
            "positions": {pid: _home_position(pid) for pid in _home_stack},
        },
        "zones":         {k: v for k, v in ZONES.items() if v != "free"},
        "tasks": [
            {
                "task_id":    t.task_id[:8],
                "state":      t.state,
                "pinky":      t.pinky_id,
                "load_wait":  t.load_wait_zone,
                "storage":    t.storage_zone,
                "log":        t.log[-5:],
            }
            for t in tasks.values()
        ],
        "dispatch_pending": _dispatch_queue.pending_count(),
        "pinky_pending":    _pinky_queue.pending_count(),
        "wait_pending":     _wait_queue.pending_count(),
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
        print("[ROS2] executor 종료됨")
    except Exception as e:
        import traceback
        print(f"[ERROR] ros_spin 크래시: {e}")
        traceback.print_exc()


from .test_routes import router as _test_router
app.include_router(_test_router)


def main():
    global ros_node
    rclpy.init()
    ros_node = TaskManagerNode()
    threading.Thread(target=ros_spin, daemon=True).start()
    uvicorn.run(app, host="0.0.0.0", port=8090)
