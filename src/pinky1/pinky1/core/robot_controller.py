# core/robot_controller.py
# 메인 컨트롤러 — 모든 모듈 통합

import json
import math
import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

try:
    from msgs.action import RobotCommand
    _PINKY_MSGS_AVAILABLE = True
except ImportError:
    _PINKY_MSGS_AVAILABLE = False

from pinky1.config.settings import ROBOT_CONFIG, ARUCO_CONFIG, LOCATIONS, ZONE_DEPART_POINTS, ARRIVAL_US_STOP, ARRIVAL_US_FORWARD, ARRIVAL_YAW_ROTATE
from pinky1.navigation.nav_manager import NavManager
from pinky1.navigation.line_tracer import LineTracer
from pinky1.sensors.sensor_manager import SensorManager
from pinky1.utils.logger import RobotLogger
from pinky1.vision.aruco_detector import ArucoDetector
from pinky1.vision.yolo_detector import YoloDetector
from pinky1.core.visual_dock import VisualDock, MIN_DOCK_SIZE


class RobotController(Node):
    """
    단일 로봇 통합 컨트롤러
    ─────────────────────────────────────────
    [로봇 제공] SensorManager  — 모든 센서 토픽 수신
    [로봇 제공] NavManager     — Nav2 액션 + 로봇 서비스
    [내가 추가] ArucoDetector  — ArUco 마커 감지
    [내가 추가] YoloDetector   — 사람/박스 감지
    [ROS2 Action] RobotCommand — 명령 수신 (Task Manager → 로봇)
    """

    # ════════════════════════════════════════════════════════════
    # INITIALIZATION
    # ════════════════════════════════════════════════════════════
    def __init__(self, robot_id: str = "pinky1"):
        super().__init__(f"controller_{robot_id}", namespace=robot_id)

        cfg           = ROBOT_CONFIG[robot_id]
        self.robot_id = robot_id
        self.ns       = cfg["namespace"]
        self.log      = RobotLogger(self)

        self.log.info("SYS", f"=== {cfg['name']} 시작 ===")

        # ── TF Buffer (노드 전체 공유 — 중복 생성 방지) ────────
        self._tf_buf      = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # ── 모듈 초기화 ────────────────────────────
        self.sensors       = SensorManager(self, self.ns)
        self.nav           = NavManager(self, self.ns)
        self.line_tracer   = LineTracer(self, self.sensors, self.nav,
                                       angular_gain=0.15, angular_d_gain=0.20,
                                       stop_distance=0.065,
                                       tf_buf=self._tf_buf)
        self.aruco         = ArucoDetector(self)
        self.yolo          = YoloDetector(self)
        self.visual_dock   = VisualDock(self, self.sensors, tf_buf=self._tf_buf)

        # ── 도킹/마커 상태 ─────────────────────────
        self._docking          = False
        self._target_marker_id = None   # 정밀 정차 대상 마커 ID

        # ── 액션/네비 상태 ─────────────────────────
        self._task_id              = ""
        self._nav_event            = None   # navigate 액션 동기 대기용
        self._nav_event_result     = False
        self._line_trace_on_arrive = False  # home 도착 시 라인트레이싱 자동 시작
        self._current_location     = None   # 마지막으로 도착 완료된 위치
        self._target_location      = None   # 현재 이동 중인 목적지
        self._align_yaw            = None   # Nav2 완료 후 정렬할 map 기준 yaw

        # ── cmd_vel 직접 제어 상태 (회전/전진 타이머) ──
        self._rotate_timer         = None
        self._rotate_end_time      = 0.0    # 회전 종료 시각 (sec)
        self._pending_navigate     = None   # 회전/전진 후 실행할 목적지
        self._forward_timer        = None
        self._forward_end_time     = 0.0    # 전진 종료 시각 (sec, 폴백용)
        self._forward_target_dist  = 0.0    # 목표 이동 거리 (m)
        self._forward_start_pos    = None   # 출발 AMCL 위치
        self._forward_speed        = 0.05   # 전진 속도 (m/s)
        self._us_forward_timer     = None   # 초음파 전진 타이머
        self._us_forward_done_cb   = None   # 초음파 정지 후 콜백
        self._yaw_rotate_target    = None
        self._yaw_rotate_timer     = None
        self._yaw_rotate_done_cb   = None

        # ── 퍼블리셔 ───────────────────────────────
        self._cmd_pub     = self.create_publisher(Twist, f"/{self.ns}/cmd_vel", 10)

        # ── 콜백 연결 ──────────────────────────────
        self.sensors.on_image           = self._on_image
        self.sensors.on_person_detected = self._on_person_by_robot
        self.sensors.on_robot_status    = self._on_robot_status

        self.yolo.on_person_detected = self._on_person_by_yolo
        self.yolo.on_box_detected    = self._on_box_detected

        self.aruco.on_detected = self._on_aruco_detected

        # ── 비전 루프 타이머 (10Hz) ─────────────────
        self.create_timer(0.1, self._vision_loop)

        # ── Action Server 시작 ────────────────────
        if _PINKY_MSGS_AVAILABLE:
            self._action_server = ActionServer(
                self,
                RobotCommand,
                "command",
                self._execute_callback,
                callback_group=ReentrantCallbackGroup())
            self.log.info("SYS", "Action Server 시작 → robot_command")
        else:
            self.log.warn("SYS", "msgs 없음 — Action Server 비활성화")

        self.log.info("SYS", "모든 모듈 초기화 완료")

    # ════════════════════════════════════════════════════════════
    # VISION LOOP
    # ════════════════════════════════════════════════════════════
    def _vision_loop(self):
        img  = self.sensors.image
        cam  = self.sensors.camera_matrix
        dist = self.sensors.dist_coeffs

        if img is None:
            # 카메라 토픽 미수신 — 50회마다 한 번씩만 경고
            self._no_img_count = getattr(self, "_no_img_count", 0) + 1
            if self._no_img_count % 50 == 1:
                self.log.warn("SYS", "카메라 이미지 없음 (카메라 토픽 확인 필요)")
            return

        self._no_img_count = 0
        self.yolo.detect(img)
        self.aruco.detect(img, cam, dist)

    def _on_image(self, msg):
        pass

    # ════════════════════════════════════════════════════════════
    # EVENT HANDLERS
    # ════════════════════════════════════════════════════════════
    def _on_person_by_yolo(self, obj):
        self.log.warn("SYS", "YOLO 사람 감지 → 긴급 정지!")
        self.nav.emergency_stop(activate=True)
        self.nav.set_emotion("sad")
        self.nav.set_led(255, 0, 0)

    def _on_person_by_robot(self, msg):
        self.log.warn("SYS", "로봇 감지 사람 → 긴급 정지!")
        self.nav.emergency_stop(activate=True)

    def _on_box_detected(self, obj):
        self.log.info("SYS",
            f"박스 감지 위치: ({obj['cx']:.0f}, {obj['cy']:.0f})")

    def _on_aruco_detected(self, markers):
        for m in markers:
            if self._target_marker_id is not None and m["id"] == self._target_marker_id:
                if self.visual_dock.active:
                    # visual_dock 진행 중이면 항상 update (Nav2 도착 후 도킹 포함)
                    self.visual_dock.update(m)
                elif not self._docking:
                    # ARRIVAL_US_FORWARD 목적지는 Nav2 도착 후 visual_dock 시작 — 이동 중 취소 안 함
                    if self._target_location in ARRIVAL_US_FORWARD:
                        return
                    # 멀리 있으면 Nav2 장애물 회피에 맡기고, 충분히 가까워졌을 때만 시각 서보 시작
                    if m["pixel_size"] < MIN_DOCK_SIZE:
                        return
                    self.log.info("SYS",
                        f"마커 {m['id']} 근접 감지 → 네비게이션 취소 후 정밀 정차 시작")
                    self.nav.cancel_navigation()
                    self._stop_search()
                    self._docking = True
                    self.visual_dock.start(done_callback=self._on_dock_done)
                    self.visual_dock.update(m)

    def _on_robot_status(self, msg):
        self.log.info("SYS", f"로봇 상태: {msg}")

    # ════════════════════════════════════════════════════════════
    # ACTION SERVER & NAVIGATION CALLBACKS
    # ════════════════════════════════════════════════════════════

    # ── 액션 서버 ──────────────────────────────────
    def _execute_callback(self, goal_handle):
        action_type = goal_handle.request.action_type
        params      = json.loads(goal_handle.request.parameters_json or "{}")
        task_id     = goal_handle.request.task_id

        self.log.info("ACT", f"goal 수신 — action={action_type} params={params} task={task_id}")

        feedback        = RobotCommand.Feedback()
        feedback.status = "executing"
        goal_handle.publish_feedback(feedback)
        self.log.info("ACT", "feedback 전송 → executing")

        cmd = {"action": action_type, "parameters": params,
               "task_id": task_id}

        result = RobotCommand.Result()

        if action_type == "navigate":
            # navigate는 _on_nav_done 콜백까지 동기 대기 (Event)
            self._nav_event        = threading.Event()
            self._nav_event_result = False
            self._run_command(cmd)
            self._nav_event.wait(timeout=120.0)

            if self._nav_event_result:
                result.event   = "arrived"
                result.message = ""
                goal_handle.succeed()
                self.log.info("ACT", f"result 반환 → arrived (task={task_id})")
            else:
                result.event   = "error"
                result.message = "Navigation failed or timed out"
                goal_handle.abort()
                self.log.warn("ACT", f"result 반환 → error (task={task_id})")
        else:
            self._run_command(cmd)
            result.event   = "done"
            result.message = ""
            goal_handle.succeed()
            self.log.info("ACT", f"result 반환 → done (task={task_id})")

        return result

    # ── 네비 완료 콜백 ─────────────────────────────
    def _send_callback(self, event: str):
        if event == "arrived":
            self._current_location = self._target_location
        elif event == "error":
            self._current_location = None
        if self._nav_event is not None:
            self._nav_event_result = (event in ("arrived", "parked"))
            self._nav_event.set()
            self._nav_event = None

    def _on_dock_done(self, success=True):
        self._docking          = False
        self._target_marker_id = None
        if success:
            self.log.info("SYS", "정밀 정차 완료")
            self._send_callback("arrived")
        else:
            self.log.warn("SYS", "정밀 정차 실패 (마커 미감지 타임아웃)")
            self._send_callback("error")

    def _on_nav_done(self, success=True):
        if not success:
            self.log.warn("SYS", "내비게이션 실패")
            self._line_trace_on_arrive = False
            self._send_callback("error")
            return

        # home 도착 → 라인트레이싱 자동 시작 (백그라운드)
        if self._line_trace_on_arrive:
            self._line_trace_on_arrive = False
            self.log.info("SYS", "home 도착 → 라인트레이싱 시작")
            self.line_tracer.start()
            self._send_callback("arrived")
            return

        # align_yaw 파라미터 있으면 yaw 정렬 후 arrived
        if self._align_yaw is not None:
            target = self._align_yaw
            self._align_yaw = None
            self.log.info("SYS", f"도착 → yaw {math.degrees(target):.1f}° 정렬 시작")
            self._start_rotate_to_yaw(target,
                done_cb=lambda: self._send_callback("arrived"))
            return

        # 도착 후 cmd_vel로 yaw 정렬
        if self._target_location in ARRIVAL_YAW_ROTATE:
            target_yaw = ARRIVAL_YAW_ROTATE[self._target_location]
            self.log.info("SYS",
                f"{self._target_location} 도착 → cmd_vel yaw {math.degrees(target_yaw):.1f}° 정렬")
            self._start_rotate_to_yaw(target_yaw,
                done_cb=lambda: self._send_callback("arrived"))
            return

        # 도착 후 초음파 감지까지 전진 후 yaw 정렬
        if self._target_location in ARRIVAL_US_STOP:
            self.log.info("SYS",
                f"{self._target_location} 도착 → 초음파 6.5cm 감지까지 전진")
            self._start_us_forward(
                done_cb=lambda: self._start_rotate_to_yaw(
                    math.pi, done_cb=lambda: self._send_callback("arrived")))
            return

        # 도착 후 마커 각도 보정 전진 → 초음파 정지 → yaw 정렬 (load_wait_1 등)
        if self._target_location in ARRIVAL_US_FORWARD:
            self.log.info("SYS",
                f"{self._target_location} 도착 → 마커 visual_dock 시작")
            self._docking = True
            target_yaw = LOCATIONS.get("zone_1", {}).get("yaw", 0.0)
            self.visual_dock.start(done_callback=self._on_dock_done,
                                   target_yaw=target_yaw)
            return

        if self._target_marker_id is None or self._docking:
            self._send_callback("arrived")
            return
        self.log.info("SYS",
            f"도착 — 마커 {self._target_marker_id} 감지 대기 중")

    # ── 회전 (시간 기반 90도) ──────────────────────
    def _stop_rotate(self):
        if self._rotate_timer is not None:
            self._rotate_timer.cancel()
            self._rotate_timer = None
        self._cmd_pub.publish(Twist())

    def _start_left_turn_90(self):
        """제자리 왼쪽 90도 회전 (angular.z 양수)."""
        angular_speed = 0.5
        duration      = (math.pi / 2) / angular_speed
        now_sec       = self.get_clock().now().nanoseconds / 1e9
        self._rotate_end_time = now_sec + duration
        if self._rotate_timer is None:
            self._rotate_timer = self.create_timer(0.1, self._left_rotate_spin)

    def _left_rotate_spin(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec >= self._rotate_end_time:
            self._stop_rotate()
            self.log.info("SYS", "왼쪽 90도 회전 완료 → 홈 복귀 이동 시작")
            location = self._pending_navigate
            self._pending_navigate = None
            self._do_navigate(location)
            return
        msg = Twist()
        msg.angular.z = 0.5
        self._cmd_pub.publish(msg)

    # ── 전진 (거리 기반, AMCL + 시간 폴백) ─────────
    def _dist_to_depart_point(self, depart_location: str) -> float:
        """현재 AMCL 위치에서 출발지 중간 좌표까지 거리 반환. 읽기 실패 시 0.20m."""
        target = ZONE_DEPART_POINTS.get(depart_location)
        if target is None:
            return 0.20
        try:
            pose = self.sensors.amcl_pose
            if pose is None:
                return 0.20
            dx = target["x"] - pose.pose.pose.position.x
            dy = target["y"] - pose.pose.pose.position.y
            return math.sqrt(dx * dx + dy * dy)
        except Exception:
            return 0.20

    def _start_forward(self, distance: float, speed: float = 0.05):
        """cmd_vel로 지정 거리만큼 전진. AMCL 기준 이동 거리 측정, 폴백은 타이머."""
        self._forward_target_dist = distance
        self._forward_speed       = speed
        self._forward_start_pos   = self._amcl_xy()
        # 폴백: AMCL 없을 때 시간 기반. 여유 2배 확보
        duration = distance / speed
        now_sec  = self.get_clock().now().nanoseconds / 1e9
        self._forward_end_time = now_sec + duration * 2.0
        if self._forward_timer is None:
            self._forward_timer = self.create_timer(0.05, self._forward_tick)

    def _forward_tick(self):
        done = False
        if self._forward_start_pos is not None:
            cur = self._amcl_xy()
            if cur is not None:
                dx = cur[0] - self._forward_start_pos[0]
                dy = cur[1] - self._forward_start_pos[1]
                traveled = math.sqrt(dx * dx + dy * dy)
                done = traveled >= self._forward_target_dist
        if not done:
            # AMCL 없으면 시간 폴백
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if self._forward_start_pos is None and now_sec >= self._forward_end_time:
                done = True

        if done:
            self._cmd_pub.publish(Twist())
            if self._forward_timer is not None:
                self._forward_timer.cancel()
                self._forward_timer = None
            self._forward_start_pos = None
            location = self._pending_navigate
            self._pending_navigate = None
            if location is None:
                self.log.info("SYS", "전진 완료 → arrived 반환")
                self._send_callback("arrived")
            else:
                self.log.info("SYS", f"전진 완료 → Nav2 시작 ({location})")
                self._do_navigate(location)
            return
        msg = Twist()
        msg.linear.x = self._forward_speed
        self._cmd_pub.publish(msg)

    def _start_us_forward(self, done_cb=None):
        """초음파 ≤ 6.5cm 될 때까지 전진."""
        self._us_forward_done_cb = done_cb
        if self._us_forward_timer is None:
            self._us_forward_timer = self.create_timer(0.05, self._us_forward_tick)

    def _us_forward_tick(self):
        us = self.sensors.us_distance
        if us is not None and us <= 0.065:
            self._cmd_pub.publish(Twist())
            self._us_forward_timer.cancel()
            self._us_forward_timer = None
            self.log.info("SYS", f"초음파 정지 {us*100:.1f}cm")
            cb = self._us_forward_done_cb
            self._us_forward_done_cb = None
            if cb:
                cb()
            return
        msg = Twist()
        msg.linear.x = 0.05
        self._cmd_pub.publish(msg)

    # ── yaw 정렬 회전 (목표 yaw 기준) ──────────────
    def _start_rotate_to_yaw(self, target_yaw: float, done_cb=None):
        self._yaw_rotate_target  = target_yaw
        self._yaw_rotate_done_cb = done_cb
        if self._yaw_rotate_timer is None:
            self._yaw_rotate_timer = self.create_timer(0.1, self._yaw_rotate_tick)

    def _yaw_rotate_tick(self):
        yaw = self._map_yaw()
        if yaw is None:
            return
        diff = self._angle_diff(self._yaw_rotate_target, yaw)
        if abs(diff) <= math.radians(3.0):
            self._cmd_pub.publish(Twist())
            self._yaw_rotate_timer.cancel()
            self._yaw_rotate_timer = None
            self.log.info("SYS",
                f"yaw 정렬 완료 → {math.degrees(self._yaw_rotate_target):.1f}°")
            cb = self._yaw_rotate_done_cb
            self._yaw_rotate_done_cb = None
            self._yaw_rotate_target  = None
            if cb:
                cb()
            return
        # 거리 비례 속도 (최소 0.15, 최대 0.5)
        speed = max(0.15, min(0.5, abs(diff) * 0.5))
        msg = Twist()
        msg.angular.z = math.copysign(speed, diff)
        self._cmd_pub.publish(msg)

    # ── yaw 조회 헬퍼 ──────────────────────────────
    def _map_yaw(self) -> float | None:
        """map 기준 yaw. TF 실패 시 AMCL 폴백."""
        try:
            t = self._tf_buf.lookup_transform(
                "map", f"{self.ns}/base_footprint",
                rclpy.time.Time())
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            return math.atan2(siny, cosy)
        except Exception:
            pose = self.sensors.amcl_pose
            if pose is not None:
                q = pose.pose.pose.orientation
                siny = 2.0 * (q.w * q.z + q.x * q.y)
                cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                return math.atan2(siny, cosy)
            return None

    def _current_map_yaw(self) -> float | None:
        """현재 map 기준 yaw. AMCL만 사용."""
        pose = self.sensors.amcl_pose
        if pose is None:
            return None
        q = pose.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _angle_diff(self, a: float, b: float) -> float:
        d = a - b
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def _amcl_xy(self):
        """현재 AMCL map 좌표 (x, y). 없으면 None."""
        pose = self.sensors.amcl_pose
        if pose is None:
            return None
        p = pose.pose.pose.position
        return (p.x, p.y)

    # ── Nav2 호출 ──────────────────────────────────
    def _do_navigate(self, location: str):
        if self._align_yaw is not None or location in ARRIVAL_YAW_ROTATE:
            # cmd_vel로 yaw 정렬할 위치 또는 align_yaw 있으면
            # 현재 yaw로 Nav2 goal 설정 → Nav2가 최종 회전 시도 안 함
            current_yaw = self._current_map_yaw() or 0.0
            self.nav.go_to_location(location, callback=self._on_nav_done,
                                    override_yaw=current_yaw)
        else:
            self.nav.go_to_location(location, callback=self._on_nav_done)

    def _stop_search(self):
        self._cmd_pub.publish(Twist())

    def _cancel_all_motion(self):
        """진행 중인 모든 cmd_vel 발행 타이머/모듈을 강제 정지.
        새 navigate/stop 명령 진입 시 좀비 타이머가 cmd_vel을 덮어쓰지 않도록 한다."""
        for attr in ("_rotate_timer", "_forward_timer",
                     "_us_forward_timer", "_yaw_rotate_timer"):
            t = getattr(self, attr, None)
            if t is not None:
                t.cancel()
                setattr(self, attr, None)
        self._forward_start_pos  = None
        self._pending_navigate   = None
        self._us_forward_done_cb = None
        self._yaw_rotate_done_cb = None
        self._yaw_rotate_target  = None
        if self.line_tracer.is_active:
            self.line_tracer.stop()
        if self.visual_dock.active:
            self.visual_dock.stop()
        self._docking = False
        self._cmd_pub.publish(Twist())

    # ════════════════════════════════════════════════════════════
    # COMMAND ROUTER
    # ════════════════════════════════════════════════════════════
    def _run_command(self, cmd: dict):
        action = cmd.get("action", "unknown")
        params = cmd.get("parameters", {})

        self.log.info("SYS", f"실행: {action} {params}")

        # ── 이동 / 도킹 / 운반 ──────────────────────
        if action == "navigate":
            self._cancel_all_motion()
            self._target_marker_id     = None
            self._task_id              = cmd.get("task_id", "")
            self._line_trace_on_arrive = (params.get("location") == "home")
            self._align_yaw            = params.get("align_yaw", None)
            location      = params.get("location", "home")
            prev_location = self._current_location
            self._target_location = location

            if location == "stop":
                self.nav.cancel_navigation()
                self.nav.emergency_stop(activate=True)
                return

            # 마커와 연결된 장소면 정밀 정차 대상 설정
            loc_to_marker = {v: k for k, v in ARUCO_CONFIG["marker_map"].items()}
            if location.startswith("marker_"):
                try:
                    self._target_marker_id = int(location.split("_")[1])
                except (IndexError, ValueError):
                    self._target_marker_id = None
            else:
                self._target_marker_id = loc_to_marker.get(location, None)
            if self._target_marker_id is not None:
                self.log.info("SYS",
                    f"목적지 {location} → 마커 {self._target_marker_id} 정밀 정차 대기")

            # zone_1/2/3 출발 → 홈 복귀 시 왼쪽 90도 회전 후 이동
            if location == "home" and prev_location in ("zone_1", "zone_2", "zone_3"):
                self.log.info("SYS",
                    f"{prev_location} 출발 → 홈 복귀 전 왼쪽 90도 회전")
                self._pending_navigate = location
                self._start_left_turn_90()
                return

            # 출발지가 ZONE_DEPART_POINTS에 있으면 cmd_vel 전진 후 Nav2 시작
            if prev_location in ZONE_DEPART_POINTS:
                dist = self._dist_to_depart_point(prev_location)
                self.log.info("SYS",
                    f"{location} 이동 → {prev_location} 출발점까지 {dist:.3f}m 전진 후 Nav2 시작")
                self._pending_navigate = location
                self._start_forward(distance=dist, speed=0.05)
                return

            self._do_navigate(location)

        elif action == "dock":
            self.nav.dock_to_marker(
                int(params.get("marker_id", 0)))

        elif action == "transport":
            self.nav.run_transport(
                params.get("pickup",   "loading_zone"),
                params.get("delivery", "zone_A"))

        # ── 정지 / 재개 ────────────────────────────
        elif action == "stop":
            self._cancel_all_motion()
            self._target_marker_id = None
            self.nav.cancel_navigation()
            self.nav.emergency_stop(activate=True)
            self.nav.set_led(255, 0, 0)

        elif action == "resume":
            self.nav.emergency_stop(activate=False)
            self.nav.set_led(0, 255, 0)

        # ── 표현 / 조명 ────────────────────────────
        elif action == "set_emotion":
            self.nav.set_emotion(
                params.get("emotion", "neutral"))

        elif action == "set_led":
            self.nav.set_led(
                int(params.get("r", 255)),
                int(params.get("g", 255)),
                int(params.get("b", 255)))

        elif action == "set_lamp":
            self.nav.set_lamp(
                params.get("mode",  "on"),
                params.get("color", "white"))

        # ── 라인 트레이싱 ──────────────────────────
        elif action == "line_trace_start":
            self.line_tracer.on_parked = lambda: self._send_callback("parked")
            self.line_tracer.start()

        elif action == "line_trace_stop":
            self.line_tracer.stop()

        elif action == "unknown":
            self.log.warn("SYS",
                f"모르는 명령: {cmd.get('message')}")
