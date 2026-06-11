# navigation/nav_manager.py
# Nav2 액션 + 로봇 제공 서비스 전체 관리

import math
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses

from pinky1.config.settings import LOCATIONS, NAV2_CONFIG
from pinky1.utils.logger import RobotLogger

try:
    from pinky_interfaces.action import TransportMission, DockToMarker
    from pinky_interfaces.srv import (
        Emotion, SetLed, SetBrightness, SetLamp, EmergencyStop)
    _PINKY_AVAILABLE = True
except ImportError:
    _PINKY_AVAILABLE = False


class NavManager:
    """
    Nav2 자율주행 + 로봇 제공 서비스 관리
    ──────────────────────────────────────
    Actions (로봇 제공):
      navigate_to_pose       단일 목적지 이동
      navigate_through_poses 경유지 이동
      transport_mission      픽업→배송 전체 미션
      dock_to_marker         마커 앞 정밀 정차

    Services (로봇 제공):
      set_emotion            표정 변경
      set_led                LED 제어
      set_brightness         밝기 조절
      set_lamp               램프 제어
      emergency_stop         긴급 정지/해제
    """

    def __init__(self, node: Node, namespace: str = ""):
        self.node = node
        self.ns   = f"/{namespace}" if namespace else ""
        self.log  = RobotLogger(node)

        self.is_navigating = False
        self.current_goal  = None
        self._goal_handle  = None

        self._setup_actions()
        self._setup_services()

    # ── 초기화 ─────────────────────────────────────
    def _setup_actions(self):
        ns = self.ns

        # Nav2 기본 액션
        self.nav_client = ActionClient(
            self.node, NavigateToPose,
            f"{ns}/navigate_to_pose")
        self.nav_through_client = ActionClient(
            self.node, NavigateThroughPoses,
            f"{ns}/navigate_through_poses")

        # 로봇 커스텀 액션
        if _PINKY_AVAILABLE:
            self.transport_client = ActionClient(
                self.node, TransportMission,
                f"{ns}/transport_mission")
            self.dock_client = ActionClient(
                self.node, DockToMarker,
                f"{ns}/dock_to_marker")
        else:
            self.transport_client = None
            self.dock_client      = None
            self.log.warn("NAV",
                "pinky_interfaces 없음 — 커스텀 액션 비활성화")

        self.log.info("NAV", "액션 클라이언트 등록 완료")

    def _setup_services(self):
        ns = self.ns

        if _PINKY_AVAILABLE:
            self.emotion_client = self.node.create_client(
                Emotion,       f"{ns}/set_emotion")
            self.led_client = self.node.create_client(
                SetLed,        f"{ns}/set_led")
            self.brightness_client = self.node.create_client(
                SetBrightness, f"{ns}/set_brightness")
            self.lamp_client = self.node.create_client(
                SetLamp,       f"{ns}/set_lamp")
            self.stop_client = self.node.create_client(
                EmergencyStop, f"{ns}/emergency_stop")
            self.log.info("NAV", "서비스 클라이언트 등록 완료")
        else:
            self.emotion_client    = None
            self.led_client        = None
            self.brightness_client = None
            self.lamp_client       = None
            self.stop_client       = None
            self.log.warn("NAV",
                "pinky_interfaces 없음 — 서비스 비활성화")

    def cancel_navigation(self):
        """현재 네비게이션 취소"""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle  = None
            self.is_navigating = False
            self.log.info("NAV", "네비게이션 취소")

    # ── 이동 액션 ──────────────────────────────────
    def go_to(self, x: float, y: float,
              yaw: float = 0.0, callback=None):
        """좌표로 이동"""
        goal      = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)
        self.current_goal  = (x, y)
        self.is_navigating = True

        self.log.info("NAV",
            f"이동 시작 → ({x:.2f}, {y:.2f}) yaw={yaw:.2f}")
        self._send_action(
            self.nav_client, goal, callback)

    def go_to_location(self, name: str, callback=None, override_yaw=None):
        """장소 이름으로 이동. override_yaw 지정 시 해당 yaw로 goal 전송 (Nav2 최종 회전 억제용)."""
        if name not in LOCATIONS:
            self.log.error("NAV", f"모르는 장소: {name}")
            if callback:
                callback(False)
            return False
        loc = LOCATIONS[name]
        yaw = override_yaw if override_yaw is not None else loc["yaw"]
        self.go_to(loc["x"], loc["y"], yaw, callback)
        return True

    def go_through(self, waypoints: list, callback=None):
        """경유지 여러 개 순서대로 이동"""
        goal       = NavigateThroughPoses.Goal()
        goal.poses = [
            self._make_pose(w["x"], w["y"], w.get("yaw", 0.0))
            for w in waypoints
        ]
        self.is_navigating = True
        self.log.info("NAV",
            f"경유지 이동 ({len(waypoints)}개)")
        self._send_action(
            self.nav_through_client, goal, callback)

    # ── 도킹 ───────────────────────────────────────
    def dock_to_marker(self, marker_id: int, callback=None):
        """ArUco 마커 앞에 정밀 정차"""
        if not self.dock_client:
            self.log.warn("NAV", "dock_to_marker 미지원")
            return
        goal           = DockToMarker.Goal()
        goal.marker_id = marker_id
        self.log.info("NAV",
            f"마커 {marker_id} 도킹 시작")
        self._send_action(
            self.dock_client, goal, callback)

    # ── 운반 미션 ──────────────────────────────────
    def run_transport(self, pickup: str,
                      delivery: str, callback=None):
        """픽업 → 배송 전체 미션"""
        if not self.transport_client:
            self.log.warn("NAV", "transport_mission 미지원")
            return
        if pickup not in LOCATIONS or \
           delivery not in LOCATIONS:
            self.log.error("NAV", "잘못된 장소 이름")
            return

        goal               = TransportMission.Goal()
        goal.pickup_pose   = self._make_pose(
            **{k: LOCATIONS[pickup][k]
               for k in ["x","y","yaw"]})
        goal.delivery_pose = self._make_pose(
            **{k: LOCATIONS[delivery][k]
               for k in ["x","y","yaw"]})

        self.log.info("NAV",
            f"운반 미션: {pickup} → {delivery}")
        self._send_action(
            self.transport_client, goal, callback)

    # ── 서비스 호출 ────────────────────────────────
    def emergency_stop(self, activate: bool = True):
        """긴급 정지 / 해제"""
        if not self.stop_client:
            self.log.warn("NAV", "emergency_stop 미지원")
            return
        req          = EmergencyStop.Request()
        req.activate = activate
        self.stop_client.call_async(req)
        status = "활성화" if activate else "해제"
        self.log.warn("NAV", f"긴급 정지 {status}!")

    def set_emotion(self, emotion: str):
        """로봇 표정 변경 (happy/sad/neutral 등)"""
        if not self.emotion_client:
            return
        req         = Emotion.Request()
        req.emotion = emotion
        self.emotion_client.call_async(req)
        self.log.info("LED", f"표정 변경: {emotion}")

    def set_led(self, r: int, g: int, b: int):
        """LED 색상 변경"""
        if not self.led_client:
            return
        req   = SetLed.Request()
        req.r = r
        req.g = g
        req.b = b
        self.led_client.call_async(req)
        self.log.info("LED", f"LED 색상: ({r},{g},{b})")

    def set_brightness(self, level: int):
        """LED 밝기 조절 (0~100)"""
        if not self.brightness_client:
            return
        req            = SetBrightness.Request()
        req.brightness = level
        self.brightness_client.call_async(req)

    def set_lamp(self, mode: str, color: str = "white"):
        """상단 램프 제어"""
        if not self.lamp_client:
            return
        req       = SetLamp.Request()
        req.mode  = mode
        req.color = color
        self.lamp_client.call_async(req)
        self.log.info("LED", f"램프: mode={mode} color={color}")

    # ── 내부 헬퍼 ──────────────────────────────────
    def _make_pose(self, x, y, yaw=0.0):
        pose                 = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp    = \
            self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        return pose

    def _send_action(self, client, goal, callback):
        future = client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback)
        future.add_done_callback(
            lambda f: self._on_accepted(f, callback))

    def _on_feedback(self, fb):
        try:
            dist = fb.feedback.distance_remaining
            if dist > 0.5:
                self.log.debug("NAV",
                    f"남은 거리: {dist:.2f}m")
        except Exception:
            pass

    def _on_accepted(self, future, callback):
        try:
            gh = future.result()
        except Exception as e:
            self.log.error("NAV", f"액션 전송 실패: {e}")
            self.is_navigating = False
            if callback:
                callback(False)
            return
        if not gh.accepted:
            self.log.error("NAV", "목표 거부됨 — 초기 위치 설정 여부 확인")
            self.is_navigating = False
            if callback:
                callback(False)
            return
        self.log.info("NAV", "목표 수락됨 — 이동 중...")
        self._goal_handle = gh
        rf = gh.get_result_async()
        rf.add_done_callback(
            lambda f: self._on_result(f, callback))

    def _on_result(self, future, callback):
        self.is_navigating = False
        self.current_goal  = None
        self._goal_handle  = None
        try:
            result = future.result()
            success = result.status == 4
            if success:
                self.log.info("NAV", "이동 완료!")
            else:
                self.log.warn("NAV",
                    f"이동 종료 (status={result.status})")
        except Exception as e:
            self.log.error("NAV", f"결과 수신 오류: {e}")
            success = False
        if callback:
            callback(success)
