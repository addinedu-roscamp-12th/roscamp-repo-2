# navigation/slam_manager.py
# SLAM 지도 생성/저장/불러오기

import subprocess
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

from config.settings import SLAM_CONFIG
from utils.logger import RobotLogger


class SlamManager:
    """
    SLAM 관리 (slam_toolbox 기반)
    - 새 지도 생성 (매핑 모드)
    - 기존 지도로 위치 추정 (로컬라이제이션 모드)
    - 지도 저장/불러오기
    - /map 토픽 수신
    """

    def __init__(self, node: Node):
        self.node     = node
        self.log      = RobotLogger(node)
        self.map      = None
        self._process = None

        # /map 토픽 구독
        self.node.create_subscription(
            OccupancyGrid, "/map",
            self._on_map, 10)

        self.log.info("SLAM", "SlamManager 초기화 완료")

    # ── 모드 선택 ──────────────────────────────────
    def start_mapping(self):
        """새 지도 생성 모드"""
        self.log.info("SLAM", "매핑 모드 시작...")
        self._process = subprocess.Popen([
            "ros2", "launch", "slam_toolbox",
            "online_async_launch.py",
            "use_sim_time:=false",
        ])

    def start_localization(self, map_path: str = None):
        """기존 지도로 위치 추정 모드"""
        path = map_path or SLAM_CONFIG["map_save_path"]
        self.log.info("SLAM",
            f"로컬라이제이션 모드: {path}.yaml")
        self._process = subprocess.Popen([
            "ros2", "launch", "slam_toolbox",
            "localization_launch.py",
            f"map:={path}.yaml",
        ])

    def save_map(self, path: str = None):
        """현재 지도 파일로 저장"""
        save = path or SLAM_CONFIG["map_save_path"]
        subprocess.run([
            "ros2", "run", "nav2_map_server",
            "map_saver_cli", "-f", save
        ])
        self.log.info("SLAM", f"지도 저장: {save}.yaml")

    def stop(self):
        """SLAM 종료"""
        if self._process:
            self._process.terminate()
            self._process = None
            self.log.info("SLAM", "SLAM 종료")

    # ── 콜백 ───────────────────────────────────────
    def _on_map(self, msg):
        self.map = msg
        self.log.debug("SLAM",
            f"지도 수신 ({msg.info.width}x{msg.info.height})")

    # ── 프로퍼티 ───────────────────────────────────
    @property
    def is_ready(self):
        return self.map is not None

    @property
    def map_size(self):
        if self.map is None:
            return None
        return {
            "width":  self.map.info.width,
            "height": self.map.info.height,
            "res":    self.map.info.resolution,
        }
