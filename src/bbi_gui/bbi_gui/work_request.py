import os
import rclpy
from PyQt5.QtWidgets import QDialog, QMessageBox
from PyQt5.QtCore import QTimer, pyqtSignal, Qt
from PyQt5.QtGui import QPixmap
from PyQt5 import uic
 
try:
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
 
try:
    from db.db_client import DBClient
    DB_AVAILABLE = True
except ImportError:
    DB_AVAILABLE = False
 
 
def get_ui_path(filename):
    if ROS2_AVAILABLE:
        return os.path.join(
            get_package_share_directory('bbi_gui'),
            'ui', filename
        )
    else:
        return os.path.join(os.path.dirname(__file__), 'ui', filename)
 
 
def get_image_path(filename):
    if ROS2_AVAILABLE:
        return os.path.join(
            get_package_share_directory('bbi_gui'),
            'images', filename
        )
    else:
        return os.path.join(os.path.dirname(__file__), 'images', filename)
 
 
class WorkRequestDialog(QDialog):
 
    def __init__(self, node=None, user_info=None, rack_info=None, parent=None):
        super().__init__(parent)
        self.node      = node
        self.user_info = user_info
        self.rack_info = rack_info
 
        uic.loadUi(get_ui_path('work_request.ui'), self)
 
        # 사용자 이름 설정
        if user_info:
            self.label_username.setText(f"{user_info.get('name', '')} 님")
 
        # 랙 정보 설정
        if rack_info:
            rack_id = rack_info.get('rack_id', '')
            parts   = rack_id.split('-')
            zone    = parts[1] if len(parts) > 1 else '-'
            number  = parts[2] if len(parts) > 2 else '-'
            self.label_rack_info.setText(f"{zone}구역 {number}번")
 
        # 로봇 이미지 로드
        self._load_robot_image()
 
        # 버튼 연결
        self.btn_start.clicked.connect(self.on_start)
        self.btn_logout.clicked.connect(self.on_logout)
 
    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)
 
    def _load_robot_image(self):
        """로봇 이미지 로드"""
        try:
            img_path = get_image_path('robot.png')
            pixmap   = QPixmap(img_path)
            self.label_robot_image.setPixmap(
                pixmap.scaled(100, 100, Qt.KeepAspectRatio, Qt.SmoothTransformation)
            )
        except Exception as e:
            print(f"[이미지 로드 오류] {e}")
 
    def on_start(self):
        """작업 시작 버튼 클릭"""
        text = self.input_request.toPlainText().strip()
 
        if not text:
            QMessageBox.warning(self, '경고', '작업 요청 내용을 입력해주세요.')
            return
 
        # 확인 메시지 창
        reply = QMessageBox.question(
            self,
            '작업 시작 확인',
            f"아래 내용으로 작업을 시작합니다.\n\n"
            f"{text}\n\n"
            f"수정할 내용이 있으신가요?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
 
        if reply == QMessageBox.Yes:
            # 수정할 내용 있음 → 메시지 창 닫고 다시 입력
            return
        else:
            # 수정 없음 → 작업 시작
            self._start_work(text)
 
    def _start_work(self, text):
        """작업 시작 처리"""
        print(f"[작업 시작] {text}")
        # TODO: ROS2 토픽으로 작업 요청 전송
        QMessageBox.information(self, '완료', '작업 요청이 전송되었습니다.')
        self.accept()
 
    def on_logout(self):
        """로그아웃 버튼 클릭"""
        reply = QMessageBox.question(
            self,
            '로그아웃',
            '로그아웃 하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.reject()
 