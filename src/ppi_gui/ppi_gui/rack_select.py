import os
import rclpy
from PyQt5.QtWidgets import QDialog, QPushButton, QGridLayout, QLabel, QMessageBox
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter
from PyQt5 import uic

try:
    import rclpy
    from rclpy.node import Node
    from ament_index_python.packages import get_package_share_directory
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

try:
    from db.db_client import DBClient
    DB_AVAILABLE = True
    print("[DB] db_client import 성공")
except ImportError:
    DB_AVAILABLE = False
    print("[DB] db_client import 실패 → 임시 데이터 사용")

def get_ui_path(filename):
    if ROS2_AVAILABLE:
        return os.path.join(
            get_package_share_directory('ppi_gui'),
            'ui', filename
        )
    else:
        return os.path.join(os.path.dirname(__file__), 'ui', filename)


# 상태별 색상
STATUS_COLORS = {
    '사용가능': '#3fb950',  # 초록
    '사용중':   '#f85149',  # 빨강
    '점검중':   '#d29922',  # 노랑
}

# 임시 랙 데이터 (DB 연동 전)
TEMP_RACK_DATA = [
    {'rack_id': 'rack-A-01', 'zone': 'A', 'status': '사용가능'},
    {'rack_id': 'rack-A-02', 'zone': 'A', 'status': '사용중'},
    {'rack_id': 'rack-A-03', 'zone': 'A', 'status': '사용가능'},
    {'rack_id': 'rack-A-04', 'zone': 'A', 'status': '점검중'},
    {'rack_id': 'rack-A-05', 'zone': 'A', 'status': '사용가능'},
    {'rack_id': 'rack-A-06', 'zone': 'A', 'status': '사용가능'},
    {'rack_id': 'rack-A-07', 'zone': 'A', 'status': '사용중'},
    {'rack_id': 'rack-A-08', 'zone': 'A', 'status': '사용가능'},
    {'rack_id': 'rack-B-01', 'zone': 'B', 'status': '사용가능'},
    {'rack_id': 'rack-B-02', 'zone': 'B', 'status': '사용가능'},
    {'rack_id': 'rack-B-03', 'zone': 'B', 'status': '점검중'},
    {'rack_id': 'rack-B-04', 'zone': 'B', 'status': '사용가능'},
    {'rack_id': 'rack-B-05', 'zone': 'B', 'status': '사용중'},
    {'rack_id': 'rack-B-06', 'zone': 'B', 'status': '사용가능'},
    {'rack_id': 'rack-B-07', 'zone': 'B', 'status': '사용가능'},
    {'rack_id': 'rack-B-08', 'zone': 'B', 'status': '사용가능'},
]


class RackButton(QPushButton):
    '''랙 메인 화면 및 상태 표시'''
    def __init__(self, rack_data, parent=None):
        super().__init__(parent)
        self.rack_data   = rack_data
        self.rack_id     = rack_data['rack_id']
        self.status      = rack_data['status']
        self.is_selected = False

        self.setFixedSize(120, 80)
        self.setCheckable(True)
        self.clicked.connect(self.on_clicked)
        self._apply_style()

    def _apply_style(self):
        color = STATUS_COLORS.get(self.status, '#8b949e')
        self.setText(self.rack_id.replace('rack-', ''))

        # 사용가능하지 않으면 아예 비활성화
        if self.status != '사용가능':
            self.setEnabled(False)

        self.setStyleSheet(f"""
            QPushButton {{
                background-color: #1c2128;
                border: 2px solid #30363d;
                border-radius: 10px;
                color: white;
                font-size: 12px;
                font-weight: bold;
            }}
            QPushButton:disabled {{
                background-color: #0d1117;
                color: #8b949e;
                border: 2px solid #21262d;
            }}
            QPushButton:checked {{
                border: 2px solid {color};
                background-color: #21262d;
            }}
            QPushButton:hover:!disabled {{
                background-color: #21262d;
            }}
        """)

    def paintEvent(self, event):
        """버튼 우측 상단에 상태 원 그리기"""
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        color = STATUS_COLORS.get(self.status, '#8b949e')
        painter.setBrush(QColor(color))
        painter.setPen(Qt.NoPen)

        # 우측 상단에 원 그리기 (반지름 6px)
        painter.drawEllipse(self.width() - 16, 8, 12, 12)
        painter.end()

    def on_clicked(self):
        pass

class RackDialog(QDialog):
    rack_data_signal = pyqtSignal(list)

    def __init__(self, node=None, user_info=None, parent=None):
        super().__init__(parent)
        self.node        = node
        self.user_info   = user_info
        self.selected_rack = None
        self.rack_buttons  = []

        uic.loadUi(get_ui_path('rack.ui'), self)

        # 사용자 이름 설정
        if user_info:
            self.label_username.setText(f"{user_info.get('name', '')} 님")

        # 시그널 연결
        self.rack_data_signal.connect(self._build_rack_grid)

        # 선택 버튼
        self.btn_select.clicked.connect(self.on_select)

        # 랙 데이터 로드
        QTimer.singleShot(500, self._load_rack_data)

    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def _load_rack_data(self):
        print(f"[디버그] DB_AVAILABLE: {DB_AVAILABLE}")
        print(f"[디버그] node: {self.node}")
        print(f"[디버그] DB_AVAILABLE and self.node: {DB_AVAILABLE and self.node is not None}")

        """DB에서 랙 데이터 로드"""
        if DB_AVAILABLE and self.node:
            print("[디버그] DB에서 랙 데이터 로드 시도")
            db_client = DBClient(self.node)
            db_client.execute_query_with_response(
                "SELECT rack_id, location, status FROM `RACK` ORDER BY rack_id",
                    callback=lambda r: self.rack_data_signal.emit(
                    r['result'] if r['success'] and r['result'] else TEMP_RACK_DATA
                )
            )
        else:
            """db에 없다면 임시 데이터로 load"""
            print("[디버그] 임시 데이터 사용")
            self._build_rack_grid(TEMP_RACK_DATA)

    def _build_rack_grid(self, rack_list):
        """랙 그리드 동적 생성"""
        # rack_panel에 기존 위젯 제거
        if self.rack_panel.layout():
            while self.rack_panel.layout().count():
                item = self.rack_panel.layout().takeAt(0)
                if item.widget():
                    item.widget().deleteLater()
        else:
            self.rack_panel.setLayout(QGridLayout())

        layout = self.rack_panel.layout()
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)

        # rack_id에서 zone 추출
        # 'rack-A-01' → 'A'
        zone_a = [r for r in rack_list if r.get('rack_id', '').split('-')[1] == 'A']
        zone_b = [r for r in rack_list if r.get('rack_id', '').split('-')[1] == 'B']

        # 구역 A 타이틀
        label_a = QLabel('A 구역')
        label_a.setStyleSheet('color: #8b949e; font-size: 13px; font-weight: bold;')
        label_a.setAlignment(Qt.AlignCenter)
        layout.addWidget(label_a, 0, 0, 1, 4)  # 4칸 차지

        # 구역 A 랙 버튼 (4x2)
        for i, rack in enumerate(zone_a[:8]):
            btn = RackButton(rack)
            btn.clicked.connect(lambda checked, b=btn: self._on_rack_selected(b))
            self.rack_buttons.append(btn)
            row = (i // 4) + 1  # 4열 기준으로 행 계산
            col = i % 4         # 0,1,2,3 반복
            layout.addWidget(btn, row, col)
        
        # A구역과 B구역 사이 구분선
        separator = QLabel()
        separator.setFixedHeight(2)
        separator.setStyleSheet('background-color: #30363d;')
        layout.addWidget(separator, 3, 0, 1, 4)  # 4칸 전체 차지

        # 구역 B 타이틀
        label_b = QLabel('B 구역')
        label_b.setStyleSheet('color: #8b949e; font-size: 13px; font-weight: bold;')
        label_b.setAlignment(Qt.AlignCenter)
        layout.addWidget(label_b, 4, 0, 1, 4)  # A구역 아래에 배치

        # 구역 B 랙 버튼 (4x2)
        for i, rack in enumerate(zone_b[:8]):
            btn = RackButton(rack)
            btn.clicked.connect(lambda checked, b=btn: self._on_rack_selected(b))
            self.rack_buttons.append(btn)
            row = (i // 4) + 5  # B구역 시작 행
            col = i % 4
            layout.addWidget(btn, row, col)

    def _on_rack_selected(self, selected_btn):
        """랙 선택 시 다른 버튼 해제"""
        for btn in self.rack_buttons:
            if btn != selected_btn:
                btn.setChecked(False)
        if selected_btn.isChecked():
            self.selected_rack = selected_btn.rack_data
        else:
            self.selected_rack = None

    def on_select(self):
        """선택 버튼 클릭"""
        if not self.selected_rack:
            QMessageBox.warning(self, '경고', '랙을 선택해주세요.')
            return

        try:
            from ppi_gui.payment import PaymentDialog
            payment = PaymentDialog(self.node, self.user_info, self.selected_rack, self)
            if payment.exec_() == QDialog.Accepted:
                self.accept()
        except Exception as e:
            print(f"[오류] {e}")
            import traceback
            traceback.print_exc()
