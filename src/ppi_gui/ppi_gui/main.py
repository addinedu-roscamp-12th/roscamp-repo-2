import sys
import os
import cv2
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow, QDialog, QMessageBox, QHeaderView, QTableWidgetItem, QListWidgetItem
from PyQt5.QtCore import QTimer, pyqtSignal, Qt
from PyQt5 import uic
from PyQt5.QtGui import QColor, QImage, QPixmap
from ppi_gui.rack_select import RackDialog
from ppi_gui.work_request import WorkRequestDialog

# ROS2 사용 가능 여부 확인
try:
    import rclpy
    from rclpy.node import Node
    from ament_index_python.packages import get_package_share_directory
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# DB 사용 가능 여부 확인
try:
    from db.db_client import DBClient
    DB_AVAILABLE = True
except ImportError:
    DB_AVAILABLE = False
    print("[경고] db 패키지를 찾을 수 없습니다.")


def get_ui_path(filename):
    """UI 파일 경로 반환"""
    if ROS2_AVAILABLE:
        return os.path.join(
            get_package_share_directory('ppi_gui'),
            'ui', filename
        )
    else:
        return os.path.join(os.path.dirname(__file__), 'ui', filename)


def generate_user_id(last_user_id):
    """마지막 user_id를 받아서 다음 user_id 생성"""
    if last_user_id is None:
        return 'user-001'
    try:
        num = int(last_user_id.split('-')[1])
        return f'user-{num + 1:03d}'
    except:
        return 'user-001'


class RegisterDialog(QDialog):
    register_result_signal = pyqtSignal(dict)
    insert_result_signal   = pyqtSignal(dict, str)

    def __init__(self, node=None, parent=None):
        super().__init__(parent)
        self.node = node
        uic.loadUi(get_ui_path('register.ui'), self)

        self._pending_name  = None
        self._pending_phone = None
        self._pending_pw    = None
        self._db_client     = None

        self.register_result_signal.connect(self._process_register_result)
        self.insert_result_signal.connect(self._on_insert_result)

        self.btn_register.clicked.connect(self.on_register)
        self.btn_back.clicked.connect(self.close)

    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def on_register(self):
        name  = self.input_name.text().strip()
        phone = self.input_phone.text().strip()
        pw    = self.input_pw.text().strip()

        if not name or not phone or not pw:
            QMessageBox.warning(self, '경고', '모든 항목을 입력해주세요.')
            return

        if len(pw) != 4 or not pw.isdigit():
            QMessageBox.warning(self, '경고', '비밀번호는 숫자 4자리여야 합니다.')
            return

        if not DB_AVAILABLE or self.node is None:
            QMessageBox.warning(self, '경고', 'DB에 연결할 수 없습니다.')
            return

        self._pending_name  = name
        self._pending_phone = phone
        self._pending_pw    = pw
        self._db_client     = DBClient(self.node)

        self._db_client.execute_query_with_response(
            "SELECT user_id FROM USER ORDER BY user_id DESC LIMIT 1",
            callback=lambda r: self.register_result_signal.emit(r)
        )

    def _process_register_result(self, response):
        try:
            last_user_id = None
            if response['success'] and response['result']:
                last_user_id = response['result'][0]['user_id']

            new_user_id = generate_user_id(last_user_id)
            created_at  = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            query = f"""
                INSERT INTO USER (user_id, phone, password, name, role, created_at)
                VALUES ('{new_user_id}', '{self._pending_phone}', '{self._pending_pw}',
                        '{self._pending_name}', 'user', '{created_at}')
            """
            self._db_client.execute_query_with_response(
                query,
                callback=lambda r: self.insert_result_signal.emit(r, new_user_id)
            )
        except Exception as e:
            QMessageBox.critical(self, '오류', f'회원가입 중 오류가 발생했습니다.\n{e}')
            print(f"[오류] {e}")

    def _on_insert_result(self, response, new_user_id):
        print(f"[INSERT 결과] {response}")
        if response['success']:
            QMessageBox.information(self, '완료', f'회원가입이 완료되었습니다.\n아이디: {new_user_id}')
            print(f"[회원가입 완료] {new_user_id} / {self._pending_name}")
            self.close()
        else:
            QMessageBox.critical(self, '오류', f'회원가입 실패: {response["message"]}')


class LoginDialog(QDialog):
    login_result_signal = pyqtSignal(dict)
    rack_check_signal   = pyqtSignal(dict)

    def __init__(self, node=None, parent=None):
        super().__init__(parent)
        self.node = node
        uic.loadUi(get_ui_path('login.ui'), self)

        self.login_success = False
        self.user_role     = None
        self.user_info     = None

        self.login_result_signal.connect(self._process_login_result)
        self.rack_check_signal.connect(self._process_rack_check)

        self.btn_login.clicked.connect(self.on_login)
        self.btn_register.clicked.connect(self.on_register)

    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def on_login(self):
        phone = self.input_phone.text().strip()
        pw    = self.input_pw.text().strip()

        if not phone or not pw:
            QMessageBox.warning(self, '경고', '전화번호와 비밀번호를 입력해주세요.')
            return

        if not DB_AVAILABLE or self.node is None:
            QMessageBox.warning(self, '경고', 'DB에 연결할 수 없습니다.')
            return

        db_client = DBClient(self.node)
        query = f"""
            SELECT user_id, name, role
            FROM USER
            WHERE phone = '{phone}' AND password = '{pw}'
        """
        db_client.execute_query_with_response(
            query,
            callback=lambda r: self.login_result_signal.emit(r)
        )

    def _process_login_result(self, response):
        if response['success'] and response['result']:
            user = response['result'][0]
            self.user_info     = user
            self.user_role     = user['role']
            self.login_success = True
            print(f"[로그인 성공] {user['name']} / {user['role']}")

            # admin이면 바로 accept
            if user['role'] == 'admin':
                self.accept()
                return

            # user면 결제 여부 확인
            db_client = DBClient(self.node)
            query = f"""
                SELECT rack_id, expired_at 
                FROM `RACK` 
                WHERE user_id = '{user['user_id']}'
                AND status = '사용중'
                AND expired_at > NOW()
                LIMIT 1
            """
            db_client.execute_query_with_response(
                query,
                callback=lambda r: self.rack_check_signal.emit(r)
            )
        else:
            QMessageBox.warning(self, '로그인 실패', '전화번호 또는 비밀번호가 올바르지 않습니다.')

    def _process_rack_check(self, response):
        """결제 여부 확인 후 화면 전환"""
        if response['success'] and response['result']:
            # 결제한 랙이 있음 → rack_info 저장
            rack = response['result'][0]
            self.user_info['rack_id']    = rack['rack_id']
            self.user_info['expired_at'] = rack['expired_at']
            self.user_info['has_rack']   = True
            print(f"[결제된 랙 있음] {rack['rack_id']} / 만료: {rack['expired_at']}")
        else:
            # 결제한 랙 없음
            self.user_info['has_rack'] = False
            print("[결제된 랙 없음] → 랙 선택 화면으로")

        self.accept()
    def on_register(self):
        register_dialog = RegisterDialog(self.node, self)
        register_dialog.exec_()


class AdminWindow(QMainWindow):
    rack_data_signal = pyqtSignal(list)
    user_data_signal = pyqtSignal(list)
    robot_data_signal = pyqtSignal(list)
    log_data_signal   = pyqtSignal(list)

    """관리자 화면"""
    def __init__(self, node=None):
        super().__init__()
        self.node = node
        uic.loadUi(get_ui_path('work_dashboard.ui'), self)

        # 카메라 스레드 시작
        try:
            from ppi_gui.camera_thread import CameraThread
            self.camera_thread = CameraThread('192.168.1.65', 9000)
            self.camera_thread.frame_received.connect(self.update_frame)
            self.camera_thread.start()
        except Exception as e:
            print(f"[카메라 오류] {e}")
        
        # 테이블 설정
        self._setup_robot_table()
        self._setup_rack_table()
        self._setup_user_table()

        # 시그널 연결
        self.rack_data_signal.connect(self._fill_rack_table)
        self.user_data_signal.connect(self._fill_user_table)
        self.robot_data_signal.connect(self._fill_robot_table)
        self.log_data_signal.connect(self._fill_log_list)

        # 사이드바 버튼 클릭 시 페이지 전환 + 데이터 새로고침
        self.btn_task.clicked.connect(self._on_click_dashboard)
        self.btn_rack.clicked.connect(self._on_click_rack)
        self.btn_user.clicked.connect(self._on_click_user)

         # 대시보드 진입 시 로봇 데이터도 로드
        self._load_robot_data()
        self._load_log_data()

    def _on_click_dashboard(self):
        self.stacked_widget.setCurrentWidget(self.page_dashboard)
        self._load_robot_data()
        self._load_log_data()

    def _on_click_rack(self):
        self.stacked_widget.setCurrentWidget(self.page_rack)
        self._load_rack_data()

    def _on_click_user(self):
        self.stacked_widget.setCurrentWidget(self.page_user)
        self._load_user_data()

    def _setup_robot_table(self):
        header = self.robot_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)  # 모든 컬럼 균등 배치
        self.robot_table.verticalHeader().setVisible(False)  # 행 번호 숨기기

        # 행 높이를 테이블 전체 높이에 맞춰 균등 분배
        self.robot_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def _load_robot_data(self):
        """DB에서 로봇 데이터 로드"""
        if not DB_AVAILABLE or self.node is None:
            print("[경고] DB에 연결할 수 없습니다.")
            return

        db_client = DBClient(self.node)
        db_client.execute_query_with_response(
            "SELECT robot_id, type, status, location, battery FROM `ROBOT` ORDER BY robot_id",
            callback=lambda r: self.robot_data_signal.emit(
                r['result'] if r['success'] and r['result'] else []
            )
        )

    def _fill_robot_table(self, robot_list):
        """robot_table 채우기"""
        self.robot_table.setRowCount(len(robot_list))

        for row, robot in enumerate(robot_list):
            robot_id = robot.get('robot_id', '-')
            r_type   = robot.get('type', '-')
            status   = robot.get('status', '-')
            location   = robot.get('location', '-')
            battery   = robot.get('battery', '-')

            values = [robot_id, r_type, status, location, battery]

            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setTextAlignment(Qt.AlignCenter)
                self.robot_table.setItem(row, col, item)

            # 배터리 컬럼 (색상 적용)
            battery_text = f"{battery}%" if battery is not None else '-'
            battery_item = QTableWidgetItem(battery_text)
            battery_item.setTextAlignment(Qt.AlignCenter)

            if battery is not None:
                try:
                    battery_value = float(battery)  # 문자열이어도 숫자로 변환
                except (ValueError, TypeError):
                    battery_value = None

                if battery_value >= 80:
                    color = QColor('#3fb950')   # 초록 (충분)
                elif battery_value >= 60:
                    color = QColor('#d29922')   # 노랑 (보통)
                else:
                    color = QColor('#f85149')   # 빨강 (부족)
                battery_item.setForeground(color)

            self.robot_table.setItem(row, 4, battery_item)

    def _setup_rack_table(self):
        header = self.rack_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        self.rack_table.verticalHeader().setVisible(False)
        self.rack_table.setEditTriggers(self.rack_table.NoEditTriggers)

        # 행 높이를 테이블 전체 높이에 맞춰 균등 분배
        self.rack_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
    
    def _load_rack_data(self):
        """DB에서 랙 데이터 로드"""
        if not DB_AVAILABLE or self.node is None:
            print("[경고] DB에 연결할 수 없습니다.")
            return

        db_client = DBClient(self.node)
        db_client.execute_query_with_response(
            "SELECT rack_id, location, status, user_id, share_period, expired_at FROM `RACK` ORDER BY rack_id",
            callback=lambda r: self.rack_data_signal.emit(
                r['result'] if r['success'] and r['result'] else []
            )
        )

    def _fill_rack_table(self, rack_list):
        """rack_table 채우기"""
        self.rack_table.setRowCount(len(rack_list))

        status_colors = {
            '사용가능': QColor('#3fb950'),  # 초록
            '점검중':   QColor('#d29922'),  # 주황
            '사용중':   QColor('#f85149'),  # 빨강
        }

        for row, rack in enumerate(rack_list):
            rack_id = rack.get('rack_id', '-')
            parts   = rack_id.split('-')
            zone    = parts[1] if len(parts) > 1 else '-'
            number  = parts[2] if len(parts) > 2 else '-'
            position_text = f"{zone}구역 {number}번"

            status       = rack.get('status', '-')
            user_id      = rack.get('user_id') or '-'
            period       = rack.get('share_period')
            period_text  = f"{period}개월" if period else '-'
            expired_at   = rack.get('expired_at')
            expired_text = str(expired_at)[:10] if expired_at else '-'

            values = [
                rack_id.replace('rack-', ''),
                position_text,
                status,
                user_id,
                period_text,
                expired_text
            ]

            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)  # 중앙 정렬

                # 상태 컬럼(col == 2)에만 색상 적용
                if col == 2 and status in status_colors:
                    item.setForeground(status_colors[status])
                    
                self.rack_table.setItem(row, col, item)

    def _setup_user_table(self):
        header = self.user_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        self.user_table.verticalHeader().setVisible(False)
        self.user_table.setEditTriggers(self.user_table.NoEditTriggers)

        # 행 높이를 테이블 전체 높이에 맞춰 균등 분배
        self.user_table.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)

    def _load_user_data(self):
        """DB에서 사용자 데이터 로드"""
        if not DB_AVAILABLE or self.node is None:
            print("[경고] DB에 연결할 수 없습니다.")
            return

        db_client = DBClient(self.node)
        db_client.execute_query_with_response(
            "SELECT user_id, name, phone, role, created_at FROM `USER` ORDER BY user_id",
            callback=lambda r: self.user_data_signal.emit(
                r['result'] if r['success'] and r['result'] else []
            )
        )

    def _fill_user_table(self, user_list):
        """user_table 채우기"""
        self.user_table.setRowCount(len(user_list))

        for row, user in enumerate(user_list):
            user_id    = user.get('user_id', '-')
            name       = user.get('name', '-')
            phone      = user.get('phone', '-')
            role       = user.get('role', '-')
            created_at = user.get('created_at')
            created_text = str(created_at)[:10] if created_at else '-'

            values = [user_id, name, phone, role, created_text]

            for col, value in enumerate(values):
                item = QTableWidgetItem(str(value))
                item.setTextAlignment(Qt.AlignCenter)
                self.user_table.setItem(row, col, item)

    def _load_log_data(self):
        """DB에서 로그 데이터 로드"""
        if not DB_AVAILABLE or self.node is None:
            print("[경고] DB에 연결할 수 없습니다.")
            return

        db_client = DBClient(self.node)
        db_client.execute_query_with_response(
            "SELECT log_id, robot_id, order_id, type, message FROM `LOG` ORDER BY log_id ASC",
            callback=lambda r: self.log_data_signal.emit(
                r['result'] if r['success'] and r['result'] else []
            )
        )

    def _fill_log_list(self, log_list):
        """log_list (QListWidget) 채우기"""
        self.log_list.clear()

        for log in log_list:
            log_id   = log.get('log_id', '-')
            robot_id = log.get('robot_id', '-')
            order_id = log.get('order_id', '-')[:8]  # 앞 8자리만
            log_type = log.get('type', '-')
            message  = log.get('message', '-')

            text = f"[{log_id}] [{robot_id}] [{order_id}] - {message}({log_type})"

            item = QListWidgetItem(text)

            # type에 따라 색상 구분
            if log_type == '적재':
                item.setForeground(QColor('#3fb950'))   # 초록
            elif log_type == '주행':
                item.setForeground(QColor('#58a6ff'))   # 파랑
            elif log_type == '오류':
                item.setForeground(QColor('#f85149'))   # 빨강
            else:
                item.setForeground(QColor('#c9d1d9'))   # 기본 흰색

            self.log_list.addItem(item)

        # 최신 로그로 스크롤
        self.log_list.scrollToBottom()

    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def update_frame(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch  = frame_rgb.shape
        qt_image  = QImage(frame_rgb.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap    = QPixmap.fromImage(qt_image)
        self.camera_view.setPixmap(
            pixmap.scaled(
                self.camera_view.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation
            )
        )

    def closeEvent(self, event):
        try:
            self.camera_thread.stop()
        except:
            pass
        event.accept()


def main():
    if ROS2_AVAILABLE:
        rclpy.init()
        node = Node('bbi_gui_node')
    else:
        node = None

    app = QApplication(sys.argv)

    # 전역 ROS2 spin 타이머 (하나만!)
    if ROS2_AVAILABLE and node:
        ros_timer = QTimer()
        ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
        ros_timer.start(100)

   # 로그인 루프 (로그아웃 시 다시 로그인 화면으로)
    while True:
        login = LoginDialog(node)
        if login.exec_() != QDialog.Accepted or not login.login_success:
            break  # 창 닫으면 종료

        if login.user_role == 'admin':
            window = AdminWindow(node)
            window.show()
            sys.exit(app.exec_())
        else:
            if login.user_info.get('has_rack'):
                rack_info = {'rack_id': login.user_info['rack_id']}
                window = WorkRequestDialog(node, login.user_info, rack_info)
            else:
                window = RackDialog(node, login.user_info)

            result = window.exec_()
            # reject() = 로그아웃 → while 루프 다시 돌아서 로그인 화면 표시
            # accept() = 정상 종료 → break
            if result == QDialog.Accepted:
                break
    sys.exit(0)

if __name__ == '__main__':
    main()
