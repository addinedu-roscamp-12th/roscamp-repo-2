import os
import uuid
from datetime import datetime
from dateutil.relativedelta import relativedelta
from PyQt5.QtWidgets import QDialog, QMessageBox
from PyQt5.QtCore import pyqtSignal
from PyQt5 import uic
from ppi_gui.work_request import WorkRequestDialog

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
            get_package_share_directory('ppi_gui'),
            'ui', filename
        )
    else:
        return os.path.join(os.path.dirname(__file__), 'ui', filename)


# 기간별 금액
PERIOD_PRICE = {
    1: 30000,
    3: 80000,
    6: 150000,
}


class PaymentDialog(QDialog):
    payment_result_signal = pyqtSignal(dict)
    rack_update_signal    = pyqtSignal(dict)

    def __init__(self, node=None, user_info=None, rack_info=None, parent=None):
        super().__init__(parent)
        self.node       = node
        self.user_info  = user_info
        self.rack_info  = rack_info
        self.selected_period = None  # 선택한 기간 (개월 수)

        uic.loadUi(get_ui_path('payment.ui'), self)

        # 사용자 이름 설정
        if user_info:
            self.label_username.setText(f"{user_info.get('name', '')} 님")

        # 랙 정보 설정
        if rack_info:
            rack_id = rack_info.get('rack_id', '-')
            parts   = rack_id.split('-')
            zone    = parts[1] if len(parts) > 1 else '-'
            number  = parts[2] if len(parts) > 2 else '-'
            self.label_rack_id.setText(f"랙 ID: {rack_id.replace('rack-', '')}")
            self.label_rack_zone.setText(f"위치: {zone}구역 {number}번")

        # 시그널 연결
        self.payment_result_signal.connect(self._on_payment_result)
        self.rack_update_signal.connect(self._on_rack_update)

        # 기간 버튼 연결
        self.btn_1month.clicked.connect(lambda: self._on_period_selected(1))
        self.btn_3month.clicked.connect(lambda: self._on_period_selected(3))
        self.btn_6month.clicked.connect(lambda: self._on_period_selected(6))

        # 결제/취소 버튼 연결
        self.btn_pay.clicked.connect(self.on_pay)
        self.btn_cancel.clicked.connect(self.on_cancel)

    def ros_spin(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def _on_period_selected(self, period):
        """기간 선택 시 금액 업데이트"""
        self.selected_period = period

        # 버튼 체크 상태 업데이트
        self.btn_1month.setChecked(period == 1)
        self.btn_3month.setChecked(period == 3)
        self.btn_6month.setChecked(period == 6)

        # 금액 표시
        price = PERIOD_PRICE.get(period, 0)
        self.label_amount.setText(f"결제 금액: {price:,}원")

    def on_pay(self):
        """결제 버튼 클릭"""
        if not self.selected_period:
            QMessageBox.warning(self, '경고', '공유 기간을 선택해주세요.')
            return

        if not DB_AVAILABLE or self.node is None:
            QMessageBox.warning(self, '경고', 'DB에 연결할 수 없습니다.')
            return

        # 결제 정보 계산
        payment_id = f"pay-{str(uuid.uuid4())[:8]}"
        user_id    = self.user_info.get('user_id', '')
        rack_id    = self.rack_info.get('rack_id', '')
        amount     = PERIOD_PRICE.get(self.selected_period, 0)
        paid_at    = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        expired_at = (datetime.now() + relativedelta(months=self.selected_period)).strftime('%Y-%m-%d %H:%M:%S')

        db_client = DBClient(self.node)

        # 1. PAYMENT 테이블 INSERT
        payment_query = f"""
            INSERT INTO PAYMENT (payment_id, user_id, rack_id, amount, paid_at)
            VALUES ('{payment_id}', '{user_id}', '{rack_id}', {amount}, '{paid_at}')
        """
        db_client.execute_query_with_response(
            payment_query,
            callback=lambda r: self.payment_result_signal.emit({
                'response': r,
                'user_id': user_id,
                'rack_id': rack_id,
                'amount': amount,
                'paid_at': paid_at,
                'expired_at': expired_at,
                'period': self.selected_period,
                'db_client': db_client
            })
        )

    def _on_payment_result(self, data):
        """PAYMENT INSERT 결과 처리"""
        response = data['response']
        print(f"[PAYMENT INSERT 결과] {response}")  # 추가

        if not response['success']:
            QMessageBox.critical(self, '결제 실패', f'결제 중 오류가 발생했습니다.\n{response["message"]}')
            return

        # 2. RACK 테이블 UPDATE
        rack_query = f"""
            UPDATE `RACK` 
            SET status = '사용중',
                user_id = '{data["user_id"]}',
                share_period = {data["period"]},
                expired_at = '{data["expired_at"]}'
            WHERE rack_id = '{data["rack_id"]}'
        """
        print(f"[RACK UPDATE 쿼리] {rack_query}")  # 추가
        
        data['db_client'].execute_query_with_response(
            rack_query,
            callback=lambda r: self.rack_update_signal.emit({
                'response': r,
                'data': data
            })
        )

    def _on_rack_update(self, result):
        """RACK UPDATE 결과 처리"""
        response = result['response']
        data     = result['data']

        print(f"[RACK UPDATE 결과] {response}")  # 추가

        if not response['success']:
            QMessageBox.critical(self, '오류', f'랙 상태 업데이트 실패\n{response["message"]}')
            return

        # 3. 결제 완료 안내
        rack_id    = data['rack_id'].replace('rack-', '')
        parts      = data['rack_id'].split('-')
        zone       = parts[1] if len(parts) > 1 else '-'
        number     = parts[2] if len(parts) > 2 else '-'
        name       = self.user_info.get('name', '')
        period     = data['period']
        expired_at = data['expired_at']
        amount     = data['amount']

        # 4. 결제 완료 메시지 (스타일 적용)
        msg = QMessageBox(self)
        msg.setWindowTitle('결제 완료')
        msg.setText(
            f"결제가 완료되었습니다!\n\n"
            f"성명: {name}\n"
            f"랙 위치: {zone}구역 {number}번\n"
            f"공유 기간: {period}개월\n"
            f"만료일: {expired_at[:10]}\n"
            f"결제 금액: {amount:,}원"
        )
        msg.setStyleSheet("""
            QMessageBox {
                background-color: #0d1117;
            }
            QMessageBox QLabel {
                color: white;
                font-size: 14px;
            }
            QPushButton {
                background-color: #1f6feb;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px 20px;
                font-size: 13px;
            }
            QPushButton:hover {
                background-color: #388bfd;
            }
        """)
        msg.exec_()
        
        # 5. 작업 요청 화면으로 전환
        work = WorkRequestDialog(self.node, self.user_info, self.rack_info, self)
        work.exec_()
        self.accept()


    def on_cancel(self):
        """취소 버튼 클릭 → 랙 선택 화면으로"""
        self.reject()
