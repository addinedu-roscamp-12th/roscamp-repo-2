"""
JetCobot2 ROS2 액션 서버 (Jazzy)
액션명: jetcobot2/command
"""
 
import time
import json
import logging
 
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
 
from pymycobot.mycobot280 import MyCobot280
from pymycobot.genre import Angle
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
 
from jetcobot2.rack_detector import RackSpaceDetector
from jetcobot_interfaces.action import RobotCommand
from jetcobot2.qr_detector import scan_qr
 
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("jetcobot2")
 
# ──────────────────────────────────────────────
# 모든 좌표를 send_angles 기준으로 정의
# ──────────────────────────────────────────────
 
# 입고용 Place 좌표 (모두 각도 - do_load)
LOAD_PLACE_COORDS = {
    'floor2_left': {
        'approach': [94.3, -60.2, -88.68, 144.75, -2.63, -48.51],
        'place':    [94.3, -60.2, -88.68, 144.75, -2.63, -48.51],  # ← 실제 각도로 교체
        'retreat':  [94.3, -60.2, -88.68, 144.75, -2.63, -48.51],  # ← 실제 각도로 교체
    },
    'floor2_right': {
        'approach': [75.93, -66.7, -66.26, 126.65, 15.2, -42.01],
        'place':    [75.93, -66.7, -66.26, 126.65, 15.2, -42.01],  # ← 실제 각도로 교체
        'retreat':  [75.93, -66.7, -66.26, 126.65, 15.2, -42.01],  # ← 실제 각도로 교체
    },
}
 
# 랙 스캔 위치 (각도만 사용 - do_load)
RACK_SCAN_ANGLES = [78.04, -102.21, -13.27, 118.91, 98.7, -42.97]
 
# QR 스캔 위치 (각도만 사용 - do_unload)
QR_SCAN_ANGLES = {
    'floor2_left':  [89.64, -132.71, 55.98, 75.76, 0.35, -45.61],
    'floor2_right': [62.31, -88.06, -40.51, 122.25, 24.69, -44.56],
}
 
# 출고용 Pick 좌표 (모두 각도 - do_unload)
UNLOAD_PICK_COORDS = {
    'floor2_left': {
        'approach': [87.27, -76.28, -53.17, 130.86, 0.43, -45.43],
        'pick':     [87.27, -76.28, -53.17, 130.86, 0.43, -45.43],  # ← 실제 각도로 교체
        'retreat':  [87.27, -76.28, -53.17, 130.86, 0.43, -45.43],  # ← 실제 각도로 교체
    },
    'floor2_right': {
        'approach': [111.26, -100.19, -2.1, 103.97, 61.52, -36.73],
        'pick':     [111.26, -100.19, -2.1, 103.97, 61.52, -36.73],  # ← 실제 각도로 교체
        'retreat':  [111.26, -100.19, -2.1, 103.97, 61.52, -36.73],  # ← 실제 각도로 교체
    },
}
 
# 운반대 Place 좌표 (모두 각도 - do_unload)
CART_PLACE_COORDS = {
    'approach': [88.33, -20.91, -121.55, 44.03, -4.3, -40.86],
    'place':    [88.33, -20.91, -121.55, 44.03, -4.3, -40.86],  # ← 실제 각도로 교체
    'up':       [88.33, -20.91, -121.55, 44.03, -4.3, -40.86],  # ← 실제 각도로 교체
}

# 출고 컨베이어 Place 좌표 (모두 각도)
CONVEYOR_PLACE_COORDS = {
    'approach': [0, 0, 0, 0, 0, 0],  # ← 측정 필요
    'place':    [0, 0, 0, 0, 0, 0],  # ← 측정 필요
    'up':       [0, 0, 0, 0, 0, 0],  # ← 측정 필요
}

# 운반대 카메라 인식 지점 (outbound_pickup용)
OUTBOUND_CAM_ANGLES = [0, 0, 0, 0, 0, 0]  # ← 측정 필요

# 운반대 Pick 좌표 (outbound_pickup용)
OUTBOUND_PICK_COORDS = {
    'approach': [0, 0, 0, 0, 0, 0],  # ← 측정 필요
    'pick':     [0, 0, 0, 0, 0, 0],  # ← 측정 필요
    'retreat':  [0, 0, 0, 0, 0, 0],  # ← 측정 필요
}
 
 
class RobotController:
    def __init__(self):
        self.mc             = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.thread_lock = True
        self.rack_detector  = RackSpaceDetector(min_contour_area=3000)
        logger.info("로봇 연결 완료")
 
    def go_home(self, feedback_cb):
        """홈 자세로 복귀 + 그리퍼 열기"""
        feedback_cb("홈 위치로 이동")
        self.mc.send_angles([85, 0, 0, 0, 0, -47], 50)
        self.mc.set_gripper_value(100, 50)
        time.sleep(2)
 
    # ──────────────────────────────────────────
    # send_angles를 쓰는 경우:
    #   고정된 위치로 이동 (QR 스캔, Pick, Place, 홈 등)
    #
    # send_coords + ctrl_moving을 쓰는 경우:
    #   cx, cy 등 동적으로 계산된 위치로 이동
    #   (현재 do_load의 운반대 박스 Pick만 해당)
    # ──────────────────────────────────────────
 
    def ctrl_moving(self, tar_coo):
        """XYZ 좌표 이동 + 오차 보정 (동적 위치에만 사용)"""
        time.sleep(1)
        self.mc.send_coords(tar_coo, 60)
        time.sleep(0.5)

        MAX_ITER  = 3      # 최대 보정 횟수
        THRESHOLD = 2.0    # 허용 오차 (mm)

        for i in range(MAX_ITER):
            coords = np.array(self.mc.get_coords())
            error  = coords - np.array(tar_coo)
            print(f"[{i+1}회] 오차: {np.round(error[:3], 2)}")

            # XYZ 오차가 허용 범위 안이면 조기 종료
            if np.all(np.abs(error[:3]) < THRESHOLD):
                print(f"[{i+1}회] 오차 허용 범위 내 → 보정 완료")
                break

            # XYZ만 보정
            corrected = np.array(tar_coo, dtype=float)
            for j in range(3):
                corrected[j] = tar_coo[j] - error[j]

            self.mc.send_coords(corrected.tolist(), 20)
            time.sleep(0.3)

        coords = np.array(self.mc.get_coords())
        error  = coords - np.array(tar_coo)
        print(f"최종 오차: {np.round(error[:3], 2)}")
 
    def _move_angles(self, angles, speed=30, wait=1.5):
        """send_angles 래퍼 - 이동 후 대기"""
        self.mc.send_angles(angles, speed)
        time.sleep(wait)
 
    def _scan_rack(self, feedback_cb, node=None):
        """랙 스캔 위치로 이동 후 빈 공간 감지"""
        feedback_cb("랙 스캔 위치로 이동")
        # ✅ send_angles만 사용
        self._move_angles(RACK_SCAN_ANGLES, speed=30, wait=2)
 
        feedback_cb("랙 빈 공간 스캔 중...")
        time.sleep(1)
        frame = node._last_frame if node is not None else None
 
        if frame is None:
            feedback_cb("카메라 프레임 없음 → 스캔 실패")
            return None
 
        result = self.rack_detector.detect(frame)
        feedback_cb(
            f"스캔 결과 → 좌측: {'비어있음' if result['left_empty'] else '박스있음'}, "
            f"우측: {'비어있음' if result['right_empty'] else '박스있음'}"
        )
 
        if result['left_empty']:
            return 'floor2_left'
        elif result['right_empty']:
            return 'floor2_right'
        else:
            return None
 
    def do_load(self, feedback_cb, detected=False, cx=0.0, cy=0.0,
            area=0.0, hor=0.0, ver=0.0, ar=0.0, node=None):
        """
        입고 동작
        ✅ send_angles: 홈, 카메라 인식 지점, 랙 스캔, Place
        ✅ cx,cy 기반 1회 보정: 운반대 박스 Pick
        """

        # [1] 홈 자세
        feedback_cb("[1] 홈 자세 이동")
        self.go_home(feedback_cb)

        # [2] 카메라 인식 지점 이동
        feedback_cb("[2] 카메라 인식 지점 이동")
        self._move_angles([95.09, -114.34, 108.28, -76.02, -5.53, -37.17], speed=50, wait=2)

        # [3] 박스 감지 확인
        feedback_cb("[3-1] 박스 감지 확인")
        time.sleep(1)
        feedback_cb(f"[3-2] 박스 감지 → cx={cx:.1f}, cy={cy:.1f}, area={area:.0f}")

        if not detected:
            feedback_cb("박스 미감지 → 픽업 중단")
            self.go_home(feedback_cb)
            return

        # [4] cx, cy 기반 목표 좌표 보정
        feedback_cb("[4] 박스 위치 보정 계산")

        # 화면 중앙 기준 오차 계산
        CENTER_X     = 320    # 카메라 해상도 640 기준
        CENTER_Y     = 240    # 카메라 해상도 480 기준
        PIXEL_TO_MM  = 0.5   # 1픽셀 = 0.5mm (캘리브레이션 필요)

        error_x = cx - CENTER_X  # 픽셀 단위 오차
        error_y = cy - CENTER_Y

        # 기본 목표 좌표
        base_coords = [56.6, 239.3, 147.7, -178.92, -8.0, 40.24]

        # cx, cy 오차로 XY 보정
        corrected_coords = base_coords.copy()
        corrected_coords[0] -= error_x * PIXEL_TO_MM  # X축 보정
        corrected_coords[1] += error_y * PIXEL_TO_MM  # Y축 보정

        feedback_cb(
            f"[4] 보정값 → error_x={error_x:.1f}px, error_y={error_y:.1f}px "
            f"→ X보정={-error_x * PIXEL_TO_MM:.1f}mm, Y보정={error_y * PIXEL_TO_MM:.1f}mm"
        )

        # [5] 보정된 좌표로 이동
        feedback_cb("[5] 보정된 목표 좌표로 이동")
        self.ctrl_moving(corrected_coords)
        time.sleep(0.5)

        # [6] 그리퍼 열기
        feedback_cb("[6] 그리퍼 열기")
        self.mc.set_gripper_value(100, 50)
        time.sleep(0.5)

        # [7] 하강
        feedback_cb("[7] 하강")
        self.mc.send_coords([61.0, 243.5, 115.6, -178.65, -4.93, 36.74], 30)
        time.sleep(0.5)

        # [8] 그리퍼 닫기 (Pick)
        feedback_cb("[8] 그리퍼 닫기 (Pick)")
        self.mc.set_gripper_value(0, 50)
        time.sleep(0.7)

        # [9] 상승
        feedback_cb("[9] 상승")
        self.mc.send_coords([60.9, 235.8, 155.4, -176.67, -6.93, 43.56], 30)
        time.sleep(0.5)

        # [10] 랙 스캔
        feedback_cb("[10] 랙 빈 공간 스캔")
        target_slot = self._scan_rack(feedback_cb, node=node)

        if target_slot is None:
            feedback_cb("빈 공간 없음 → Place 중단")
            self.go_home(feedback_cb)
            return

        feedback_cb(f"빈 공간 확인 → {target_slot} 에 적재")
        coords = LOAD_PLACE_COORDS[target_slot]

        # [11] 접근 위치
        feedback_cb("[11] 접근 위치 이동")
        self._move_angles(coords['approach'], speed=30, wait=1.5)

        # [12] Place 위치
        feedback_cb("[12] Place 위치 이동")
        self._move_angles(coords['place'], speed=20, wait=1.5)

        # [13] 그리퍼 열기 (Place)
        feedback_cb("[13] 그리퍼 열기 (Place)")
        self.mc.set_gripper_value(100, 50)
        time.sleep(0.7)

        # [14] 후진
        feedback_cb("[14] 후진")
        self._move_angles(coords['retreat'], speed=20, wait=1.5)

        # [15] 홈 복귀
        feedback_cb("[15] 홈 자세 복귀")
        self.go_home(feedback_cb)

        feedback_cb("load 완료")
        logger.info(f"load 동작 완료 → {target_slot}")
 
    def do_unload(self, feedback_cb, detected=False, cx=0.0, cy=0.0,
                  area=0.0, hor=0.0, ver=0.0, ar=0.0,
                  target_item: str = "물병", node=None):
        """
        출고 동작
        ✅ send_angles: 모든 이동 (QR 스캔, Pick, Place, 홈)
        ⚠️  ctrl_moving: 없음 (모두 고정 위치)
        """
        # [1] 홈 자세 ✅ send_angles
        feedback_cb("[1] 홈 자세 이동")
        self.go_home(feedback_cb)
 
        # [2~5] 왼쪽 → 오른쪽 QR 스캔 ✅ send_angles
        found    = None
        position = None
 
        for slot in ['floor2_left', 'floor2_right']:
            label = '왼쪽' if slot == 'floor2_left' else '오른쪽'
            feedback_cb(f"[2] {label} QR 스캔 위치로 이동")
 
            # ✅ send_angles만 사용
            self._move_angles(QR_SCAN_ANGLES[slot], speed=30, wait=2)
 
            for i in range(3):
                feedback_cb(f"[3] {label} QR 스캔 중... ({i+1}/3)")
                frame = node._last_frame if node is not None else None
 
                if frame is None:
                    feedback_cb("카메라 프레임 없음")
                    time.sleep(1)
                    continue
 
                result = scan_qr(frame)
 
                if result and result.get('item') == target_item:
                    found    = result
                    position = slot
                    feedback_cb(
                        f"QR 감지 성공! box_id={result['box_id']}, "
                        f"item={result['item']}, position={position}"
                    )
                    break
                else:
                    if result:
                        feedback_cb(f"다른 박스: {result.get('item')} → {target_item} 찾는 중")
                    else:
                        feedback_cb(f"QR 미감지 → 재시도 {i+1}/3")
                    time.sleep(1)
 
            if found:
                break
 
        if found is None:
            feedback_cb(f"'{target_item}' 박스를 찾지 못함 → 중단")
            self.go_home(feedback_cb)
            return
 
        pick_coords = UNLOAD_PICK_COORDS[position]
        feedback_cb(f"'{target_item}' 박스 위치: {position}")
 
        # [7] Pick 접근 ✅ send_angles
        feedback_cb("[7] Pick 접근 위치 이동")
        self._move_angles(pick_coords['approach'], speed=30, wait=1.5)
 
        # [8] Pick 위치 ✅ send_angles
        feedback_cb("[8] Pick 위치 이동")
        self._move_angles(pick_coords['pick'], speed=20, wait=1.5)
 
        # 그리퍼 열기
        feedback_cb("[8-1] 그리퍼 열기")
        self.mc.set_gripper_value(100, 50)
        time.sleep(0.5)
 
        # 그리퍼 닫기 (Pick)
        feedback_cb("[8-2] 그리퍼 닫기 (Pick)")
        self.mc.set_gripper_value(0, 50)
        time.sleep(0.7)
 
        # [9] 후진 ✅ send_angles
        feedback_cb("[9] 후진")
        self._move_angles(pick_coords['retreat'], speed=20, wait=1.5)
 
        # [10] 운반대 접근 ✅ send_angles
        feedback_cb("[10] 운반대 접근 위치로 이동")
        self._move_angles(CART_PLACE_COORDS['approach'], speed=30, wait=1.5)
 
        # [11] 운반대 Place 위치 ✅ send_angles
        feedback_cb("[11] 운반대 Place 위치로 이동")
        self._move_angles(CART_PLACE_COORDS['place'], speed=20, wait=1.5)
 
        # 그리퍼 열기 (Place)
        feedback_cb("[11-1] 그리퍼 열기 (Place)")
        self.mc.set_gripper_value(100, 50)
        time.sleep(0.7)
 
        # [12] 상승 ✅ send_angles
        feedback_cb("[12] 상승")
        self._move_angles(CART_PLACE_COORDS['up'], speed=20, wait=1.5)
 
        # [13] 홈 복귀 ✅ send_angles
        feedback_cb("[13] 홈 자세 복귀")
        self.go_home(feedback_cb)
 
        feedback_cb("unload 완료")
        logger.info(f"unload 동작 완료 → {target_item} ({position})")
    
    def outbound_pickup(self, feedback_cb, detected=False, cx=0.0, cy=0.0,
                    area=0.0, hor=0.0, ver=0.0, ar=0.0):
        """
        출고 컨베이어 시나리오
        운반대에 실린 박스 → 컨베이어 벨트로 Pick & Place
        1.  홈 자세
        2.  카메라 인식 지점으로 이동 (운반대 위)
        3.  박스 감지 확인
        4.  Pick 접근 위치로 이동
        5.  그리퍼 열기
        6.  Pick 위치로 하강
        7.  그리퍼 닫기 (Pick)
        8.  상승 (후진)
        9.  컨베이어 접근 위치로 이동
        10. 컨베이어 Place 위치로 이동
        11. 그리퍼 열기 (Place)
        12. 상승
        13. 홈 복귀
        """

        # [1] 홈 자세
        feedback_cb("[1] 홈 자세 이동")
        self.go_home(feedback_cb)

        # [2] 카메라 인식 지점으로 이동 ✅ send_angles
        feedback_cb("[2] 카메라 인식 지점 이동")
        self._move_angles(OUTBOUND_CAM_ANGLES, speed=50, wait=2)

        # [3] 박스 감지 확인
        feedback_cb("[3-1] 박스 감지 확인")
        time.sleep(1)
        feedback_cb(f"[3-2] 박스 감지 → cx={cx:.1f}, cy={cy:.1f}, area={area:.0f}")

        if not detected:
            feedback_cb("박스 미감지 → 픽업 중단")
            self.go_home(feedback_cb)
            return

        # [4] cx, cy 기반 Pick 좌표 보정
        feedback_cb("[4] 박스 위치 보정 계산")

        CENTER_X    = 320
        CENTER_Y    = 240
        PIXEL_TO_MM = 0.5  # ← 캘리브레이션 필요

        error_x = cx - CENTER_X
        error_y = cy - CENTER_Y

        feedback_cb(
            f"[4] 보정값 → error_x={error_x:.1f}px, error_y={error_y:.1f}px"
        )

        # [5] Pick 접근 위치로 이동 ✅ send_angles
        feedback_cb("[5] Pick 접근 위치 이동")
        self._move_angles(OUTBOUND_PICK_COORDS['approach'], speed=30, wait=1.5)

        # [6] Pick 위치로 이동 ✅ send_angles
        feedback_cb("[6] Pick 위치 이동")
        self._move_angles(OUTBOUND_PICK_COORDS['pick'], speed=20, wait=1.5)

        # [7] 그리퍼 열기
        feedback_cb("[7] 그리퍼 열기")
        self.mc.set_gripper_value(100, 50)
        time.sleep(0.5)

        # [8] 그리퍼 닫기 (Pick)
        feedback_cb("[8] 그리퍼 닫기 (Pick)")
        self.mc.set_gripper_value(0, 50)
        time.sleep(0.7)

        # [9] 후진 ✅ send_angles
        feedback_cb("[9] 후진")
        self._move_angles(OUTBOUND_PICK_COORDS['retreat'], speed=20, wait=1.5)

        # [10] 컨베이어 접근 위치로 이동 ✅ send_angles
        feedback_cb("[10] 컨베이어 접근 위치로 이동")
        self._move_angles(CONVEYOR_PLACE_COORDS['approach'], speed=30, wait=1.5)

        # [11] 컨베이어 Place 위치로 이동 ✅ send_angles
        feedback_cb("[11] 컨베이어 Place 위치로 이동")
        self._move_angles(CONVEYOR_PLACE_COORDS['place'], speed=20, wait=1.5)

        # [12] 그리퍼 열기 (Place)
        feedback_cb("[12] 그리퍼 열기 (Place)")
        self.mc.set_gripper_value(100, 50)
        time.sleep(0.7)

        # [13] 상승 ✅ send_angles
        feedback_cb("[13] 상승")
        self._move_angles(CONVEYOR_PLACE_COORDS['up'], speed=20, wait=1.5)

        # [14] 홈 복귀 ✅ send_angles
        feedback_cb("[14] 홈 자세 복귀")
        self.go_home(feedback_cb)

        feedback_cb("outbound_pickup 완료")
        logger.info("outbound_pickup 동작 완료")
 
 
class JetCobot2ActionServer(Node):
    def __init__(self):
        super().__init__('jetcobot2_action_server')
        self._bridge     = CvBridge()
        self._last_frame = None
 
        self.robot = RobotController()
 
        self._action_server = ActionServer(
            self,
            RobotCommand,
            'jetcobot2/command',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
 
        self.create_subscription(Image,  'camera/image',   self._camera_callback, 10)
        self.create_subscription(String, '/manual_command', self._manual_cb,       10)
        self.get_logger().info("액션 서버 시작: jetcobot2/command")
 
    def _camera_callback(self, msg):
        self._last_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
 
    def _manual_cb(self, msg):
        action_type = msg.data
        self.get_logger().info(f"수동 명령 수신: {action_type}")
        def fb(m): logger.info(f"[manual] {m}")
        if action_type == "do_load":
            self.robot.do_load(fb)
        elif action_type == "do_unload":
            self.robot.do_unload(fb)
        elif action_type == "outbound_pickup":
            self.robot.outbound_pickup(fb)
 
    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Goal 수신 → action_type={goal_request.action_type}, "
            f"task_id={goal_request.task_id}"
        )
        return GoalResponse.ACCEPT
 
    def cancel_callback(self, goal_handle):
        self.get_logger().info("취소 요청 수신")
        return CancelResponse.ACCEPT
 
    def execute_callback(self, goal_handle):
        req         = goal_handle.request
        task_id     = req.task_id
        action_type = req.action_type
        parameters  = json.loads(req.parameters_json) if req.parameters_json else {}
 
        self.get_logger().info(
            f"[{task_id}] 실행 시작 → action_type={action_type}, params={parameters}"
        )
 
        def send_feedback(status: str):
            fb        = RobotCommand.Feedback()
            fb.status = status
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f"[{task_id}] Feedback: {status}")
 
        result      = RobotCommand.Result()
        detected    = parameters.get("detected",    False)
        result_cx   = parameters.get("result_cx",   0.0)
        result_cy   = parameters.get("result_cy",   0.0)
        result_area = parameters.get("result_area", 0.0)
        result_hor  = parameters.get("result_hor",  0.0)
        result_ver  = parameters.get("result_ver",  0.0)
        result_ar   = parameters.get("result_ar",   0.0)
 
        try:
            if goal_handle.is_cancel_requested:
                result.event   = "canceled"
                result.message = "취소됨"
                goal_handle.canceled()
                return result
 
            if action_type == "do_load":
                self.robot.do_load(
                    send_feedback, detected,
                    result_cx, result_cy, result_area,
                    result_hor, result_ver, result_ar,
                    node=self
                )
            elif action_type == "do_unload":
                target_item = parameters.get("target_item", "물병")
                self.robot.do_unload(
                    send_feedback, detected,
                    result_cx, result_cy, result_area,
                    result_hor, result_ver, result_ar,
                    target_item=target_item,
                    node=self
                )
            elif action_type == "outbound_pickup": 
                self.robot.outbound_pickup(
                    send_feedback, detected,
                    result_cx, result_cy, result_area,
                    result_hor, result_ver, result_ar
                )
            else:
                self.get_logger().warning(f"[{task_id}] 알 수 없는 action_type: {action_type}")
                result.event   = "error"
                result.message = f"Unknown action_type: {action_type}"
                goal_handle.abort()
                return result
 
            result.event   = "done"
            result.message = f"action '{action_type}' 완료"
            goal_handle.succeed()
            self.get_logger().info(f"[{task_id}] 완료 → event=done 전송")
 
        except Exception as e:
            self.get_logger().error(f"[{task_id}] 오류: {e}")
            result.event   = "error"
            result.message = str(e)
            goal_handle.abort()
 
        return result
 
 
def main(args=None):
    rclpy.init(args=args)
    node     = JetCobot2ActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()