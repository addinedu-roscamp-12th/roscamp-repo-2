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
from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Empty
 
from jetcobot2.rack_detector import RackSpaceDetector
from ppibigi_interfaces.action import RobotCommand
from jetcobot2.qr_detector import scan_qr, draw_qr_debug
from jetcobot2.cv_lib import get_filtered_cx_cy
 
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("jetcobot2")
 
# ──────────────────────────────────────────────
# 모든 좌표를 send_angles 기준으로 정의
# ──────────────────────────────────────────────

# 공통 초기화 위치
HOME_INIT_ANGLE = [87.45, 6.15, -1.05, -1.75, -0.7, -45.26] # 홈 자세 위치
WORK_INIT_ANGLE = [87.45, 6.15, -1.05, -1.75, -0.7, -45.26] # 작업 초기화 위치 (각도, do_load & do_unload)
OUT_WORK_INIT_ANGLE = [-88.59, 1.66, -0.79, 1.58, 5.09, -43.06] # 출고 작업 초기화 위치(outbound_place)
#---------------------------------------------------------------------------------------
# do_load
LOAD_PICK_COORDS = [91.66, -32.43, -4.57, -23.02, -2.54, -40.86] # load pick 작업 위치 (각도, do_load)
RACK_SCAN_ANGLE = [91.75, -3.33, -87.18, 0.79, -50.0, 46.75] # 랙 스캔 위치 (각도, do_load)
LOAD_PLACE_COORDS = {
        'approach': [103.05, -106.87, -1.23, 112.23, -107.66, -48.6],  
        'place':    [77.95, -96.59, -19.59, 91.58, -80.15, -19.86],  
        'retreat':  [103.79, -103.44, -1.23, 106.43, -104.23, -45.7],  
} # load Place 좌표 (각도, do_load)
#---------------------------------------------------------------------------------------
# do_unload
QR_SCAN_ANGLES = {
    'floor2_left':  [100.81, -123.22, -0.43, 121.64, -100.38, -39.9],
    'floor2_right': [100.81, -123.22, -0.43, 121.64, -105.38, -39.9],
} # QR 스캔 위치 (각도, do_unload)
UNLOAD_PICK_COORDS = {
        'approach': [103.05, -106.87, -1.23, 112.23, -107.66, -48.6],   
        'pick':     [75.41, -97.64, -1.58, 76.11, -76.55, -17.22],  
        'retreat':  [103.79, -103.44, -1.23, 106.43, -104.23, -45.7],  
} # 출고용 Pick 좌표 (각도, do_unload)
CART_PLACE_COORDS = {
    'approach': [88.15, -67.23, -1.49, 29.97, -5.36, -46.05],
    'place':    [87.97, -69.34, -1.14, 29.79, -4.13, -46.05],  
    'up':       [87.97, -56.42, -1.49, 22.14, -4.48, -46.4],  
} # 운반대 Place 좌표 (각도, do_unload)
#---------------------------------------------------------------------------------------
# outbound_place
CONVEYOR_PLACE_COORDS = {
    'approach': [-83.05, -4.83, 1.58, -60.2, -4.74, -32.87],  
    'place':    [-87.89, -30.14, -0.08, -15.55, -5.18, -41.57],  
    'up':       [-87.8, -23.9, -0.17, -9.66, -4.21, -39.19],  
} # 출고 컨베이어 Place 좌표 (각도, outbound_place)
#---------------------------------------------------------------------------------------


class RobotController:
    def __init__(self):
        self.mc             = MyCobot280('/dev/ttyJETCOBOT', 1000000)
        self.mc.thread_lock = True
        self.rack_detector  = RackSpaceDetector(
            white_ratio_threshold=0.07,
            roi_x1=30,
            roi_y1=10,
            roi_x2=600,
            roi_y2=220,
            block_size = 51,
            c_value = -10
        )
        logger.info("로봇 연결 완료")
 
    def go_home(self, feedback_cb):
        """홈 자세로 복귀 + 그리퍼 열기"""
        feedback_cb("홈 위치로 이동")
        self.mc.send_angles(HOME_INIT_ANGLE, 50)
        self.mc.set_gripper_value(100, 50)
        time.sleep(2)
  
    def ctrl_moving(self, tar_coo, feedback_cb = None):
        """XYZ 좌표 이동 + 오차 보정""" 
        time.sleep(1)
        self.mc.send_coords(tar_coo, 60)
        time.sleep(1)  # ← 대기 시간 늘리기 (이동 완료 전에 get_coords 하면 오차 큼)

        MAX_ITER  = 3
        THRESHOLD = 5.0

        for i in range(MAX_ITER):
            coords = np.array(self.mc.get_coords())
            error  = coords - np.array(tar_coo)

            if feedback_cb:
                feedback_cb(f"[보정 {i+1}회] 오차: {np.round(error[:3], 2)}")

            if np.all(np.abs(error[:2]) < THRESHOLD):  # Z 제외 (X/Y만 판단)
                if feedback_cb:
                    feedback_cb(f"[보정 {i+1}회] 오차 허용 범위 내 → 보정 완료")
                break

            # coords 기준으로 오차만큼 빼기 (= tar_coo로 재이동)
            corrected = np.array(tar_coo, dtype=float)
            corrected[:3] = np.array(coords[:3]) - error[:3]

            self.mc.send_coords(corrected.tolist(), 20)
            time.sleep(1)

        coords = np.array(self.mc.get_coords())
        error  = coords - np.array(tar_coo)
        if feedback_cb:
            feedback_cb(f"최종 오차: {np.round(error[:3], 2)}")
 
    def _move_angles(self, angles, speed=30, wait=2.0):
        """send_angles 래퍼 - 이동 후 대기"""
        self.mc.send_angles(angles, speed)
        time.sleep(wait)
 
    def _scan_rack(self, feedback_cb, node=None):
        feedback_cb("랙 스캔 위치로 이동")
        if node: node._set_detect(False)
        self._move_angles(RACK_SCAN_ANGLE, speed=30, wait=2)

        feedback_cb("카메라 안정화 대기 중...")

        # 가벼운 원본 프레임만 publish
        stabilize_start = time.time()
        while time.time() - stabilize_start < 2.0:
            frame = node._last_frame if node is not None else None
            if frame is not None and node:
                node.publish_debug_frame(frame)
            time.sleep(0.2)

        feedback_cb("랙 빈 공간 스캔 중...")

        results = []
        for i in range(5):
            time.sleep(0.3)
            frame = node._last_frame if node is not None else None
            if frame is None:
                continue
            result = self.rack_detector.detect(frame)
            # Flask 화면에 랙 스캔 ROI/상태 시각화
            if node: node.publish_debug_frame(result['debug_frame'])
            results.append(result)

        if not results:
            feedback_cb("카메라 프레임 없음 → 스캔 실패")
            if node: node._set_detect(True)
            if node: node.clear_debug_frame()
            return None
    
       
        # dict에서 left_empty/right_empty를 꺼내서 기존 다수결 로직 유지
        left_votes  = sum(1 for r in results if r['left_empty'])
        right_votes = sum(1 for r in results if r['right_empty'])
        total       = len(results)

        left_empty  = left_votes  > total // 2
        right_empty = right_votes > total // 2

        if node: node._set_detect(True)

        feedback_cb(
            f"스캔 결과 → "
            f"좌측: {'비어있음' if left_empty  else '박스있음'} ({left_votes}/{total}), "
            f"우측: {'비어있음' if right_empty else '박스있음'} ({right_votes}/{total})"
        )

        time.sleep(1.0) # 결과를 화면에서 더 보여줌

        if node: node.clear_debug_frame()

        if left_empty:
            return 'floor2_left'
        elif right_empty:
            return 'floor2_right'
        else:
            return None
        
    def check_gripper_grasp(self, feedback_cb, empty_threshold=10, max_retry=2):
        """
        그리퍼로 물체를 잡았는지 확인
        Args:
            empty_threshold: 빈 상태로 간주할 그리퍼 값 (이하면 미집힘)
            max_retry: 재시도 횟수
        
        Returns:
            bool: 물체를 잡았으면 True
        """
        for attempt in range(max_retry + 1):
            time.sleep(0.5)  # 그리퍼 안정화 대기
            gripper_value = self.mc.get_gripper_value()
            feedback_cb(f"그리퍼 값 확인: {gripper_value} (시도 {attempt+1}/{max_retry+1})")

            if gripper_value > empty_threshold:
                feedback_cb(f"물체 감지됨 (gripper={gripper_value})")
                return True

            if attempt < max_retry:
                feedback_cb("물체 미감지 → 재시도")
                self.mc.set_gripper_value(100, 50)  # 그리퍼 열기
                time.sleep(0.5)
                self.mc.set_gripper_value(0, 50)    # 다시 닫기 (Pick 재시도)
                time.sleep(0.7)

        feedback_cb("물체 Pick 실패 (최대 재시도 초과)")
        return False

    def do_load(self, feedback_cb, node=None):
        """입고 동작"""
        # --------------------------------------------------------------------------------
        # do_load() pick 파트 시작
        # 카메라 인식 지점 이동
        feedback_cb("카메라 인식 지점 이동")
        if node: node._set_detect(False)
        self._move_angles(LOAD_PICK_COORDS, speed = 40, wait = 2.0)
        if node: node._set_detect(True)

        # 필터링 - N프레임 중앙값으로 cx, cy 획득
        feedback_cb("박스 위치 필터링 중...")
        cx, cy = get_filtered_cx_cy(
            frame_getter=lambda: node._last_frame,
            n_frames=5,
            feedback_cb=feedback_cb
        )

        if cx is None:
            feedback_cb("박스 미감지 → 픽업 중단")
            self.go_home(feedback_cb)
            if node: node._set_detect(True)
            return

        feedback_cb(
            f"필터링 완료 → "
            f"cx={cx:.1f}, cy={cy:.1f}"
        )

        # cx, cy 기반 목표 좌표 보정
        feedback_cb("박스 위치 보정 계산")

        COORD_LIMITS = {
            'x': (-281.45, 281.45),
            'y': (-281.45, 281.45),
            'z': (-70.0,   435.0),
        }
        # 화면 중앙 기준 오차 계산
        CENTER_X     = 320    # 카메라 해상도 640 기준
        CENTER_Y     = 240    # 카메라 해상도 480 기준
        PIXEL_TO_MM  = 0.2   # 2픽셀 = 0.2mm

        error_x = cx - CENTER_X
        error_y = cy - CENTER_Y

        # 기본 목표 좌표 - 박스를 pick할 좌표 
        base_coords = [78.0, 269.3, 234.4, -149.58, -29.89, 32.69]

        # cx, cy 오차로 XY 보정
        corrected_coords = base_coords.copy()
        corrected_coords[0] -= error_x * PIXEL_TO_MM  # X축 보정
        corrected_coords[1] += error_y * PIXEL_TO_MM  # Y축 보정

        # 범위 초과 방지 클리핑
        corrected_coords[0] = max(COORD_LIMITS['x'][0],
                            min(COORD_LIMITS['x'][1], corrected_coords[0]))
        corrected_coords[1] = max(COORD_LIMITS['y'][0],
                            min(COORD_LIMITS['y'][1], corrected_coords[1]))
        corrected_coords[2] = max(COORD_LIMITS['z'][0],
                            min(COORD_LIMITS['z'][1], corrected_coords[2]))

        feedback_cb(
            f"보정값 → "
            f"error_x={error_x:.1f}px, error_y={error_y:.1f}px "
            f"→ X보정={-error_x * PIXEL_TO_MM:.1f}mm, "
            f"Y보정={error_y * PIXEL_TO_MM:.1f}mm"
        )

        # ctrl_moving - 보정 좌표로 이동 + 관절 오차 보정
        feedback_cb("보정된 목표 좌표로 이동")
        self.ctrl_moving(corrected_coords, feedback_cb=feedback_cb)
        time.sleep(0.7)

        # 그리퍼 닫기 (Pick)
        feedback_cb("그리퍼 닫기 (Pick)")
        self.mc.set_gripper_value(0, 50)
        time.sleep(0.7)

        # Pick 성공 여부 확인
        if not self.check_gripper_grasp(feedback_cb, empty_threshold=10, max_retry=2):
            feedback_cb("Pick 실패 → 동작 중단")
            self.go_home(feedback_cb)
            if node: node._set_detect(True)
            return

        # 상승
        feedback_cb("상승")
        self._move_angles([86.92, -40.86, -1.23, 1.31, 1.58, -45.08], speed=30, wait=2.0)
        time.sleep(0.5)
        # do_load() pick 파트 종료
        # -----------------------------------------------------------------------------------

        # ---------------------------------------------------------------------------------
        # do_load() rack scan 파트 시작
        # 랙 스캔
        feedback_cb("랙 빈 공간 스캔")
        target_slot = self._scan_rack(feedback_cb, node=node)

        if target_slot is None:
            feedback_cb("빈 공간 없음 → Place 중단")
            self.go_home(feedback_cb)
            return

        feedback_cb(f"빈 공간 확인 → {target_slot} 에 적재")

        # do_load() rack scan 파트 종료
        # ---------------------------------------------------------------------------------

        # ---------------------------------------------------------------------------------
        # do_load() place 파트 시작
        # Place 작업 전 초기화 위치
        feedback_cb("Place 작업 전 초기화 위치 이동")
        self._move_angles(WORK_INIT_ANGLE, speed=30, wait=2.0)

        # Place 접근 위치
        feedback_cb("Place 접근 위치 이동")
        self._move_angles(LOAD_PLACE_COORDS['approach'], speed=30, wait=4.0)

        # Place 위치
        feedback_cb("Place 위치 이동")
        self._move_angles(LOAD_PLACE_COORDS['place'], speed=30, wait=2.0)

        # 그리퍼 열기 (Place)
        feedback_cb("그리퍼 열기 (Place)")
        self.mc.set_gripper_value(80, 50)
        time.sleep(0.7)

        # 후진
        feedback_cb("후진")
        self._move_angles(LOAD_PLACE_COORDS['retreat'], speed=30, wait=2.0)
        
        # 홈 복귀
        feedback_cb("홈 자세 복귀")
        self.go_home(feedback_cb)

        # 재작업을 위한 카메라 인식 지점 이동
        feedback_cb("카메라 인식 지점 이동")
        if node: node._set_detect(False)
        self._move_angles(LOAD_PICK_COORDS, speed=50, wait=2.0)
        if node: node._set_detect(True)

        feedback_cb("load 완료")
        logger.info(f"load 동작 완료")
        # do_load() place 파트 종료
        # ---------------------------------------------------------------------------------
 
    def do_unload(self, feedback_cb, target_item: str = "물병", node=None):
        """
        출고 동작
        ✅ send_angles: 모든 이동 (QR 스캔, Pick, Place, 홈)
        ⚠️  ctrl_moving: 없음 (모두 고정 위치)
        """

        # QR 스캔/Pick/Place 전체 구간은 process_frame() 검출이 불필요하므로
        # flask_node의 부하를 줄이기 위해 detect를 꺼둠
        if node: node._set_detect(False)

        # ---------------------------------------------------------------------------------
        # do_unload() QR 코드 스캔 파트 시작
        # 왼쪽 → 오른쪽 QR 스캔 
        found    = None
        position = None
 
        for slot in ['floor2_left', 'floor2_right']:
            label = '왼쪽' if slot == 'floor2_left' else '오른쪽'
            feedback_cb(f"[1] {label} QR 스캔 위치로 이동")
 
            # QR 스캔 위치로 이동 (왼쪽 , 오른쪽)
            self._move_angles(QR_SCAN_ANGLES[slot], speed=30, wait=2)
            # 이동 명령은 그대로 보내되, 이동하는 동안에도 주기적으로
            # 현재 카메라 프레임을 디버그 화면으로 publish
            move_start = time.time()
            while time.time() - move_start < 2.0:
                frame = node._last_frame if node is not None else None
                if frame is not None and node:
                    node.publish_debug_frame(frame)
                time.sleep(0.3)

            for i in range(3):
                feedback_cb(f"[2] {label} QR 스캔 중... ({i+1}/3)")
                frame = node._last_frame if node is not None else None
 
                if frame is None:
                    feedback_cb("카메라 프레임 없음")
                    time.sleep(1)
                    continue
 
                # scan_qr 사용
                result = scan_qr(frame, target_item=target_item, max_angle_deg=45.0, debug=False)

                # Flask 화면에 QR 디버그 시각화 표시
                debug_frame, _ = draw_qr_debug(frame, target_item=target_item, max_angle_deg=45.0)
                if node: node.publish_debug_frame(debug_frame)

                if result:
                    found    = result
                    position = slot

                    feedback_cb(
                        f"QR 감지 성공! box_id={result['box_id']}, "
                        f"item={result['item']}, position={position}, "
                        f"거리={result.get('distance_cm', '?')}cm "
                        f"({'정상' if result.get('in_range') else '거리 주의'})"
                    )
                    break
                else:
                    feedback_cb(f"QR 미감지 → 재시도 {i+1}/3")
                    time.sleep(1)

            if found:
                break
        
        if found is None:
            feedback_cb(f"'{target_item}' 박스를 찾지 못함 → 중단")
            if node: node.clear_debug_frame()
            self.go_home(feedback_cb)
            return
        
        time.sleep(1.0) # 결과를 화면에서 더 보여줌

        # QR 스캔 파트 끝나면
        if node: node.clear_debug_frame()

        # do_unload() QR 코드 스캔 파트 종료
        # --------------------------------------------------------------------------------

        # --------------------------------------------------------------------------------
        # do_unload() 운반대 pick 파트 시작
        # Pick 작업 전 초기화 위치
        feedback_cb("Pick 작업 전 초기화 위치 이동")
        self._move_angles(WORK_INIT_ANGLE, speed=30, wait=2.0)

        # Pick 접근 위치
        feedback_cb("Pick 접근 위치 이동")
        self._move_angles(UNLOAD_PICK_COORDS['approach'], speed=30, wait=4.0)
 
        # Pick 위치
        feedback_cb("Pick 위치 이동")
        self._move_angles(UNLOAD_PICK_COORDS['pick'], speed=30, wait=2.0)
 
        # 그리퍼 닫기 (Pick)
        feedback_cb("그리퍼 닫기 (Pick)")
        self.mc.set_gripper_value(0, 50)
        time.sleep(0.7)

        # Pick 성공 여부 확인 추가
        if not self.check_gripper_grasp(feedback_cb, empty_threshold=10, max_retry=2):
            feedback_cb("Pick 실패 → 동작 중단")
            if node: node._set_detect(True)
            self.go_home(feedback_cb)
            return
 
        # 후진
        feedback_cb("후진")
        self._move_angles(UNLOAD_PICK_COORDS['retreat'], speed=30, wait=2.0)

        # do_unload() 운반대 pick 파트 종료
        # --------------------------------------------------------------------------------

        # --------------------------------------------------------------------------------
        # do_unload() 운반대 place 파트 시작
        # Place 작업 전 초기화 위치
        feedback_cb("Place 작업 전 초기화 위치 이동")
        self._move_angles(WORK_INIT_ANGLE, speed=30, wait=2.0)

        # 운반대 접근 위치
        feedback_cb("운반대 접근 위치로 이동")
        self._move_angles(CART_PLACE_COORDS['approach'], speed=30, wait=2.0)
 
        # 운반대 Place 위치 
        feedback_cb("운반대 Place 위치로 이동")
        self._move_angles(CART_PLACE_COORDS['place'], speed=30, wait=2.0)
 
        # 그리퍼 열기 (Place)
        feedback_cb("그리퍼 열기 (Place)")
        self.mc.set_gripper_value(100, 50)
        time.sleep(0.7)
 
        # 상승
        feedback_cb("상승")
        self._move_angles(CART_PLACE_COORDS['up'], speed=30, wait=2.0)
 
        if node: node._set_detect(True) # 정상 종료 
        
        # 홈 복귀
        feedback_cb("홈 자세 복귀")
        self.go_home(feedback_cb)
 
        feedback_cb("unload 완료")
        logger.info(f"unload 동작 완료 → {target_item}")
        # do_unload() 운반대 place 파트 종료
        # --------------------------------------------------------------------------------
    
    def outbound_place(self, feedback_cb, node=None):
        """
        출고 작업
        운반대에 실린 박스 → 컨베이어 벨트로 Pick & Place
        """
        
        if node: node._set_detect(False)
        
        # Pick 작업 전 초기화 위치
        feedback_cb("Pick 작업 전 초기화 위치 이동")
        self._move_angles(OUT_WORK_INIT_ANGLE, speed=30, wait=2.0)

        # 그리퍼 닫기 (Pick)
        self.mc.set_gripper_value(0, 50)
        time.sleep(0.7)

        # Pick 한 상태
        feedback_cb("Pick한 상태 확인")
        time.sleep(0.7)

        # 컨베이어 Place 위치로 이동 
        feedback_cb("컨베이어 Place 위치로 이동")
        self._move_angles(CONVEYOR_PLACE_COORDS['place'], speed=30, wait=2.0)

        # 그리퍼 열기 (Place)
        feedback_cb("그리퍼 열기 (Place)")
        self.mc.set_gripper_value(100, 50)
        time.sleep(0.7)

        # 상승 
        feedback_cb("상승")
        self._move_angles(CONVEYOR_PLACE_COORDS['up'], speed=30, wait=2.0)

        if node: node._set_detect(True)
        
        # 홈 복귀 
        feedback_cb("홈 자세 복귀")
        self._move_angles(OUT_WORK_INIT_ANGLE, speed=30, wait=2.0)
        # self.go_home(feedback_cb)

        feedback_cb("outbound_place 완료")
        logger.info("outbound_place 동작 완료")
 
 
class JetCobot2ActionServer(Node):
    def __init__(self):
        super().__init__('jetcobot2_action_server')
        self._bridge     = CvBridge()
        self._last_frame = None
 
        self.robot = RobotController()

        # 디버그 프레임 발행자 추가
        self._debug_stream_pub       = self.create_publisher(Image, 'debug_stream', 10)
        self._debug_stream_clear_pub = self.create_publisher(Empty, 'debug_stream_clear', 10)

 
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
        self.create_subscription(String, 'manual_command', self._manual_cb,      10)

        # ✅ 감지 허용 신호 발행자 추가
        self._detect_enable_pub = self.create_publisher(Bool, 'detect_enable', 10)
        self.get_logger().info("액션 서버 시작: jetcobot2/command")

        def fb(m): logger.info(f"[init] {m}")
        self.get_logger().info("초기화: 홈 자세 이동")
        self.robot.go_home(fb)
    
    def publish_debug_frame(self, frame):
        """디버그 시각화 프레임을 /debug_stream 토픽으로 발행 (flask_node가 구독)"""
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self._debug_stream_pub.publish(msg)

    def clear_debug_frame(self):
        """/debug_stream_clear 토픽으로 override 해제 신호 발행"""
        self._debug_stream_clear_pub.publish(Empty())

    def _camera_callback(self, msg):
        self._last_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
 
    def _manual_cb(self, msg):
        action_type = msg.data
        self.get_logger().info(f"수동 명령 수신: {action_type}")
        def fb(m): logger.info(f"[manual] {m}")

        self.robot.go_home(fb)

        if action_type == "do_load":
           self.robot.do_load(fb, node=self)
        elif action_type == "do_unload":
            self.robot.do_unload(fb, node=self)
        elif action_type == "outbound_place":
            self.robot.outbound_place(fb, node=self)

    def _set_detect(self, enable: bool):
        """감지 허용/금지 신호 발행"""
        msg      = Bool()
        msg.data = enable
        self._detect_enable_pub.publish(msg)

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
 
        try:
            if goal_handle.is_cancel_requested:
                result.event   = "canceled"
                result.message = "취소됨"
                goal_handle.canceled()
                return result
 
            if action_type == "do_load":
                self.robot.do_load(send_feedback, node=self)

            elif action_type == "do_unload":
                target_item = parameters.get("target_item", "물병")
                self.robot.do_unload(send_feedback, target_item=target_item, node=self)

            elif action_type == "outbound_place": 
                self.robot.outbound_place(send_feedback, node=self)

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