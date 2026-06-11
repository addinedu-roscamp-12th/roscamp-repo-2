# vision/aruco_detector.py
# ArUco 마커 감지 + 3D 위치 계산

import cv2
import numpy as np
from cv_bridge import CvBridge

from pinky1.config.settings import ARUCO_CONFIG
from pinky1.utils.logger import RobotLogger


class ArucoDetector:
    """
    ArUco 마커 감지
    - 카메라 이미지에서 마커 ID, 위치, 각도 계산
    - /camera/camera_info 로 캘리브레이션 자동 수신
    - 결과를 NavManager.dock_to_marker 와 연동
    """

    def __init__(self, node):
        self.node    = node
        self.log     = RobotLogger(node)
        self.bridge  = CvBridge()

        self.aruco_dict   = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, ARUCO_CONFIG["dict"]))
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.aruco_params.minMarkerPerimeterRate = 0.05
        self.aruco_params.errorCorrectionRate    = 0.5

        self.marker_size = ARUCO_CONFIG["marker_size"]
        self.marker_map  = ARUCO_CONFIG["marker_map"]

        # 감지 콜백 (외부 등록)
        self.on_detected = None
        self._seen_ids   = set()  # 이미 로그 찍은 마커 ID

        self.log.info("ARUCO", "ArUco 감지기 초기화 완료")

    # ── 감지 ───────────────────────────────────────────────────
    def detect(self, image_msg, camera_matrix, dist_coeffs):
        """
        이미지에서 마커 감지
        Returns:
            list of {
                id, location, distance,
                tvec [x,y,z], rvec [rx,ry,rz]
            }
        """
        if image_msg is None:
            return []

        # 1) 마커 추출
        frame = self.bridge.imgmsg_to_cv2(
            image_msg, desired_encoding="bgr8")
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        if ids is None:
            self._seen_ids.clear()
            return []

        # 2) 신규 감지 ID 추적
        current_ids = set(ids.flatten().tolist())
        new_ids     = current_ids - self._seen_ids
        self._seen_ids = current_ids

        results = []
        for i, mid in enumerate(ids.flatten()):
            # 3) 마커별 메타데이터 구성
            pts = corners[i][0]
            cx  = int(pts[:, 0].mean())
            cy  = int(pts[:, 1].mean())
            pixel_size = int(np.linalg.norm(pts[0] - pts[2]))

            res = {
                "id":         int(mid),
                "location":   self.marker_map.get(int(mid), "unknown"),
                "cx":         cx,
                "cy":         cy,
                "pixel_size": pixel_size,
                "corners":    pts.tolist(),  # [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]
                "distance":   None,
                "tvec":       None,
                "rvec":       None,
            }

            # 4) 자세 추정 (camera_matrix 있을 때)
            if camera_matrix is not None and dist_coeffs is not None:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i:i+1], self.marker_size,
                    camera_matrix, dist_coeffs)
                res["distance"] = round(float(np.linalg.norm(tvec[0][0])), 3)
                res["tvec"]     = tvec[0][0].tolist()
                res["rvec"]     = rvec[0][0].tolist()

            # 5) 등록되지 않은 마커 필터링 — marker_map에 없으면 무시
            if res["location"] == "unknown":
                continue
            results.append(res)
            if int(mid) in new_ids:
                dist_str = f"{res['distance']:.2f}m" if res["distance"] else ""
                self.log.info("ARUCO",
                    f"마커 {mid} 감지 | {res['location']} {dist_str}")

        # 6) 콜백 호출
        if results and self.on_detected:
            self.on_detected(results)

        return results

