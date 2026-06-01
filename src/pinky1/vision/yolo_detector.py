# vision/yolo_detector.py
# YOLO 객체 감지 (사람, 박스)

from cv_bridge import CvBridge
from config.settings import YOLO_CONFIG
from utils.logger import RobotLogger

if YOLO_CONFIG.get("enabled", False):
    from ultralytics import YOLO


class YoloDetector:
    """
    YOLO 객체 감지
    - 사람 감지 → on_person_detected 콜백 → 긴급 정지
    - 박스 감지 → on_box_detected 콜백
    - /camera/image_raw 이미지 입력
    """

    def __init__(self, node):
        self.node   = node
        self.log    = RobotLogger(node)
        self.bridge = CvBridge()

        self.model   = YOLO(YOLO_CONFIG["model"]) if YOLO_CONFIG.get("enabled", False) else None
        self.conf    = YOLO_CONFIG["confidence"]
        self.classes = YOLO_CONFIG.get("classes", None)

        # 외부 콜백 등록
        self.on_person_detected = None
        self.on_box_detected    = None

        self.log.info("YOLO",
            f"YOLO 로드 완료 ({YOLO_CONFIG['model']})")

    def detect(self, image_msg):
        if not YOLO_CONFIG.get("enabled", True):
            return {"detections": []}
        if image_msg is None:
            return {"detections": []}

        frame   = self.bridge.imgmsg_to_cv2(
            image_msg, desired_encoding="bgr8")
        results = self.model(
            frame, conf=self.conf,
            classes=self.classes, verbose=False)

        detections = []

        for r in results:
            for box in r.boxes:
                cls  = int(box.cls[0])
                conf = float(box.conf[0])
                xyxy = box.xyxy[0].tolist()
                name = self.model.names.get(cls, str(cls))
                obj  = {
                    "class_id":   cls,
                    "class_name": name,
                    "confidence": round(conf, 2),
                    "bbox":       xyxy,
                    "cx": (xyxy[0] + xyxy[2]) / 2,
                    "cy": (xyxy[1] + xyxy[3]) / 2,
                }
                detections.append(obj)
                self.log.info("YOLO",
                    f"{name} 감지! conf={conf:.2f}")
                if self.on_person_detected:
                    self.on_person_detected(obj)

        return {"detections": detections}
