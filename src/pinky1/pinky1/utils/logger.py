# utils/logger.py
# 태그별 색상 로거

class RobotLogger:
    """
    태그별로 구분되는 로거
    사용법: self.log.info("NAV", "이동 시작")
    """

    TAGS = {
        "SYS":    "[SYS]   ",
        "NAV":    "[NAV]   ",
        "SENSOR": "[SENSOR]",
        "ARUCO":  "[ARUCO] ",
        "YOLO":   "[YOLO]  ",
        "SLAM":   "[SLAM]  ",
        "LLM":    "[LLM]   ",
        "LED":    "[LED]   ",
    }

    def __init__(self, node):
        self.node = node

    def _tag(self, tag):
        return self.TAGS.get(tag, f"[{tag}]")

    def info(self, tag: str, msg: str):
        self.node.get_logger().info(f"{self._tag(tag)} {msg}")

    def warn(self, tag: str, msg: str):
        self.node.get_logger().warn(f"{self._tag(tag)} {msg}")

    def error(self, tag: str, msg: str):
        self.node.get_logger().error(f"{self._tag(tag)} {msg}")

    def debug(self, tag: str, msg: str):
        self.node.get_logger().debug(f"{self._tag(tag)} {msg}")
