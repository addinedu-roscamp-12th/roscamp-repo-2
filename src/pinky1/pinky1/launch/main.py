# launch/main.py
# 전체 시스템 실행 진입점

import sys
import os
import threading

from pinky1.config.settings import ROS_DOMAIN_ID
os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)

import cv2
import rclpy
from rclpy.executors import MultiThreadedExecutor
from pinky1.core.robot_controller import RobotController


def run_single(robot_id: str = "pinky1"):
    rclpy.init()
    robot    = RobotController(robot_id)
    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    threading.Thread(target=executor.spin, daemon=True).start()

    try:
        while rclpy.ok():
            frame = robot.yolo.latest_frame
            if frame is not None:
                cv2.imshow("YOLO Detection", frame)
            cv2.waitKey(30)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()


# ── 실행 ───────────────────────────────────────────
# python3 launch/main.py
# python3 launch/main.py pinky1
if __name__ == "__main__":
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "pinky1"
    run_single(robot_id)
