FROM ros:jazzy-ros-base

# pinky_msgs 액션 패키지 빌드
WORKDIR /ros2_ws/src
COPY pinky_msgs ./pinky_msgs
WORKDIR /ros2_ws
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
RUN . /opt/ros/jazzy/setup.sh && colcon build --packages-select pinky_msgs

# 태스크매니저 설치
RUN pip3 install fastapi uvicorn pydantic --break-system-packages
COPY task_manager.py /app/task_manager.py
WORKDIR /app

CMD ["/bin/bash", "-c", \
     "source /opt/ros/jazzy/setup.bash && \
      source /ros2_ws/install/setup.bash && \
      python3 task_manager.py"]
