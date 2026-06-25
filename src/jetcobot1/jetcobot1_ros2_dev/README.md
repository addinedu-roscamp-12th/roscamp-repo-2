# jetcobot1_ros2_dev (개발 중 / 미검증)

기존 jetcobot1을 ROS2 액션 서버(`jetcobot1/command`) 기반으로
전환하는 작업 중인 버전입니다. 실제 하드웨어 검증 전이며,
colcon 빌드 대상에서 제외되어 있습니다 (참고/기록용).

- action_server.py: Task Manager의 RobotCommand 액션을 받아
  pickup/load/unload로 분기
- pause_control.py: 기존 PAUSE_EVENT + ROS2 SetPause 서비스 추가
- voice_pause_client.py: STT만 수행, 분류는 Task Manager LLM에 위임
  (voice_command_server.py는 액션서버로 대체되어 삭제됨)
