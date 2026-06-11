'''
명령어
deactivate
cd jetcobot2_ws

colcon build --packages-select jetcobot2
source install/setup.bash
ros2 launch jetcobot2 launch.py

카메라 안켜진다면?

sudo fuser -v /dev/jetcocam0
sudo kill -9 <pid>

테스트 명령

ros2 topic pub --once /manual_command std_msgs/msg/String "data: 'inbound_pickup'"

Action Type 변경
# 입고 모드
ros2 topic pub /cv_mode std_msgs/msg/String \
  '{"data": "{\"action_type\": \"do_load\"}"}'

# 출고 모드
ros2 topic pub /cv_mode std_msgs/msg/String \
  '{"data": "{\"action_type\": \"do_unload\", \"target_item\": \"물병\"}"}'

# 출고 컨베이어 모드
ros2 topic pub /cv_mode std_msgs/msg/String \
  '{"data": "{\"action_type\": \"outbound_pickup\"}"}'
'''