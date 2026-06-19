# launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # ✅ 로봇 ID를 launch 인자로 받음 (기본값: robot1)
    # 실행 시: ros2 launch jetcobot2 launch.py robot_id:=robot1
    #          ros2 launch jetcobot2 launch.py robot_id:=robot2
    robot_id = LaunchConfiguration('robot_id', default='jetcobot2')

    declare_robot_id = DeclareLaunchArgument(
        'robot_id',
        default_value='jetcobot2',
        description='로봇 고유 ID (robot1, robot2 등)'
    )

    flask_node = Node(
        package='jetcobot2',
        executable='flask_node',
        namespace=robot_id,         # 네임스페이스 적용
        name='flask_node',
        output='screen',
    )

    cv_node = Node(
        package='jetcobot2',
        executable='cv_node',
        namespace=robot_id,         # 네임스페이스 적용
        name='cv_node',
        output='screen',
    )

    action_server = Node(
        package='jetcobot2',
        executable='action_server',
        namespace=robot_id,         # 네임스페이스 적용
        name='jetcobot2_action_server',
        output='screen',
    )

    return LaunchDescription([
        declare_robot_id,
        flask_node,
        cv_node,
        action_server,
    ])