from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    """
    LaunchDescription 에 노드를 등록해 반환.
    ROS2 launch 시스템이 이 함수를 호출해 노드들을 실행.
    """

    # ✅ cv_node는 3초 후 실행 (action_server 준비 대기)
    cv = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='jetcobot2',
                executable='cv_node',
                name='cv_node',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    flask_node = Node(
        package='jetcobot2',
        executable='flask_node',       # setup.py entry_points 이름과 일치
        name='flask_node',
        output='screen',
        emulate_tty=True,
    )

        # arm_node : 로봇팔 Action Server 
    arm = Node(
        package='jetcobot2',       # 패키지 이름
        executable='action_server',            # setup.cfg 의 entry_points 이름과 일치
        name='jetcobot2_action_server',                  # ros2 node list 에서 보이는 이름
        output='screen',                  # 로그를 터미널에 출력
        emulate_tty=True,                 # 색깔 로그 유지
    )

    return LaunchDescription([arm, cv, flask_node])
