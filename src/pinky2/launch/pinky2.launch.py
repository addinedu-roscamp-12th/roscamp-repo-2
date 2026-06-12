from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pinky2',
            executable='auto_parking_node',
            name='auto_parking_node',
            output='screen',
        ),
        Node(
            package='pinky2',
            executable='exit_parking_node',
            name='exit_parking_node',
            output='screen',
        ),
        Node(
            package='pinky2',
            executable='pinky2_orchestrator_node',
            name='pinky2_orchestrator_node',
            output='screen',
        ),
    ])
