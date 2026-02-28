from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tdt_vision',
            executable='nyush_detect_node.py',
            name='nyush_detect_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
