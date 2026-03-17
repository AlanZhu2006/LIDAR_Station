from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    publish_width_arg = DeclareLaunchArgument(
        'publish_width',
        default_value='0',
        description='Camera image width to publish. 0 keeps the native sensor width.',
    )
    publish_height_arg = DeclareLaunchArgument(
        'publish_height',
        default_value='0',
        description='Camera image height to publish. 0 keeps the native sensor height.',
    )
    poll_period_ms_arg = DeclareLaunchArgument(
        'poll_period_ms',
        default_value='20',
        description='Camera polling period in milliseconds.',
    )
    grab_timeout_ms_arg = DeclareLaunchArgument(
        'grab_timeout_ms',
        default_value='5',
        description='Camera frame grab timeout in milliseconds.',
    )
    target_fps_arg = DeclareLaunchArgument(
        'target_fps',
        default_value='30.0',
        description='Requested camera acquisition frame rate.',
    )

    return LaunchDescription([
        publish_width_arg,
        publish_height_arg,
        poll_period_ms_arg,
        grab_timeout_ms_arg,
        target_fps_arg,
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera_node',
            output='screen',
            parameters=[{
                'publish_width': LaunchConfiguration('publish_width'),
                'publish_height': LaunchConfiguration('publish_height'),
                'poll_period_ms': LaunchConfiguration('poll_period_ms'),
                'grab_timeout_ms': LaunchConfiguration('grab_timeout_ms'),
                'target_fps': LaunchConfiguration('target_fps'),
            }],
        ),
    ])
