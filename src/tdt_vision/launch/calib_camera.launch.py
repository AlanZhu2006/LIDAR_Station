"""
实机相机五点标定 launch：只启动标定节点 + map_server，不启动 rosbag。
请先在另一终端运行: ros2 run hik_camera hik_camera_node
标定节点会订阅 /camera_image。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.actions import Shutdown, IncludeLaunchDescription
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    radar_calib_node = ComposableNode(
        package='tdt_vision',
        plugin='tdt_radar::Calibrate',
        name='radar_calib_node',
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[radar_calib_node],
        output='both',
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('tdt_vision'), 'launch', 'map_server_launch.py')
        ]),
    )

    return LaunchDescription([
        container,
        map_server_launch,
    ])
