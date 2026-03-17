# 最小测试：仅 localization（用于排查 "一顿一顿" 是否来自 dynamic_cloud）
# 用法：ros2 launch dynamic_cloud lidar_minimal.launch.py map_file:=mapping_ws/test.pcd
# 配合：Livox + 此 launch + RViz（不启动 dynamic_cloud/cluster/kalman）
# 若此组合流畅，则卡顿来自 dynamic_cloud 或 cluster
import os
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    map_file = LaunchConfiguration('map_file', default='config/RM2024.pcd')
    voxel_leaf_size = LaunchConfiguration('voxel_leaf_size', default='0.35')
    accumulate_time = LaunchConfiguration('accumulate_time', default='3')

    localization_node = ComposableNode(
        package='localization',
        plugin='tdt_radar::Localization',
        name='localization_node',
        parameters=[{'map_file': map_file, 'voxel_leaf_size': voxel_leaf_size, 'accumulate_time': accumulate_time, 'invert_tf': True, 'use_grid_map': False}],
        extra_arguments=[{'use_intra_process_comms': True}, {'use_multi_threaded_executor': True}],
    )

    container = ComposableNodeContainer(
        name='lidar_minimal_container',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[localization_node],
        output='both',
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value='config/RM2024.pcd'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.35'),
        DeclareLaunchArgument('accumulate_time', default_value='3'),
        container,
    ])
