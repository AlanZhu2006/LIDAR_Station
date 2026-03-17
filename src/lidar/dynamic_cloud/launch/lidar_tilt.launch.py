"""
雷达倾斜安装启动：TF 变换 + pointcloud_transform（参考 README_LIDAR 10.5 节）
使用方式：ros2 launch dynamic_cloud lidar_tilt.launch.py
建图后需对 PCD 运行 rotate_pcd.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def get_launch_file_dir():
    return os.path.join(os.path.dirname(__file__))

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_tilt_correction = LaunchConfiguration('use_tilt_correction', default='true')
    lidar_pitch_deg = LaunchConfiguration('lidar_pitch_deg', default='50.0')
    map_file = LaunchConfiguration('map_file', default='config/RM2024.pcd')
    ceiling_z_max = LaunchConfiguration('ceiling_z_max', default='100.0')
    voxel_leaf_size = LaunchConfiguration('voxel_leaf_size', default='0.25')
    kd_tree_threshold_sq = LaunchConfiguration('kd_tree_threshold_sq', default='0.15')
    process_every_n = LaunchConfiguration('process_every_n', default='1')
    accumulate_time = LaunchConfiguration('accumulate_time', default='3')
    cluster_tolerance = LaunchConfiguration('cluster_tolerance', default='0.25')
    min_cluster_size = LaunchConfiguration('min_cluster_size', default='12')
    max_cluster_size = LaunchConfiguration('max_cluster_size', default='1000')
    cluster_voxel_leaf_size = LaunchConfiguration('cluster_voxel_leaf_size', default='0.0')
    auto_align = LaunchConfiguration('auto_align', default='true')
    debug_camera_match = LaunchConfiguration('debug_camera_match', default='false')
    camera_detect_radius = LaunchConfiguration('camera_detect_radius', default='1.0')
    track_match_radius = LaunchConfiguration('track_match_radius', default='1.0')
    publish_stationary_targets = LaunchConfiguration('publish_stationary_targets', default='false')
    lidar_pitch_rad = PythonExpression(['(', lidar_pitch_deg, ') * 3.141592653589793 / 180.0'])

    # static_transform: livox_frame -> livox_frame_ground (x y z yaw pitch roll)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', lidar_pitch_rad, '0', 'livox_frame', 'livox_frame_ground'],
        condition=IfCondition(use_tilt_correction),
    )

    pointcloud_transform_node = Node(
        package='pointcloud_transform',
        executable='pointcloud_transform_node',
        name='pointcloud_transform_node',
        parameters=[{
            'source_frame': 'livox_frame',
            'target_frame': 'livox_frame_ground',
            'input_topic': '/livox/lidar',
            'output_topic': '/livox/lidar_corrected',
        }],
        condition=IfCondition(use_tilt_correction),
    )

    lidar_main = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_launch_file_dir(), '/lidar.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map_file': map_file,
            'ceiling_z_max': ceiling_z_max,
            'voxel_leaf_size': voxel_leaf_size,
            'kd_tree_threshold_sq': kd_tree_threshold_sq,
            'process_every_n': process_every_n,
            'accumulate_time': accumulate_time,
            'cluster_tolerance': cluster_tolerance,
            'min_cluster_size': min_cluster_size,
            'max_cluster_size': max_cluster_size,
            'cluster_voxel_leaf_size': cluster_voxel_leaf_size,
            'point_cloud_topic': '/livox/lidar_corrected',
            'point_cloud_frame_id': 'livox_frame_ground',
            'tf_child_frame': 'livox_frame',
            'auto_align': auto_align,
            'debug_camera_match': debug_camera_match,
            'camera_detect_radius': camera_detect_radius,
            'track_match_radius': track_match_radius,
            'publish_stationary_targets': publish_stationary_targets,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用仿真时间'),
        DeclareLaunchArgument('map_file', default_value='config/RM2024.pcd', description='PCD 地图路径'),
        DeclareLaunchArgument('ceiling_z_max', default_value='100.0', description='输入点云 z 方向天花板过滤阈值'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.25', description='地图体素降采样大小'),
        DeclareLaunchArgument('kd_tree_threshold_sq', default_value='0.15', description='动态点判定平方距离阈值，减小后更容易保留低矮机器人'),
        DeclareLaunchArgument('process_every_n', default_value='1', description='每 N 帧执行一次 KD-Tree'),
        DeclareLaunchArgument('accumulate_time', default_value='3', description='动态点云累积窗口大小'),
        DeclareLaunchArgument('cluster_tolerance', default_value='0.25', description='欧式聚类距离阈值'),
        DeclareLaunchArgument('min_cluster_size', default_value='12', description='最小聚类点数'),
        DeclareLaunchArgument('max_cluster_size', default_value='1000', description='最大聚类点数'),
        DeclareLaunchArgument('cluster_voxel_leaf_size', default_value='0.0', description='聚类前体素降采样大小，0表示关闭'),
        DeclareLaunchArgument('use_tilt_correction', default_value='true', description='启用倾斜修正'),
        DeclareLaunchArgument('lidar_pitch_deg', default_value='50.0', description='雷达向前倾角度数'),
        DeclareLaunchArgument('auto_align', default_value='true', description='自动校正 RViz 显示地平面'),
        DeclareLaunchArgument('debug_camera_match', default_value='false', description='启用融合匹配调试日志'),
        DeclareLaunchArgument('camera_detect_radius', default_value='1.0', description='相机-雷达匹配半径（米）'),
        DeclareLaunchArgument('track_match_radius', default_value='1.0', description='LiDAR Kalman 轨迹关联半径（米）'),
        DeclareLaunchArgument('publish_stationary_targets', default_value='false', description='调试时是否发布静止目标到 /livox/lidar_kalman'),
        static_tf,
        pointcloud_transform_node,
        lidar_main,
    ])
