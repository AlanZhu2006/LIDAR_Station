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
    accumulate_time = LaunchConfiguration('accumulate_time', default='2')
    publish_accumulated_dynamic_cloud = LaunchConfiguration('publish_accumulated_dynamic_cloud', default='true')
    cluster_tolerance = LaunchConfiguration('cluster_tolerance', default='0.25')
    min_cluster_size = LaunchConfiguration('min_cluster_size', default='8')
    max_cluster_size = LaunchConfiguration('max_cluster_size', default='1000')
    cluster_voxel_leaf_size = LaunchConfiguration('cluster_voxel_leaf_size', default='0.0')
    min_cluster_diagonal = LaunchConfiguration('min_cluster_diagonal', default='0.08')
    max_cluster_diagonal = LaunchConfiguration('max_cluster_diagonal', default='2.50')
    min_cluster_height = LaunchConfiguration('min_cluster_height', default='0.03')
    max_cluster_height = LaunchConfiguration('max_cluster_height', default='2.20')
    min_cluster_xy_span = LaunchConfiguration('min_cluster_xy_span', default='0.04')
    cluster_use_bbox_center = LaunchConfiguration('cluster_use_bbox_center', default='false')
    auto_align = LaunchConfiguration('auto_align', default='true')
    debug_camera_match = LaunchConfiguration('debug_camera_match', default='false')
    camera_detect_radius = LaunchConfiguration('camera_detect_radius', default='1.0')
    track_match_radius = LaunchConfiguration('track_match_radius', default='1.0')
    publish_stationary_targets = LaunchConfiguration('publish_stationary_targets', default='false')
    publish_unclassified_targets = LaunchConfiguration('publish_unclassified_targets', default='true')
    min_unclassified_history = LaunchConfiguration('min_unclassified_history', default='2')
    min_motion_displacement = LaunchConfiguration('min_motion_displacement', default='0.12')
    min_motion_time_span = LaunchConfiguration('min_motion_time_span', default='0.25')
    min_motion_history = LaunchConfiguration('min_motion_history', default='3')
    min_motion_speed = LaunchConfiguration('min_motion_speed', default='0.08')
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
            'publish_accumulated_dynamic_cloud': publish_accumulated_dynamic_cloud,
            'cluster_tolerance': cluster_tolerance,
            'min_cluster_size': min_cluster_size,
            'max_cluster_size': max_cluster_size,
            'cluster_voxel_leaf_size': cluster_voxel_leaf_size,
            'min_cluster_diagonal': min_cluster_diagonal,
            'max_cluster_diagonal': max_cluster_diagonal,
            'min_cluster_height': min_cluster_height,
            'max_cluster_height': max_cluster_height,
            'min_cluster_xy_span': min_cluster_xy_span,
            'cluster_use_bbox_center': cluster_use_bbox_center,
            'point_cloud_topic': '/livox/lidar_corrected',
            'point_cloud_frame_id': 'livox_frame_ground',
            'tf_child_frame': 'livox_frame',
            'auto_align': auto_align,
            'debug_camera_match': debug_camera_match,
            'camera_detect_radius': camera_detect_radius,
            'track_match_radius': track_match_radius,
            'publish_stationary_targets': publish_stationary_targets,
            'publish_unclassified_targets': publish_unclassified_targets,
            'min_unclassified_history': min_unclassified_history,
            'min_motion_displacement': min_motion_displacement,
            'min_motion_time_span': min_motion_time_span,
            'min_motion_history': min_motion_history,
            'min_motion_speed': min_motion_speed,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='是否使用仿真时间'),
        DeclareLaunchArgument('map_file', default_value='config/RM2024.pcd', description='PCD 地图路径'),
        DeclareLaunchArgument('ceiling_z_max', default_value='100.0', description='输入点云 z 方向天花板过滤阈值'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.25', description='地图体素降采样大小'),
        DeclareLaunchArgument('kd_tree_threshold_sq', default_value='0.15', description='动态点判定平方距离阈值，减小后更容易保留低矮机器人'),
        DeclareLaunchArgument('process_every_n', default_value='1', description='每 N 帧执行一次 KD-Tree'),
        DeclareLaunchArgument('accumulate_time', default_value='2', description='动态点云累积窗口大小'),
        DeclareLaunchArgument('publish_accumulated_dynamic_cloud', default_value='true', description='是否发布累积后的动态点云到 cluster'),
        DeclareLaunchArgument('cluster_tolerance', default_value='0.25', description='欧式聚类距离阈值'),
        DeclareLaunchArgument('min_cluster_size', default_value='8', description='最小聚类点数'),
        DeclareLaunchArgument('max_cluster_size', default_value='1000', description='最大聚类点数'),
        DeclareLaunchArgument('cluster_voxel_leaf_size', default_value='0.0', description='聚类前体素降采样大小，0表示关闭'),
        DeclareLaunchArgument('min_cluster_diagonal', default_value='0.08', description='有效聚类的最小平面对角线（米）'),
        DeclareLaunchArgument('max_cluster_diagonal', default_value='2.50', description='有效聚类的最大平面对角线（米）'),
        DeclareLaunchArgument('min_cluster_height', default_value='0.03', description='有效聚类的最小高度（米）'),
        DeclareLaunchArgument('max_cluster_height', default_value='2.20', description='有效聚类的最大高度（米）'),
        DeclareLaunchArgument('min_cluster_xy_span', default_value='0.04', description='有效聚类的最小 X/Y 跨度（米）'),
        DeclareLaunchArgument('cluster_use_bbox_center', default_value='false', description='是否用包围盒中心代替质心作为 cluster 输出'),
        DeclareLaunchArgument('use_tilt_correction', default_value='true', description='启用倾斜修正'),
        DeclareLaunchArgument('lidar_pitch_deg', default_value='50.0', description='雷达向前倾角度数'),
        DeclareLaunchArgument('auto_align', default_value='true', description='自动校正 RViz 显示地平面'),
        DeclareLaunchArgument('debug_camera_match', default_value='false', description='启用融合匹配调试日志'),
        DeclareLaunchArgument('camera_detect_radius', default_value='1.0', description='相机-雷达匹配半径（米）'),
        DeclareLaunchArgument('track_match_radius', default_value='1.0', description='LiDAR Kalman 轨迹关联半径（米）'),
        DeclareLaunchArgument('publish_stationary_targets', default_value='false', description='调试时是否发布静止目标到 /livox/lidar_kalman'),
        DeclareLaunchArgument('publish_unclassified_targets', default_value='true', description='调试时是否发布未识别的灰色 Kalman 目标'),
        DeclareLaunchArgument('min_unclassified_history', default_value='2', description='发布灰色未识别 Kalman 目标前的最小历史帧数'),
        DeclareLaunchArgument('min_motion_displacement', default_value='0.12', description='未识别轨迹判定为移动目标前的最小位移（米）'),
        DeclareLaunchArgument('min_motion_time_span', default_value='0.25', description='未识别轨迹判定为移动目标前的最小观测时长（秒）'),
        DeclareLaunchArgument('min_motion_history', default_value='3', description='未识别轨迹判定为移动目标前的最小历史帧数'),
        DeclareLaunchArgument('min_motion_speed', default_value='0.08', description='未识别轨迹判定为移动目标前的最小平均速度（米/秒）'),
        static_tf,
        pointcloud_transform_node,
        lidar_main,
    ])
