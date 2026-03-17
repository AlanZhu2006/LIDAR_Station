import os
import sys
import yaml
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction, Shutdown, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch

def dump_params(param_file_path, node_name):
    with open(param_file_path, 'r') as file:
        return [yaml.safe_load(file)[node_name]['ros__parameters']]

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_file = LaunchConfiguration('map_file', default='config/RM2024.pcd')
    ceiling_z_max = LaunchConfiguration('ceiling_z_max', default='100.0')
    voxel_leaf_size = LaunchConfiguration('voxel_leaf_size', default='0.25')
    kd_tree_threshold_sq = LaunchConfiguration('kd_tree_threshold_sq', default='0.15')
    process_every_n = LaunchConfiguration('process_every_n', default='1')  # 1=每帧处理，减少闪烁；2/3=跳帧省算力
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
    point_cloud_topic = LaunchConfiguration('point_cloud_topic', default='/livox/lidar')
    point_cloud_frame_id = LaunchConfiguration('point_cloud_frame_id', default='livox_frame')
    tf_child_frame = LaunchConfiguration('tf_child_frame', default='livox_frame')
    static_tf_pitch_rad = LaunchConfiguration('static_tf_pitch_rad', default='0')
    auto_align = LaunchConfiguration('auto_align', default='true')
    camera_detect_radius = LaunchConfiguration('camera_detect_radius', default='1.0')
    track_match_radius = LaunchConfiguration('track_match_radius', default='1.0')
    publish_stationary_targets = LaunchConfiguration('publish_stationary_targets', default='false')
    publish_unclassified_targets = LaunchConfiguration('publish_unclassified_targets', default='true')
    min_unclassified_history = LaunchConfiguration('min_unclassified_history', default='2')
    min_motion_displacement = LaunchConfiguration('min_motion_displacement', default='0.12')
    min_motion_time_span = LaunchConfiguration('min_motion_time_span', default='0.25')
    min_motion_history = LaunchConfiguration('min_motion_history', default='3')
    min_motion_speed = LaunchConfiguration('min_motion_speed', default='0.08')
    
    def get_localization_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='localization_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_file': map_file,
                'point_cloud_topic': point_cloud_topic,
                'tf_child_frame': tf_child_frame,
            }],
            extra_arguments=[{'use_intra_process_comms': True},
                             {'use_multi_threaded_executor': True}],
        )
        
    def get_dynamic_cloud_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='dynamic_cloud_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_file': map_file,
                'ceiling_z_max': ceiling_z_max,
                'voxel_leaf_size': voxel_leaf_size,
                'kd_tree_threshold_sq': kd_tree_threshold_sq,
                'process_every_n': process_every_n,
                'accumulate_time': accumulate_time,
                'publish_accumulated_dynamic_cloud': publish_accumulated_dynamic_cloud,
                'point_cloud_topic': point_cloud_topic,
            }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_cluster_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='cluster_node',
            parameters=[{
                'use_sim_time': use_sim_time,
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
            }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_kalman_filter_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='kalman_filter_node',
            parameters=[{
                'use_sim_time': use_sim_time,
                'debug_camera_match': LaunchConfiguration('debug_camera_match', default='false'),
                'camera_detect_radius': camera_detect_radius,
                'track_match_radius': track_match_radius,
                'publish_stationary_targets': publish_stationary_targets,
                'publish_unclassified_targets': publish_unclassified_targets,
                'min_unclassified_history': min_unclassified_history,
                'min_motion_displacement': min_motion_displacement,
                'min_motion_time_span': min_motion_time_span,
                'min_motion_history': min_motion_history,
                'min_motion_speed': min_motion_speed,
            }],
            extra_arguments=[{'use_intra_process_comms': True}]
        )
        
    def get_foxglove_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='foxglove_bridge_node',
            parameters=[ {'send_buffer_limit': 1000000000}],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_container(*nodes):
        # 打印当前路径
        print(os.getcwd())
        return ComposableNodeContainer(
            name='lidar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=list(nodes),
            output='both',
            emulate_tty=True,
            on_exit=Shutdown(),
        )
    
    # static_tf 立即发布 rm_frame->livox_frame，确保 RViz/动态点云在启动时即有 TF（localization 配准后覆盖）
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', static_tf_pitch_rad, '0', 'rm_frame', tf_child_frame],
    )

    display_aligner = Node(
        package='dynamic_cloud',
        executable='display_aligner_node.py',
        name='display_aligner_node',
        parameters=[{'map_topic': '/livox/map'}],
        condition=IfCondition(auto_align),
    )

    # 创建节点描述
    localization_node = get_localization_node('localization', 'tdt_radar::Localization')
    dynamic_cloud_node = get_dynamic_cloud_node('dynamic_cloud', 'tdt_radar::DynamicCloud')
    cluster_node = get_cluster_node('cluster', 'tdt_radar::Cluster')
    kalman_filter_node = get_kalman_filter_node('kalman_filter', 'tdt_radar::KalmanFilter')
    # foxglove_node = get_foxglove_node('foxglove_bridge', 'foxglove_bridge::FoxgloveBridge')

    # 创建节点容器
    lidar_detector = get_container(
                                    localization_node,
                                    dynamic_cloud_node,
                                    cluster_node,
                                    kalman_filter_node
                                    # foxglove_node
                                   )
    # cmd = launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'play', 'config/merged_bag/merged_bag_0.db3', '--loop', '--start-offset', '250'])

    # 延迟 0.5 秒启动 lidar_detector，确保 static_tf 先发布 rm_frame
    delayed_lidar = TimerAction(period=0.5, actions=[lidar_detector])

    return LaunchDescription([
            DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
            DeclareLaunchArgument('map_file', default_value='config/RM2024.pcd', description='PCD map path'),
            DeclareLaunchArgument('ceiling_z_max', default_value='100.0', description='Max z for ceiling filter'),
            DeclareLaunchArgument('voxel_leaf_size', default_value='0.25', description='Voxel grid leaf size'),
            DeclareLaunchArgument('kd_tree_threshold_sq', default_value='0.15', description='KD-tree squared distance threshold (~0.39m), smaller=more sensitive to low robots'),
            DeclareLaunchArgument('process_every_n', default_value='1', description='Process every N frames; 1=no skip, 2/3=skip for perf'),
            DeclareLaunchArgument('accumulate_time', default_value='2', description='Dynamic cloud accumulation window size'),
            DeclareLaunchArgument('publish_accumulated_dynamic_cloud', default_value='true', description='Publish accumulated dynamic cloud instead of current-frame dynamic cloud'),
            DeclareLaunchArgument('cluster_tolerance', default_value='0.25', description='Euclidean cluster tolerance in meters'),
            DeclareLaunchArgument('min_cluster_size', default_value='8', description='Minimum number of points per cluster'),
            DeclareLaunchArgument('max_cluster_size', default_value='1000', description='Maximum number of points per cluster'),
            DeclareLaunchArgument('cluster_voxel_leaf_size', default_value='0.0', description='Voxel downsample size before clustering; 0 disables'),
            DeclareLaunchArgument('min_cluster_diagonal', default_value='0.08', description='Minimum XY diagonal for a valid cluster'),
            DeclareLaunchArgument('max_cluster_diagonal', default_value='2.50', description='Maximum XY diagonal for a valid cluster'),
            DeclareLaunchArgument('min_cluster_height', default_value='0.03', description='Minimum height for a valid cluster'),
            DeclareLaunchArgument('max_cluster_height', default_value='2.20', description='Maximum height for a valid cluster'),
            DeclareLaunchArgument('min_cluster_xy_span', default_value='0.04', description='Minimum X/Y span for a valid cluster'),
            DeclareLaunchArgument('cluster_use_bbox_center', default_value='false', description='Use bounding-box center instead of centroid as cluster output'),
            DeclareLaunchArgument('point_cloud_topic', default_value='/livox/lidar', description='Input PointCloud2 topic'),
            DeclareLaunchArgument('point_cloud_frame_id', default_value='livox_frame', description='Input point cloud frame id used by startup TF'),
            DeclareLaunchArgument('tf_child_frame', default_value='livox_frame', description='TF child frame published by localization/startup TF'),
            DeclareLaunchArgument('static_tf_pitch_rad', default_value='0', description='Static TF pitch (rad), -50°=-0.873'),
            DeclareLaunchArgument('auto_align', default_value='true', description='Auto-level RViz display using rm_frame_display'),
            DeclareLaunchArgument('debug_camera_match', default_value='false', description='Enable camera_match debug logs when red/blue not appearing'),
            DeclareLaunchArgument('camera_detect_radius', default_value='1.0', description='Camera-LiDAR match radius in meters'),
            DeclareLaunchArgument('track_match_radius', default_value='1.0', description='LiDAR Kalman track association radius in meters'),
            DeclareLaunchArgument('publish_stationary_targets', default_value='false', description='Publish stationary targets in /livox/lidar_kalman for debugging'),
            DeclareLaunchArgument('publish_unclassified_targets', default_value='true', description='Publish unclassified gray Kalman targets for debugging'),
            DeclareLaunchArgument('min_unclassified_history', default_value='2', description='Minimum history length before publishing an unclassified gray Kalman target'),
            DeclareLaunchArgument('min_motion_displacement', default_value='0.12', description='Minimum displacement in meters required before an unclassified track is treated as moving'),
            DeclareLaunchArgument('min_motion_time_span', default_value='0.25', description='Minimum observation span in seconds required before an unclassified track is treated as moving'),
            DeclareLaunchArgument('min_motion_history', default_value='3', description='Minimum history length required before an unclassified track is treated as moving'),
            DeclareLaunchArgument('min_motion_speed', default_value='0.08', description='Minimum average speed in m/s required before an unclassified track is treated as moving'),
            static_tf,
            display_aligner,
            delayed_lidar
            ])
