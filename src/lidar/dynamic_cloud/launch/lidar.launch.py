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
    accumulate_time = LaunchConfiguration('accumulate_time', default='3')
    cluster_tolerance = LaunchConfiguration('cluster_tolerance', default='0.25')
    min_cluster_size = LaunchConfiguration('min_cluster_size', default='12')
    max_cluster_size = LaunchConfiguration('max_cluster_size', default='1000')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic', default='/livox/lidar')
    point_cloud_frame_id = LaunchConfiguration('point_cloud_frame_id', default='livox_frame')
    tf_child_frame = LaunchConfiguration('tf_child_frame', default='livox_frame')
    static_tf_pitch_rad = LaunchConfiguration('static_tf_pitch_rad', default='0')
    auto_align = LaunchConfiguration('auto_align', default='true')
    camera_detect_radius = LaunchConfiguration('camera_detect_radius', default='1.0')
    publish_stationary_targets = LaunchConfiguration('publish_stationary_targets', default='false')
    
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
                'publish_stationary_targets': publish_stationary_targets,
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
            executable='component_container',
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
            DeclareLaunchArgument('accumulate_time', default_value='3', description='Dynamic cloud accumulation window size'),
            DeclareLaunchArgument('cluster_tolerance', default_value='0.25', description='Euclidean cluster tolerance in meters'),
            DeclareLaunchArgument('min_cluster_size', default_value='12', description='Minimum number of points per cluster'),
            DeclareLaunchArgument('max_cluster_size', default_value='1000', description='Maximum number of points per cluster'),
            DeclareLaunchArgument('point_cloud_topic', default_value='/livox/lidar', description='Input PointCloud2 topic'),
            DeclareLaunchArgument('point_cloud_frame_id', default_value='livox_frame', description='Input point cloud frame id used by startup TF'),
            DeclareLaunchArgument('tf_child_frame', default_value='livox_frame', description='TF child frame published by localization/startup TF'),
            DeclareLaunchArgument('static_tf_pitch_rad', default_value='0', description='Static TF pitch (rad), -50°=-0.873'),
            DeclareLaunchArgument('auto_align', default_value='true', description='Auto-level RViz display using rm_frame_display'),
            DeclareLaunchArgument('debug_camera_match', default_value='false', description='Enable camera_match debug logs when red/blue not appearing'),
            DeclareLaunchArgument('camera_detect_radius', default_value='1.0', description='Camera-LiDAR match radius in meters'),
            DeclareLaunchArgument('publish_stationary_targets', default_value='false', description='Publish stationary targets in /livox/lidar_kalman for debugging'),
            static_tf,
            display_aligner,
            delayed_lidar
            ])
