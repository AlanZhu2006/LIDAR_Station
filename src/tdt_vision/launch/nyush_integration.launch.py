import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nyush_path_arg = DeclareLaunchArgument(
        'nyush_path',
        default_value=os.environ.get('NYUSH_PATH', '/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation'),
        description='Absolute path to the NYUSH radar station project root',
    )
    map_mode_arg = DeclareLaunchArgument(
        'map_mode',
        default_value='testmap',
        description='NYUSH map mode: battle or testmap',
    )
    state_arg = DeclareLaunchArgument(
        'state',
        default_value='B',
        description='Self color used by NYUSH assets: B or R',
    )
    field_width_arg = DeclareLaunchArgument(
        'field_width_m',
        default_value='6.79',
        description='Physical field width in meters for map scaling',
    )
    field_height_arg = DeclareLaunchArgument(
        'field_height_m',
        default_value='3.82',
        description='Physical field height in meters for map scaling',
    )
    calibration_width_arg = DeclareLaunchArgument(
        'calibration_width_px',
        default_value='0',
        description='Calibration image width in pixels. Use 0 to auto-match current camera_image width.',
    )
    calibration_height_arg = DeclareLaunchArgument(
        'calibration_height_px',
        default_value='0',
        description='Calibration image height in pixels. Use 0 to auto-match current camera_image height.',
    )
    test_map_path_arg = DeclareLaunchArgument(
        'test_map_path',
        default_value='',
        description='Optional override for NYUSH test map image path',
    )
    test_calib_map_path_arg = DeclareLaunchArgument(
        'test_calib_map_path',
        default_value='',
        description='Optional override for NYUSH test calibration map image path',
    )
    test_array_path_arg = DeclareLaunchArgument(
        'test_array_path',
        default_value='',
        description='Optional override for NYUSH calibration array path',
    )
    test_mask_path_arg = DeclareLaunchArgument(
        'test_mask_path',
        default_value='',
        description='Optional override for NYUSH mask image path',
    )
    brightness_gamma_arg = DeclareLaunchArgument(
        'brightness_gamma',
        default_value='0.7',
        description='detect_image 亮度 gamma，<1 提亮',
    )
    brightness_bias_arg = DeclareLaunchArgument(
        'brightness_bias',
        default_value='20',
        description='detect_image 亮度偏移',
    )
    brightness_clahe_arg = DeclareLaunchArgument(
        'brightness_clahe',
        default_value='true',
        description='使用 CLAHE 自适应增强',
    )
    window_size_arg = DeclareLaunchArgument(
        'window_size',
        default_value='3',
        description='SlidingWindowFilter 窗口大小，2=低延迟，3=更平滑',
    )
    max_inactive_time_arg = DeclareLaunchArgument(
        'max_inactive_time',
        default_value='2.0',
        description='SlidingWindowFilter 超时清空秒数',
    )
    car_conf_arg = DeclareLaunchArgument(
        'car_conf',
        default_value='0.15',
        description='车框 detector confidence threshold',
    )
    car_iou_arg = DeclareLaunchArgument(
        'car_iou',
        default_value='0.5',
        description='车框 detector IoU threshold',
    )
    armor_conf_arg = DeclareLaunchArgument(
        'armor_conf',
        default_value='0.45',
        description='装甲板 detector confidence threshold',
    )
    armor_iou_arg = DeclareLaunchArgument(
        'armor_iou',
        default_value='0.2',
        description='装甲板 detector IoU threshold',
    )
    armor_roi_expand_ratio_arg = DeclareLaunchArgument(
        'armor_roi_expand_ratio',
        default_value='0.12',
        description='装甲板 ROI 左右/上方扩张比例（相对 car bbox）',
    )
    armor_roi_bottom_expand_ratio_arg = DeclareLaunchArgument(
        'armor_roi_bottom_expand_ratio',
        default_value='0.18',
        description='装甲板 ROI 下方额外扩张比例（相对 car bbox）',
    )
    enhance_armor_roi_on_miss_arg = DeclareLaunchArgument(
        'enhance_armor_roi_on_miss',
        default_value='true',
        description='装甲板首次 miss 时，使用增强后的 ROI 自动重试一次',
    )
    diagnostic_log_every_sec_arg = DeclareLaunchArgument(
        'diagnostic_log_every_sec',
        default_value='2.0',
        description='NYUSH 运行时状态日志节流间隔；0=关闭',
    )
    publish_debug_image_arg = DeclareLaunchArgument(
        'publish_debug_image',
        default_value='true',
        description='是否发布 detect_image 调试图像',
    )
    publish_debug_map_arg = DeclareLaunchArgument(
        'publish_debug_map',
        default_value='false',
        description='是否发布 nyush_map_image 调试地图',
    )
    debug_image_every_n_arg = DeclareLaunchArgument(
        'debug_image_every_n',
        default_value='1',
        description='调试图像发布频率，每 N 帧发布 1 帧',
    )
    process_width_arg = DeclareLaunchArgument(
        'process_width',
        default_value='0',
        description='NYUSH 内部检测处理宽度；0=使用原始 camera_image 宽度',
    )
    process_height_arg = DeclareLaunchArgument(
        'process_height',
        default_value='0',
        description='NYUSH 内部检测处理高度；0=使用原始 camera_image 高度',
    )

    nyush_world_node = Node(
        package='tdt_vision',
        executable='nyush_world_node.py',
        name='nyush_world_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'nyush_path': LaunchConfiguration('nyush_path'),
            'map_mode': LaunchConfiguration('map_mode'),
            'state': LaunchConfiguration('state'),
            'field_width_m': LaunchConfiguration('field_width_m'),
            'field_height_m': LaunchConfiguration('field_height_m'),
            'calibration_width_px': LaunchConfiguration('calibration_width_px'),
            'calibration_height_px': LaunchConfiguration('calibration_height_px'),
            'test_map_path': LaunchConfiguration('test_map_path'),
            'test_calib_map_path': LaunchConfiguration('test_calib_map_path'),
            'test_array_path': LaunchConfiguration('test_array_path'),
            'test_mask_path': LaunchConfiguration('test_mask_path'),
            'brightness_gamma': LaunchConfiguration('brightness_gamma'),
            'brightness_bias': LaunchConfiguration('brightness_bias'),
            'brightness_clahe': LaunchConfiguration('brightness_clahe'),
            'window_size': LaunchConfiguration('window_size'),
            'max_inactive_time': LaunchConfiguration('max_inactive_time'),
            'car_conf': LaunchConfiguration('car_conf'),
            'car_iou': LaunchConfiguration('car_iou'),
            'armor_conf': LaunchConfiguration('armor_conf'),
            'armor_iou': LaunchConfiguration('armor_iou'),
            'armor_roi_expand_ratio': LaunchConfiguration('armor_roi_expand_ratio'),
            'armor_roi_bottom_expand_ratio': LaunchConfiguration('armor_roi_bottom_expand_ratio'),
            'enhance_armor_roi_on_miss': LaunchConfiguration('enhance_armor_roi_on_miss'),
            'diagnostic_log_every_sec': LaunchConfiguration('diagnostic_log_every_sec'),
            'publish_debug_image': LaunchConfiguration('publish_debug_image'),
            'publish_debug_map': LaunchConfiguration('publish_debug_map'),
            'debug_image_every_n': LaunchConfiguration('debug_image_every_n'),
            'process_width': LaunchConfiguration('process_width'),
            'process_height': LaunchConfiguration('process_height'),
        }],
    )

    return LaunchDescription([
        nyush_path_arg,
        map_mode_arg,
        state_arg,
        field_width_arg,
        field_height_arg,
        calibration_width_arg,
        calibration_height_arg,
        test_map_path_arg,
        test_calib_map_path_arg,
        test_array_path_arg,
        test_mask_path_arg,
        brightness_gamma_arg,
        brightness_bias_arg,
        brightness_clahe_arg,
        window_size_arg,
        max_inactive_time_arg,
        car_conf_arg,
        car_iou_arg,
        armor_conf_arg,
        armor_iou_arg,
        armor_roi_expand_ratio_arg,
        armor_roi_bottom_expand_ratio_arg,
        enhance_armor_roi_on_miss_arg,
        diagnostic_log_every_sec_arg,
        publish_debug_image_arg,
        publish_debug_map_arg,
        debug_image_every_n_arg,
        process_width_arg,
        process_height_arg,
        nyush_world_node,
    ])
