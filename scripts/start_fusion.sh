#!/bin/bash
# RadarStation 雷达站 - 视觉+雷达融合 一键启动（对应 LIDAR.txt 模式三）
# 用法：./scripts/start_fusion.sh [--low-latency]
cd ~/Desktop/RadarStation

LOW_LATENCY=false
[[ "$1" == "--low-latency" ]] && LOW_LATENCY=true

# 清理残留 Livox 进程（避免 bind failed）
pkill -f livox_ros_driver2_node 2>/dev/null || true
sleep 1

# 配置雷达网口 IP
sudo ip addr add 192.168.1.5/24 dev enx00e04c2536b0 2>/dev/null || true

SETUP="source /opt/ros/humble/setup.bash && source ~/Desktop/RadarStation/install/setup.bash && cd ~/Desktop/RadarStation"

if $LOW_LATENCY; then
  PUB_FREQ=10.0
  VOXEL=0.3
  ACCUM=3
  CAM_W=960
  CAM_H=720
  DEBUG_EVERY_N=3
  DEFAULT_PROCESS_W=960
  DEFAULT_PROCESS_H=720
  echo ">>> 低延迟模式: publish_freq=$PUB_FREQ, voxel=$VOXEL, accumulate=$ACCUM, camera=${CAM_W}x${CAM_H}, debug_image_every_n=$DEBUG_EVERY_N"
else
  PUB_FREQ=10.0
  VOXEL=0.35
  ACCUM=3
  CAM_W=1280
  CAM_H=960
  DEBUG_EVERY_N=2
  DEFAULT_PROCESS_W=960
  DEFAULT_PROCESS_H=720
  echo ">>> 默认模式: publish_freq=$PUB_FREQ, voxel=$VOXEL, accumulate=$ACCUM, camera=${CAM_W}x${CAM_H}, debug_image_every_n=$DEBUG_EVERY_N"
fi

gnome-terminal --title="Livox" -- bash -c "$SETUP && ros2 launch livox_ros_driver2 msg_MID360_launch.py xfer_format:=0 publish_freq:=$PUB_FREQ; exec bash" &
MAP_PATH="$HOME/Desktop/RadarStation/mapping_ws/test.pcd"
DEFAULT_MAP="$HOME/Desktop/RadarStation/config/RM2024.pcd"
NYUSH_PATH="${NYUSH_PATH:-$HOME/Desktop/NYUSH_Robotics_RM_RadarStation}"
LIDAR_PITCH_DEG="50.0"
USE_TILT_CORRECTION="${USE_TILT_CORRECTION:-false}"
KD_TREE_THRESHOLD_SQ="${KD_TREE_THRESHOLD_SQ:-0.15}"
CLUSTER_TOLERANCE="${CLUSTER_TOLERANCE:-0.25}"
MIN_CLUSTER_SIZE="${MIN_CLUSTER_SIZE:-8}"
DEBUG_CAMERA_MATCH="${DEBUG_CAMERA_MATCH:-false}"
CAMERA_DETECT_RADIUS="${CAMERA_DETECT_RADIUS:-1.0}"
PUBLISH_STATIONARY_TARGETS="${PUBLISH_STATIONARY_TARGETS:-false}"
SELF_COLOR="${SELF_COLOR:-R}"
FIELD_WIDTH_M="${FIELD_WIDTH_M:-6.37}"
FIELD_HEIGHT_M="${FIELD_HEIGHT_M:-11.32}"
WINDOW_SIZE="${WINDOW_SIZE:-2}"
MAX_INACTIVE_TIME="${MAX_INACTIVE_TIME:-2.0}"
CAR_CONF="${CAR_CONF:-0.15}"
CAR_IOU="${CAR_IOU:-0.50}"
ARMOR_CONF="${ARMOR_CONF:-0.45}"
ARMOR_IOU="${ARMOR_IOU:-0.20}"
ARMOR_ROI_EXPAND_RATIO="${ARMOR_ROI_EXPAND_RATIO:-0.12}"
ARMOR_ROI_BOTTOM_EXPAND_RATIO="${ARMOR_ROI_BOTTOM_EXPAND_RATIO:-0.18}"
ENHANCE_ARMOR_ROI_ON_MISS="${ENHANCE_ARMOR_ROI_ON_MISS:-true}"
DIAGNOSTIC_LOG_EVERY_SEC="${DIAGNOSTIC_LOG_EVERY_SEC:-2.0}"
PUBLISH_DEBUG_IMAGE="${PUBLISH_DEBUG_IMAGE:-true}"
PUBLISH_DEBUG_MAP="${PUBLISH_DEBUG_MAP:-true}"
IMAGE_VIEW_WAIT_TIMEOUT_SEC="${IMAGE_VIEW_WAIT_TIMEOUT_SEC:-30}"
BRIGHTNESS_GAMMA="${BRIGHTNESS_GAMMA:-0.8}"
BRIGHTNESS_BIAS="${BRIGHTNESS_BIAS:-25}"
BRIGHTNESS_CLAHE="${BRIGHTNESS_CLAHE:-true}"
PROCESS_WIDTH="${PROCESS_WIDTH:-$DEFAULT_PROCESS_W}"
PROCESS_HEIGHT="${PROCESS_HEIGHT:-$DEFAULT_PROCESS_H}"

if [[ ! -f "$MAP_PATH" ]]; then
  MAP_PATH="$DEFAULT_MAP"
fi

echo ">>> LiDAR map: $MAP_PATH"
echo ">>> Detection tuning: kd_tree_threshold_sq=$KD_TREE_THRESHOLD_SQ, cluster_tolerance=$CLUSTER_TOLERANCE, min_cluster_size=$MIN_CLUSTER_SIZE"
echo ">>> Fusion debug: debug_camera_match=$DEBUG_CAMERA_MATCH, camera_detect_radius=$CAMERA_DETECT_RADIUS, publish_stationary_targets=$PUBLISH_STATIONARY_TARGETS"
echo ">>> Vision tuning: self_color=$SELF_COLOR, field=${FIELD_WIDTH_M}x${FIELD_HEIGHT_M}m, calibration=${CAM_W}x${CAM_H}"
echo ">>> NYUSH tuning: path=$NYUSH_PATH, window_size=$WINDOW_SIZE, max_inactive_time=$MAX_INACTIVE_TIME, car_conf=$CAR_CONF, armor_conf=$ARMOR_CONF"
echo ">>> NYUSH armor ROI: expand=$ARMOR_ROI_EXPAND_RATIO, bottom_expand=$ARMOR_ROI_BOTTOM_EXPAND_RATIO, retry_enhance=$ENHANCE_ARMOR_ROI_ON_MISS"
echo ">>> NYUSH debug: publish_debug_image=$PUBLISH_DEBUG_IMAGE, publish_debug_map=$PUBLISH_DEBUG_MAP, debug_image_every_n=$DEBUG_EVERY_N, process=${PROCESS_WIDTH}x${PROCESS_HEIGHT}, diagnostic_log_every_sec=$DIAGNOSTIC_LOG_EVERY_SEC"
echo ">>> rqt_image_view wait timeout: ${IMAGE_VIEW_WAIT_TIMEOUT_SEC}s"
if [[ "$USE_TILT_CORRECTION" == "true" ]]; then
  echo ">>> LiDAR launch: lidar_tilt.launch.py (pitch=$LIDAR_PITCH_DEG deg)"
  LIDAR_CMD="LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar_tilt.launch.py map_file:=$MAP_PATH ceiling_z_max:=100.0 voxel_leaf_size:=$VOXEL accumulate_time:=$ACCUM auto_align:=true lidar_pitch_deg:=$LIDAR_PITCH_DEG kd_tree_threshold_sq:=$KD_TREE_THRESHOLD_SQ cluster_tolerance:=$CLUSTER_TOLERANCE min_cluster_size:=$MIN_CLUSTER_SIZE debug_camera_match:=$DEBUG_CAMERA_MATCH camera_detect_radius:=$CAMERA_DETECT_RADIUS publish_stationary_targets:=$PUBLISH_STATIONARY_TARGETS"
else
  echo ">>> LiDAR launch: lidar.launch.py (raw cloud already level)"
  LIDAR_CMD="LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py map_file:=$MAP_PATH ceiling_z_max:=100.0 voxel_leaf_size:=$VOXEL accumulate_time:=$ACCUM auto_align:=true kd_tree_threshold_sq:=$KD_TREE_THRESHOLD_SQ cluster_tolerance:=$CLUSTER_TOLERANCE min_cluster_size:=$MIN_CLUSTER_SIZE debug_camera_match:=$DEBUG_CAMERA_MATCH camera_detect_radius:=$CAMERA_DETECT_RADIUS publish_stationary_targets:=$PUBLISH_STATIONARY_TARGETS"
fi
gnome-terminal --title="Lidar" -- bash -c "$SETUP && $LIDAR_CMD; exec bash" &
gnome-terminal --title="Camera" -- bash -c "$SETUP && ros2 run hik_camera hik_camera_node --ros-args -p publish_width:=$CAM_W -p publish_height:=$CAM_H -p poll_period_ms:=20 -p grab_timeout_ms:=5 -p target_fps:=30.0 -p brightness_gamma:=0.8 -p brightness_bias:=25; exec bash" &
gnome-terminal --title="NYUSH" -- bash -c "$SETUP && ros2 launch tdt_vision nyush_integration.launch.py nyush_path:=$NYUSH_PATH map_mode:=testmap state:=$SELF_COLOR field_width_m:=$FIELD_WIDTH_M field_height_m:=$FIELD_HEIGHT_M calibration_width_px:=$CAM_W calibration_height_px:=$CAM_H brightness_gamma:=$BRIGHTNESS_GAMMA brightness_bias:=$BRIGHTNESS_BIAS brightness_clahe:=$BRIGHTNESS_CLAHE window_size:=$WINDOW_SIZE max_inactive_time:=$MAX_INACTIVE_TIME car_conf:=$CAR_CONF car_iou:=$CAR_IOU armor_conf:=$ARMOR_CONF armor_iou:=$ARMOR_IOU armor_roi_expand_ratio:=$ARMOR_ROI_EXPAND_RATIO armor_roi_bottom_expand_ratio:=$ARMOR_ROI_BOTTOM_EXPAND_RATIO enhance_armor_roi_on_miss:=$ENHANCE_ARMOR_ROI_ON_MISS diagnostic_log_every_sec:=$DIAGNOSTIC_LOG_EVERY_SEC publish_debug_image:=$PUBLISH_DEBUG_IMAGE publish_debug_map:=$PUBLISH_DEBUG_MAP debug_image_every_n:=$DEBUG_EVERY_N process_width:=$PROCESS_WIDTH process_height:=$PROCESS_HEIGHT; exec bash" &
gnome-terminal --title="RViz" -- bash -c "$SETUP && rviz2 -d ~/Desktop/RadarStation/src/livox_ros_driver2/config/pointcloud_lidar.rviz; exec bash" &
gnome-terminal --title="rqt_image_view" -- bash -c "$SETUP && PUBLISH_DEBUG_IMAGE=$PUBLISH_DEBUG_IMAGE PUBLISH_DEBUG_MAP=$PUBLISH_DEBUG_MAP IMAGE_VIEW_WAIT_TIMEOUT_SEC=$IMAGE_VIEW_WAIT_TIMEOUT_SEC bash ./scripts/wait_for_image_topics.sh; exec bash" &
