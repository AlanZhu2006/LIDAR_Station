#!/bin/bash
# RadarStation 雷达站 - 视觉+雷达融合 一键启动（对应 LIDAR.txt 模式三）
# 用法：./scripts/start_fusion.sh [--low-latency]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
LIVOX_IF="${LIVOX_IF:-enx00e04c2536b0}"
LIVOX_LIDAR_IP="${LIVOX_LIDAR_IP:-192.168.1.3}"
LIVOX_HOST_IP="${LIVOX_HOST_IP:-192.168.1.5}"
LIVOX_CONFIG_TEMPLATE="$WS_ROOT/src/livox_ros_driver2/config/MID360_config.json"
RUNTIME_LIVOX_CONFIG="/tmp/RadarStation_mid360_${LIVOX_LIDAR_IP//./_}.json"

cd "$WS_ROOT"

LOW_LATENCY=false
[[ "${1:-}" == "--low-latency" ]] && LOW_LATENCY=true

CONFLICT_PATTERNS=(
  "component_container_mt"
  "component_container"
  "nyush_world_node"
  "hik_camera_node"
  "livox_ros_driver2_node"
  "rviz2"
  "rqt_image_view"
  "pointcloud_transform_node"
  "display_aligner_node.py"
)

launch_command() {
  local title="$1"
  local cmd="$2"
  if command -v gnome-terminal >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    gnome-terminal --title="$title" -- bash -lc "$cmd; exec bash" &
  else
    local safe_title
    safe_title="$(printf '%s' "$title" | tr -cs 'A-Za-z0-9' '_')"
    local log_file="/tmp/${safe_title}.log"
    nohup bash -lc "$cmd" >"$log_file" 2>&1 </dev/null &
    echo "$title 日志: $log_file"
  fi
}

stop_conflicting_processes() {
  local stopped=0
  for pattern in "${CONFLICT_PATTERNS[@]}"; do
    if pgrep -f "$pattern" >/dev/null 2>&1; then
      echo "停止旧进程: $pattern"
      pkill -f "$pattern" >/dev/null 2>&1 || true
      stopped=1
    fi
  done
  if (( stopped )); then
    sleep 2
  fi
}

ensure_livox_network() {
  if ! ip link show dev "$LIVOX_IF" >/dev/null 2>&1; then
    echo "[ERROR] 未找到 Livox 网口: $LIVOX_IF"
    exit 1
  fi

  if ip -4 addr show dev "$LIVOX_IF" | grep -Fq "inet $LIVOX_HOST_IP/"; then
    echo ">>> Livox 网口已配置: $LIVOX_IF -> $LIVOX_HOST_IP/24"
    return 0
  fi

  echo ">>> 配置 Livox 网口 IP: $LIVOX_IF -> $LIVOX_HOST_IP/24"
  if ! sudo ip addr replace "$LIVOX_HOST_IP"/24 dev "$LIVOX_IF"; then
    echo "[ERROR] 配置 Livox 网口失败: $LIVOX_IF -> $LIVOX_HOST_IP/24"
    exit 1
  fi

  if ! ip -4 addr show dev "$LIVOX_IF" | grep -Fq "inet $LIVOX_HOST_IP/"; then
    echo "[ERROR] Livox 网口仍未获得期望地址: $LIVOX_IF -> $LIVOX_HOST_IP/24"
    ip -4 addr show dev "$LIVOX_IF" || true
    exit 1
  fi
}

render_livox_config() {
  if [ ! -f "$LIVOX_CONFIG_TEMPLATE" ]; then
    echo "[ERROR] 未找到 Livox 配置模板: $LIVOX_CONFIG_TEMPLATE"
    exit 1
  fi

  python3 - "$LIVOX_CONFIG_TEMPLATE" "$RUNTIME_LIVOX_CONFIG" "$LIVOX_LIDAR_IP" "$LIVOX_HOST_IP" <<'PY'
import json
import sys

template_path, output_path, lidar_ip, host_ip = sys.argv[1:5]
with open(template_path, "r", encoding="utf-8") as f:
    data = json.load(f)

configs = data.get("lidar_configs", [])
if not configs:
    raise SystemExit("MID360_config.json 缺少 lidar_configs")
configs[0]["ip"] = lidar_ip

host = data.get("MID360", {}).get("host_net_info", {})
for key in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
    if key in host:
        host[key] = host_ip

with open(output_path, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=2)
    f.write("\n")
PY
}

echo ">>> 清理旧的 fusion 进程"
stop_conflicting_processes

render_livox_config

ensure_livox_network

SETUP="source /opt/ros/humble/setup.bash && source $WS_ROOT/install/setup.bash && cd $WS_ROOT"

if $LOW_LATENCY; then
  PUB_FREQ=10.0
  VOXEL=0.3
  ACCUM=1
  CAM_W=960
  CAM_H=720
  DEBUG_EVERY_N=3
  DEFAULT_PROCESS_W=960
  DEFAULT_PROCESS_H=720
  DEFAULT_PROCESS_EVERY_N=2
  DEFAULT_CLUSTER_VOXEL_LEAF_SIZE=0.12
  DEFAULT_AUTO_ALIGN=false
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
  DEFAULT_PROCESS_EVERY_N=1
  DEFAULT_CLUSTER_VOXEL_LEAF_SIZE=0.0
  DEFAULT_AUTO_ALIGN=true
  echo ">>> 默认模式: publish_freq=$PUB_FREQ, voxel=$VOXEL, accumulate=$ACCUM, camera=${CAM_W}x${CAM_H}, debug_image_every_n=$DEBUG_EVERY_N"
fi

MAP_PATH="$WS_ROOT/mapping_ws/test.pcd"
DEFAULT_MAP="$WS_ROOT/config/RM2024.pcd"
NYUSH_PATH="${NYUSH_PATH:-$HOME/Desktop/NYUSH_Robotics_RM_RadarStation}"
LIDAR_PITCH_DEG="50.0"
USE_TILT_CORRECTION="${USE_TILT_CORRECTION:-false}"
KD_TREE_THRESHOLD_SQ="${KD_TREE_THRESHOLD_SQ:-0.15}"
CLUSTER_TOLERANCE="${CLUSTER_TOLERANCE:-0.25}"
MIN_CLUSTER_SIZE="${MIN_CLUSTER_SIZE:-8}"
DEBUG_CAMERA_MATCH="${DEBUG_CAMERA_MATCH:-false}"
CAMERA_DETECT_RADIUS="${CAMERA_DETECT_RADIUS:-1.0}"
TRACK_MATCH_RADIUS="${TRACK_MATCH_RADIUS:-1.0}"
PUBLISH_STATIONARY_TARGETS="${PUBLISH_STATIONARY_TARGETS:-false}"
SELF_COLOR="${SELF_COLOR:-R}"
FIELD_WIDTH_M="${FIELD_WIDTH_M:-6.79}"
FIELD_HEIGHT_M="${FIELD_HEIGHT_M:-3.82}"
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
PROCESS_EVERY_N="${PROCESS_EVERY_N:-$DEFAULT_PROCESS_EVERY_N}"
CLUSTER_VOXEL_LEAF_SIZE="${CLUSTER_VOXEL_LEAF_SIZE:-$DEFAULT_CLUSTER_VOXEL_LEAF_SIZE}"
AUTO_ALIGN="${AUTO_ALIGN:-$DEFAULT_AUTO_ALIGN}"
CALIBRATION_WIDTH_PX="${CALIBRATION_WIDTH_PX:-0}"
CALIBRATION_HEIGHT_PX="${CALIBRATION_HEIGHT_PX:-0}"

if [[ ! -f "$MAP_PATH" ]]; then
  MAP_PATH="$DEFAULT_MAP"
fi

echo ">>> LiDAR map: $MAP_PATH"
echo ">>> Livox network: if=$LIVOX_IF, host_ip=$LIVOX_HOST_IP, lidar_ip=$LIVOX_LIDAR_IP"
echo ">>> Detection tuning: kd_tree_threshold_sq=$KD_TREE_THRESHOLD_SQ, cluster_tolerance=$CLUSTER_TOLERANCE, min_cluster_size=$MIN_CLUSTER_SIZE"
echo ">>> Fusion debug: debug_camera_match=$DEBUG_CAMERA_MATCH, camera_detect_radius=$CAMERA_DETECT_RADIUS, track_match_radius=$TRACK_MATCH_RADIUS, publish_stationary_targets=$PUBLISH_STATIONARY_TARGETS"
if [[ "$CALIBRATION_WIDTH_PX" == "0" || "$CALIBRATION_HEIGHT_PX" == "0" ]]; then
  CALIBRATION_DESC="auto(metadata/raw)"
else
  CALIBRATION_DESC="${CALIBRATION_WIDTH_PX}x${CALIBRATION_HEIGHT_PX}"
fi
echo ">>> Vision tuning: self_color=$SELF_COLOR, field=${FIELD_WIDTH_M}x${FIELD_HEIGHT_M}m, camera=${CAM_W}x${CAM_H}, calibration=${CALIBRATION_DESC}"
echo ">>> NYUSH tuning: path=$NYUSH_PATH, window_size=$WINDOW_SIZE, max_inactive_time=$MAX_INACTIVE_TIME, car_conf=$CAR_CONF, armor_conf=$ARMOR_CONF"
echo ">>> NYUSH armor ROI: expand=$ARMOR_ROI_EXPAND_RATIO, bottom_expand=$ARMOR_ROI_BOTTOM_EXPAND_RATIO, retry_enhance=$ENHANCE_ARMOR_ROI_ON_MISS"
echo ">>> NYUSH debug: publish_debug_image=$PUBLISH_DEBUG_IMAGE, publish_debug_map=$PUBLISH_DEBUG_MAP, debug_image_every_n=$DEBUG_EVERY_N, process=${PROCESS_WIDTH}x${PROCESS_HEIGHT}, diagnostic_log_every_sec=$DIAGNOSTIC_LOG_EVERY_SEC"
echo ">>> LiDAR runtime: process_every_n=$PROCESS_EVERY_N, accumulate_time=$ACCUM, cluster_voxel_leaf_size=$CLUSTER_VOXEL_LEAF_SIZE, auto_align=$AUTO_ALIGN"
echo ">>> rqt_image_view wait timeout: ${IMAGE_VIEW_WAIT_TIMEOUT_SEC}s"
if [[ "$USE_TILT_CORRECTION" == "true" ]]; then
  echo ">>> LiDAR launch: lidar_tilt.launch.py (pitch=$LIDAR_PITCH_DEG deg)"
  LIDAR_CMD="LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar_tilt.launch.py map_file:=$MAP_PATH ceiling_z_max:=100.0 voxel_leaf_size:=$VOXEL process_every_n:=$PROCESS_EVERY_N accumulate_time:=$ACCUM auto_align:=$AUTO_ALIGN lidar_pitch_deg:=$LIDAR_PITCH_DEG kd_tree_threshold_sq:=$KD_TREE_THRESHOLD_SQ cluster_tolerance:=$CLUSTER_TOLERANCE min_cluster_size:=$MIN_CLUSTER_SIZE cluster_voxel_leaf_size:=$CLUSTER_VOXEL_LEAF_SIZE debug_camera_match:=$DEBUG_CAMERA_MATCH camera_detect_radius:=$CAMERA_DETECT_RADIUS track_match_radius:=$TRACK_MATCH_RADIUS publish_stationary_targets:=$PUBLISH_STATIONARY_TARGETS"
else
  echo ">>> LiDAR launch: lidar.launch.py (raw cloud already level)"
  LIDAR_CMD="LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py map_file:=$MAP_PATH ceiling_z_max:=100.0 voxel_leaf_size:=$VOXEL process_every_n:=$PROCESS_EVERY_N accumulate_time:=$ACCUM auto_align:=$AUTO_ALIGN kd_tree_threshold_sq:=$KD_TREE_THRESHOLD_SQ cluster_tolerance:=$CLUSTER_TOLERANCE min_cluster_size:=$MIN_CLUSTER_SIZE cluster_voxel_leaf_size:=$CLUSTER_VOXEL_LEAF_SIZE debug_camera_match:=$DEBUG_CAMERA_MATCH camera_detect_radius:=$CAMERA_DETECT_RADIUS track_match_radius:=$TRACK_MATCH_RADIUS publish_stationary_targets:=$PUBLISH_STATIONARY_TARGETS"
fi
launch_command "Livox" "$SETUP && ros2 launch livox_ros_driver2 msg_MID360_launch.py xfer_format:=0 publish_freq:=$PUB_FREQ user_config_path:=$RUNTIME_LIVOX_CONFIG"
launch_command "Lidar" "$SETUP && $LIDAR_CMD"
launch_command "Camera" "$SETUP && ros2 run hik_camera hik_camera_node --ros-args -p publish_width:=$CAM_W -p publish_height:=$CAM_H -p poll_period_ms:=20 -p grab_timeout_ms:=5 -p target_fps:=30.0 -p brightness_gamma:=$BRIGHTNESS_GAMMA -p brightness_bias:=$BRIGHTNESS_BIAS"
launch_command "NYUSH" "$SETUP && ros2 launch tdt_vision nyush_integration.launch.py nyush_path:=$NYUSH_PATH map_mode:=testmap state:=$SELF_COLOR field_width_m:=$FIELD_WIDTH_M field_height_m:=$FIELD_HEIGHT_M calibration_width_px:=$CALIBRATION_WIDTH_PX calibration_height_px:=$CALIBRATION_HEIGHT_PX brightness_gamma:=$BRIGHTNESS_GAMMA brightness_bias:=$BRIGHTNESS_BIAS brightness_clahe:=$BRIGHTNESS_CLAHE window_size:=$WINDOW_SIZE max_inactive_time:=$MAX_INACTIVE_TIME car_conf:=$CAR_CONF car_iou:=$CAR_IOU armor_conf:=$ARMOR_CONF armor_iou:=$ARMOR_IOU armor_roi_expand_ratio:=$ARMOR_ROI_EXPAND_RATIO armor_roi_bottom_expand_ratio:=$ARMOR_ROI_BOTTOM_EXPAND_RATIO enhance_armor_roi_on_miss:=$ENHANCE_ARMOR_ROI_ON_MISS diagnostic_log_every_sec:=$DIAGNOSTIC_LOG_EVERY_SEC publish_debug_image:=$PUBLISH_DEBUG_IMAGE publish_debug_map:=$PUBLISH_DEBUG_MAP debug_image_every_n:=$DEBUG_EVERY_N process_width:=$PROCESS_WIDTH process_height:=$PROCESS_HEIGHT"
launch_command "RViz" "$SETUP && rviz2 -d $WS_ROOT/src/livox_ros_driver2/config/pointcloud_lidar.rviz"
launch_command "rqt_image_view" "$SETUP && PUBLISH_DEBUG_IMAGE=$PUBLISH_DEBUG_IMAGE PUBLISH_DEBUG_MAP=$PUBLISH_DEBUG_MAP IMAGE_VIEW_WAIT_TIMEOUT_SEC=$IMAGE_VIEW_WAIT_TIMEOUT_SEC bash ./scripts/wait_for_image_topics.sh"
