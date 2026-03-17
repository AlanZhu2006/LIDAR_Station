#!/bin/bash
# =============================================================================
# RadarStation 雷达站 - FAST-LIO 建图一键启动
# =============================================================================
# 自动用 xfer_format=1（CustomMsg）启动 Livox，无需手动改文件
# 建图完成后：Ctrl+C 停止 → 点云自动保存 → 复制到 mapping_ws/test.pcd
# =============================================================================

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MAPPING_WS="$WS_ROOT/mapping_ws"
LIVOX_IF="${LIVOX_IF:-enx00e04c2536b0}"
LIVOX_LIDAR_IP="${LIVOX_LIDAR_IP:-192.168.1.3}"
LIVOX_CONFIG_TEMPLATE="$WS_ROOT/src/livox_ros_driver2/config/MID360_config.json"
RUNTIME_LIVOX_CONFIG="/tmp/RadarStation_mid360_${LIVOX_LIDAR_IP//./_}.json"

cd "$WS_ROOT"

SETUP_MAIN="source /opt/ros/humble/setup.bash && source $WS_ROOT/install/setup.bash && cd $WS_ROOT"
SETUP_MAPPING="source /opt/ros/humble/setup.bash && source $MAPPING_WS/install/setup.bash && cd $MAPPING_WS"
SETUP_ALL="source /opt/ros/humble/setup.bash && source $WS_ROOT/install/setup.bash && source $MAPPING_WS/install/setup.bash && cd $WS_ROOT"
LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0"

CONFLICT_PATTERNS=(
  "fastlio_mapping"
  "livox_ros_driver2_node"
  "dynamic_cloud_node"
  "cluster_node"
  "kalman_filter_node"
  "localization_node"
  "component_container"
  "nyush_world_node"
  "hik_camera_node"
)

ros2_check() {
  local cmd="$1"
  bash -lc "$SETUP_ALL && $cmd"
}

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

wait_for_ros2_check() {
  local description="$1"
  local cmd="$2"
  local timeout_sec="${3:-20}"
  local deadline=$((SECONDS + timeout_sec))
  while (( SECONDS < deadline )); do
    if ros2_check "$cmd" >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  echo "[ERROR] $description"
  return 1
}

stop_conflicting_processes() {
  local stopped=0
  for pattern in "${CONFLICT_PATTERNS[@]}"; do
    if pgrep -f "$pattern" >/dev/null 2>&1; then
      echo "停止冲突进程: $pattern"
      pkill -f "$pattern" >/dev/null 2>&1 || true
      stopped=1
    fi
  done

  if (( stopped )); then
    sleep 2
  fi
}

ensure_livox_network() {
  local host_ip="${LIVOX_HOST_IP:-192.168.1.5}"

  if ! ip link show dev "$LIVOX_IF" >/dev/null 2>&1; then
    echo "[ERROR] 未找到 Livox 网口: $LIVOX_IF"
    return 1
  fi

  if ip -4 addr show dev "$LIVOX_IF" | grep -Fq "inet $host_ip/"; then
    echo "Livox 网口已配置: $LIVOX_IF -> $host_ip/24"
    return 0
  fi

  echo "配置 Livox 网口 IP: $LIVOX_IF -> $host_ip/24"
  if ! sudo ip addr replace "$host_ip"/24 dev "$LIVOX_IF"; then
    echo "[ERROR] 配置 Livox 网口失败: $LIVOX_IF -> $host_ip/24"
    return 1
  fi

  if ! ip -4 addr show dev "$LIVOX_IF" | grep -Fq "inet $host_ip/"; then
    echo "[ERROR] Livox 网口仍未获得期望地址: $LIVOX_IF -> $host_ip/24"
    ip -4 addr show dev "$LIVOX_IF" || true
    return 1
  fi
}

render_livox_config() {
  if [ ! -f "$LIVOX_CONFIG_TEMPLATE" ]; then
    echo "[ERROR] 未找到 Livox 配置模板: $LIVOX_CONFIG_TEMPLATE"
    return 1
  fi

  python3 - "$LIVOX_CONFIG_TEMPLATE" "$RUNTIME_LIVOX_CONFIG" "$LIVOX_LIDAR_IP" <<'PY'
import json
import sys

template_path, output_path, lidar_ip = sys.argv[1:4]
with open(template_path, "r", encoding="utf-8") as f:
    data = json.load(f)

configs = data.get("lidar_configs", [])
if not configs:
    raise SystemExit("MID360_config.json 缺少 lidar_configs")

configs[0]["ip"] = lidar_ip

with open(output_path, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=2)
    f.write("\n")
PY
}

fastlio_rviz_arg() {
  if command -v gnome-terminal >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
    echo "rviz:=true"
  else
    echo "rviz:=false"
  fi
}

echo "=============================================="
echo "  FAST-LIO 建图"
echo "=============================================="
echo "  1. 空场建图（场地上不要有机器人）"
echo "  2. 拿着雷达走一圈，覆盖全场"
echo "  3. Ctrl+C 停止后，点云保存到："
echo "     $MAPPING_WS/src/FAST_LIO/PCD/scans.pcd"
echo "  4. 复制到 test.pcd："
echo "     cp $MAPPING_WS/src/FAST_LIO/PCD/scans.pcd $MAPPING_WS/test.pcd"
echo "=============================================="

echo "清理与建图冲突的融合/检测进程..."
stop_conflicting_processes
render_livox_config
echo "建图使用雷达 IP: $LIVOX_LIDAR_IP"

ensure_livox_network

# Livox 用 xfer_format=1（CustomMsg，FAST-LIO 需要）
launch_command "Livox(建图)" "$SETUP_MAIN && ros2 launch livox_ros_driver2 msg_MID360_launch.py xfer_format:=1 publish_freq:=10.0 user_config_path:=$RUNTIME_LIVOX_CONFIG"
wait_for_ros2_check "Livox 驱动未切到建图模式（xfer_format!=1）" \
  "ros2 param get /livox_lidar_publisher xfer_format | grep -q 'Integer value is: 1'" 15
wait_for_ros2_check "当前 /livox/lidar 不是 CustomMsg，可能还有别的链路占用该话题" \
  "ros2 topic list -t | grep -Fq '/livox/lidar [livox_ros_driver2/msg/CustomMsg]'" 15

# FAST-LIO
FASTLIO_RVIZ_ARG="$(fastlio_rviz_arg)"
launch_command "FAST-LIO" "$SETUP_MAPPING && LD_PRELOAD=$LD_PRELOAD ros2 launch fast_lio mapping.launch.py config_path:=$MAPPING_WS/src/FAST_LIO/config config_file:=mid360.yaml $FASTLIO_RVIZ_ARG"
wait_for_ros2_check "FAST-LIO 尚未输出 /Odometry" \
  "ros2 topic list -t | grep -Fq '/Odometry [nav_msgs/msg/Odometry]'" 15
wait_for_ros2_check "FAST-LIO 未提供 /map_save 服务" \
  "ros2 service list | grep -q '^/map_save$'" 15
wait_for_ros2_check "FAST-LIO 当前没有发布 /tf" \
  "ros2 topic info -v /tf | grep -q 'Publisher count: 1'" 15

echo ""
echo "已启动 Livox(xfer_format=1) 和 FAST-LIO"
echo "FAST-LIO 当前读取配置：$MAPPING_WS/src/FAST_LIO/config/mid360.yaml"
echo "启动自检通过：/livox/lidar=CustomMsg, /laser_mapping 已启动, /map_save 可用"
echo "建图完成后按 Ctrl+C 停止 FAST-LIO，然后执行："
echo "  cp $MAPPING_WS/src/FAST_LIO/PCD/scans.pcd $MAPPING_WS/test.pcd"
echo "倾斜安装时还需运行：python3 scripts/rotate_pcd.py"
echo ""
