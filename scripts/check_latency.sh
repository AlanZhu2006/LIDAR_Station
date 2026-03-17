#!/bin/bash
# 融合运行时，在另一终端执行此脚本，检查各话题频率和延迟
# 用法：./scripts/check_latency.sh

echo "=========================================="
echo "  延迟诊断（需先启动融合）"
echo "=========================================="
echo ""

cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

print_param() {
  local node="$1"
  local param="$2"
  local label="$3"
  local raw
  raw=$(timeout 3 ros2 param get "$node" "$param" 2>/dev/null | tail -n 1 || true)
  if [[ -z "$raw" ]]; then
    echo "   $label: 节点未运行或参数不可读"
    return
  fi
  echo "   $label: ${raw#*: }"
}

echo "1. 话题频率（运行 5 秒后 Ctrl+C 可提前结束）"
echo "   camera_image:"
timeout 5 ros2 topic hz /camera_image 2>/dev/null || echo "   无数据"
echo "   livox/lidar:"
timeout 5 ros2 topic hz /livox/lidar 2>/dev/null || echo "   无数据"
echo "   resolve_result:"
timeout 5 ros2 topic hz /resolve_result 2>/dev/null || echo "   无数据"
echo "   livox/lidar_cluster:"
timeout 5 ros2 topic hz /livox/lidar_cluster 2>/dev/null || echo "   无数据"
echo "   livox/lidar_kalman:"
timeout 5 ros2 topic hz /livox/lidar_kalman 2>/dev/null || echo "   无数据"

echo ""
echo "2. 当前节点参数（若节点在运行）"
print_param /nyush_world_node window_size "NYUSH window_size"
print_param /nyush_world_node max_inactive_time "NYUSH max_inactive_time"
print_param /nyush_world_node car_conf "NYUSH car_conf"
print_param /nyush_world_node armor_conf "NYUSH armor_conf"
print_param /kalman_filter_node debug_camera_match "Kalman debug_camera_match"
echo ""
echo "3. 详细分析见 docs/LATENCY_ANALYSIS.md"
echo "=========================================="
