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

echo "0. 真实 ROS 图（--no-daemon，避免缓存误判）"
timeout 3 ros2 node list --no-daemon 2>/dev/null | sed 's/^/   /' || echo "   无节点"
echo ""

python3 scripts/check_latency.py --duration "${LATENCY_SAMPLE_SEC:-4.0}" || echo "   采样失败"

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
