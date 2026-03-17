#!/bin/bash
# 诊断动态目标检测链路：检查各 topic 是否有数据
# 用法：在运行 lidar.launch 和雷达驱动时，另开终端执行此脚本

set -e
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

echo "=== 动态目标检测链路诊断 ==="
echo "请确保：1) 雷达驱动已启动  2) lidar.launch 已启动  3) 场地内有移动的小车"
echo ""

echo "1. 检查 /livox/lidar (原始点云)"
timeout 2 ros2 topic hz /livox/lidar 2>/dev/null || echo "   [无数据] 请先启动雷达驱动"

echo ""
echo "2. 检查 /livox/lidar_dynamic (动态点云)"
timeout 2 ros2 topic hz /livox/lidar_dynamic 2>/dev/null || echo "   [无数据] 检查 TF 或 dynamic_cloud"

echo ""
echo "3. 检查 /livox/lidar_cluster (聚类中心)"
timeout 2 ros2 topic hz /livox/lidar_cluster 2>/dev/null || echo "   [无数据] 检查 cluster 或 lidar_dynamic 是否为空"

echo ""
echo "4. 检查 TF (rm_frame <- livox_frame)"
timeout 1 ros2 run tf2_ros tf2_echo rm_frame livox_frame 2>&1 | head -5 || echo "   [TF 异常] 检查 localization 是否对齐"

echo ""
echo "5. 单次采样 lidar_dynamic 点数"
MSG=$(timeout 1 ros2 topic echo /livox/lidar_dynamic --once 2>/dev/null | grep -c "x:" || echo "0")
echo "   约 $MSG 个点"

echo ""
echo "6. 单次采样 lidar_cluster 点数"
MSG=$(timeout 1 ros2 topic echo /livox/lidar_cluster --once 2>/dev/null | grep -c "x:" || echo "0")
echo "   约 $MSG 个 cluster 中心"

echo ""
echo "=== 若 lidar_dynamic 为空：检查 kd_tree_threshold_sq（可试 kd_tree_threshold_sq:=0.35）==="
echo "=== 若 lidar_cluster 为空但 lidar_dynamic 有数据：检查 cluster MinClusterSize（当前 12）==="
