#!/bin/bash
# 诊断 "一顿一顿" 根因：监控发布频率、TF 更新、CPU
# 用法：先启动 start_fusion.sh，再开新终端运行 ./scripts/diagnose_stutter.sh

cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "诊断 1: /livox/lidar 发布频率 (10秒)"
echo "  期望: ~10 Hz，若波动大或偏低则异常"
echo "=========================================="
timeout 10 ros2 topic hz /livox/lidar 2>/dev/null || true

echo ""
echo "=========================================="
echo "诊断 2: TF (rm_frame->livox_frame) 更新间隔"
echo "  期望: 每 ~50ms 有一次，若出现大间隔则卡顿"
echo "=========================================="
echo "运行 5 秒，观察 tf_echo 输出时间戳间隔..."
timeout 5 ros2 run tf2_ros tf2_echo rm_frame livox_frame 2>&1 | head -80 || true

echo ""
echo "=========================================="
echo "诊断 3: 查看 Lidar 终端 GICP 日志"
echo "  关注: GICP time=XXms (单次耗时)"
echo "  若 >150ms 则 GICP 是瓶颈"
echo "=========================================="
echo "请手动查看 Lidar 终端窗口的 GICP 输出"
