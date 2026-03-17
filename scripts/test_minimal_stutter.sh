#!/bin/bash
# 最小测试：仅 Livox + Localization + RViz（无 dynamic_cloud/cluster/kalman）
# 用于排查 "一顿一顿" 是否来自 dynamic_cloud
# 若此测试流畅 → 问题在 dynamic_cloud；若仍卡顿 → 问题在 livox/localization/RViz

cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash

echo ">>> 最小测试：Livox + Localization + RViz（无 dynamic_cloud）"
echo ">>> 观察 /livox/lidar 是否还一顿一顿"
echo ""

sudo ip addr add 192.168.1.5/24 dev enx00e04c2536b0 2>/dev/null || true

gnome-terminal --title="Livox" -- bash -c "source /opt/ros/humble/setup.bash && source ~/Desktop/RadarStation/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py publish_freq:=10.0; exec bash" &
sleep 2
gnome-terminal --title="Lidar(仅localization)" -- bash -c "source /opt/ros/humble/setup.bash && source ~/Desktop/RadarStation/install/setup.bash && LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar_minimal.launch.py map_file:=mapping_ws/test.pcd; exec bash" &
sleep 2
gnome-terminal --title="RViz" -- bash -c "source /opt/ros/humble/setup.bash && source ~/Desktop/RadarStation/install/setup.bash && rviz2 -d ~/Desktop/RadarStation/src/livox_ros_driver2/config/pointcloud_lidar.rviz; exec bash" &

echo ">>> 已启动。若 /livox/lidar 仍卡顿 → 问题在 livox/localization/RViz"
echo ">>> 若流畅 → 问题在 dynamic_cloud，需优化 GetDynamicCloud"
