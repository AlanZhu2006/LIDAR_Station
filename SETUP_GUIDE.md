# T-DT-2024-Radar 快速配置指南

> 详细文档请参阅 [README.md](./README.md)

---

## 快速检查清单

### 环境依赖

- [x] ROS2 Humble
- [x] CUDA 11.x+
- [x] TensorRT 8.x - 10.x
- [x] Livox SDK2
- [x] MVS SDK（海康相机）
- [x] Python 依赖：`numpy<2`, `IPython`, `onnxruntime-gpu`

### 编译

```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
colcon build
```

### TensorRT 头文件（如缺失）

```bash
cd /tmp
git clone --depth 1 --branch v10.0.0 https://github.com/NVIDIA/TensorRT.git tensorrt_headers
git clone --depth 1 https://github.com/onnx/onnx-tensorrt.git onnx_tensorrt
cp onnx_tensorrt/NvOnnxParser.h tensorrt_headers/include/
```

---

## 快速启动命令

### 雷达测试（rosbag）

```bash
# 终端 1 - 点云处理
cd ~/Desktop/T-DT-2024-Radar && source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py

# 终端 2 - rosbag 回放
ros2 bag play bag/merged_bag_0.db3 --loop

# 终端 3 - 可视化
rviz2
```

### 实际相机测试（NYUSH）

```bash
# 终端 1 - 相机驱动
cd ~/Desktop/T-DT-2024-Radar && source install/setup.bash
ros2 run hik_camera hik_camera_node

# 终端 2 - NYUSH 检测
ros2 launch tdt_vision nyush_detect.launch.py

# 终端 3 - 查看结果
ros2 run rqt_image_view rqt_image_view  # 选择 /detect_image
```

### 完整系统（雷达 + 视觉）

```bash
# 终端 1 - 雷达驱动
sudo ip addr add 192.168.1.5/24 dev enp4s0
cd ~/Desktop/T-DT-2024-Radar && source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 终端 2 - 点云处理
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py

# 终端 3 - 相机驱动
ros2 run hik_camera hik_camera_node

# 终端 4 - NYUSH 检测
ros2 launch tdt_vision nyush_detect.launch.py

# 终端 5 - RViz
rviz2
```

---

## 当前系统状态

| 模块 | 状态 | 说明 |
|------|------|------|
| 雷达点云处理 | ✅ 正常 | GICP、动态点云、聚类、卡尔曼 |
| T-DT 视觉 (rosbag) | ✅ 正常 | 高分辨率 rosbag 正常 |
| T-DT 视觉 (实际相机) | ❌ 不兼容 | 需 4096×3000 |
| **NYUSH 视觉 (ROS2)** | ✅ 正常 | 实际相机检测正常 |
| 融合 | ⚠️ 需标定 | 需五点标定 |

---

## 关键文件

| 文件 | 说明 |
|------|------|
| `src/tdt_vision/detect/scripts/nyush_detect_node.py` | NYUSH ROS2 检测节点 |
| `src/tdt_vision/launch/nyush_detect.launch.py` | NYUSH 启动文件 |
| `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/` | NYUSH 模型文件 |
| `config/RM2024.pcd` | 场地地图 |
| `config/out_matrix.yaml` | 相机外参（标定输出）|

---

## 话题一览

### 输入
| 话题 | 来源 |
|------|------|
| `/livox/lidar` | 雷达驱动 |
| `/camera_image` | 相机驱动 |

### 输出
| 话题 | 来源 |
|------|------|
| `/livox/lidar_dynamic` | 动态点云 |
| `/livox/cluster` | 聚类结果 |
| `/livox/lidar_kalman` | 卡尔曼跟踪（XYZRGB）|
| `/detect_result` | 视觉检测结果 |
| `/detect_image` | 检测标注图像 |

---

## 下一步

- [ ] **相机外参标定** - 需在实际场地五点标定
- [ ] **参数调优** - 聚类、卡尔曼、融合阈值
- [ ] **分布式部署测试**

---

*详细文档请参阅 [README.md](./README.md)*

*最后更新: 2026-02-28*
