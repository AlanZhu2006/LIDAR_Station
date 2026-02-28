<div align="center">

# NYU Robotics Radar Station

> NYU Robotics RoboMaster 雷达站代码
> 
> 基于 T-DT 2024 Radar 项目

<a href="./LICENSE"><img alt="License" src="https://img.shields.io/badge/License-MIT-yellow"></a>

</div>

---

## 目录

- [项目介绍](#项目介绍)
- [当前系统状态](#当前系统状态)
- [系统要求](#系统要求)
- [快速开始](#快速开始)
- [详细安装指南](#详细安装指南)
- [编译项目](#编译项目)
- [运行项目](#运行项目)
- [RViz 可视化配置](#rviz-可视化配置)
- [话题说明](#话题说明)
- [视觉方案详解](#视觉方案详解)
- [雷达方案详解](#雷达方案详解)
- [传感器融合](#传感器融合)
- [LiDAR 建图](#lidar-建图)
- [相机外参标定](#相机外参标定)
- [多台电脑分布式运行](#多台电脑分布式运行)
- [常见问题](#常见问题)
- [项目结构](#项目结构)
- [代码修改记录](#代码修改记录)
- [更新日志](#更新日志)

---

## 项目介绍

本项目通过激光雷达和单目相机的目标检测，进行传感器后融合，实现了传感器之间的完全解耦合，避免了联合标定带来的误差。

**原项目技术报告**: [T-DT 2024 雷达技术报告](https://bbs.robomaster.com/wiki/260375/27115)

### 主要特性

- 即插即用，不依赖联合标定
- 不依赖相机和雷达之间的帧间匹配
- 雷达全自动 GICP 配准
- 低耦合，易于维护和扩展
- **双视觉方案**：支持 T-DT ROS2 (C++) 方案和 NYUSH ROS2 (Python) 方案

### 硬件配置

| 设备 | 型号 |
|------|------|
| 激光雷达 | Livox Mid-360 |
| 单目相机 | Hikvision MV-CA016-10UC (1440×1080) |
| CPU | i7-12700KF 或同等性能 |
| GPU | NVIDIA RTX 3070 或同等性能 |

---

## 当前系统状态

| 模块 | 状态 | 说明 |
|------|------|------|
| **LiDAR 建图 (FAST-LIO)** | ✅ 正常 | 使用 FAST-LIO 建图，生成 PCD 地图文件 |
| 雷达点云处理 | ✅ 正常 | GICP 配准、动态点云、聚类、卡尔曼 |
| T-DT 视觉 (rosbag) | ✅ 正常 | 高分辨率 rosbag 测试正常 |
| T-DT 视觉 (实际相机) | ❌ 不兼容 | 模型需 4096×3000，相机 1440×1080 |
| **NYUSH 视觉 (ROS2)** | ✅ 正常 | 已移植到 ROS2，实际相机检测正常 |
| 相机-雷达融合 | ⚠️ 需标定 | 需在实际场地五点标定 |

---

## 系统要求

| 软件 | 版本要求 |
|------|----------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| CUDA | 11.x 或更高 |
| cuDNN | 8.x 或更高 |
| TensorRT | 8.x - 10.x |
| OpenCV | 4.5.4 |
| PCL | 1.12.1 |
| Python | 3.10 (ROS2 Humble) |

---

## 快速开始

如果你只想快速测试项目（使用 rosbag 数据），按以下步骤操作：

```bash
# 1. 克隆项目
git clone https://github.com/your-repo/T-DT-2024-Radar.git
cd T-DT-2024-Radar

# 2. 安装依赖（见详细安装指南）

# 3. 编译
source /opt/ros/humble/setup.bash
colcon build

# 4. 运行（需要先下载 rosbag 测试数据）
# 终端 1 - 点云处理
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py

# 终端 2 - rosbag 回放
source install/setup.bash
ros2 bag play bag/merged_bag_0.db3 --loop

# 终端 3 - 可视化
rviz2
```

---

## 详细安装指南

### 1. 安装 ROS2 Humble

```bash
# 设置 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 软件源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. 安装系统依赖

```bash
# ROS2 相关包
sudo apt install -y \
    ros-humble-nav2-map-server \
    ros-humble-foxglove-bridge \
    ros-humble-cv-bridge \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-rqt-image-view

# PCL 修复
sudo apt install --reinstall libpcl-io1.12

# 编译工具
sudo apt install -y build-essential cmake git

# Python 依赖（NYUSH ROS2 节点需要）
sudo apt install python3-pip
sudo pip3 install 'numpy<2' IPython onnxruntime-gpu
```

### 3. 安装 NVIDIA 依赖

#### CUDA & cuDNN
请参考 [NVIDIA 官方文档](https://developer.nvidia.com/cuda-downloads) 安装 CUDA 和 cuDNN。

#### TensorRT
```bash
# 安装 TensorRT（通过 apt）
sudo apt install -y tensorrt

# 或者手动下载安装：https://developer.nvidia.com/tensorrt
```

#### TensorRT 头文件配置（TensorRT 10.x 需要）

```bash
cd /tmp
git clone --depth 1 --branch v10.0.0 https://github.com/NVIDIA/TensorRT.git tensorrt_headers
git clone --depth 1 https://github.com/onnx/onnx-tensorrt.git onnx_tensorrt
cp onnx_tensorrt/NvOnnxParser.h tensorrt_headers/include/

# 在 CMakeLists.txt 中添加 include 路径（项目已配置）
```

### 4. 安装 Livox SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 5. 安装海康威视 MVS SDK（可选，用于实际相机）

从 [海康威视官网](https://www.hikrobotics.com/cn/machinevision/service/download) 下载 MVS SDK 并安装到 `/opt/MVS/`。

### 6. 下载测试数据

测试 rosbag 数据（包含比赛场地录制）：

**百度网盘**: https://pan.baidu.com/s/1ogRvs3v1OMCVUbAlUsOGQA?pwd=52rm

```bash
# 方法一：使用 BaiduPCS-Go（推荐）
wget https://github.com/qjfoidnh/BaiduPCS-Go/releases/download/v3.9.7/BaiduPCS-Go-v3.9.7-linux-amd64.zip
unzip BaiduPCS-Go-v3.9.7-linux-amd64.zip
cd BaiduPCS-Go-v3.9.7-linux-amd64

# 登录（使用 BDUSS cookie，从浏览器获取）
./BaiduPCS-Go login -bduss="你的BDUSS值"

# 下载
./BaiduPCS-Go d "/我的资源/适应性训练第一把.zip" --saveto ~/Desktop/T-DT-2024-Radar/

# 解压
cd ~/Desktop/T-DT-2024-Radar
7z x 适应性训练第一把.zip
mkdir -p bag && mv merged_bag_0.db3 bag/
```

---

## 编译项目

### 完整编译

```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
colcon build
```

### 编译单个功能包

```bash
# 雷达处理模块
colcon build --packages-select localization
colcon build --packages-select dynamic_cloud
colcon build --packages-select cluster
colcon build --packages-select kalman_filter

# 视觉检测模块
colcon build --packages-select tdt_vision
```

### 编译 Livox 驱动

```bash
cd src/livox_ros_driver2
./build.sh humble
```

### 编译问题排查

**TensorRT 头文件缺失**:
```bash
# 如果 /tmp/tensorrt_headers 被清理
cd /tmp
git clone --depth 1 --branch v10.0.0 https://github.com/NVIDIA/TensorRT.git tensorrt_headers
git clone --depth 1 https://github.com/onnx/onnx-tensorrt.git onnx_tensorrt
cp onnx_tensorrt/NvOnnxParser.h tensorrt_headers/include/
```

---

## 运行项目

### 模式一：rosbag 回放测试（雷达）

**终端 1 - 点云处理**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

**终端 2 - rosbag 回放**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play bag/merged_bag_0.db3 --loop
```

**终端 3 - RViz 可视化**:
```bash
source /opt/ros/humble/setup.bash
rviz2
```

### 模式二：rosbag 回放测试（T-DT 视觉）

**单终端启动**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/home/nyu/Desktop/T-DT-2024-Radar/bag/merged_bag_0.db3
```

启动后会：
- 自动弹出 OpenCV 窗口显示检测结果（1280x960）
- 发布 `/detect_image` 话题

### 模式三：使用实际雷达（Mid-360）

**终端 1 - 配置网络并启动雷达驱动**:
```bash
# 配置网卡 IP（雷达默认 IP 为 192.168.1.114）
sudo ip addr add 192.168.1.5/24 dev enp4s0

# 测试连接
ping 192.168.1.114

# 启动驱动
cd ~/Desktop/T-DT-2024-Radar
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端 2 - 点云处理**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

**终端 3 - RViz 可视化**:
```bash
source /opt/ros/humble/setup.bash
rviz2
```

### 模式四：使用实际相机（NYUSH ROS2 方案，推荐）

**终端 1 - 海康相机驱动**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

**终端 2 - NYUSH 视觉检测（ROS2 节点）**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_detect.launch.py
```

**终端 3 - 查看检测结果**:
```bash
ros2 run rqt_image_view rqt_image_view
# 选择 /detect_image 话题
```

### 模式五：完整系统（实际雷达 + 实际相机 + 融合）

**终端 1 - 雷达驱动**:
```bash
sudo ip addr add 192.168.1.5/24 dev enp4s0
cd ~/Desktop/T-DT-2024-Radar
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端 2 - 点云处理**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

**终端 3 - 海康相机驱动**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

**终端 4 - NYUSH 视觉检测**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_detect.launch.py
```

**终端 5 - RViz 可视化**:
```bash
source /opt/ros/humble/setup.bash
rviz2
```

---

## RViz 可视化配置

### Fixed Frame 设置
- **推荐**: `rm_frame`（场地坐标系，地面水平）
- **备选**: `livox_frame`（雷达坐标系，可能有倾斜）

### 添加 PointCloud2 显示

| 话题 | 说明 | 推荐颜色 | Size |
|------|------|----------|------|
| `/livox/lidar` | 原始点云 | 白色/灰色 | 0.01 |
| `/livox/map` | 场地地图 | 绿色 | 0.02 |
| `/livox/lidar_dynamic` | 动态点云（机器人） | 红色 | 0.02 |
| `/livox/cluster` | 聚类结果 | 黄色 | 0.1 |
| `/livox/lidar_kalman` | 卡尔曼跟踪结果 | **RGB8** | 0.15 |

### 添加 Image 显示

| 话题 | 说明 | 注意事项 |
|------|------|----------|
| `/camera_image` | 原始相机图像 | - |
| `/detect_image` | 检测结果图像（带框） | Reliability Policy 设为 **Best Effort** |

### 其他设置
- **Decay Time**: 设为 `0.5` 或 `1`（防止闪烁）
- **Reliability Policy**: 如果点云/图像不显示，尝试改为 `Best Effort`

---

## 话题说明

### 雷达话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | PointCloud2 | 原始点云数据（frame_id: livox_frame） |
| `/livox/map` | PointCloud2 | 场地地图点云（frame_id: rm_frame） |
| `/livox/lidar_dynamic` | PointCloud2 | 动态点云（frame_id: rm_frame） |
| `/livox/cluster` | PointCloud2 | 聚类中心点（frame_id: rm_frame） |
| `/livox/lidar_kalman` | PointCloud2 | 卡尔曼跟踪结果（frame_id: rm_frame，XYZRGB） |
| `/tf` | TF | 动态坐标变换 |
| `/tf_static` | TF | 静态坐标变换 |

### 视觉话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera_image` | Image | 原始相机图像 |
| `/detect_image` | Image | 检测结果图像（带检测框） |
| `/detect_result` | DetectResult | 检测结果（机器人位置） |
| `/resolve_result` | DetectResult | 解算后的 3D 位置 |

### 融合话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/kalman_detect` | DetectResult | 卡尔曼节点输出 |
| `/radar2sentry` | Radar2Sentry | 发送给串口的最终结果 |

---

## 视觉方案详解

本项目支持两套视觉检测方案，均已集成到 ROS2 框架：

### 方案对比

| 特性 | NYUSH ROS2 (Python) | T-DT ROS2 (C++) |
|------|---------------------|-----------------|
| **语言** | Python | C++ |
| **节点名** | `nyush_detect_node` | `radar_detect_node` |
| **模型输入** | 640×640 | 1280×1280 + 192×192 + 224×224 |
| **检测流程** | 2阶段 | 3阶段 |
| **推理框架** | ONNX Runtime + TensorRT | 纯 TensorRT (CUDA) |
| **适用相机** | 任意分辨率 | 高分辨率 (4096×3000) |
| **实际相机兼容性** | ✅ 良好 | ❌ 需高分辨率 |
| **推荐场景** | 实际相机 | rosbag/高分辨率相机 |

### NYUSH ROS2 方案（推荐用于实际相机）

**模型位置**: `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/`

**检测流程**:
```
相机图像 (任意分辨率, 如 1440×1080)
    │
    ▼ 订阅 /camera_image
┌─────────────────────────────────┐
│  car.engine (640×640 YOLOv5)    │ ← 检测机器人
│  类别: car, armor, ignore,      │   置信度阈值: 0.1
│        watcher, base            │
└─────────────────────────────────┘
    │
    ▼ 裁剪机器人区域
┌─────────────────────────────────┐
│  armor.engine (640×640 YOLOv5)  │ ← 检测并分类装甲板
│  类别: B1-B7, R1-R7             │   置信度阈值: 0.5
│  （直接输出颜色+编号）            │
└─────────────────────────────────┘
    │
    ▼ 发布
├── /detect_result (DetectResult)  ← 检测结果消息
└── /detect_image (Image)          ← 带标注的图像
```

**显示效果**:
- 绿色矩形框：检测到的车辆边界框
- 绿色文字 `car:0.85`：车辆标签 + 置信度
- 蓝色/红色文字 `B5:0.94`：装甲板标签 + 置信度

**运行方式**:
```bash
# 终端 1 - 相机驱动
ros2 run hik_camera hik_camera_node

# 终端 2 - NYUSH 检测节点
ros2 launch tdt_vision nyush_detect.launch.py

# 终端 3 - 查看结果
ros2 run rqt_image_view rqt_image_view  # 选择 /detect_image
```

**关键文件**:
- 检测节点: `src/tdt_vision/detect/scripts/nyush_detect_node.py`
- 启动文件: `src/tdt_vision/launch/nyush_detect.launch.py`
- 模型文件: `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/`
  - `car.engine` - 车辆检测模型
  - `armor.engine` - 装甲板检测模型

### T-DT ROS2 方案（用于 rosbag/高分辨率相机）

**检测流程**:
```
相机图像 (4096×3000)
    │
    ▼
┌─────────────────────────────────┐
│  yolo.engine (1280×1280)        │ ← 检测机器人
│  类别: red_car, blue_car        │   置信度阈值: 0.3
└─────────────────────────────────┘
    │
    ▼ 裁剪机器人区域
┌─────────────────────────────────┐
│  armor_yolo.engine (192×192)    │ ← 检测装甲板位置
└─────────────────────────────────┘
    │
    ▼ 裁剪装甲板区域
┌─────────────────────────────────┐
│  classify.engine (224×224)      │ ← DenseNet121 分类编号
│  DenseNet121                    │   输出: 0-7 编号
└─────────────────────────────────┘
    │
    ▼
输出: /detect_result, /detect_image
```

**运行方式**:
```bash
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/path/to/rosbag.db3
```

---

## 雷达方案详解

### 硬件配置

| 项目 | 参数 |
|------|------|
| 雷达型号 | Livox Mid-360 |
| IP 地址 | 192.168.1.114 |
| 主机 IP | 192.168.1.5 |
| 点云格式 | PointCloud2 |
| 帧率 | 10 Hz |

### 处理流程

```
┌─────────────────────────────────────────────────────────────────┐
│                       Livox Mid-360                             │
│                     IP: 192.168.1.114                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼ /livox/lidar (PointCloud2)
┌─────────────────────────────────────────────────────────────────┐
│  localization (GICP 配准)                                       │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│  输入: /livox/lidar (原始点云)                                  │
│  参考: config/RM2024.pcd (场地地图)                             │
│  算法: Generalized ICP (GICP)                                   │
│  输出: TF (rm_frame → livox_frame)                             │
│  作用: 将雷达坐标系对齐到场地坐标系                              │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼ TF 变换
┌─────────────────────────────────────────────────────────────────┐
│  dynamic_cloud (动态点云提取)                                   │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│  输入: /livox/lidar (变换后的点云)                              │
│  参考: config/RM2024.pcd (场地地图)                             │
│  算法: KD-Tree 最近邻搜索                                       │
│  输出: /livox/lidar_dynamic (动态点云)                          │
│  作用: 对比地图，提取移动物体（机器人）的点云                     │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼ /livox/lidar_dynamic
┌─────────────────────────────────────────────────────────────────┐
│  cluster (欧式聚类)                                             │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│  输入: /livox/lidar_dynamic                                     │
│  算法: Euclidean Cluster Extraction                             │
│  参数: ClusterTolerance, MinClusterSize, MaxClusterSize         │
│  输出: /livox/cluster (聚类中心点)                              │
│  作用: 将分散的点云聚合成独立目标                                │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼ /livox/cluster
┌─────────────────────────────────────────────────────────────────┐
│  kalman_filter (卡尔曼滤波 + 传感器融合)                        │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│  输入: /livox/cluster (雷达检测)                                │
│        /resolve_result (相机检测，如果有)                       │
│  算法: Extended Kalman Filter                                   │
│  融合: 位置匹配 (阈值 detect_r = 1 米)                          │
│  输出: /livox/lidar_kalman (带颜色的跟踪结果)                   │
│  作用: 平滑轨迹，融合相机颜色信息                                │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼ /livox/lidar_kalman (XYZRGB)
                         RViz 可视化
```

### 关键参数

| 参数 | 位置 | 默认值 | 说明 |
|------|------|--------|------|
| 地图文件 | `config/RM2024.pcd` | - | 场地点云地图 |
| 过滤范围 X | `dynamic_cloud.cpp` | -50~50 | 点云 X 范围（测试模式已放宽）|
| 过滤范围 Y | `dynamic_cloud.cpp` | -50~50 | 点云 Y 范围（测试模式已放宽）|
| 过滤范围 Z | `dynamic_cloud.cpp` | -5~10 | 点云 Z 范围（测试模式已放宽）|
| 聚类容差 | `cluster` 参数 | 0.5 | ClusterTolerance |
| 融合距离 | `filter_plus.h` | 1 米 | detect_r，相机-雷达匹配阈值 |

### 坐标系说明

| 坐标系 | 说明 |
|--------|------|
| `livox_frame` | 雷达坐标系，原点在雷达中心 |
| `rm_frame` | 场地坐标系，通过 GICP 配准建立 |
| `camera_frame` | 相机坐标系 |

---

## 传感器融合

### 融合架构

```
┌─────────────────┐                    ┌─────────────────┐
│   海康相机      │                    │   Livox Mid-360 │
│  1440×1080      │                    │   IP:192.168.1.114│
└────────┬────────┘                    └────────┬────────┘
         │                                      │
         ▼ /camera_image                        ▼ /livox/lidar
┌─────────────────┐                    ┌─────────────────┐
│ hik_camera_node │                    │ livox_ros_driver│
└────────┬────────┘                    └────────┬────────┘
         │                                      │
         ▼                                      ▼
┌─────────────────┐                    ┌─────────────────┐
│nyush_detect_node│                    │  localization   │
│  (视觉检测)     │                    │  (GICP 配准)    │
└────────┬────────┘                    └────────┬────────┘
         │                                      │
         │ /detect_result                       ▼
         │                             ┌─────────────────┐
         │                             │  dynamic_cloud  │
         │                             └────────┬────────┘
         │                                      │
         │                                      ▼
         │                             ┌─────────────────┐
         │                             │    cluster      │
         │                             └────────┬────────┘
         │                                      │
         │         /resolve_result              │ /livox/cluster
         │                │                     │
         ▼                ▼                     ▼
         └────────────────┴─────────────────────┘
                          │
                          ▼
                 ┌─────────────────┐
                 │  kalman_filter  │ ← 位置匹配（阈值 1 米）
                 │  (传感器融合)   │ → 匹配成功：使用相机颜色（红/蓝）
                 └────────┬────────┘ → 匹配失败：使用默认颜色
                          │
                          ▼
                 /livox/lidar_kalman (XYZRGB)
```

### 融合原理

1. **雷达检测**：通过聚类获取机器人 3D 位置
2. **视觉检测**：通过 YOLO 获取机器人颜色/编号，通过外参解算 3D 位置
3. **位置匹配**：比较雷达位置和相机解算位置
   - 距离 < 1 米：匹配成功，使用相机颜色
   - 距离 > 1 米：匹配失败，使用默认颜色
4. **卡尔曼滤波**：平滑轨迹，输出带颜色的跟踪结果

---

## LiDAR 建图

雷达站需要一个预先建好的场地点云地图（PCD 文件）用于 GICP 配准。本项目使用 FAST-LIO 进行建图。

### 建图工作空间

建图使用独立的工作空间，位于 `mapping_ws/` 目录：

```
mapping_ws/
├── src/
│   └── FAST_LIO/           # FAST-LIO ROS2 版本
│       ├── config/
│       │   └── mid360.yaml # Mid-360 配置文件
│       └── PCD/            # 保存的点云地图
└── install/
```

### 安装 FAST-LIO (首次使用)

```bash
# 创建建图工作空间
cd ~/Desktop/T-DT-2024-Radar
mkdir -p mapping_ws/src && cd mapping_ws/src

# 克隆 ROS2 版本的 FAST-LIO
git clone https://github.com/Ericsii/FAST_LIO.git -b ros2

# 初始化子模块
cd FAST_LIO
git submodule update --init --recursive

# 编译
cd ~/Desktop/T-DT-2024-Radar/mapping_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 建图流程

**步骤 1：配置网络并启动 Livox 驱动**

```bash
# 终端 1：配置网络
sudo ip addr add 192.168.1.5/24 dev enp4s0
ping 192.168.1.114  # 测试雷达连接

# 启动驱动（必须使用 CustomMsg 格式）
cd ~/Desktop/T-DT-2024-Radar
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

> **重要**：建图时 `xfer_format` 必须设为 `1`（CustomMsg 格式），在 `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` 中修改。

**步骤 2：启动 FAST-LIO**

```bash
# 终端 2：启动建图
cd ~/Desktop/T-DT-2024-Radar/mapping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

等待出现 `IMU Initial Done` 消息表示初始化完成。

**步骤 3：采集地图数据**

- 拿着雷达在**空场地**（无机器人）缓慢行走
- 覆盖整个场地区域
- 在 RViz 中观察点云累积情况
- 建议采集时间：3-5 分钟

**步骤 4：保存地图**

```bash
# 终端 3：调用保存服务（FAST-LIO 运行期间）
source /opt/ros/humble/setup.bash
ros2 service call /map_save std_srvs/srv/Trigger
```

FAST-LIO 会输出：
```
[INFO] Saving map to ./test.pcd...
```

**步骤 5：检查并移动地图文件**

```bash
# 检查生成的地图
ls -la ~/Desktop/T-DT-2024-Radar/mapping_ws/test.pcd

# 查看点云（可选）
pcl_viewer ~/Desktop/T-DT-2024-Radar/mapping_ws/test.pcd

# 移动到项目配置目录
cp ~/Desktop/T-DT-2024-Radar/mapping_ws/test.pcd ~/Desktop/T-DT-2024-Radar/config/RM2024.pcd
```

### FAST-LIO 配置说明

配置文件位置：`mapping_ws/src/FAST_LIO/config/mid360.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| `lid_topic` | `/livox/lidar` | LiDAR 话题 |
| `imu_topic` | `/livox/imu` | IMU 话题 |
| `lidar_type` | `1` | Livox 系列 |
| `pcd_save.pcd_save_en` | `true` | 启用 PCD 保存 |
| `map_file_path` | `./test.pcd` | 保存路径 |

### 常见问题

**问题 1：No point, skip this scan!**

- **原因**：IMU 数据未正确接收或 IMU 初始化未完成
- **解决**：
  1. 检查 IMU 话题：`ros2 topic hz /livox/imu`（应有 ~200Hz）
  2. 等待 `IMU Initial Done` 消息出现
  3. 确保 `xfer_format` 设为 `1`（CustomMsg 格式）

**问题 2：libusb_set_option undefined symbol**

- **原因**：MVS SDK 的 libusb 与系统版本冲突
- **解决**：启动命令前加 `LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0`

**问题 3：PCD 文件大小为 0 或很小**

- **原因**：未调用保存服务就退出了
- **解决**：在 FAST-LIO 运行期间调用 `ros2 service call /map_save std_srvs/srv/Trigger`

**问题 4：RViz 显示 "frame camera_init does not exist"**

- **原因**：FAST-LIO 使用 `camera_init` 作为固定坐标系
- **解决**：在 RViz 中将 Fixed Frame 设为 `camera_init` 或 `body`

### Livox 驱动模式切换

| 场景 | xfer_format | 话题类型 |
|------|-------------|----------|
| **建图（FAST-LIO）** | `1` | `livox_ros_driver2/msg/CustomMsg` |
| **检测（T-DT）** | `0` | `sensor_msgs/msg/PointCloud2` |

修改位置：`src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`

```python
# 建图时
xfer_format = 1  # CustomMsg

# 检测时
xfer_format = 0  # PointCloud2
```

---

## 相机外参标定

相机外参标定用于将相机图像中的 2D 坐标转换为场地 3D 坐标，是相机-雷达融合的关键。

### 标定文件

- **输入**: 相机内参 `config/camera_params.yaml`
- **输出**: 外参 `config/out_matrix.yaml`

### 标定方法

```bash
ros2 launch tdt_vision calib_rosbag.launch.py rosbag_file:=/path/to/rosbag.db3
```

**操作步骤**:
1. 按 **Enter** 开始标定
2. 依次点击 5 个标定点：
   - R0/B0 左上
   - R0/B0 右上
   - 己方前哨站血条最高点
   - 敌方基地引导灯
   - 敌方前哨站引导灯
3. 每次点击后可用 **WASD** 微调
4. 按 **N** 保存当前点
5. 保存 5 个点后自动计算并保存外参

### 注意事项

- 标定时相机位置必须固定
- 相机移动后需要重新标定
- 使用 rosbag 测试时，如果外参不匹配会导致融合失败（颜色五颜六色）

---

## 多台电脑分布式运行

### ROS2 多机通信配置

ROS2 Humble 默认使用 DDS 进行多机通信，只需确保：

1. **相同网络**: 所有电脑在同一局域网
2. **相同 ROS_DOMAIN_ID**: 设置相同的域 ID

```bash
# 在每台电脑上设置相同的 DOMAIN_ID（默认为 0）
export ROS_DOMAIN_ID=0

# 添加到 ~/.bashrc 使其永久生效
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

### 分布式部署示例

**电脑 A（雷达处理）**:
```bash
# 运行雷达驱动和点云处理
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch dynamic_cloud lidar.launch.py
```

**电脑 B（视觉处理）**:
```bash
# 运行相机驱动和视觉检测
ros2 run hik_camera hik_camera_node
ros2 launch tdt_vision nyush_detect.launch.py
```

**电脑 C（可视化）**:
```bash
# 运行 RViz 显示所有话题
rviz2
```

### 网络检查

```bash
# 查看本机 IP
ip addr

# 检查其他节点是否可见
ros2 node list

# 检查话题是否可见
ros2 topic list
```

### 时间同步

多机通信需要时间同步，推荐使用 chrony：

```bash
sudo apt install chrony
sudo systemctl enable chrony
sudo systemctl start chrony
```

---

## 常见问题

### 1. 雷达无法连接

```bash
# 检查网卡 IP
ip addr show enp4s0

# 配置 IP（每次重启需要重新配置）
sudo ip addr add 192.168.1.5/24 dev enp4s0

# 测试连接
ping 192.168.1.114
```

### 2. bind failed 错误

```bash
# 检查端口占用
sudo lsof -i :56100

# 杀掉残留进程
pkill -f livox
```

### 3. libusb 符号错误

```bash
# 运行时添加 LD_PRELOAD
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch ...

# 或临时重命名 MVS 的 libusb
sudo mv /opt/MVS/lib/64/libusb-1.0.so.0 /opt/MVS/lib/64/libusb-1.0.so.0.bak
```

### 4. GICP 崩溃

- 确保从项目根目录启动
- 检查 `config/RM2024.pcd` 地图文件存在

### 5. RViz 点云不显示

- 检查 Fixed Frame 是否正确（推荐 `rm_frame`）
- 检查 TF 是否正常：`ros2 run tf2_ros tf2_echo rm_frame livox_frame`
- 调整 Decay Time 为 0.5-1 秒
- 尝试将 Reliability Policy 改为 `Best Effort`

### 6. RViz Image 显示 "No Image"

- 检查话题是否有数据：`ros2 topic hz /detect_image`
- 将 Reliability Policy 改为 `Best Effort`
- 使用 rqt_image_view：`ros2 run rqt_image_view rqt_image_view`

### 7. 融合颜色不正确（五颜六色）

**原因**: 相机外参未正确标定，导致相机解算的 3D 位置与雷达检测位置不匹配。

**解决方案**:
1. 在实际场地进行五点标定
2. 临时方案：增大匹配距离阈值（修改 `src/fusion/kalman_filter/include/filter_plus.h` 中的 `detect_r`）

### 8. NYUSH 检测节点报错 ModuleNotFoundError

```bash
# 安装缺失的 Python 包
sudo pip3 install 'numpy<2' IPython onnxruntime-gpu
```

### 9. TensorRT engine 版本不兼容

```bash
# 重新生成 engine 文件
cd /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
source .venv/bin/activate
python onnx2engine.py
```

---

## 项目结构

### T-DT-2024-Radar（ROS2 主项目）

```
T-DT-2024-Radar/
├── config/                     # 配置文件
│   ├── RM2024.pcd             # 场地地图（用于 GICP 配准）
│   ├── camera_params.yaml     # 相机内参
│   ├── out_matrix.yaml        # 相机外参（标定输出）
│   └── detect_params.yaml     # 检测参数
├── model/                      # T-DT AI 模型
│   ├── ONNX/                  # ONNX 模型
│   └── TensorRT/              # TensorRT 引擎
├── bag/                        # rosbag 测试数据
├── mapping_ws/                 # 建图工作空间（FAST-LIO）
│   ├── src/
│   │   └── FAST_LIO/          # FAST-LIO ROS2 版本
│   │       ├── config/
│   │       │   └── mid360.yaml # Mid-360 配置
│   │       └── PCD/           # 保存的点云地图
│   └── install/
├── map/                        # 地图存储目录（可选）
├── src/
│   ├── lidar/                 # 雷达处理模块
│   │   ├── localization/      # GICP 配准
│   │   ├── dynamic_cloud/     # 动态点云提取
│   │   ├── cluster/           # 聚类
│   │   └── ...
│   ├── tdt_vision/            # 视觉检测模块
│   │   ├── detect/            # T-DT YOLO 检测 (C++)
│   │   │   └── scripts/       # NYUSH 检测节点 (Python)
│   │   │       └── nyush_detect_node.py
│   │   ├── resolve/           # 位置解算
│   │   ├── yolo/              # TensorRT 推理
│   │   └── launch/            # 启动文件
│   │       ├── run_rosbag.launch.py
│   │       └── nyush_detect.launch.py
│   ├── fusion/                # 传感器融合
│   │   └── kalman_filter/     # 卡尔曼滤波
│   ├── hik_camera/            # 海康相机 ROS2 驱动
│   ├── livox_ros_driver2/     # Livox 雷达驱动
│   └── interface/             # 自定义消息接口
│       └── vision_interface/
│           └── msg/
│               └── DetectResult.msg
└── install/                    # 编译输出
```

### NYUSH_Robotics_RM_RadarStation（NYUSH 模型源）

```
NYUSH_Robotics_RM_RadarStation/
├── models/                     # NYUSH AI 模型（被 nyush_detect_node.py 使用）
│   ├── car.engine             # 机器人检测 (640×640)
│   ├── car.onnx
│   ├── armor.engine           # 装甲板检测+分类 (640×640)
│   └── armor.onnx
├── yaml/                       # 模型配置
│   ├── car.yaml               # 车辆检测类别
│   └── armor.yaml             # 装甲板类别 (B1-B7, R1-R7)
├── main.py                     # 独立 Python 程序入口
├── detect_function.py          # YOLOv5 检测器封装
├── hik_camera.py              # 海康相机驱动 (Python)
├── onnx2engine.py             # ONNX 转 TensorRT 工具
├── calibration.py             # 标定程序
└── .venv/                      # Python 虚拟环境 (Python 3.9)
```

---

## 代码修改记录

### 环境依赖修复

- [x] TensorRT 10.x 头文件配置
- [x] ONNX Parser 头文件配置
- [x] libusb 冲突解决（MVS SDK）
- [x] libpcl-io 修复

### Livox Mid-360 雷达配置

- [x] Livox-SDK2 编译安装
- [x] livox_ros_driver2 编译
- [x] 雷达配置文件修改
- [x] 点云格式切换支持：`xfer_format = 0` (PointCloud2) / `xfer_format = 1` (CustomMsg)

### LiDAR 建图 (FAST-LIO)

- [x] FAST-LIO ROS2 版本安装（Ericsii/FAST_LIO ros2 分支）
- [x] Mid-360 配置文件 (`mid360.yaml`)
- [x] IMU 初始化调试
- [x] 地图保存服务 (`/map_save`)
- [x] libusb 冲突解决（LD_PRELOAD）

### 编译问题修复

- [x] Livox SDK v2.3.0 编译修复（添加 `#include <memory>`）
- [x] TensorRT nvonnxparser 链接修复

### 运行时问题修复

- [x] localization 空点云检查（防止 GICP 崩溃）
- [x] localization 过滤范围放宽（适应非比赛场地测试）
- [x] dynamic_cloud 过滤范围放宽
- [x] TF 时间戳问题修复（静态 TF 广播器）

### 视觉模块

- [x] detect 节点添加图像发布功能
- [x] 启用 debug 模式绘制检测框
- [x] 添加 OpenCV 窗口显示高清检测结果
- [x] 创建 NYUSH ROS2 检测节点 `nyush_detect_node.py`
- [x] 创建 NYUSH 启动文件 `nyush_detect.launch.py`

---

## 致谢

本项目基于东北大学 T-DT 实验室的开源代码开发，感谢原作者的贡献。

- **原项目技术报告**: [https://bbs.robomaster.com/wiki/260375/27115](https://bbs.robomaster.com/wiki/260375/27115)

## 联系方式

- **团队**: NYU Robotics

---

## 更新日志

### 2026-02-28 (第二次更新)
- **LiDAR 建图功能完成**
  - 安装 FAST-LIO ROS2 版本（独立工作空间 `mapping_ws/`）
  - 配置 Mid-360 参数（`mid360.yaml`）
  - 解决 IMU 初始化问题（需等待 `IMU Initial Done`）
  - 解决 libusb 符号冲突（`LD_PRELOAD` 方案）
  - 实现地图保存服务调用（`ros2 service call /map_save`）
  - 添加 Livox 驱动模式切换说明（建图用 CustomMsg，检测用 PointCloud2）
- 更新 README 添加完整建图指南

### 2026-02-28
- **NYUSH 视觉检测完成 ROS2 移植**
  - 创建 `nyush_detect_node.py` ROS2 Python 节点
  - 订阅 `/camera_image`，发布 `/detect_result` 和 `/detect_image`
  - 支持实际相机（1440×1080）检测
  - 检测框显示车辆和装甲板信息（绿色框 + 蓝/红标签）
- 添加实际相机测试支持
- 集成 NYUSH 视觉检测方案
- 添加视觉方案对比文档
- 更新雷达和视觉流程说明
- TensorRT engine 重新转换（适配 TensorRT 10.5.0）

---

*最后更新: 2026-02-28*
