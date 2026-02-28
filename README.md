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
- [系统要求](#系统要求)
- [快速开始](#快速开始)
- [详细安装指南](#详细安装指南)
- [编译项目](#编译项目)
- [运行项目](#运行项目)
- [RViz 可视化配置](#rviz-可视化配置)
- [话题说明](#话题说明)
- [算法流程](#算法流程)
- [相机外参标定](#相机外参标定)
- [常见问题](#常见问题)
- [项目结构](#项目结构)

---

## 项目介绍

本项目通过激光雷达和单目相机的目标检测，进行传感器后融合，实现了传感器之间的完全解耦合，避免了联合标定带来的误差。

**原项目技术报告**: [T-DT 2024 雷达技术报告](https://bbs.robomaster.com/wiki/260375/27115)

### 主要特性

- 即插即用，不依赖联合标定
- 不依赖相机和雷达之间的帧间匹配
- 三层神经网络实现更好的鲁棒性
- 雷达全自动 GICP 配准
- 低耦合，易于维护和扩展

### 硬件配置

| 设备 | 型号 |
|------|------|
| 激光雷达 | Livox Mid-360 / Avia |
| 单目相机 | Hikvision CH-120-10UC |
| CPU | i7-12700KF 或同等性能 |
| GPU | NVIDIA RTX 系列（需支持 TensorRT） |

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

### 模式二：rosbag 回放测试（视觉）

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

**查看检测图像**:
```bash
ros2 run rqt_image_view rqt_image_view
# 选择 /detect_image 话题
```

### 模式三：完整系统（雷达 + 视觉融合）

**终端 1 - 点云处理**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

**终端 2 - 视觉检测**:
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/home/nyu/Desktop/T-DT-2024-Radar/bag/merged_bag_0.db3
```

**终端 3 - RViz 可视化**:
```bash
source /opt/ros/humble/setup.bash
rviz2
```

### 模式四：使用实际雷达（Mid-360）

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

## 算法流程

### 雷达处理流程

```
原始点云 (/livox/lidar, frame_id: livox_frame)
    │
    ▼
┌─────────────────┐
│  localization   │ ← 加载场地地图 (config/RM2024.pcd)
│  (GICP 配准)    │ → 发布 TF: rm_frame → livox_frame
└─────────────────┘
    │
    ▼
┌─────────────────┐
│  dynamic_cloud  │ ← KD树对比地图，提取动态点
│  (动态点提取)   │ → 发布 /livox/lidar_dynamic
└─────────────────┘
    │
    ▼
┌─────────────────┐
│    cluster      │ ← 欧式聚类
│  (目标检测)     │ → 发布 /livox/cluster
└─────────────────┘
    │
    ▼
┌─────────────────┐
│  kalman_filter  │ ← 卡尔曼滤波跟踪
│  (目标跟踪)     │ → 发布 /livox/lidar_kalman (XYZRGB)
└─────────────────┘
```

### 视觉处理流程

```
相机图像 (/camera_image)
    │
    ▼
┌─────────────────┐
│  radar_detect   │ ← YOLO v5 检测机器人 (1280x1280)
│  (目标检测)     │ ← YOLO v5 检测装甲板 (192x192)
│                 │ ← DenseNet121 分类数字 (224x224)
│                 │ → 发布 /detect_result, /detect_image
└─────────────────┘
    │
    ▼
┌─────────────────┐
│  radar_resolve  │ ← 五点标定外参 (config/out_matrix.yaml)
│  (位置解算)     │ → 发布 /resolve_result（3D 位置）
└─────────────────┘
```

### 传感器融合流程

```
雷达检测 (/livox/cluster)    相机检测 (/resolve_result)
         │                            │
         └──────────┬─────────────────┘
                    ▼
          ┌─────────────────┐
          │  kalman_filter  │ ← 位置匹配（阈值 1 米）
          │  (传感器融合)   │ → 匹配成功：使用相机颜色（红/蓝）
          └─────────────────┘ → 匹配失败：使用默认颜色
                    │
                    ▼
          发布 /livox/lidar_kalman (XYZRGB)
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

---

## 项目结构

```
T-DT-2024-Radar/
├── config/                     # 配置文件
│   ├── RM2024.pcd             # 场地地图
│   ├── camera_params.yaml     # 相机内参
│   ├── out_matrix.yaml        # 相机外参（标定输出）
│   └── detect_params.yaml     # 检测参数
├── model/                      # AI 模型
│   ├── ONNX/                  # ONNX 模型
│   └── TensorRT/              # TensorRT 引擎（自动生成）
├── bag/                        # rosbag 测试数据
├── src/
│   ├── lidar/                 # 雷达处理模块
│   │   ├── localization/      # GICP 配准
│   │   ├── dynamic_cloud/     # 动态点云提取
│   │   ├── cluster/           # 聚类
│   │   └── ...
│   ├── tdt_vision/            # 视觉检测模块
│   │   ├── detect/            # YOLO 检测
│   │   ├── resolve/           # 位置解算
│   │   └── yolo/              # YOLO TensorRT 推理
│   ├── fusion/                # 传感器融合
│   │   └── kalman_filter/     # 卡尔曼滤波
│   ├── livox_ros_driver2/     # Livox 雷达驱动
│   └── interface/             # 自定义消息接口
└── install/                    # 编译输出
```

---

## 致谢

本项目基于东北大学 T-DT 实验室的开源代码开发，感谢原作者的贡献。

- **原项目技术报告**: [https://bbs.robomaster.com/wiki/260375/27115](https://bbs.robomaster.com/wiki/260375/27115)

## 联系方式

- **团队**: NYU Robotics

---

*最后更新: 2026-02-28*
