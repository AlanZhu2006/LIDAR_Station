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
- [命令速查完整说明](#命令速查完整说明)
- [RViz 可视化配置](#rviz-可视化配置)
- [话题说明](#话题说明)
- [视觉方案详解](#视觉方案详解)
- [雷达方案详解](#雷达方案详解)
- [传感器融合](#传感器融合)
- [模拟雷达识别功能](#模拟雷达识别功能)
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

**一体化工作流**：优先阅读 `docs/RADAR_STATION_INTEGRATED_WORKFLOW.md`

**命令速查**：见下方 [命令速查完整说明](#命令速查完整说明)，或 `docs/LIDAR.txt`、`docs/CAMERA.txt`

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
| **NYUSH 视觉 (ROS2)** | ✅ 正常 | 可直接输出世界坐标并接入 TDT 融合 |
| 相机-雷达融合 | ⚠️ 部分打通 | LiDAR 对齐正常；NYUSH `detect_image`、`/resolve_result`、`/nyush_map_image` 已接通；颜色融合仍受视觉世界坐标稳定性影响 |

### 2026-03 当前实机推荐配置

这一段以当前仓库代码和 `scripts/start_fusion.sh` 的默认行为为准，优先级高于后文仍未完全清理的旧说明。

- **运行地图统一使用**：`mapping_ws/test.pcd`
- **默认 LiDAR 启动方式**：`lidar.launch.py`
- **默认不开倾斜修正**：当前这套实机数据里原始 `/livox/lidar` 已基本水平，只有在原始点云本身 visibly tilted 时才启用 `lidar_tilt.launch.py`
- **RViz 自动摆正只影响显示**：`auto_align:=true` 会发布 `rm_frame_display`，便于看平地面，但不替代地图/GICP 配准
- **动态检测默认改为机器人优先**：`start_fusion.sh` 现在默认使用 `kd_tree_threshold_sq=0.15`、`min_cluster_size=8`，比旧版更容易保留低矮移动机器人
- **视觉侧默认 testmap 尺度**：`field_width_m=3.82`、`field_height_m=6.79`
- **视觉侧颜色方向必须匹配己方**：`SELF_COLOR=R` 或 `SELF_COLOR=B`

推荐直接使用：

```bash
cd ~/Desktop/RadarStation && source install/setup.bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

常用变体：

```bash
# 原始 /livox/lidar 本身就是斜的时才开
USE_TILT_CORRECTION=true SELF_COLOR=R ./scripts/start_fusion.sh

# 若人能出轨迹、机器人难出轨迹，先把检测调激进
KD_TREE_THRESHOLD_SQ=0.12 MIN_CLUSTER_SIZE=6 SELF_COLOR=R ./scripts/start_fusion.sh

# 若 testmap 是别的缩比场地，显式覆盖视觉坐标缩放
FIELD_WIDTH_M=9.6 FIELD_HEIGHT_M=5.4 SELF_COLOR=R ./scripts/start_fusion.sh
```

### 2026-03-17 交接总结

这一节是当前仓库和实机联调结果的统一结论，优先级高于后文残留的旧流程说明。

**今天实际完成的代码 / 脚本改动**

- `scripts/start_fusion.sh`
  - 统一整套实机默认链路：`mapping_ws/test.pcd`、`lidar.launch.py`、Camera `1280x960`、NYUSH `testmap`
  - 把 `SELF_COLOR`、`field_width_m`、`field_height_m`、`calibration_width_px`、`calibration_height_px` 等关键参数统一从脚本传进 NYUSH
  - 新增运行时调试参数：`DEBUG_CAMERA_MATCH`、`CAMERA_DETECT_RADIUS`、`PUBLISH_STATIONARY_TARGETS`
  - 新增 NYUSH 内部处理分辨率参数：`PROCESS_WIDTH`、`PROCESS_HEIGHT`
  - `rqt_image_view` 改为等待图像话题就绪后再打开，避免一启动就看不到 `/nyush_map_image`
- `scripts/wait_for_image_topics.sh`
  - 新增图像话题等待脚本，默认等待 `/detect_image` 和 `/nyush_map_image`
- `scripts/check_latency.sh`
  - 联调时可直接查看当前节点参数和链路状态，减少“以为启动了其实没吃到新参数”的误判
- `src/tdt_vision/launch/nyush_integration.launch.py`
  - 接入 `calibration_width_px / calibration_height_px`
  - 接入 `publish_debug_map`
  - 接入 `process_width / process_height`
- `src/tdt_vision/detect/scripts/nyush_world_node.py`
  - 修复标定分辨率参数类型问题
  - 默认暴露 `field_width_m=3.82`、`field_height_m=6.79`
  - 降低车检测阈值、扩大 armor ROI，并在 armor miss 时对增强后的 ROI 自动重试
  - 新增状态日志：`NYUSH status: cars=..., armors=..., tracks_ready=..., resolve_nonzero=...`
  - `detect_image` 现在只处理最新帧，不再让旧帧在回调里排队
  - NYUSH 内部检测默认改成 `960x720`，再映射回 `1280x960` 标定尺度
- `src/hik_camera/src/hik_camera_node.cpp`
  - 相机亮度增强从“每帧浮点 gamma + bias”改为 LUT 查表，减少 `detect_image` 额外延迟
- `src/fusion/kalman_filter/src/kalman_filter.cpp`
  - 相机-雷达匹配半径改成运行时参数 `camera_detect_radius`
  - 新增 `debug_camera_match` 日志，直接打印最近距离和最大时间差
  - 新增 `publish_stationary_targets`，联调时可把“几乎不动的点”也发到 `/livox/lidar_kalman`
  - 新增 `kalman_publish: ...` 日志，区分“被静止过滤了”还是“确实没匹配到颜色”
- `src/lidar/dynamic_cloud/launch/lidar.launch.py` / `lidar_tilt.launch.py`
  - 把 `camera_detect_radius` 和 `publish_stationary_targets` 一路传进 `kalman_filter`

**今天已经确认可用的部分**

- LiDAR 主链路已恢复正常：`/livox/lidar`、`/livox/map`、`/livox/lidar_dynamic`、`/livox/lidar_cluster`、`/livox/lidar_kalman` 可以正常运行。
- 当前这套实机数据里，原始 `/livox/lidar` 本身基本水平，所以默认不要再开 tilt correction。
- `start_fusion.sh` 现已统一使用 `mapping_ws/test.pcd`，默认走 `lidar.launch.py`。
- `dynamic_cloud` / `cluster` 已按低矮机器人做过一轮调参，默认比旧版更容易出机器人轨迹。
- `display_aligner_node.py` 可执行权限问题已修复，`rm_frame_display` 能正常用于 RViz 摆正显示。
- NYUSH 视觉链已经打通：`/detect_image`、`/resolve_result`、`/nyush_map_image` 都能发布。
- `detect_image` 延迟问题已经做过一轮实改：相机 LUT 优化 + NYUSH 最新帧处理 + 内部 `960x720` 处理分辨率。

**今天联调时已经确认的运行事实**

- 早期 `debug_camera_match` 日志显示：`最大时间差 0.36s ~ 0.49s < 1s`，说明时间不同步不是主因。
- 早期 `debug_camera_match` 日志也显示过：最近距离大约 `1.39m`，说明空间位置曾经就没有完全对上。
- 当前 `/resolve_result` 已经可以发非零值，但还不稳定，会在“短暂非零”和“全 0”之间切换。
- 当前 `/livox/lidar_kalman` 的灰色点不代表 Kalman 坏了，而是“雷达轨迹已经有了，但相机身份还没成功注入”。
- 当前实测过的一组数据：
  - `/resolve_result`: `R4 ≈ (3.81, 6.56)`
  - `/livox/lidar_kalman`: 灰色点约为 `(3.55, 1.83)` 和 `(0.00, -0.04)`
  - 这说明相机世界坐标和 LiDAR 当前主目标在 `y` 方向仍然差了大约 `4.7m`
- 当前也确认过：某些目标几乎不动时，会被 Kalman 当成静止目标过滤掉；如果不开 `PUBLISH_STATIONARY_TARGETS=true`，联调时容易误以为“LiDAR 没出点”。

**今天已经修掉的问题**

- `display_aligner_node.py executable not found`
- `start_fusion.sh` 与实际运行地图、LiDAR launch 不一致
- LiDAR 倾斜修正链路接错，导致地图和点云互相打架
- `calibration_width_px / calibration_height_px` 类型不匹配，导致 `nyush_world_node` 启动失败
- NYUSH 运行时标定分辨率与实际相机分辨率不一致，导致世界坐标整体偏得很远
- `rqt_image_view` 过早打开，导致一开始看不到 `/nyush_map_image`
- `detect_image` 延迟明显：已通过相机 LUT 优化和 NYUSH 最新帧处理方式做过一轮修复

**当前仍未完全解决的问题**

- `/nyush_map_image` 上的红/蓝点已经能出现，但还不够稳定。
- `/resolve_result` 仍然会在“短暂非零”和“全 0”之间切换。
- 当前主 blocker 不是 LiDAR 不出点，而是 NYUSH 世界坐标和 LiDAR cluster 仍未对齐；上色失败的根因仍在视觉世界坐标映射。
- 因为 `/resolve_result` 不稳定，`kalman_filter` 很多时候拿不到持续有效的视觉世界坐标，所以 `/livox/lidar_kalman` 仍可能长期保持灰色，没有稳定红蓝标识。
- 当前 `kalman_filter` 的相机匹配门槛仍是距离 `< 1m`；如果 NYUSH 点和 LiDAR cluster 长期差 1 米以上，就不会上色。
- 当前最可疑的剩余问题是：
  - `testmap` 标定点顺序不一致
  - 对焦后画面分布变化，但没有重做标定
  - `/nyush_map_image` 的点虽然“出现了”，但并没有落在真实目标附近

**当前推荐启动方式**

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

`start_fusion.sh` 当前会默认做这些事：

- Livox：`xfer_format:=0`
- LiDAR：`map_file:=mapping_ws/test.pcd`
- LiDAR：`kd_tree_threshold_sq:=0.15`、`cluster_tolerance:=0.25`、`min_cluster_size:=8`
- LiDAR：`debug_camera_match:=false`
- LiDAR：`camera_detect_radius:=1.0`
- LiDAR：`publish_stationary_targets:=false`
- Camera：`publish_width:=1280`、`publish_height:=960`
- NYUSH：`map_mode:=testmap`
- NYUSH：`field_width_m:=3.82`、`field_height_m:=6.79`
- NYUSH：`calibration_width_px:=1280`、`calibration_height_px:=960`
- NYUSH：`window_size:=2`、`max_inactive_time:=2.0`
- NYUSH：`car_conf:=0.15`、`armor_conf:=0.45`
- NYUSH：`armor_roi_expand_ratio:=0.12`、`armor_roi_bottom_expand_ratio:=0.18`
- NYUSH：`process_width:=960`、`process_height:=720`
- NYUSH：`publish_debug_map:=true`
- RViz：载入 `src/livox_ros_driver2/config/pointcloud_lidar.rviz`

今天联调可直接用：

```bash
# 先抓融合层最近距离/时间差
DEBUG_CAMERA_MATCH=true SELF_COLOR=R ./scripts/start_fusion.sh

# 若 /resolve_result 经常回到全 0，先从 NYUSH 稳定性入手
ARMOR_CONF=0.45 WINDOW_SIZE=2 MAX_INACTIVE_TIME=2.5 SELF_COLOR=R ./scripts/start_fusion.sh

# 若 detect_image 里经常“车框有了但装甲板没出”，进一步放宽 ROI
ARMOR_ROI_EXPAND_RATIO=0.18 ARMOR_ROI_BOTTOM_EXPAND_RATIO=0.25 SELF_COLOR=R ./scripts/start_fusion.sh

# 若 debug_camera_match 显示最近距离略大于 1m，可临时放宽匹配半径验证
DEBUG_CAMERA_MATCH=true CAMERA_DETECT_RADIUS=1.5 SELF_COLOR=R ./scripts/start_fusion.sh

# 若目标基本不动，Kalman 默认会把静止目标过滤掉；联调时可临时打开
DEBUG_CAMERA_MATCH=true CAMERA_DETECT_RADIUS=1.5 PUBLISH_STATIONARY_TARGETS=true SELF_COLOR=R ./scripts/start_fusion.sh

# 若 detect_image 延迟明显，先让 NYUSH 在 960x720 内部处理最新帧
PROCESS_WIDTH=960 PROCESS_HEIGHT=720 SELF_COLOR=R ./scripts/start_fusion.sh

# 若仍觉得 detect_image 处理重，直接上低延迟模式
SELF_COLOR=R ./scripts/start_fusion.sh --low-latency
```

**明天继续之前，先确认这 6 项**

1. `ros2 topic echo --once /resolve_result` 不是全 0。
2. `rqt_image_view` 里能看到 `/detect_image` 和 `/nyush_map_image`。
3. `/nyush_map_image` 的点落在地图上大致正确的位置。
4. `/livox/lidar_cluster` 在 RViz 里能稳定看到动态目标。
5. `/livox/lidar_kalman` 若仍全灰，优先查 `/resolve_result` 是否持续非零，而不是先动 LiDAR 参数。
6. 若目标基本不动，联调时记得加 `PUBLISH_STATIONARY_TARGETS=true`，否则可能被静止过滤误导。

**现场最短排查顺序**

```bash
ros2 topic list | grep -E "resolve_result|lidar_cluster|lidar_kalman|detect_image|nyush_map_image"
ros2 node list | grep -E "nyush|kalman|hik|lidar"
ros2 topic echo --once /resolve_result
ros2 topic echo --once /livox/lidar_kalman
```

- 若 `/resolve_result` 是全 0：先看 NYUSH 检测稳定性，不要先改 LiDAR。
- 若 `/resolve_result` 非零，但 `/livox/lidar_kalman` 仍全灰：先看 `/nyush_map_image` 的点位是否落在真实目标附近。
- 若 `/nyush_map_image` 点位明显偏离：优先重做 NYUSH `testmap` 标定，而不是继续调 `camera_detect_radius`。

**明天的续作事项**

1. 打开 `kalman_filter` 的 `debug_camera_match`，抓一轮“最近距离 / 时间差”日志，确认是“没识别到”还是“距离 > 1m”。
2. 稳定 NYUSH 世界坐标输出：
   - 继续检查 `armor.engine` 的检测稳定性
   - 必要时调 `armor_conf`、`window_size`、`max_inactive_time`
3. 如果 `/nyush_map_image` 点位仍偏，重做 NYUSH `testmap` 标定，而不是回到旧的 T-DT 五点外参流程。
4. 若目标基本不动，联调时继续用 `PUBLISH_STATIONARY_TARGETS=true`，先确认“有没有匹配到颜色”，再恢复静止过滤。
5. 只有在 `/resolve_result` 已经持续稳定非零后，再评估是否要继续放宽 `camera_detect_radius`。
6. README 之外，后续可以把 `docs/LIDAR.txt` 和 `docs/CAMERA.txt` 再做第二轮精简，彻底移除历史冲突命令。

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
git clone https://github.com/your-repo/RadarStation.git
cd RadarStation

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
./BaiduPCS-Go d "/我的资源/适应性训练第一把.zip" --saveto ~/Desktop/RadarStation/

# 解压
cd ~/Desktop/RadarStation
7z x 适应性训练第一把.zip
mkdir -p bag && mv merged_bag_0.db3 bag/
```

---

## 编译项目

### 完整编译

```bash
cd ~/Desktop/RadarStation
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
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

**终端 2 - rosbag 回放**:
```bash
cd ~/Desktop/RadarStation
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
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/home/nyu/Desktop/RadarStation/bag/merged_bag_0.db3
```

启动后会：
- 自动弹出 OpenCV 窗口显示检测结果（1280x960）
- 发布 `/detect_image` 话题

### 模式三：使用实际雷达（Mid-360）

**终端 1 - 配置网络并启动雷达驱动**:
```bash
# 配置网卡 IP（雷达默认 IP 为 192.168.1.114）
sudo ip addr add 192.168.1.5/24 dev enx00e04c2536b0

# 测试连接
ping 192.168.1.114

# 启动驱动
cd ~/Desktop/RadarStation
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端 2 - 点云处理**:
```bash
cd ~/Desktop/RadarStation
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
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

**终端 2 - NYUSH 世界坐标节点（推荐）**:
```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=R \
  field_width_m:=3.82 \
  field_height_m:=6.79 \
  calibration_width_px:=1280 \
  calibration_height_px:=960 \
  publish_debug_map:=true
```

**终端 3 - 查看世界坐标结果**:
```bash
ros2 topic echo --once /resolve_result
```

**终端 4 - 查看地图调试图（推荐）**:
```bash
ros2 run rqt_image_view rqt_image_view
# 选择 /detect_image 和 /nyush_map_image
```

### 模式五：完整系统（当前推荐，实际雷达 + 实际相机 + 融合）

**最简单方式：一键启动**

```bash
cd ~/Desktop/RadarStation
source install/setup.bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

`start_fusion.sh` 当前默认行为：

- LiDAR 地图：`mapping_ws/test.pcd`
- 原始点云按**已水平**处理，默认启动 `lidar.launch.py`
- 动态检测默认：`kd_tree_threshold_sq=0.15`、`cluster_tolerance=0.25`、`min_cluster_size=8`
- NYUSH 默认：`map_mode:=testmap`、`field_width_m:=3.82`、`field_height_m:=6.79`
- NYUSH 默认：`calibration_width_px:=1280`、`calibration_height_px:=960`
- NYUSH 默认：`publish_debug_map:=true`
- RViz 默认载入：`src/livox_ros_driver2/config/pointcloud_lidar.rviz`

**推荐环境变量开关**

```bash
# 己方颜色，必须与实际一致
SELF_COLOR=R ./scripts/start_fusion.sh
SELF_COLOR=B ./scripts/start_fusion.sh

# 只有原始 /livox/lidar 本身是斜的时才开
USE_TILT_CORRECTION=true SELF_COLOR=R ./scripts/start_fusion.sh

# 低矮机器人难检出时，进一步放宽
KD_TREE_THRESHOLD_SQ=0.12 MIN_CLUSTER_SIZE=6 SELF_COLOR=R ./scripts/start_fusion.sh
```

**分终端方式（便于排障）**

终端 1，Livox：
```bash
sudo ip addr add 192.168.1.5/24 dev enx00e04c2536b0
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py xfer_format:=0 publish_freq:=10.0
```

终端 2，LiDAR 处理链：
```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py \
  map_file:=mapping_ws/test.pcd \
  ceiling_z_max:=100.0 \
  voxel_leaf_size:=0.35 \
  accumulate_time:=3 \
  kd_tree_threshold_sq:=0.15 \
  cluster_tolerance:=0.25 \
  min_cluster_size:=8 \
  auto_align:=true
```

如果原始 `/livox/lidar` 本身就是斜的，再改用：
```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar_tilt.launch.py \
  map_file:=mapping_ws/test.pcd \
  ceiling_z_max:=100.0 \
  voxel_leaf_size:=0.35 \
  accumulate_time:=3 \
  kd_tree_threshold_sq:=0.15 \
  cluster_tolerance:=0.25 \
  min_cluster_size:=8 \
  lidar_pitch_deg:=50 \
  auto_align:=true
```

终端 3，相机：
```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node \
  --ros-args \
  -p publish_width:=1280 \
  -p publish_height:=960 \
  -p target_fps:=30.0
```

终端 4，NYUSH 世界坐标：
```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=R \
  field_width_m:=3.82 \
  field_height_m:=6.79 \
  calibration_width_px:=1280 \
  calibration_height_px:=960 \
  publish_debug_map:=true
```

若己方不是红方，把 `state:=R` 改为 `state:=B`。

---

## 命令速查完整说明

本节整合 `docs/LIDAR.txt` 与 `docs/CAMERA.txt` 的完整命令，便于在 README 内直接查阅。

### lidar.launch.py / lidar_tilt.launch.py 参数说明（2026-03 当前）

用法：`ros2 launch dynamic_cloud lidar.launch.py 参数名:=值`

| 参数 | 默认值 | 说明 |
|------|--------|------|
| map_file | config/RM2024.pcd | 场地点云地图路径 |
| ceiling_z_max | 100.0 | 输入点云 z 方向 ceiling 过滤阈值；当前默认等效关闭 |
| kd_tree_threshold_sq | 0.15 | 与地图最近邻平方距离阈值；越小越容易保留低矮机器人，越大越不容易误检静止物 |
| process_every_n | 1 | 每 N 帧做一次 kd-tree；1=每帧处理，2/3=更省算力 |
| voxel_leaf_size | 0.25 | 地图体素降采样大小 |
| accumulate_time | 3 | 动态点云累积窗口；一键启动当前也用 3 |
| cluster_tolerance | 0.25 | 欧式聚类距离阈值 |
| min_cluster_size | 12 | 聚类最小点数；`start_fusion.sh` 当前默认覆盖为 8 |
| max_cluster_size | 1000 | 聚类最大点数 |
| auto_align | true | 自动发布 `rm_frame_display`，仅用于 RViz 显示摆正 |
| debug_camera_match | false | 输出相机-雷达匹配调试日志 |

**常用组合（追加到命令末尾）**：
```bash
map_file:=mapping_ws/test.pcd ceiling_z_max:=100.0 kd_tree_threshold_sq:=0.15 min_cluster_size:=8
process_every_n:=1 voxel_leaf_size:=0.35 accumulate_time:=3

# 若机器人仍难检出
kd_tree_threshold_sq:=0.12 min_cluster_size:=6

# 若静止障碍误检偏多
kd_tree_threshold_sq:=0.18
```

### 雷达模式命令汇总

**模式一：实际雷达（Livox Mid-360）**
```bash
sudo ip addr add 192.168.1.5/24 dev enx00e04c2536b0
cd ~/Desktop/RadarStation && source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# 当前默认：原始 /livox/lidar 已水平，直接用 lidar.launch.py
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py \
  map_file:=mapping_ws/test.pcd ceiling_z_max:=100.0 kd_tree_threshold_sq:=0.15 min_cluster_size:=8

# 只有在原始 /livox/lidar 本身 visibly tilted 时才启用倾斜修正
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar_tilt.launch.py \
  map_file:=mapping_ws/test.pcd ceiling_z_max:=100.0 lidar_pitch_deg:=50 kd_tree_threshold_sq:=0.15 min_cluster_size:=8

rviz2 -d ~/Desktop/RadarStation/src/livox_ros_driver2/config/pointcloud_lidar.rviz
```

**模式二：FAST-LIO 建图**
```bash
cd ~/Desktop/RadarStation && ./scripts/start_mapping.sh
# 建图后：cp mapping_ws/src/FAST_LIO/PCD/scans.pcd mapping_ws/test.pcd
# 当前推荐：运行时统一只使用 mapping_ws/test.pcd
# 仅当原始 /livox/lidar 明显倾斜时，才考虑建图后旋转 PCD 并在运行时配合 lidar_tilt.launch.py
```

**模式三：视觉 + 雷达融合**
```bash
SELF_COLOR=R ./scripts/start_fusion.sh

# 若己方不是红方：SELF_COLOR=B ./scripts/start_fusion.sh
# 若原始 /livox/lidar 本身是斜的：USE_TILT_CORRECTION=true SELF_COLOR=R ./scripts/start_fusion.sh
# 若机器人难检出：KD_TREE_THRESHOLD_SQ=0.12 MIN_CLUSTER_SIZE=6 SELF_COLOR=R ./scripts/start_fusion.sh
```

**模式四：rosbag 回放**
```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py map_file:=config/RM2024.pcd ceiling_z_max:=100.0
ros2 bag play bag/merged_bag_0.db3 --loop --clock
rviz2 -d ~/Desktop/RadarStation/src/livox_ros_driver2/config/pointcloud.rviz
```

**模式五：最小测试（排查卡顿）**
```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar_minimal.launch.py map_file:=mapping_ws/test.pcd
```

### 相机模式命令汇总

**模式一：rosbag + T-DT 视觉（4096×3000）**
```bash
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/path/to/merged_bag_0.db3
```

**模式二：实际相机 + NYUSH 检测**
```bash
# 终端1：ros2 run hik_camera hik_camera_node
# 终端2：ros2 launch tdt_vision nyush_detect.launch.py
```

**模式三：完整融合（相机 + 雷达 + nyush_integration）**
```bash
# 终端1：相机
ros2 run hik_camera hik_camera_node

# 终端2：nyush_integration（发布 /resolve_result，融合必需）
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap state:=R field_width_m:=3.82 field_height_m:=6.79 \
  calibration_width_px:=1280 calibration_height_px:=960 \
  publish_debug_map:=true

# 雷达侧见上方「模式三：视觉 + 雷达融合」
```

`nyush_integration` 关键参数：

- `state`：己方颜色，必须与实际一致
- `field_width_m` / `field_height_m`：世界坐标缩放，当前 testmap 默认是 `3.82 x 6.79`
- `map_mode`：当前只建议用 `testmap`
- `calibration_width_px` / `calibration_height_px`：必须与运行时 `camera_image` 分辨率一致；当前脚本默认 `1280 x 960`
- `publish_debug_map`：建议保持 `true`，便于看 `/nyush_map_image`

### 快速验证 /resolve_result（仅相机，无需雷达）

```bash
# 终端1
ros2 run hik_camera hik_camera_node

# 终端2
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap state:=R field_width_m:=3.82 field_height_m:=6.79 \
  calibration_width_px:=1280 calibration_height_px:=960 \
  publish_debug_map:=true

# 验证
ros2 topic echo --once /resolve_result
# 有检测时应看到 red_x,red_y,blue_x,blue_y 非零（场地坐标，单位米）

# 可视化验证
ros2 run rqt_image_view rqt_image_view
# 选择 /nyush_map_image，检查点是否落在地图正确区域
```

### 模拟雷达识别展示（2D 地图）

```bash
# 在融合运行后
ros2 run debug_map debug_map_node
# 订阅 /kalman_detect，在 config/RM2024.png 上绘制红/蓝方位置和编号
# 需先 colcon build --packages-select debug_map
```

### 融合机制简述

- **雷达**：cluster 质心 → Kalman 轨迹，位置始终来自雷达
- **相机**：/resolve_result（rm_frame 2D）→ camera_match 与 KF history 中最近雷达点距离<1m 则匹配
- **前提**：resolve_result 必须是 rm_frame 坐标，需 nyush_integration（不是 nyush_detect）
- 详见 [融合机制详解](#融合机制详解关键)

### 关键话题

| 话题 | 说明 |
|------|------|
| /livox/lidar | 原始点云 |
| /livox/map | 场地地图 |
| /livox/lidar_dynamic | 动态点云 |
| /livox/lidar_cluster | 聚类中心，Kalman 订阅 |
| /livox/lidar_kalman | 卡尔曼跟踪（XYZRGB）|
| /kalman_detect | 融合结果，供 debug_map |
| /resolve_result | 相机世界坐标，融合必需 |

### 常见问题速查

| 问题 | 解决 |
|------|------|
| 静止物误检 | 先保留 `ceiling_z_max:=100.0`，再把 `kd_tree_threshold_sq` 从 `0.15` 慢慢加回 `0.18/0.22` |
| Kalman 全灰 | 需 nyush_integration 发布 /resolve_result |
| 移动时卡顿 | process_every_n:=3、voxel_leaf_size:=0.3；排查用 lidar_minimal |
| libusb 错误 | LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 |
| 建图用 CustomMsg | xfer_format:=1；检测用 PointCloud2，xfer_format:=0 |

---

## RViz 可视化配置

### Fixed Frame 设置
- **当前 RViz 默认**: `rm_frame_display`（只用于显示摆正）
- **算法真实坐标系**: `rm_frame`
- **备选**: `livox_frame`（雷达坐标系，通常不建议用于最终排障）

### 添加 PointCloud2 显示

| 话题 | 说明 | 推荐颜色 | Size |
|------|------|----------|------|
| `/livox/lidar` | 原始点云 | 白色/灰色 | 0.01 |
| `/livox/map` | 场地地图 | 绿色 | 0.02 |
| `/livox/lidar_dynamic` | 动态点云（机器人） | 红色 | 0.02 |
| `/livox/lidar_cluster` | 聚类结果 | 黄色 | 0.1 |
| `/livox/lidar_kalman` | 卡尔曼跟踪结果 | **RGB8** | 0.15 |

### 添加 Image 显示

| 话题 | 说明 | 注意事项 |
|------|------|----------|
| `/camera_image` | 原始相机图像 | - |
| `/detect_image` | 检测结果图像（带框） | Reliability Policy 设为 **Best Effort** |
| `/nyush_map_image` | NYUSH 地图调试图 | 用来判断标定后的世界坐标是否落在地图正确位置 |

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
| `/livox/lidar_cluster` | PointCloud2 | 聚类中心点（frame_id: rm_frame） |
| `/livox/lidar_kalman` | PointCloud2 | 卡尔曼跟踪结果（frame_id: rm_frame，XYZRGB） |
| `/tf` | TF | 动态坐标变换 |
| `/tf_static` | TF | 静态坐标变换 |

### 视觉话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera_image` | Image | 原始相机图像 |
| `/detect_image` | Image | 检测结果图像（带检测框） |
| `/nyush_map_image` | Image | NYUSH 地图调试图，显示世界坐标点 |
| `/detect_result` | DetectResult | 检测结果（仅旧链路保留，不作为当前 fusion 入口） |
| `/resolve_result` | DetectResult | NYUSH 输出的场地 2D 世界坐标，当前 fusion 入口 |

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

# 终端 2 - NYUSH 世界坐标节点
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=R \
  field_width_m:=3.82 \
  field_height_m:=6.79 \
  calibration_width_px:=1280 \
  calibration_height_px:=960 \
  publish_debug_map:=true

# 终端 3 - 查看结果
ros2 topic echo --once /resolve_result
```

**关键文件**:
- 世界坐标节点: `src/tdt_vision/detect/scripts/nyush_world_node.py`
- 启动文件: `src/tdt_vision/launch/nyush_integration.launch.py`
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
│  输出: /livox/lidar_cluster (聚类中心点)                        │
│  作用: 将分散的点云聚合成独立目标                                │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼ /livox/lidar_cluster
┌─────────────────────────────────────────────────────────────────┐
│  kalman_filter (卡尔曼滤波 + 传感器融合)                        │
│  ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
│  输入: /livox/lidar_cluster (雷达检测)                          │
│        /resolve_result (相机检测，如果有)                       │
│  算法: Extended Kalman Filter                                   │
│  融合: 位置匹配 (阈值默认 1 米，可用 camera_detect_radius 覆盖) │
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
| 地图文件 | `lidar.launch.py` | `config/RM2024.pcd` | 场地点云地图 |
| **ceiling_z_max** | `lidar.launch.py` | 100.0 m | 当前 Mid-360 为约50度倾角安装，默认设高，等效关闭该传感器坐标系过滤 |
| **kd_tree_threshold_sq** | `lidar.launch.py` | 0.15 | 与地图距离平方阈值；对低矮机器人更敏感，静止物误检多时再加回 0.18/0.22 |
| **process_every_n** | `lidar.launch.py` | 1 | 每 N 帧做 kd-tree，1 为每帧处理 |
| **voxel_leaf_size** | `lidar.launch.py` | 0.25 | 地图体素降采样 m，`start_fusion.sh` 当前覆盖为 0.35 |
| **accumulate_time** | `lidar.launch.py` | 3 | 动态点云累积窗口 |
| **cluster_tolerance** | `cluster` | 0.25 | 欧式聚类距离阈值 |
| **min_cluster_size** | `cluster` | 12 | 最小聚类点数；`start_fusion.sh` 当前覆盖为 8 |
| **max_cluster_size** | `cluster` | 1000 | 最大聚类点数 |
| 体素降采样 | `dynamic_cloud.cpp` | 0.25 | kd-tree 前体素大小 |
| 融合距离 | `kalman_filter` 运行时参数 | 1 米 | `camera_detect_radius`，相机-雷达匹配阈值 |

**启动时覆盖示例**：
```bash
ros2 launch dynamic_cloud lidar.launch.py map_file:=mapping_ws/test.pcd \
  ceiling_z_max:=100.0 kd_tree_threshold_sq:=0.15 min_cluster_size:=8
```

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
         │         /resolve_result              │ /livox/lidar_cluster
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

### 融合数据流详解（重要）

**位置来源**：雷达提供，**不是**相机  
**身份来源**：相机提供，**不是**雷达  

```
雷达路径（位置）：
  livox/lidar → localization → dynamic_cloud → cluster → /livox/lidar_cluster
                                                              ↓
                                                         Kalman 订阅
                                                              ↓
  /livox/lidar_kalman（点云）← Kalman 发布 ← 用 cluster 质心创建/更新轨迹
  /kalman_detect（DetectResult）← 同上

相机路径（身份）：
  相机图像 → 装甲板检测 → resolve/NYUSH → /resolve_result（红/蓝 2D 位置）
                                                              ↓
                                                         Kalman 订阅 detect_callback
                                                              ↓
  camera_match：把相机检测与已有雷达轨迹按位置匹配，给轨迹打上红/蓝和编号
```

| 来源 | 作用 |
|------|------|
| **雷达** | 提供**位置**：检测动态目标、聚类、卡尔曼跟踪，输出在 `rm_frame` 下的坐标 |
| **相机 (resolve_result)** | 提供**身份**：把红/蓝装甲板与雷达轨迹匹配，给轨迹打上颜色和编号 |

**RViz 显示**：`/livox/lidar_kalman` 中每个点的位置来自雷达，颜色来自相机匹配  
- **红** = 相机匹配到红方装甲板  
- **蓝** = 相机匹配到蓝方装甲板  
- **灰** = 雷达有轨迹，但相机尚未匹配（无相机时全部为灰色）

### 融合机制详解（关键）

融合发生在 **Kalman 节点**，通过 **位置匹配** 将相机的身份（红/蓝+编号）绑定到雷达轨迹上。

#### 1. 雷达轨迹的建立与更新

```
/livox/lidar_cluster（cluster 质心，rm_frame 下的 x,y）
    ↓
Kalman callback：对每个 cluster 点
    - 若与某 KF 的 predict_point 距离 < car_max_speed*dt + detect_r → 更新该 KF
    - 若无匹配 → 新建 KF
    ↓
KFs 列表：每个 KF 维护 history[(时间, 位置)]、detect_history[(颜色, 编号)]
```

- **match 条件**：`Distance(predict_point, cluster_point) < car_max_speed * dt + detect_r`（当前默认 `detect_r = 1m`，可通过 `camera_detect_radius` 运行时覆盖）
- **位置**：始终来自雷达 cluster，输出时用 `get_output_point()`（最近观测）

#### 2. 相机身份的注入（camera_match）

```
/resolve_result（red_x[0~5], red_y[0~5], blue_x[0~5], blue_y[0~5]，rm_frame 下的 2D 坐标）
    ↓
detect_callback：对每个红/蓝点 (i=0~5 对应英雄/工程/步兵/哨兵)
    ↓
对每个 KF 调用 camera_match(time, point, color, number)
```

**camera_match 逻辑**（`kalman_filter` + `filter_plus.h`）：

1. **时间对齐**：在 KF 的 `history` 中找与相机时间戳**最近**的雷达观测点（时间差 < 1s）
2. **空间匹配**：若该雷达点与相机点距离 < `detect_r`（默认 1m，可通过 `camera_detect_radius` 覆盖）→ 匹配成功
3. **写入身份**：`detect_history.push_back((color, number))`，color=0 蓝/2 红，number=0~5

```
相机：红方 3 号装甲板在 (12.5, 8.2)
雷达：某 KF 在 t≈相机时间 时位置 (12.3, 8.4)
距离 0.28m < 1m → 匹配成功 → 该 KF 标记为红方、编号 3（步兵）
```

#### 3. 输出时的身份使用

- **get_color()**：统计 `detect_history` 中红/蓝出现次数，多的为当前颜色
- **get_number()**：统计同颜色下各编号出现次数，多的为当前编号
- **输出**：`/livox/lidar_kalman` 点云 RGB、`/kalman_detect` 的 red_x[number]/blue_x[number]

#### 4. 融合前提条件

| 条件 | 说明 |
|------|------|
| **同一坐标系** | resolve_result 必须是 **rm_frame** 下的 2D 坐标（x,y），与 cluster 一致 |
| **同一尺度** | `field_width_m` / `field_height_m` 必须与当前 testmap 的真实米制一致；当前脚本默认是 `3.82 x 6.79` |
| **颜色方向正确** | `nyush_integration` 的 `state` 必须与己方颜色一致，否则红蓝语义会反 |
| **相机外参标定** | NYUSH 世界坐标节点需正确标定，否则相机解算位置与雷达不对齐 |
| **时间戳** | camera_match 用时间差 < 1s 找最近雷达点，时间不同步会影响匹配 |
| **detect_r** | 匹配半径默认 1m，标定误差大时可临时调大运行时参数 `camera_detect_radius` |

#### 5. 融合失败时的表现

- **无 /resolve_result**：所有轨迹为灰色，位置正确
- **标定偏差大**：相机点与雷达点距离 > 1m，匹配失败，轨迹仍为灰色
- **尺度不一致**：`/resolve_result` 有值，但 `/kalman_detect` 长期全 0，轨迹持续灰色
- **state 传错**：颜色可能整体反了，或 debug_map / RViz 上红蓝方逻辑异常
- **时间不同步**：找不到 1s 内的雷达点，匹配失败

---

## 模拟雷达识别功能

### 展示要求（已达成）

> **模拟雷达识别功能**：参赛队自行划定备赛场地部分区域为区域地图，区域地图中放置旋转且移动的步兵机器人、英雄机器人、工程机器人、哨兵机器人，展示雷达可以精确识别机器人，并将其位置准确标识在区域地图上的功能。

### 当前能力对照

| 要求 | 实现 | 说明 |
|------|------|------|
| 区域地图 | ✅ | `map_file` 可指定任意 PCD 地图（如 `config/RM2024.pcd` 或自建图） |
| 旋转且移动的机器人 | ✅ | dynamic_cloud 提取动态点，cluster 聚类，kalman 跟踪 |
| 精确识别机器人 | ✅ | 雷达 + 相机融合，可区分红/蓝方及编号 |
| 位置准确标识在区域地图上 | ✅ | 输出为 `rm_frame` 地图坐标，debug_map 在 2D 地图上显示 |

### 机器人类型与编号

RoboMaster 装甲板编号与机器人类型对应（索引 0–5）：

| 索引 | 机器人类型 |
|------|------------|
| 0 | 英雄 |
| 1 | 工程 |
| 2–4 | 步兵 |
| 5 | 哨兵 |

相机识别装甲板编号后，通过 `camera_match` 与雷达轨迹融合，即可得到对应机器人类型。

### 展示启动流程

1. 启动完整融合：Livox + 相机 + localization + dynamic_cloud + cluster + kalman_filter  
2. 相机发布 `/resolve_result`：包含红/蓝装甲板 2D 位置及编号  
3. 启动 debug_map：订阅 `/kalman_detect`，在 `config/RM2024.png` 上绘制红/蓝方位置和编号  

```bash
# 启动 debug_map 节点（若未在 launch 中）
ros2 run debug_map debug_map_node
```

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
cd ~/Desktop/RadarStation
mkdir -p mapping_ws/src && cd mapping_ws/src

# 克隆 ROS2 版本的 FAST-LIO
git clone https://github.com/Ericsii/FAST_LIO.git -b ros2

# 初始化子模块
cd FAST_LIO
git submodule update --init --recursive

# 编译
cd ~/Desktop/RadarStation/mapping_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 建图流程

**步骤 1：配置网络并启动 Livox 驱动**

```bash
# 终端 1：配置网络
sudo ip addr add 192.168.1.5/24 dev enx00e04c2536b0
ping 192.168.1.114  # 测试雷达连接

# 启动驱动（必须使用 CustomMsg 格式）
cd ~/Desktop/RadarStation
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

> **重要**：建图时 `xfer_format` 必须设为 `1`（CustomMsg 格式），在 `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` 中修改。

**步骤 2：启动 FAST-LIO**

```bash
# 终端 2：启动建图
cd ~/Desktop/RadarStation/mapping_ws
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
ls -la ~/Desktop/RadarStation/mapping_ws/test.pcd

# 查看点云（可选）
pcl_viewer ~/Desktop/RadarStation/mapping_ws/test.pcd

# 移动到项目配置目录
cp ~/Desktop/RadarStation/mapping_ws/test.pcd ~/Desktop/RadarStation/config/RM2024.pcd
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

**问题 5：建图出来点云斜/畸变（非直角）**

- **原因**：Mid-360 需 `scan_line: 6`，原配置为 4 会导致点云处理错误
- **解决**：`mapping_ws/src/FAST_LIO/config/mid360.yaml` 中已改为 `scan_line: 6`，需重新建图
- **RViz 自动对齐**：默认 `auto_align:=true`，订阅地图后 RANSAC 拟合地面并发布 `rm_frame_display`；当前仓库自带 RViz 配置默认 Fixed Frame 就是 `rm_frame_display`。注意这只影响显示，不会改变 localization / dynamic_cloud 的真实坐标系：
  ```bash
  # 自动对齐（默认）
  ros2 launch dynamic_cloud lidar.launch.py

  # 关闭自动对齐，改用手动参数
  ros2 launch dynamic_cloud lidar.launch.py auto_align:=false display_correction_roll_rad:=0.03
  ```

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

当前实机融合使用的是 **NYUSH 标定流程**，不是旧的 T-DT `out_matrix.yaml` 五点外参流程。

### 当前生效的标定文件

- 标定脚本：`/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/calibration.py`
- testmap 标定结果：`/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/array_test_custom.npy`
- 标定参考地图：`/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/images/my_map.jpg`
- 运行显示地图：`/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/images/my_map(m).jpg`

### 当前推荐标定流程（NYUSH）

```bash
cd /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
python3 calibration.py
```

终端输入建议：

```text
camera mode selection (test/hik/video): hik
state selection (R/B): R
map profile (battle/testmap): testmap
number of heights [1]: 1
```

然后在界面里：

1. 点“开始标定”。
2. 左边相机图、右边地图图按同一顺序点击 4 对对应点。
3. 4 个点尽量分散，覆盖整个可视区域。
4. 当前建议先只标 `1` 层，先把地面机器人 fusion 跑通。
5. 点“保存计算”，结果写到 `array_test_custom.npy`。

### 标定点选择建议

- 只选同一平面上的固定点。
- 优先选边界清楚、几何结构稳定的角点。
- 不要把 4 个点都点在一小块区域。
- 左图和右图的点击顺序必须完全一致，比如始终按“左上 → 右上 → 右下 → 左下”。

### 标定后必须做的事

标定写入 `array_test_custom.npy` 后，运行中的 `nyush_world_node` 不会自动热更新，必须重启 NYUSH 或直接重启整套融合：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

### 标定验收方法

先不要看 LiDAR 上色，先单独确认 NYUSH 世界坐标对不对：

```bash
ros2 topic echo --once /resolve_result
ros2 run rqt_image_view rqt_image_view
```

重点看：

- `/detect_image`：框和装甲板标签是否稳定
- `/nyush_map_image`：红/蓝点是否落在地图上机器人真实所在区域

如果 `/detect_image` 正常、`/nyush_map_image` 的点明显偏很远，那么问题仍在标定，不在 LiDAR。

### 分辨率注意事项

NYUSH 世界坐标节点现在依赖 `calibration_width_px` / `calibration_height_px` 与运行时 `camera_image` 分辨率一致。

当前 `start_fusion.sh` 默认相机分辨率是 `1280 x 960`，并且会自动传：

- `calibration_width_px:=1280`
- `calibration_height_px:=960`

如果你手动改了相机输出分辨率，也要同步改这两个参数，否则世界坐标会整体偏很远。

### 旧的 T-DT 五点标定说明

`config/out_matrix.yaml` 和 `calib_rosbag.launch.py` 属于旧的 T-DT 视觉链路，当前 NYUSH + testmap 融合默认不走这套。除非你明确要回到 T-DT 检测链，否则不要再按旧的五点流程做标定。

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
# 运行相机驱动和 NYUSH 世界坐标节点
ros2 run hik_camera hik_camera_node
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=R \
  field_width_m:=3.82 \
  field_height_m:=6.79 \
  calibration_width_px:=1280 \
  calibration_height_px:=960 \
  publish_debug_map:=true
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
ip addr show enx00e04c2536b0

# 配置 IP（每次重启需要重新配置）
sudo ip addr add 192.168.1.5/24 dev enx00e04c2536b0

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

- 检查 Fixed Frame 是否正确（当前仓库自带 RViz 配置默认是 `rm_frame_display`）
- 检查 TF 是否正常：`ros2 run tf2_ros tf2_echo rm_frame livox_frame`
- 调整 Decay Time 为 0.5-1 秒
- 尝试将 Reliability Policy 改为 `Best Effort`

### 6. RViz Image 显示 "No Image"

- 检查话题是否有数据：`ros2 topic hz /detect_image`
- 将 Reliability Policy 改为 `Best Effort`
- 使用 rqt_image_view：`ros2 run rqt_image_view rqt_image_view`

### 7. launch 报 `display_aligner_node.py` executable not found

这通常出现在 `colcon build --symlink-install` 后，源码脚本丢了执行权限。

```bash
chmod +x src/lidar/dynamic_cloud/scripts/display_aligner_node.py
colcon build --packages-select dynamic_cloud --symlink-install
```

然后重新启动 LiDAR launch。

### 8. 融合颜色不正确（五颜六色）

**常见原因**:

1. NYUSH 世界坐标未正确标定
2. `nyush_integration` 的 `state` 传错（R/B 反了）
3. `field_width_m` / `field_height_m` 与当前 testmap 不一致
4. `/resolve_result` 与 `/livox/lidar_cluster` 不在同一 `rm_frame` 尺度下
5. `calibration_width_px` / `calibration_height_px` 与当前相机分辨率不一致

**解决方案**:
1. 用 `python3 calibration.py` 重做 NYUSH testmap 标定
2. 检查 `ros2 topic echo --once /resolve_result` 是否持续有非零点
3. 检查 `/nyush_map_image` 的点是否已经落在地图正确区域
4. 检查 `state:=R/B` 是否与己方一致
5. 检查 `field_width_m` / `field_height_m`
6. 检查 `calibration_width_px` / `calibration_height_px` 是否和当前 `camera_image` 一致
7. 临时方案：增大匹配距离阈值（运行时传 `CAMERA_DETECT_RADIUS=1.5`，不要再直接改 `filter_plus.h`）

### 9. NYUSH 检测节点报错 ModuleNotFoundError

```bash
# 安装缺失的 Python 包
sudo pip3 install 'numpy<2' IPython onnxruntime-gpu
```

### 10. TensorRT engine 版本不兼容

```bash
# 重新生成 engine 文件
cd /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
source .venv/bin/activate
python onnx2engine.py
```

### 11. 静止物体被误识别为 cluster/kalman 目标

- 当前默认已加入 Kalman 静止过滤：位移小、持续时间长的目标不会继续发布
- 若只是为了联调确认“相机是否已经给这个 cluster 上色”，可临时加 `PUBLISH_STATIONARY_TARGETS=true`
- 若静止物仍误检：先保持 `ceiling_z_max:=100.0`，把 `kd_tree_threshold_sq` 从 `0.15` 慢慢加回 `0.18` 或 `0.22`
- 若人能被检测到，但低矮机器人难被检测到：这是**反方向**问题，不要继续增大 `kd_tree_threshold_sq`

### 12. 人能出 Kalman，机器人不容易出

这通常不是雷达“完全没看到机器人”，而是动态判定和聚类参数对低矮目标过于保守。

- 原因 1：`kd_tree_threshold_sq` 太大时，离地较近的机器人点会被地面当成静态地图吸走
- 原因 2：`min_cluster_size` 太大时，小机器人稀疏点云凑不成簇

推荐从这里开始：

```bash
KD_TREE_THRESHOLD_SQ=0.15 MIN_CLUSTER_SIZE=8 SELF_COLOR=R ./scripts/start_fusion.sh
```

若仍不够激进：

```bash
KD_TREE_THRESHOLD_SQ=0.12 MIN_CLUSTER_SIZE=6 SELF_COLOR=R ./scripts/start_fusion.sh
```

### 13. Kalman 颜色含义？为什么是灰色？

- **红**=红方车、**蓝**=蓝方车，由相机装甲板检测+雷达位置匹配决定
- **灰**=未匹配（雷达先检测到，相机尚未识别到装甲板）
- 需相机+resolve 发布 `/resolve_result` 才会变红蓝
- 若加了 `PUBLISH_STATIONARY_TARGETS=true` 之后仍然是灰色，说明“这个轨迹已经被发出来了，但颜色身份仍然没匹配上”

### 15. `/nyush_map_image` 上的点会闪一下然后消失

这通常不是 RViz 或 rqt 的 bug，而是 NYUSH 检测结果不稳定：

- 绿框来自车检测，只要车框还在就会显示。
- 地图上的红/蓝点依赖装甲板分类和世界坐标映射。
- 当前 `window_size` 默认是 `2`，如果连续几帧没有拿到稳定的装甲板类别，`/resolve_result` 就会重新变成全 0，地图点会消失。

先检查：

```bash
ros2 topic echo --once /resolve_result
```

如果经常全 0，先稳 NYUSH 检测，再看 fusion 上色。

### 16. `detect_image` 有框，但 `/livox/lidar_kalman` 仍然没有红蓝颜色

这说明视觉链和 LiDAR 链都各自有输出，但融合层还没匹配上。

当前 `kalman_filter` 的相机匹配条件是：

- 时间差 `< 1s`
- 空间距离 `< 1m`

只要 `/resolve_result` 不稳定，或者 NYUSH 世界坐标和 LiDAR cluster 仍差 1 米以上，`/livox/lidar_kalman` 就会继续显示灰色。

当前 2026-03-17 实机联调里已经抓到过一组典型数据：

- `/resolve_result`：`R4 ≈ (3.81, 6.56)`
- `/livox/lidar_kalman` 灰色主点：`≈ (3.55, 1.83)`

这说明“颜色不上”并不等于 Kalman 坏了，而是相机世界坐标当前仍没有对到 LiDAR 主目标上。

推荐排查顺序：

1. 先确认 `/resolve_result` 不是全 0。
2. 再看 `/nyush_map_image` 是否已经在地图正确位置。
3. 然后再开 `kalman_filter` 的 `debug_camera_match` 看最近距离和时间差。
4. 若目标基本不动，联调时加 `PUBLISH_STATIONARY_TARGETS=true`，避免被静止过滤误导。

### 14. Kalman 漂移 / 点云一顿一顿 / 移动时卡顿

- 当前默认已改成 `process_every_n:=1`、`accumulate_time:=3`，优先保证检测连续性
- 若仍卡顿：把 `process_every_n` 提到 `2/3`，或增大 `voxel_leaf_size`
- 排查：`lidar_minimal.launch` 若流畅则卡顿来自 dynamic_cloud

---

## 项目结构

### RadarStation（ROS2 主项目）

```
RadarStation/
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
- [x] `display_aligner_node.py` 执行权限修复
- [x] LiDAR 默认启动链路改为当前实机推荐配置（默认关闭 tilt correction）
- [x] `start_fusion.sh` 统一地图、LiDAR、Camera、NYUSH 启动参数
- [x] NYUSH 标定分辨率参数接入 `start_fusion.sh`
- [x] NYUSH `calibration_width_px` / `calibration_height_px` 参数类型修复

### 视觉模块

- [x] detect 节点添加图像发布功能
- [x] 启用 debug 模式绘制检测框
- [x] 添加 OpenCV 窗口显示高清检测结果
- [x] 创建 NYUSH ROS2 检测节点 `nyush_detect_node.py`
- [x] 创建 NYUSH 启动文件 `nyush_detect.launch.py`
- [x] 创建 NYUSH 世界坐标节点 `nyush_world_node.py`
- [x] 接入 `/nyush_map_image` 调试图
- [x] 标定界面亮度增强（CLAHE + gamma + bias）
- [x] NYUSH 运行时标定分辨率自动/显式对齐

### 2026-03-17 实机联调进展

| 模块 | 文件 | 修改 / 结论 |
|------|------|-------------|
| start_fusion | `scripts/start_fusion.sh` | 统一 `test.pcd`、Camera 1280x960、NYUSH 标定分辨率、`publish_debug_map:=true`、`camera_detect_radius`、`publish_stationary_targets`、`process_width/process_height` |
| image_view | `scripts/wait_for_image_topics.sh` | 等待 `/detect_image` 和 `/nyush_map_image` 就绪后再打开 `rqt_image_view` |
| diagnose | `scripts/check_latency.sh` | 增强链路排查，直接查看联调相关参数与状态 |
| LiDAR | `src/lidar/dynamic_cloud/launch/lidar.launch.py` | 默认走水平点云链路，`rm_frame_display` 只做显示摆正，并透传融合调试参数 |
| LiDAR | `src/lidar/dynamic_cloud/launch/lidar_tilt.launch.py` | 仅保留给“原始点云本身明显倾斜”的情况，并透传融合调试参数 |
| cluster | `src/lidar/cluster/src/cluster.cpp` | 暴露聚类参数，修复质心初始化，改善低矮机器人检出 |
| hik_camera | `src/hik_camera/src/hik_camera_node.cpp` | 相机提亮改成 LUT，降低 `detect_image` 延迟 |
| NYUSH | `src/tdt_vision/launch/nyush_integration.launch.py` | 接入 `calibration_width_px / calibration_height_px`、`publish_debug_map`、`process_width / process_height` |
| NYUSH | `src/tdt_vision/detect/scripts/nyush_world_node.py` | 修复类型冲突和分辨率错配，扩大 armor ROI，支持 miss 重试，状态日志，最新帧处理，内部低分辨率推理 |
| NYUSH | `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/calibration.py` | 标定界面提亮，便于实机点击 |
| Fusion | `src/fusion/kalman_filter/src/kalman_filter.cpp` | `debug_camera_match`、`camera_detect_radius`、`publish_stationary_targets`，并输出 `kalman_publish` 日志 |
| Fusion | `src/fusion/kalman_filter/include/filter_plus.h` | 当前默认灰色表示“尚未匹配颜色”；实际不上的主因仍在世界坐标对齐，不在 Kalman 颜色逻辑本身 |

### 2026-02-27 雷达优化与模拟雷达识别（开发进度）

| 模块 | 文件 | 修改 |
|------|------|------|
| dynamic_cloud | `src/lidar/dynamic_cloud/` | 天花板过滤(ceiling_z_max)、体素 0.25、process_every_n 跳帧、kd_tree_threshold_sq |
| kalman_filter | `src/fusion/kalman_filter/include/filter_plus.h` | is_stationary()、get_output_point()、dt 限制、Q/R、灰色 |
| kalman_filter | `src/fusion/kalman_filter/src/kalman_filter.cpp` | 静止过滤、输出用 get_output_point() |
| cluster | `src/lidar/cluster/src/cluster.cpp` | MinClusterSize 5→18 |
| localization | `src/lidar/localization/src/localization.cpp` | GICP 迭代 35、收敛 1e-5、TF blend 0.2 |
| launch | `src/lidar/dynamic_cloud/launch/lidar.launch.py` | ceiling_z_max、kd_tree_threshold_sq、process_every_n |
| docs | `docs/LIDAR.txt` | 静止误判、Kalman 颜色、卡顿 Q&A |
| docs | `README.md` | 融合数据流、模拟雷达识别、参数表、常见问题、更新日志 |

---

## 致谢

本项目基于东北大学 T-DT 实验室的开源代码开发，感谢原作者的贡献。

- **原项目技术报告**: [https://bbs.robomaster.com/wiki/260375/27115](https://bbs.robomaster.com/wiki/260375/27115)

## 联系方式

- **团队**: NYU Robotics

---

## 更新日志

### 2026-03-17

- **LiDAR 实机链路重新对齐**
  - 确认当前原始 `/livox/lidar` 基本水平，默认运行改回 `lidar.launch.py`
  - `start_fusion.sh` 统一使用 `mapping_ws/test.pcd`
  - `display_aligner_node.py` 执行权限修复，`rm_frame_display` 可正常显示摆正
- **机器人检出参数更新**
  - `kd_tree_threshold_sq` 默认调整为 `0.15`
  - `cluster_tolerance` 参数化
  - `min_cluster_size` 默认允许由启动脚本覆盖为 `8`
- **NYUSH 融合链打通**
  - `start_fusion.sh` 默认传入 `state`、`field_width_m`、`field_height_m`
  - 新增默认 `publish_debug_map:=true`
  - 新增 `wait_for_image_topics.sh`，`rqt_image_view` 改成等待 `/detect_image` 和 `/nyush_map_image` 就绪后再打开
- **NYUSH 标定与分辨率问题修复**
  - `nyush_world_node.py` 增加 `calibration_width_px` / `calibration_height_px`
  - 修复 launch 参数类型冲突
  - 运行时默认对齐到当前相机分辨率 `1280 x 960`
  - 修复“标定后世界坐标整体差很远”的主要分辨率错配问题
- **NYUSH 检测稳定性增强**
  - 车框阈值降到 `0.15`
  - 默认扩大 armor ROI：`0.12 / 0.18`
  - armor 首次 miss 时自动对增强后的 ROI 再试一次
  - 新增 `NYUSH status: ...` 诊断日志，区分“车框有了但 armor 没出”和“armor 有了但窗口未填满”
- **融合调试能力增强**
  - `kalman_filter` 新增 `debug_camera_match`
  - `camera_detect_radius` 改成运行时参数，默认 `1.0m`
  - 新增 `publish_stationary_targets`，用于联调静止目标
  - 新增 `kalman_publish: ...` 日志，区分“静止被过滤”与“没有颜色匹配”
- **detect_image 延迟优化**
  - `hik_camera_node` 的亮度增强改成 LUT
  - `nyush_world_node` 改成只处理最新帧
  - NYUSH 内部检测默认使用 `960x720`，再映射回 `1280x960` 标定尺度
- **当前剩余问题**
  - `/resolve_result` 仍不够稳定
  - `/nyush_map_image` 的点会出现但可能闪烁
  - `/livox/lidar_kalman` 颜色融合仍未完全稳定
  - 当前已抓到过 `R4 ≈ (3.81, 6.56)` 对 `lidar_kalman ≈ (3.55, 1.83)` 的不一致样本，说明主 blocker 仍是视觉世界坐标未对齐

### 2026-02-27 雷达优化与模拟雷达识别

#### 一、修改文件清单

| 文件 | 修改内容 |
|------|----------|
| `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp` | 天花板过滤、体素、跳帧、kd_tree 参数化 |
| `src/lidar/dynamic_cloud/include/dynamic_cloud.h` | 新增 ceiling_z_max_、kd_tree_threshold_sq_、process_every_n_、frame_counter_、last_dynamic_cloud_ |
| `src/lidar/dynamic_cloud/launch/lidar.launch.py` | 新增 ceiling_z_max、kd_tree_threshold_sq、process_every_n 启动参数 |
| `src/fusion/kalman_filter/include/filter_plus.h` | is_stationary()、get_output_point()、dt 限制、Q/R 调参、灰色、transitionMatrix 动态更新 |
| `src/fusion/kalman_filter/src/kalman_filter.cpp` | 静止过滤、get_output_point() 替代 predict_point |
| `src/lidar/cluster/src/cluster.cpp` | MinClusterSize 5→18 |
| `src/lidar/localization/src/localization.cpp` | GICP 迭代 35、收敛 1e-5、TF blend 0.2、日志节流 |
| `docs/LIDAR.txt` | 新增 Q&A（静止误判、Kalman 颜色、卡顿） |
| `README.md` | 融合数据流、模拟雷达识别、参数表、常见问题、更新日志 |

#### 二、详细修改说明

**1. 天花板过滤（dynamic_cloud）**

- **问题**：天花板被误判为动态，cluster 出现在天上
- **修改**：在**雷达坐标系**（livox_frame）下，transform 之前丢弃 `z > ceiling_z_max` 的点
- **原逻辑**：在地图坐标系下 `z > 3.2` 丢弃（已移除）
- **新逻辑**：`receive_cloud` 中 `pt.z <= ceiling_z_max` 才保留；当前 50 度倾角安装默认改为 100.0m，等效关闭
- **参数**：`ceiling_z_max`，launch 可覆盖

**2. 静止目标过滤（Kalman）**

- **问题**：地图未包含的固定物（柱子、障碍）被当成动态目标
- **修改**：`filter_plus.h` 新增 `is_stationary(displacement_thresh=0.2, min_time_span=1.2, min_history=10)`
- **逻辑**：若轨迹在 1.2s 内位移 < 0.2m，视为静止，不发布到 `/livox/lidar_kalman` 和 `/kalman_detect`
- **调用**：`kalman_filter.cpp` 中发布前 `if (KFs[i].is_stationary()) continue;`

**3. 误判过滤**

- **kd_tree_threshold_sq**：与地图最近邻平方距离阈值，当前 launch 默认回到 `0.15`，优先保留低矮机器人
- **cluster MinClusterSize**：launch 默认 `12`，`start_fusion.sh` 当前覆盖为 `8`，减少小机器人漏检

**4. Kalman 漂移修复（filter_plus.h）**

- **输出**：新增 `get_output_point()`，优先用 `history.back()` 最近观测，减少漂移
- **dt 限制**：`update_predict_point()` 中 `dt_ = min(dt_real, 0.15f)`，避免大间隔外推
- **Q/R**：`sigma_q` 50→15，`sigma_r` 0.1→0.3，减弱对 cluster 质心跳变的敏感
- **transitionMatrix**：每次 predict 前按当前 dt_ 更新
- **未匹配颜色**：`rand()` 随机色 → 固定灰色 (128,128,128)

**5. 性能优化（移动时卡顿）**

- **dynamic_cloud 体素**：0.15→0.2→0.25，减少 kd-tree 输入点数
- **process_every_n**：每 N 帧做一次 kd-tree，其余帧复用 `last_dynamic_cloud_`，默认 2（约减半计算）
- **GICP**：`setMaximumIterations(50→35)`，`setTransformationEpsilon(1e-6→1e-5)`
- **TF blend**：0.35→0.2，TF 插值更平滑
- **日志**：GICP、dynamic_cloud 的 throttle 间隔增大

**6. 模拟雷达识别功能**

- **展示要求**：区域地图 + 步兵/英雄/工程/哨兵 + 位置标识
- **实现**：雷达提供位置，相机提供红/蓝和编号，debug_map 在 2D 地图上绘制
- **文档**：新增「模拟雷达识别功能」章节、融合数据流详解

**7. 文档**

- 融合数据流：明确雷达提供位置、相机提供身份
- 参数表：补充 ceiling_z_max、kd_tree_threshold_sq、process_every_n
- 常见问题：静止误判、Kalman 颜色、卡顿

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

*最后更新: 2026-03-17*
