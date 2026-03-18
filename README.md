<div align="center">

# NYU Robotics Radar Station

> NYU Robotics RoboMaster 雷达站代码
> 基于 T-DT 2024 Radar 项目
> 当前主链路：Livox Mid-360 + Hik Camera + NYUSH ROS2 视觉 + TDT 融合

<a href="./LICENSE"><img alt="License" src="https://img.shields.io/badge/License-MIT-yellow"></a>

</div>

---

## 目录

- [项目定位](#项目定位)
- [先看这三件事](#先看这三件事)
- [今天的进展（2026-03-18）](#今天的进展2026-03-18)
- [当前系统状态](#当前系统状态)
- [当前推荐配置（2026-03）](#当前推荐配置2026-03)
- [系统架构总览](#系统架构总览)
- [仓库与外部依赖](#仓库与外部依赖)
- [环境要求](#环境要求)
- [安装与编译](#安装与编译)
- [快速开始](#快速开始)
- [标准工作流：从建图到融合](#标准工作流从建图到融合)
- [地图、标定与坐标系说明](#地图标定与坐标系说明)
- [今天修掉的关键逻辑](#今天修掉的关键逻辑)
- [运行后如何验收](#运行后如何验收)
- [常用调试命令](#常用调试命令)
- [常见问题](#常见问题)
- [当前仍未完全解决的问题](#当前仍未完全解决的问题)
- [重要文档索引](#重要文档索引)
- [项目结构](#项目结构)
- [更新日志](#更新日志)

---

## 项目定位

本项目用于 RoboMaster 雷达站场景下的目标检测与后融合。当前仓库的实际目标不是“做一个泛化平台”，而是围绕当前实机链路，把以下几件事稳定跑通：

- LiDAR 基于 `mapping_ws/test.pcd` 完成定位、动态点提取、聚类、卡尔曼
- 相机通过 NYUSH 视觉链路输出世界坐标 `/resolve_result`
- LiDAR 与相机在 `rm_frame` 中进行后融合
- 使用 2D 地图和 RViz 做联调与验收

当前推荐阅读顺序：

1. 先读本 README，了解当前有效结论和启动方式
2. 再看 `docs/RADAR_STATION_INTEGRATED_WORKFLOW.md`
3. 如果是对齐问题，重点看 `docs/CURRENT_BUG.md`
4. 如果是时间语义 / 延迟问题，重点看 `docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md`

---

## 先看这三件事

### 1. 当前主入口

实机默认主入口是：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

### 2. 当前主地图与主对齐文件

- LiDAR 运行地图：`mapping_ws/test.pcd`
- NYUSH 运行地图：`/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/images/my_map(m).jpg`
- `testmap -> rm_frame` 对齐文件：`config/local/testmap_to_rm_frame.yaml`

### 3. 当前最重要的真实结论

- `testmap_metric -> rm_frame` 的镜像问题今天已经修正，运行时现在会使用 `runtime_transform.matrix_3x3`
- 当前本地 `testmap_to_rm_frame.yaml` 已重新求解，选择的是 `flip_x`
- 当前本地对齐结果：
  - `rmse = 0.0361 m`
  - `max = 0.0478 m`
  - `4 / 4` inliers
- 运行时现在会拒绝明显错误的对齐文件，不会再静默加载坏矩阵
- 但倾斜安装链路和一些硬编码比赛场地几何仍然没有完全清理

---

## 今天的进展（2026-03-18）

今天做的不是单点修补，而是把 `testmap`、`test.pcd`、标定、运行时加载、2D 可视化这整条链路从头到尾梳了一遍。

### 今天完成的核心工作

1. 审核了从相机标定到运行时融合的完整 `testmap` 工作流
   - 核对了外部 NYUSH 仓库的 `calibration.py`
   - 核对了 `my_map.jpg` 与 `my_map(m).jpg` 的关系
   - 核对了 `nyush_world_node.py` 中 `testmap` 的像素到米转换
   - 核对了 `calibrate_testmap_lidar_alignment.py` 的求解模型
   - 核对了 `config/local/testmap_to_rm_frame.yaml` 的实际拟合质量

2. 找到了之前“只在地图一侧拟合正确”的真正原因
   - 不是 `my_map.jpg` / `my_map(m).jpg` 混用了
   - 而是 `testmap_metric` 和 LiDAR `rm_frame` 在 `x` 方向存在镜像关系
   - 旧求解器只允许旋转、平移、等比例缩放，不允许镜像
   - 所以旧 RANSAC 会只吃进一侧点，把另一侧当成离群点

3. 实现了新的对齐求解逻辑
   - `src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py`
   - 新增 `--orientation-mode auto|normal|flip_x`
   - `auto` 模式会同时尝试：
     - 正常同手性拟合
     - `x` 翻转后的拟合
   - 并保存真正应该在运行时应用的 `runtime_transform.matrix_3x3`

4. 实现了运行时保护逻辑
   - `src/tdt_vision/detect/scripts/nyush_world_node.py`
   - 优先读取 `runtime_transform.matrix_3x3`
   - 如果 YAML 里有对应点，会重新按保存点计算误差
   - 若误差过大，会拒绝加载这个对齐文件
   - 不再出现“文件存在就默认强上”的情况

5. 统一了 2D 地图显示逻辑
   - `src/fusion/debug_map/debug_map.cpp`
   - 现在和运行时使用同一个矩阵字段
   - 如果对齐文件明显不可信，2D 地图不会继续显示错误的 NYUSH warp 背景

6. 重新生成了当前本地对齐文件
   - `config/local/testmap_to_rm_frame.yaml`
   - 结果为：
     - `selected = flip_x`
     - `rmse = 0.036085625341434356 m`
     - `max = 0.047770712207540963 m`
     - `inliers = 4 / 4`

7. 补充了文档
   - `docs/CURRENT_BUG.md`
   - `docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md`

### 今天产出的实际结论

- 现在这套 `my_map(m)` 与 `test.pcd` 的主对齐问题已经从数学模型层面修正
- 当前本地 `testmap_to_rm_frame.yaml` 也已经更新为可接受版本
- 也就是说，就“`testmap -> rm_frame` 是否应当对齐”这一件事来说，当前仓库应该是对齐可用状态
- 但以下问题依然独立存在：
  - 倾斜安装工作流没有彻底打通
  - tilt 正负号约定不统一
  - 时间对齐不是严格的相机/LiDAR 同步配对
  - 部分辅助输出仍硬编码官方 RM 场地几何

---

## 当前系统状态

| 模块 | 状态 | 说明 |
|------|------|------|
| LiDAR 建图（FAST-LIO） | ✅ 可用 | 生成 `mapping_ws/test.pcd` |
| LiDAR 主链路 | ✅ 可用 | 定位、动态点、聚类、卡尔曼可跑 |
| NYUSH ROS2 视觉 | ✅ 可用 | `detect_image`、`resolve_result`、`nyush_map_image` 可发布 |
| `testmap -> rm_frame` 对齐 | ✅ 今日修复 | 现支持 `flip_x`，当前本地 YAML 已重求解 |
| 2D 调试地图 | ✅ 可用 | `debug_map` 现使用统一 runtime transform |
| T-DT 视觉（rosbag） | ✅ 可用 | 用于高分辨率 bag 调试 |
| T-DT 视觉（实际相机） | ❌ 不推荐 | 分辨率需求与当前实际相机不匹配 |
| 倾斜安装流程 | ⚠️ 未完全闭环 | 旋转地图与运行时 map 选择仍有缺口 |
| 时间对齐语义 | ⚠️ 有风险 | 当前不是严格 pairwise sync |

---

## 当前推荐配置（2026-03）

以下内容以当前仓库代码和 `scripts/start_fusion.sh` 为准。

| 项目 | 当前推荐值 | 说明 |
|------|------------|------|
| LiDAR 地图 | `mapping_ws/test.pcd` | 当前实机主地图 |
| 默认 LiDAR launch | `dynamic_cloud/lidar.launch.py` | 当前默认不开 tilt |
| `USE_TILT_CORRECTION` | `false` | 仅当原始 `/livox/lidar` 明显倾斜时才开 |
| `FIELD_WIDTH_M` | `6.79` | 对应 `my_map(m).jpg` 横向宽度 |
| `FIELD_HEIGHT_M` | `3.82` | 对应 `my_map(m).jpg` 纵向高度 |
| `APPLY_TESTMAP_RM_ALIGNMENT` | `true` | 运行时默认启用 testmap 对齐 |
| 对齐配置 | `config/local/testmap_to_rm_frame.yaml` | 当前已更新为 `flip_x` 解 |
| `WINDOW_SIZE` | `1` | NYUSH 当前默认低延迟 |
| `MAX_PROCESSING_FPS` | `12.0` | 相机侧处理帧率上限 |
| `CAMERA_DETECT_RADIUS` | `1.0` | LiDAR-相机匹配半径 |
| `PUBLISH_STATIONARY_TARGETS` | `false` | 联调时可临时打开 |

当前推荐实机启动：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

常用变体：

```bash
# 看相机/LiDAR 最近距离与时间差
DEBUG_CAMERA_MATCH=true SELF_COLOR=R ./scripts/start_fusion.sh

# 目标几乎不动时，临时允许静止目标也发布到 kalman
PUBLISH_STATIONARY_TARGETS=true SELF_COLOR=R ./scripts/start_fusion.sh

# 仅当原始 /livox/lidar 本身明显倾斜时才开
USE_TILT_CORRECTION=true SELF_COLOR=R ./scripts/start_fusion.sh

# 临时放宽相机匹配半径做联调验证
CAMERA_DETECT_RADIUS=1.5 SELF_COLOR=R ./scripts/start_fusion.sh
```

---

## 系统架构总览

### 1. LiDAR 链路

```text
Livox Mid-360
  -> livox_ros_driver2
  -> localization / rm_frame
  -> dynamic_cloud
  -> cluster
  -> kalman_filter
  -> /livox/lidar_kalman + /kalman_detect
```

### 2. 相机链路

```text
Hik Camera
  -> hik_camera_node
  -> /camera_image
  -> nyush_world_node.py
  -> /detect_image + /nyush_map_image + /resolve_result
```

### 3. 融合链路

```text
/livox/lidar_cluster
  + /resolve_result
  -> kalman_filter camera_match()
  -> /livox/lidar_kalman
  -> /kalman_detect
```

### 4. 地图与坐标链路

```text
camera_image
  -> NYUSH homography
  -> my_map.jpg（竖版 calibration map）
  -> rotate 90 deg CCW
  -> my_map(m).jpg（横版 runtime map）
  -> raw testmap_metric
  -> runtime_transform.matrix_3x3
  -> rm_frame
```

### 5. 当前最关键的理解

- `/nyush_map_image` 显示的是原始 `testmap` 空间
- `/resolve_result` 在启用对齐后，发布的是已经映射到 `rm_frame` 的坐标
- `debug_map` 现在会使用与 `/resolve_result` 相同的 runtime transform
- 如果 `testmap_to_rm_frame.yaml` 明显错误，运行时会拒绝它，而不是静默套用

---

## 仓库与外部依赖

### 本仓库

- 路径：`/home/nyu/Desktop/RadarStation`
- 作用：ROS2 主工程，包含 LiDAR、相机、融合、启动脚本、调试文档

### 外部 NYUSH 仓库

- 路径：`/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation`
- 作用：
  - 存放 NYUSH 模型
  - 存放 `my_map.jpg` / `my_map(m).jpg`
  - 承担 `calibration.py` 标定流程

### 当前和对齐最相关的文件

- `scripts/start_fusion.sh`
- `scripts/start_mapping.sh`
- `scripts/rotate_pcd.py`
- `src/tdt_vision/detect/scripts/nyush_world_node.py`
- `src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py`
- `src/fusion/debug_map/debug_map.cpp`
- `config/local/testmap_to_rm_frame.yaml`

---

## 环境要求

| 类别 | 要求 |
|------|------|
| 系统 | Ubuntu 22.04 |
| ROS | ROS2 Humble |
| Python | 3.10 |
| CUDA | 11.x 或更高 |
| TensorRT | 8.x-10.x |
| OpenCV | 4.x |
| PCL | 1.12.x |
| LiDAR | Livox Mid-360 |
| Camera | Hikvision MV-CA016-10UC 或同类 |

---

## 安装与编译

### 1. ROS2 与系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-nav2-map-server \
  ros-humble-foxglove-bridge \
  ros-humble-cv-bridge \
  ros-humble-pcl-conversions \
  ros-humble-tf2-ros \
  ros-humble-rqt-image-view \
  build-essential cmake git python3-pip
```

### 2. Python 依赖

```bash
sudo pip3 install 'numpy<2' IPython onnxruntime-gpu
```

如果要使用今天修过的 `calibrate_testmap_lidar_alignment.py`，还需要：

```bash
sudo pip3 install open3d
```

### 3. Livox SDK2

```bash
cd ~
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 4. 海康 MVS SDK

用于实际相机时，需要从海康官网下载并安装到 `/opt/MVS/`。

### 5. 编译

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

按需编译单包：

```bash
colcon build --packages-select tdt_vision --symlink-install
colcon build --packages-select debug_map --symlink-install
colcon build --packages-select dynamic_cloud --symlink-install
colcon build --packages-select kalman_filter --symlink-install
```

---

## 快速开始

### 1. 当前推荐：一键实机融合

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

### 2. 只验证 NYUSH 世界坐标

```bash
cd /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
python3 calibration.py

cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=R \
  field_width_m:=6.79 \
  field_height_m:=3.82 \
  publish_debug_map:=true \
  apply_testmap_rm_alignment:=true \
  alignment_config_path:=/home/nyu/Desktop/RadarStation/config/local/testmap_to_rm_frame.yaml
```

### 3. 只验证 LiDAR 主链路

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 \
ros2 launch dynamic_cloud lidar.launch.py \
  map_file:=mapping_ws/test.pcd \
  ceiling_z_max:=100.0
```

### 4. rosbag 回放

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash

# 终端 1
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 \
ros2 launch dynamic_cloud lidar.launch.py \
  map_file:=config/RM2024.pcd \
  ceiling_z_max:=100.0

# 终端 2
ros2 bag play bag/merged_bag_0.db3 --loop --clock
```

---

## 标准工作流：从建图到融合

### 步骤 1：LiDAR 建图

```bash
cd ~/Desktop/RadarStation
scripts/start_mapping.sh
```

建图结果默认保存到：

- `mapping_ws/test.pcd`

注意：

- 建图时不要同时运行 `start_fusion.sh`
- 当前默认运行地图就是 `mapping_ws/test.pcd`

### 步骤 2：相机标定

在外部 NYUSH 仓库执行：

```bash
cd /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
python3 calibration.py
```

这一步会基于竖版 `my_map.jpg` 完成相机到地图的 homography 标定。

### 步骤 3：`testmap -> rm_frame` 对齐

如果你更换了：

- `my_map(m).jpg`
- `mapping_ws/test.pcd`
- 或者重新点击了 testmap / LiDAR 对应点

则需要重新做 testmap 与 LiDAR 的对齐：

```bash
cd ~/Desktop/RadarStation
python3 src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py --orientation-mode auto
```

当前推荐始终使用 `--orientation-mode auto`，因为当前这套 map/pcd 关系已经证明可能存在 `x` 方向镜像。

### 步骤 4：启动融合

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

### 步骤 5：验收

至少确认以下内容：

1. `/resolve_result` 不是全 0
2. `/nyush_map_image` 上的点位大致落在真实目标附近
3. `/livox/lidar_cluster` 在 RViz 中能稳定看到动态目标
4. `/livox/lidar_kalman` 不再长期全灰
5. `/map_2d` 中 2D 点位与背景大致一致

---

## 地图、标定与坐标系说明

### 1. `test.pcd`

- 当前主地图：`mapping_ws/test.pcd`
- 它是 LiDAR 定位和动态点提取的工作地图
- 如果更换这张图，`testmap -> rm_frame` 对齐应重新做

### 2. `my_map.jpg` 与 `my_map(m).jpg`

- `my_map.jpg`：竖版 calibration map
- `my_map(m).jpg`：横版 runtime map

当前代码中的逻辑是：

```text
camera image
  -> homography 到 my_map.jpg
  -> rotate 90 deg CCW
  -> my_map(m).jpg
  -> raw testmap_metric
```

也就是说：

- `my_map.jpg -> my_map(m).jpg` 是旋转关系，不是镜像关系
- 今天修掉的问题发生在更后面，即 `testmap_metric -> rm_frame`

### 3. 当前 `testmap` 尺度

当前实机 `my_map(m).jpg` 使用：

- `field_width_m = 6.79`
- `field_height_m = 3.82`

不要再把它写反成 `3.82 x 6.79`。

### 4. 当前 `testmap -> rm_frame` 对齐文件

当前本地文件：

- `config/local/testmap_to_rm_frame.yaml`

当前状态：

- `format_version = 2`
- `orientation_hypothesis.selected = flip_x`
- `runtime_transform.matrix_3x3` 为当前运行时真正使用的矩阵

运行时接受条件：

- 至少 `3` 个对应点
- 至少 `max(3, ceil(0.75 * num_pairs))` 个点在 `0.25 m` 内
- `rmse <= 0.5 m`
- `max <= 1.0 m`

如果不满足，运行时会拒绝这个 YAML。

---

## 今天修掉的关键逻辑

### 1. `testmap -> rm_frame` 镜像问题

修复位置：

- `src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py`
- `src/tdt_vision/detect/scripts/nyush_world_node.py`
- `src/fusion/debug_map/debug_map.cpp`
- `config/local/testmap_to_rm_frame.yaml`

修复效果：

- 求解器现在能发现 `flip_x`
- 运行时现在加载的是 `runtime_transform`
- 运行时现在会拒绝坏 YAML
- 当前本地 YAML 已是通过验证的版本

### 2. README 与文档结构

今天同步做了文档层面的重构：

- README 改为当前有效流程优先
- `docs/CURRENT_BUG.md` 记录现存关键 bug
- `docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md` 记录时间语义审计结论

### 3. 当前对齐问题的最终结论

今天这轮修复之后，对齐问题应该这样理解：

- `m = my_map(m)` 的 `testmap_metric`
- `t = test.pcd` 的 LiDAR `rm_frame`
- 当前真实几何关系更接近 `t_x = C - m_x`
- 所以需要在拟合时允许 `flip_x`
- 修复后，这条链路在数学上已经自洽

---

## 运行后如何验收

### 关键话题

| 话题 | 作用 |
|------|------|
| `/camera_image` | 原始相机图像 |
| `/detect_image` | NYUSH 检测可视化 |
| `/nyush_map_image` | NYUSH 原始 testmap 显示 |
| `/resolve_result` | 已对齐到 `rm_frame` 的视觉世界坐标 |
| `/livox/lidar_cluster` | LiDAR 动态聚类结果 |
| `/livox/lidar_kalman` | 融合后的 Kalman 输出 |
| `/kalman_detect` | 调试 / 可视化用的融合输出 |
| `/map_2d` | 2D 调试地图 |

### 最少验收步骤

```bash
ros2 topic echo --once /resolve_result
ros2 topic echo --once /livox/lidar_kalman
ros2 topic list | grep -E "resolve_result|nyush_map_image|lidar_cluster|lidar_kalman|map_2d"
```

当前应当看到：

- `/resolve_result` 存在非零值
- `/nyush_map_image` 能持续刷新
- `/livox/lidar_cluster` 有动态目标
- `/livox/lidar_kalman` 至少在有匹配时出现稳定颜色

### 2D / RViz 验收重点

- RViz 的 `Fixed Frame` 用 `rm_frame`
- `/nyush_map_image` 看的是原始 `testmap`
- `/map_2d` 看的是 warp 到 `rm_frame` 后的背景
- 如果 `/nyush_map_image` 正常但 `/map_2d` 上完全对不上，优先查对齐文件

---

## 常用调试命令

### 1. 快速看核心节点和话题

```bash
ros2 node list | grep -E "nyush|hik|kalman|lidar|debug"
ros2 topic list | grep -E "camera_image|detect_image|nyush_map_image|resolve_result|lidar_cluster|lidar_kalman|map_2d"
```

### 2. 看相机/LiDAR 匹配日志

```bash
DEBUG_CAMERA_MATCH=true SELF_COLOR=R ./scripts/start_fusion.sh
```

### 3. 看当前启动参数和链路状态

```bash
./scripts/check_latency.sh
```

### 4. 等图像话题就绪后再开图像窗口

```bash
./scripts/wait_for_image_topics.sh
```

### 5. 完整雷达环境检查

```bash
./scripts/check_radar_setup.sh
```

---

## 常见问题

### 1. `/resolve_result` 仍然全 0

优先顺序：

1. 看 `/detect_image` 是否稳定有框
2. 看 `/nyush_map_image` 上是否持续有点
3. 看 `SELF_COLOR` 是否正确
4. 看当前标定是否和相机分辨率一致
5. 再决定是否重做 `calibration.py`

不要在 `/resolve_result` 全 0 的情况下先去调 LiDAR 聚类参数。

### 2. `/detect_image` 有框，但 `/livox/lidar_kalman` 仍长期全灰

这通常说明：

- LiDAR 轨迹已经有了
- 但 `/resolve_result` 没有持续稳定地落到 LiDAR 目标附近

建议先做：

```bash
DEBUG_CAMERA_MATCH=true CAMERA_DETECT_RADIUS=1.5 PUBLISH_STATIONARY_TARGETS=true SELF_COLOR=R ./scripts/start_fusion.sh
```

如果这样能临时上色，说明问题更可能在视觉坐标稳定性或匹配半径。

### 3. `/nyush_map_image` 点会闪一下然后消失

优先检查：

- `window_size`
- `max_inactive_time`
- `armor_conf`
- 相机画面里装甲板是否稳定

### 4. RViz 点云不显示

检查：

- `Fixed Frame` 是否为 `rm_frame`
- PointCloud2 的 QoS Reliability 是否为 `Best Effort`
- `/livox/lidar` 是否真的在发布

### 5. 只有在确认原始点云本身倾斜时才开 tilt

当前仓库里和 tilt 相关的剩余问题还没彻底闭环，所以不要把 `USE_TILT_CORRECTION=true` 当成默认选项。

---

## 当前仍未完全解决的问题

### 1. 倾斜安装工作流未完全闭环

- `rotate_pcd.py` 生成 rotated PCD
- 但主启动流仍默认使用 `mapping_ws/test.pcd`
- 如果真的走 tilt 模式，需要你自己确认 map 文件和 live cloud 使用的是一致版本

### 2. tilt 正负号约定仍不统一

当前仓库里仍然同时存在 `+50 deg` 和 `-50 deg` 的历史说明，需要后续统一。

### 3. 时间对齐不是严格同步配对

今天没有改这部分核心机制。当前融合不是严格 pairwise sync，而是“最新相机结果 + LiDAR 历史窗口 + 半径/时间门限”。

详细见：

- `docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md`

### 4. 部分辅助输出仍绑定官方 RM 场地几何

核心融合主链已经使用 `rm_frame`，但某些辅助逻辑仍然写死了官方地图尺寸，需要后续继续参数化。

---

## 重要文档索引

| 文档 | 用途 |
|------|------|
| `docs/RADAR_STATION_INTEGRATED_WORKFLOW.md` | 实机整体工作流 |
| `docs/LIDAR.txt` | LiDAR 命令速查 |
| `docs/CAMERA.txt` | 相机命令速查 |
| `docs/CURRENT_BUG.md` | 当前已知关键 bug 与修复状态 |
| `docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md` | 时间语义 / 延迟审计 |

如果你只想排当前最关键的问题：

- 对齐问题：看 `docs/CURRENT_BUG.md`
- 时间语义问题：看 `docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md`
- 实机流程：看 `docs/RADAR_STATION_INTEGRATED_WORKFLOW.md`

---

## 项目结构

```text
RadarStation/
├── scripts/
│   ├── start_fusion.sh
│   ├── start_mapping.sh
│   ├── rotate_pcd.py
│   ├── wait_for_image_topics.sh
│   ├── check_latency.sh
│   └── check_radar_setup.sh
├── config/
│   ├── RM2024.pcd
│   └── local/
│       ├── testmap_to_rm_frame.yaml
│       ├── testmap_to_rm_frame_topdown.png
│       └── testmap_to_rm_frame_preview.png
├── mapping_ws/
│   └── test.pcd
├── src/
│   ├── hik_camera/
│   ├── lidar/
│   ├── fusion/
│   ├── tdt_vision/
│   ├── livox_ros_driver2/
│   └── utils/
└── docs/
    ├── CURRENT_BUG.md
    ├── RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md
    ├── RADAR_STATION_INTEGRATED_WORKFLOW.md
    ├── LIDAR.txt
    └── CAMERA.txt
```

关键源码位置：

- NYUSH 视觉入口：`src/tdt_vision/detect/scripts/nyush_world_node.py`
- testmap/LiDAR 手动对齐工具：`src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py`
- 2D 调试地图：`src/fusion/debug_map/debug_map.cpp`
- 一键启动：`scripts/start_fusion.sh`

---

## 更新日志

### 2026-03-18

- 完成了 `testmap` + `test.pcd` 从标定到运行时的完整审计
- 证明 `my_map.jpg -> my_map(m).jpg` 是旋转关系，不是镜像问题来源
- 定位到真正问题在 `testmap_metric -> rm_frame` 的 `x` 向镜像关系
- 为 `calibrate_testmap_lidar_alignment.py` 新增 `orientation-mode auto|normal|flip_x`
- 让运行时读取 `runtime_transform.matrix_3x3`
- 为 `nyush_world_node.py` 新增对齐质量校验和坏 YAML 拒绝逻辑
- 让 `debug_map.cpp` 使用统一矩阵字段并在坏对齐时拒绝错误背景
- 重新生成当前本地 `config/local/testmap_to_rm_frame.yaml`
- 当前本地对齐结果更新为：
  - `selected = flip_x`
  - `rmse = 0.0361 m`
  - `max = 0.0478 m`
  - `4 / 4` inliers
- 补充 `docs/CURRENT_BUG.md`
- 补充 `docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md`
- 重构 README，改为当前有效流程优先

### 2026-03-17

- 统一 `start_fusion.sh` 的实机主链路
- 将 `mapping_ws/test.pcd` 作为当前主运行地图
- 默认使用 `lidar.launch.py`
- 打通 NYUSH `detect_image`、`/resolve_result`、`/nyush_map_image`
- 调整 NYUSH 处理分辨率和相机图像亮度优化
- 增加 `debug_camera_match`、`camera_detect_radius`、`publish_stationary_targets` 等联调参数

---

## 致谢

- T-DT 2024 Radar 项目
- NYUSH Robotics Radar Station 项目
- Livox ROS Driver2 / FAST-LIO / ROS2 社区
