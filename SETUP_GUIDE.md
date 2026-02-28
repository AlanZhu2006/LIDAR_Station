# T-DT-2024-Radar 项目配置与运行指南

## 已完成事项

### 1. 环境依赖安装

- [x] ROS2 Humble 环境配置
- [x] TensorRT 10.x 头文件配置（从 GitHub 克隆到 `/tmp/tensorrt_headers`）
- [x] ONNX Parser 头文件配置
- [x] libusb 冲突解决（MVS SDK 的旧版 libusb 与系统版本冲突）
- [x] nav2_map_server 安装：`sudo apt install ros-humble-nav2-map-server`
- [x] foxglove_bridge 安装：`sudo apt install ros-humble-foxglove-bridge`
- [x] libpcl-io 修复：`sudo apt install --reinstall libpcl-io1.12`
- [x] BaiduPCS-Go 安装（用于下载百度网盘 rosbag 文件）

### 2. Livox Mid-360 雷达配置

- [x] Livox-SDK2 编译安装
- [x] livox_ros_driver2 编译（使用 `./build.sh humble`）
- [x] 雷达配置文件修改：`src/livox_ros_driver2/config/MID360_config.json`
- [x] 点云格式修改：`xfer_format = 0`（PointCloud2 格式）
  - 文件：`src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`

### 3. 代码修改

#### 3.1 解决编译问题
- [x] Livox SDK v2.3.0 编译修复（添加 `#include <memory>`）
  - 文件：`src/livox_driver/livox_sdk_vendor/CMakeLists.txt`
- [x] TensorRT nvonnxparser 链接修复
  - 文件：`src/tdt_vision/CMakeLists.txt`、`src/tdt_vision/yolo/CMakeLists.txt`

#### 3.2 解决运行时问题
- [x] localization 空点云检查（防止 GICP 崩溃）
  - 文件：`src/lidar/localization/src/localization.cpp`
- [x] localization 过滤范围放宽（适应非比赛场地测试）
  - 原条件：`x(5,30), y(-10,8), z<7`
  - 新条件：`x(-50,50), y(-50,50), z(-5,10)`
- [x] dynamic_cloud 过滤范围放宽
  - 文件：`src/lidar/dynamic_cloud/src/dynamic_cloud.cpp`
  - 原条件：`x(3,28), y(0,15), z(0,1.4)`
  - 新条件：`x(-50,50), y(-50,50), z(-5,10)`

#### 3.3 TF 时间戳问题修复
- [x] localization 使用点云时间戳发布 TF
  - 文件：`src/lidar/localization/src/localization.cpp`
  - 修改：`publishTF()` 函数使用消息的 `header.stamp`
- [x] 添加静态 TF 广播器（10Hz 持续发布）
  - 使用 `tf2_ros::StaticTransformBroadcaster` 确保 TF 持久有效
  - TF 时间戳设为 `rclcpp::Time(0)` 避免时间戳不匹配问题
- [x] launch 文件添加 `use_sim_time` 参数支持
  - 文件：`src/lidar/dynamic_cloud/launch/lidar.launch.py`

#### 3.4 视觉模块修复
- [x] detect 节点添加图像发布功能
  - 文件：`src/tdt_vision/detect/src/detect.cpp`
  - 添加 `image_pub->publish()` 发布检测结果图像
- [x] 启用 debug 模式绘制检测框
  - 文件：`src/tdt_vision/detect/include/detect.h`
  - 设置 `debug = 1` 启用检测框绘制
- [x] 添加 OpenCV 窗口显示高清检测结果
  - 自动弹出 1280x960 窗口显示检测图像

### 4. 网络配置

- [x] 雷达网卡 IP 配置：`sudo ip addr add 192.168.1.5/24 dev enp4s0`
- [x] 雷达 IP：`192.168.1.114`

### 5. 测试数据

- [x] 下载比赛场地 rosbag 数据
  - 百度网盘链接：https://pan.baidu.com/s/1ogRvs3v1OMCVUbAlUsOGQA?pwd=52rm
  - 文件：`适应性训练第一把.zip` (3.76GB)
  - 解压后：`bag/merged_bag_0.db3` (4.7GB)
  - 时长：7分16秒，包含 `/livox/lidar`、`/compressed_image` 等话题

### 6. 视觉模块测试

- [x] YOLO 模型已转换（`yolo.engine`、`armor_yolo.engine`、`classify.engine`）
- [x] 使用 rosbag 测试视觉检测
- [x] 检测结果可视化（OpenCV 窗口 + ROS 话题）

### 7. 相机-雷达融合测试

- [x] 完整系统启动测试（雷达处理 + 视觉检测 + rosbag 回放）
- [x] 融合话题验证
  - `/resolve_result` - 相机 3D 位置解算结果（正常发布）
  - `/livox/lidar_kalman` - 卡尔曼跟踪结果（正常发布）
- [x] 融合结果分析
  - **现象**：kalman 点云显示五颜六色，未能正确显示红/蓝色
  - **原因**：相机外参未针对 rosbag 数据标定，导致匹配失败
  - 详见「常见问题 - 融合颜色不正确」

---

## 运行流程

### 方式一：雷达点云处理（rosbag 回放）

只需要 **3 个终端**：

**终端 1 - 点云处理**：
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

**终端 2 - rosbag 回放**：
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play bag/merged_bag_0.db3 --loop
```

**终端 3 - RViz 可视化**：
```bash
source /opt/ros/humble/setup.bash
rviz2
```

### 方式二：视觉检测测试（rosbag 回放）

**单终端启动**：
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/home/nyu/Desktop/T-DT-2024-Radar/bag/merged_bag_0.db3
```

启动后会：
- 自动弹出 OpenCV 窗口显示检测结果（1280x960）
- 发布 `/detect_image` 话题（可在 RViz 或 rqt_image_view 查看）

**查看检测图像**：
```bash
ros2 run rqt_image_view rqt_image_view
# 选择 /detect_image 话题
```

### 方式三：完整系统（雷达 + 视觉）

**终端 1 - 点云处理**：
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

**终端 2 - 视觉检测**：
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/home/nyu/Desktop/T-DT-2024-Radar/bag/merged_bag_0.db3
```

**终端 3 - rosbag 回放**（如果不使用 run_rosbag.launch.py 内置播放）：
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 bag play bag/merged_bag_0.db3 --loop
```

**终端 4 - RViz 可视化**：
```bash
source /opt/ros/humble/setup.bash
rviz2
```

### 方式四：使用实际雷达

**终端 1 - 雷达驱动**：
```bash
sudo ip addr add 192.168.1.5/24 dev enp4s0
cd ~/Desktop/T-DT-2024-Radar
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端 2 - 点云处理**：
```bash
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

**终端 3 - RViz 可视化**：
```bash
source /opt/ros/humble/setup.bash
rviz2
```

---

## RViz 配置

### Fixed Frame 设置
- 推荐使用 `rm_frame`（场地坐标系，地面水平）
- 备选 `livox_frame`（雷达坐标系，可能有倾斜）

### 添加 PointCloud2 显示

| 话题 | 说明 | 推荐颜色 | Size |
|------|------|----------|------|
| `/livox/lidar` | 原始点云 | 白色/灰色 | 0.01 |
| `/livox/map` | 场地地图 | 绿色 | 0.02 |
| `/livox/lidar_dynamic` | 动态点云（机器人） | 红色 | 0.02 |
| `/livox/cluster` | 聚类结果 | 黄色 | 0.1 |
| `/livox/kalman` | 卡尔曼跟踪结果 | RGB8 | 0.15 |

### 添加 Image 显示

| 话题 | 说明 | 注意事项 |
|------|------|----------|
| `/camera_image` | 原始相机图像 | - |
| `/detect_image` | 检测结果图像（带框） | Reliability Policy 设为 Best Effort |
| `/compressed_image` | 压缩图像（来自 rosbag） | - |

### 其他设置
- **Decay Time**：设为 `0.5` 或 `1`（防止闪烁）
- **Reliability Policy**：如果点云/图像不显示，尝试改为 `Best Effort`

---

## 话题说明

### 雷达话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | PointCloud2 | 原始点云数据（frame_id: livox_frame） |
| `/livox/map` | PointCloud2 | 场地地图点云（frame_id: rm_frame） |
| `/livox/lidar_dynamic` | PointCloud2 | 动态点云（frame_id: rm_frame） |
| `/livox/cluster` | PointCloud2 | 聚类中心点（frame_id: rm_frame） |
| `/livox/kalman` | PointCloud2 | 卡尔曼跟踪结果（frame_id: rm_frame，XYZRGB） |
| `/filter_map` | PointCloud2 | 过滤后的源点云（用于调试） |
| `/tf` | TF | 动态坐标变换 |
| `/tf_static` | TF | 静态坐标变换（由 localization 发布） |

### 视觉话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/camera_image` | Image | 原始相机图像 |
| `/detect_image` | Image | 检测结果图像（带检测框） |
| `/compressed_image` | CompressedImage | 压缩图像（来自 rosbag） |
| `/detect_result` | DetectResult | 检测结果（机器人位置） |
| `/resolve_result` | DetectResult | 解算后的 3D 位置 |
| `/camera_point2D` | - | 2D 检测点 |
| `/camera_point3D` | - | 3D 检测点 |

---

## 算法流程

### 雷达处理流程

```
原始点云 (/livox/lidar, frame_id: livox_frame)
    │
    ▼
┌─────────────────┐
│  localization   │ ← 加载场地地图 (config/RM2024.pcd)
│  (GICP 配准)    │ → 发布 TF: rm_frame → livox_frame (静态TF, 10Hz)
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
│  (目标检测)     │ → 发布 /livox/cluster（聚类中心）
└─────────────────┘
    │
    ▼
┌─────────────────┐
│  kalman_filter  │ ← 卡尔曼滤波跟踪
│  (目标跟踪)     │ → 发布 /livox/kalman (XYZRGB)
└─────────────────┘
```

### 视觉处理流程

```
相机图像 (/camera_image 或 /compressed_image)
    │
    ▼
┌─────────────────┐
│  radar_detect   │ ← YOLO v5 检测机器人
│  (目标检测)     │ ← YOLO v5 检测装甲板
│                 │ ← DenseNet121 分类装甲板数字
│                 │ → 发布 /detect_result, /detect_image
└─────────────────┘
    │
    ▼
┌─────────────────┐
│  radar_resolve  │ ← 五点标定外参
│  (位置解算)     │ → 发布 /resolve_result（3D 位置）
└─────────────────┘
    │
    ▼
┌─────────────────┐
│  kalman_filter  │ ← 与雷达检测结果融合
│  (传感器融合)   │ → 发布 /kalman_detect, /radar2sentry
└─────────────────┘
```

### 检测结果说明

- **蓝色框**：蓝方机器人
- **红色框**：红方机器人
- **白色框**：未识别颜色的机器人
- **数字标签**：装甲板数字 (1-5) 和置信度

---

## 预期效果

### 雷达效果
1. `/livox/lidar` 和 `/livox/map` 应该**重叠对齐**（GICP 配准成功）
2. `/livox/lidar_dynamic` 应该只显示**移动的机器人**，静态场地被过滤掉
3. `/livox/cluster` 显示检测到的**机器人簇**（每个机器人一个点）
4. `/livox/kalman` 显示**跟踪的机器人轨迹**（带颜色标识红/蓝队）

### 视觉效果
1. OpenCV 窗口显示高清检测图像（1280x960）
2. 检测框标注机器人位置
3. 装甲板数字和置信度显示

---

## 待完成事项 (TODO)

### 高优先级
- [x] 相机-雷达融合系统启动测试（已完成，但融合效果需外参标定）
- [ ] **相机外参标定（五点标定）** ← 当前最重要
  - 需要在实际比赛场地进行
  - 标定后才能正确融合红/蓝色
  - 标定文件：`config/out_matrix.yaml`
- [ ] 完整比赛模式测试
  - 恢复 localization/dynamic_cloud 的原始过滤条件
  - 真实比赛场地测试

### 中优先级
- [ ] 参数调优
  - 聚类参数（ClusterTolerance, MinClusterSize）
  - 卡尔曼滤波参数
  - GICP 配准参数
  - YOLO 检测阈值
  - 融合匹配距离阈值（当前 `detect_r = 1` 米）
- [ ] 配置永久静态 IP（使用 netplan）
- [ ] 实际相机驱动配置（Hikvision）

### 低优先级
- [ ] 性能优化
- [ ] 文档完善
- [ ] 多相机/雷达支持

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
- 确保从项目根目录启动（`cd ~/Desktop/T-DT-2024-Radar`）
- 检查 `config/RM2024.pcd` 地图文件存在
- 非比赛场地测试时，已放宽过滤条件

### 5. RViz 点云不显示
- 检查 Fixed Frame 是否正确（推荐 `rm_frame`）
- 检查 TF 是否正常发布：`ros2 run tf2_ros tf2_echo rm_frame livox_frame`
- 调整 Decay Time 为 0.5-1 秒
- 尝试将 Reliability Policy 改为 `Best Effort`

### 6. `/livox/lidar` 在 `rm_frame` 下不显示
- 这是 TF 时间戳问题，已通过静态 TF 广播器修复
- 确保使用最新编译的 localization 节点
- 重启点云处理节点后等待几秒让 GICP 完成首次配准

### 7. 场地/地图倾斜
- 这是正常的，地图坐标系按 LiDAR 安装角度定义
- 可以在 RViz 中手动调整视角
- 或添加额外的 TF 校正：
  ```bash
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 -0.35 0 world rm_frame
  ```
  然后在 RViz 中使用 `world` 作为 Fixed Frame

### 8. RViz Image 显示 "No Image"
- 检查话题是否有数据：`ros2 topic hz /detect_image`
- 将 Image 的 Reliability Policy 改为 `Best Effort`
- 或使用 rqt_image_view：`ros2 run rqt_image_view rqt_image_view`

### 9. TensorRT 头文件缺失
```bash
# 如果 /tmp/tensorrt_headers 被清理，重新下载
cd /tmp
git clone --depth 1 --branch v10.0.0 https://github.com/NVIDIA/TensorRT.git tensorrt_headers
git clone --depth 1 https://github.com/onnx/onnx-tensorrt.git onnx_tensorrt
cp onnx_tensorrt/NvOnnxParser.h tensorrt_headers/include/
```

### 10. 融合颜色不正确（五颜六色）

**现象**：`/livox/lidar_kalman` 点云显示五颜六色，而不是红/蓝色。

**原因**：相机-雷达融合依赖以下条件：
1. **相机外参正确标定** - `config/out_matrix.yaml`
2. **位置匹配距离 < 1 米** - kalman_filter 的 `detect_r = 1`

当使用 rosbag 测试时，外参是针对录制时的相机位置标定的。如果：
- 相机位置/角度改变
- 外参未针对当前场景标定

则相机解算的 3D 位置与雷达检测位置相差过大，无法匹配，导致显示随机颜色。

**解决方案**：
1. **五点标定**（推荐）：在实际场地放置 5 个已知坐标的标定点，重新计算外参
2. **增大匹配距离**（临时）：修改 `src/fusion/kalman_filter/include/filter_plus.h` 中的 `detect_r`
   ```cpp
   float detect_r = 3;  // 从 1 改为 3 米（会增加误匹配风险）
   ```

**融合原理说明**：
```
相机检测 → YOLO 识别机器人颜色/编号 → 五点标定解算 3D 位置 → resolve_result
                                                               │
                                                               ▼
雷达检测 → 聚类 → 卡尔曼跟踪 ──────────────────────────────────► 位置匹配（< 1 米）
                                                               │
                                                               ▼
                                                        匹配成功：使用相机颜色
                                                        匹配失败：使用默认颜色
```

---

## 编译命令

```bash
# 完整编译
cd ~/Desktop/T-DT-2024-Radar
source /opt/ros/humble/setup.bash
colcon build

# 编译单个包
colcon build --packages-select localization
colcon build --packages-select dynamic_cloud
colcon build --packages-select cluster
colcon build --packages-select kalman_filter
colcon build --packages-select tdt_vision

# livox_ros_driver2 需要用专用脚本
cd src/livox_ros_driver2
./build.sh humble
```

---

## 下载 rosbag 测试数据

### 方法一：使用 BaiduPCS-Go（推荐）

```bash
# 下载 BaiduPCS-Go
wget https://github.com/qjfoidnh/BaiduPCS-Go/releases/download/v3.9.7/BaiduPCS-Go-v3.9.7-linux-amd64.zip
unzip BaiduPCS-Go-v3.9.7-linux-amd64.zip
cd BaiduPCS-Go-v3.9.7-linux-amd64

# 登录（使用 BDUSS cookie）
./BaiduPCS-Go login -bduss="你的BDUSS值"

# 下载文件
./BaiduPCS-Go d "/我的资源/适应性训练第一把.zip" --saveto ~/Desktop/T-DT-2024-Radar/

# 解压
cd ~/Desktop/T-DT-2024-Radar
7z x 适应性训练第一把.zip
mkdir -p bag && mv merged_bag_0.db3 bag/
```

### 方法二：本地下载后 SCP 传输

```bash
# 在本地 Windows/Mac 下载后，使用 scp 传输
scp -P 端口号 "适应性训练第一把.zip" 用户名@服务器IP:~/Desktop/T-DT-2024-Radar/
```

---

## 五点标定说明

相机外参标定用于将相机图像中的 2D 坐标转换为场地 3D 坐标，是相机-雷达融合的关键。

### 标定文件

- **输出文件**：`config/out_matrix.yaml`
- **内容**：包含 `world_rvec`（旋转向量）和 `world_tvec`（平移向量）

### 标定步骤（概要）

1. 在场地上放置 5 个标定点（已知场地坐标）
2. 让相机能看到这 5 个标定点
3. 运行标定程序，输入 5 个点的场地坐标和图像坐标
4. 程序使用 PnP 算法计算相机外参
5. 将结果保存到 `config/out_matrix.yaml`

### 标定程序位置

标定相关代码位于 `src/tdt_vision/resolve/` 目录。

### 注意事项

- 标定时相机位置必须固定
- 相机移动后需要重新标定
- 使用 rosbag 测试时，外参可能与录制时不同，导致融合失败

---

## 当前系统状态总结

| 模块 | 状态 | 备注 |
|------|------|------|
| 雷达点云处理 | ✅ 正常 | localization、dynamic_cloud、cluster、kalman 均正常 |
| 视觉检测 | ✅ 正常 | YOLO 检测、装甲板分类均正常 |
| 相机-雷达融合 | ⚠️ 需标定 | 系统正常运行，但外参需重新标定 |
| rosbag 测试 | ✅ 正常 | 可正常回放比赛场地数据 |

**下一步**：在实际比赛场地进行五点标定，完成相机外参校准。

---

*最后更新：2026-02-28*
