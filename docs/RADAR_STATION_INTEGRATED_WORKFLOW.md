# 雷达站一体化工作流

本文件整合以下资料，并以当前仓库里的实际脚本和 launch 文件为准：

- `docs/CAMERA.txt`
- `docs/LIDAR.txt`
- `docs/NYUSH_VISION_TDT_LIDAR_USAGE_GUIDE.md`
- `scripts/start_fusion.sh`
- `scripts/start_mapping.sh`
- `src/tdt_vision/launch/nyush_integration.launch.py`
- `src/lidar/dynamic_cloud/launch/lidar.launch.py`
- `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`

如果旧文档和当前代码不一致，以当前代码和本文件为准。

---

## 1. 推荐总流程

这套雷达站的推荐使用顺序不是“全开就跑”，而是按下面顺序逐步验通：

1. 先编译主工作空间与建图工作空间
2. 先用 Mid-360 建一张当前场地的 `.pcd` 地图
3. 固定相机与雷达安装位置后，做 NYUSH 相机标定
4. 单独验证相机世界坐标输出 `/resolve_result`
5. 单独验证 LiDAR 定位、动态点、聚类、Kalman
6. 最后再启动完整融合

推荐运行链路如下：

```text
/camera_image
  -> nyush_integration.launch.py
  -> /resolve_result

/livox/lidar
  -> localization
  -> dynamic_cloud
  -> cluster
  -> kalman_filter
  -> /livox/lidar_kalman
  -> /kalman_detect (rm_frame fused positions for debug_map)

/resolve_result + /livox/lidar_cluster
  -> kalman_filter 中的位置匹配
  -> 给雷达轨迹绑定红蓝和编号
```

---

## 2. 先记住的关键规则

### 2.1 融合时必须用 `nyush_integration`

- 融合时必须启动 `ros2 launch tdt_vision nyush_integration.launch.py`
- 不要用 `nyush_detect.launch.py` 代替
- 原因：`nyush_detect` 只输出检测结果；融合需要的是 `/resolve_result` 世界坐标

### 2.2 `map_mode` 只有两个合法值

- 当前 `nyush_world_node.py` 只接受 `battle` 或 `testmap`
- 不要传 `battlemap`

### 2.3 `state` 表示“己方颜色”

- `state:=R` 表示己方是红方
- `state:=B` 表示己方是蓝方
- 这里不是敌方颜色

### 2.4 Livox 两种模式不要混用

- 建图用：`xfer_format:=1`
- 检测/融合用：`xfer_format:=0`

如果这里设错了，后面的节点会直接不兼容。

### 2.5 当前仓库默认网段是固定的

`src/livox_ros_driver2/config/MID360_config.json` 当前写的是：

- 主机 IP：`192.168.1.5`
- 雷达 IP：`192.168.1.3`

所以你接 Mid-360 的那个网卡，必须配置到 `192.168.1.5/24`。

### 2.6 当前脚本默认网卡名不是自动探测

仓库里的脚本默认用：

- `enx00e04c2536b0`

如果你的机器不是这个名字，先自己确认：

```bash
ip -br link
```

然后把下面命令里的网卡名替换掉，或者先：

```bash
export LIVOX_IF=你的网卡名
```

### 2.7 `ceiling_z_max` 现在按倾斜安装处理

- 当前 Mid-360 为约 `50°` 倾角安装
- `dynamic_cloud` 里的 `ceiling_z_max` 过滤仍然发生在 `livox_frame`
- 所以仓库默认值已改为 `100.0`
- 这等效于先关闭传感器坐标系天花板过滤，避免误删有效点

### 2.8 `LD_PRELOAD` 不是可选装饰

`dynamic_cloud`、`FAST-LIO`、`pcl_viewer` 相关流程里保留：

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0
```

这是为了绕过 MVS SDK 带来的 `libusb` 冲突。

---

## 3. 关键文件和产物

### 3.1 主工作空间

```bash
~/Desktop/RadarStation
```

### 3.2 NYUSH 工程

```bash
/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
```

### 3.3 运行前必须存在的文件

- LiDAR 场地图：
  - `config/RM2024.pcd` 或你自己的 `mapping_ws/test.pcd`
- NYUSH 模型：
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/car.engine`
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/armor.engine`
- NYUSH 标定结果：
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/array_test_custom.npy` 或 battle 对应文件

### 3.4 推荐理解

- `config/RM2024.pcd`：仓库默认地图
- `mapping_ws/test.pcd`：你们自己当前场地建出来的地图

如果你在自己测试场上跑，优先使用你刚建好的 `mapping_ws/test.pcd`。

---

## 4. 编译

### 4.1 编译主工作空间

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  tdt_vision \
  hik_camera \
  localization \
  dynamic_cloud \
  cluster \
  kalman_filter \
  debug_map \
  --symlink-install
source install/setup.bash
```

### 4.2 编译建图工作空间

```bash
cd ~/Desktop/RadarStation/mapping_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 5. 第一阶段：先建图

### 5.1 一键建图

```bash
cd ~/Desktop/RadarStation
export LIVOX_IF=${LIVOX_IF:-enx00e04c2536b0}
./scripts/start_mapping.sh
```

当前脚本会做这些事：

- 把 `192.168.1.5/24` 配到 `$LIVOX_IF`
- 用 `xfer_format:=1` 启动 Livox
- 启动 `FAST-LIO`

### 5.2 手动建图时的等价命令

终端 1：

```bash
sudo ip addr add 192.168.1.5/24 dev ${LIVOX_IF:-enx00e04c2536b0}
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py xfer_format:=1 publish_freq:=10.0
```

终端 2：

```bash
cd ~/Desktop/RadarStation/mapping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

### 5.3 建图结束后的标准操作

停止 FAST-LIO 后，把结果复制为当前场地地图：

```bash
cp ~/Desktop/RadarStation/mapping_ws/src/FAST_LIO/PCD/scans.pcd \
   ~/Desktop/RadarStation/mapping_ws/test.pcd
```

### 5.4 建图时必须满足

- 场上不要有机器人
- 雷达安装高度和正式运行一致
- 走完整个场地
- 不要剧烈晃动

---

## 6. 第二阶段：做 NYUSH 相机标定

相机和雷达一旦改动安装位置，就先重新标定，不要直接调融合。

```bash
cd /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
python3 calibration.py
```

常用输入：

- `camera mode`: `hik`
- `state`: `R` 或 `B`
- `map profile`: `battle` 或 `testmap`

测试场常见输出文件：

- 地图图像：
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/images/my_map(m).jpg`
- 标定数组：
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/array_test_custom.npy`

---

## 7. 第三阶段：先单独验证相机世界坐标

### 7.1 启动相机

终端 1：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

### 7.2 启动 NYUSH 世界坐标节点

终端 2：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B
```

如果你是红方，把 `state:=B` 改成 `state:=R`。

### 7.3 需要重点检查

```bash
ros2 topic echo --once /camera_image
ros2 topic echo --once /resolve_result
ros2 topic hz /resolve_result
```

你要看到的是：

- `/camera_image` 正常出图
- `/resolve_result` 正常发布
- 位置在场地上大致合理，不要左右镜像、整块偏移、频繁跳变

### 7.4 如果是缩比测试场

`nyush_integration.launch.py` 默认按 `28.0m x 15.0m` 缩放。  
如果你的 `testmap` 对应的是缩比场地，要显式传：

```bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B \
  field_width_m:=你的场地宽度 \
  field_height_m:=你的场地高度
```

---

## 8. 第四阶段：先单独验证 LiDAR 链路

### 8.1 先把网卡和 Livox 驱动跑起来

终端 1：

```bash
sudo ip addr add 192.168.1.5/24 dev ${LIVOX_IF:-enx00e04c2536b0}
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py xfer_format:=0 publish_freq:=5.0
```

这里先用 `publish_freq:=5.0` 做稳定性验证。  
如果后面确认机器能跟上，再提到 `10.0` 做低延迟。

### 8.2 启动 T-DT LiDAR 处理链

终端 2：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py \
  map_file:=mapping_ws/test.pcd
```

如果你不是用自己建的图，就把 `map_file` 改成 `config/RM2024.pcd`。

### 8.3 这个 launch 实际会启动

- `localization`
- `dynamic_cloud`
- `cluster`
- `kalman_filter`

### 8.4 需要检查的 topic

```bash
ros2 topic echo --once /livox/lidar
ros2 topic echo --once /livox/map
ros2 topic echo --once /livox/lidar_cluster
ros2 topic echo --once /livox/lidar_kalman
```

判定标准：

- 地图和实时点云在同一场地坐标下对齐
- 移动物体能进入 `/livox/lidar_cluster`
- 没有相机时，Kalman 可以有轨迹，但不会稳定带编号和红蓝

---

## 9. 第五阶段：完整融合

### 9.1 手动启动是最稳的

建议第一次验全链路时，不要直接用一键脚本，先手动起。

终端 1，Livox：

```bash
sudo ip addr add 192.168.1.5/24 dev ${LIVOX_IF:-enx00e04c2536b0}
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py xfer_format:=0 publish_freq:=5.0
```

终端 2，LiDAR 处理：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py \
  map_file:=mapping_ws/test.pcd \
  ceiling_z_max:=100.0 \
  voxel_leaf_size:=0.25 \
  accumulate_time:=6
```

终端 3，相机：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

终端 4，NYUSH 世界坐标：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B
```

终端 5，RViz：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d ~/Desktop/RadarStation/src/livox_ros_driver2/config/pointcloud_lidar.rviz
```

终端 6，可选 2D 展示：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run debug_map debug_map
```

### 9.2 一键脚本什么时候用

确认手动流程没问题后，可以再用：

```bash
cd ~/Desktop/RadarStation
source install/setup.bash
./scripts/start_fusion.sh
```

或低延迟：

```bash
./scripts/start_fusion.sh --low-latency
```

### 9.3 当前一键脚本的真实行为

`scripts/start_fusion.sh` 当前命令行参数仍只支持：

- 无参数
- `--low-latency`

但支持用环境变量覆盖关键值，例如：

- `SELF_COLOR`
- `USE_TILT_CORRECTION`
- `KD_TREE_THRESHOLD_SQ`
- `MIN_CLUSTER_SIZE`
- `DEBUG_CAMERA_MATCH`
- `WINDOW_SIZE`
- `MAX_INACTIVE_TIME`
- `CAR_CONF`
- `ARMOR_CONF`
- `PUBLISH_DEBUG_IMAGE`
- `PUBLISH_DEBUG_MAP`

默认情况下它会使用这些值：

- `state:=R`
- `map_file:=mapping_ws/test.pcd`
- `publish_freq:=10.0`
- `voxel_leaf_size:=0.35`
- `accumulate_time:=3`
- 同时启动 RViz 和 `rqt_image_view`

所以这几种情况不要直接无脑用它：

- 你己方不是红方
- 你不想用 `mapping_ws/test.pcd`
- 你的机器在 `10Hz` 下会 queue full

这些情况下，优先使用上面的手动启动命令。

---

## 10. 融合成功的判定

### 10.1 必查 topic

```bash
ros2 topic list | rg "camera_image|resolve_result|livox/lidar|livox/lidar_cluster|livox/lidar_kalman|kalman_detect"
```

至少应存在：

- `/camera_image`
- `/resolve_result`
- `/livox/lidar`
- `/livox/lidar_cluster`
- `/livox/lidar_kalman`
- `/kalman_detect`  # debug_map 现在把它画在对齐后的 NYUSH 2D 地图上

### 10.2 频率检查

```bash
ros2 topic hz /camera_image
ros2 topic hz /resolve_result
ros2 topic hz /livox/lidar_cluster
ros2 topic hz /livox/lidar_kalman
```

### 10.3 RViz 里要看什么

`pointcloud_lidar.rviz` 当前固定坐标系是 `rm_frame`。  
融合成功时你应该看到：

- `/livox/map` 与场地对应
- `/livox/lidar_cluster` 跟着目标移动
- `/livox/lidar_kalman` 的轨迹逐渐稳定
- 相机匹配成功后，轨迹颜色和编号变稳定

### 10.4 机制上怎么判断匹配成功

当前 `kalman_filter` 订阅：

- `/livox/lidar_cluster`
- `/resolve_result`

匹配逻辑在 `src/fusion/kalman_filter/include/filter_plus.h`：

- 先找时间上最接近的雷达历史点
- 再看距离是否小于 `detect_r`
- 当前默认 `detect_r = 1`

也就是相机世界坐标和雷达轨迹大约要在 1 米内，Kalman 才会把颜色和编号绑定上去。

---

## 11. 推荐排错顺序

### 11.1 `/resolve_result` 没有

先查：

```bash
ros2 topic echo --once /camera_image
ros2 node list
```

通常是这些问题：

- 相机没起来
- `nyush_path` 错
- NYUSH 模型不存在
- 传了错误的 `map_mode`

### 11.2 `/livox/lidar_cluster` 没有

先查：

```bash
ros2 topic echo --once /livox/lidar
```

通常是：

- 网卡 IP 没配对
- Mid-360 没通
- `xfer_format` 设错
- `lidar.launch.py` 没真正启动

### 11.3 有轨迹但颜色乱、编号乱

优先怀疑：

- NYUSH 标定错
- `state` 传错
- `field_width_m` / `field_height_m` 不对
- LiDAR 地图没对齐
- 相机和雷达安装位置变了但没重新做前两步

### 11.4 画面卡、queue full、延迟大

先按这个顺序处理：

1. 把 Livox 降到 `publish_freq:=5.0`
2. `lidar.launch.py` 里把 `process_every_n` 提到 `3`
3. 把 `voxel_leaf_size` 提到 `0.3`
4. 把 `accumulate_time` 降到 `5` 或 `3`
5. 保持 `ceiling_z_max:=100.0`，不要再用旧的 `0.3/0.5` 经验值
6. 在 RViz 里先关掉相机图层

### 11.5 FAST-LIO 正常，但融合不正常

这通常不是雷达驱动问题，而是下面三者至少有一个没对齐：

- 当前 `.pcd` 地图不对
- NYUSH 标定不对
- `state` / 场地尺寸参数不对

---

## 12. 最终建议

如果你要把这套系统“按应有方式”长期使用，最稳的习惯是：

1. 每到一个新场地，先建当前场地图
2. 相机和雷达固定后，再做 NYUSH 标定
3. 先单测 `/resolve_result`
4. 再单测 `/livox/lidar_cluster`
5. 最后才开融合
6. 首次联调用手动方式
7. 全链路稳定后，再使用 `start_fusion.sh`

这套顺序能把问题切得很清楚，不会把“标定错”“地图错”“state 错”“驱动没通”全部混在一起。
