# "queue is full" 排查指南

## 1. 可能来源

| 来源 | 说明 |
|-----|------|
| **RViz message filter** | 最常见！RViz 订阅点云时，若 Filter size 太小，会打印 "queue is full"。显示在 RViz 底部 Status 或 Displays 的 Status 列。**已通过增大 Filter size 缓解**。 |
| **Livox SDK** (`liblivox_lidar_sdk_shared.so`) | SDK 内部队列，当点云回调处理太慢时可能打印 |
| **livox_ros_driver2** | Driver 的 `LidarDataQueue` 满时**静默丢弃**，原代码不打印 |

若在 **RViz 窗口**（Status 列或底部状态栏）看到 "queue is full" → 来自 **RViz 的 message filter**，已修改 `pointcloud_lidar.rviz` 将 Filter size 从 5/10 增至 30。

## 2. 数据流

```
Livox 雷达 --UDP--> Livox SDK 内部队列 --> 回调 --> PushLidarData --> Driver 的 LidarDataQueue --> Publish 线程 --> ROS topic
```

- **SDK 队列满**：回调执行太慢，SDK 收包速度 > 回调返回速度
- **Driver 队列满**：Publish 线程消费太慢，push 速度 > 取数据发布速度

## 3. 排查步骤

### 3.1 确认是否已重新编译

修改 `comm.cpp` 的 `CalculatePacketQueueSize` 后必须重新编译：

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2
source install/setup.bash
```

### 3.2 确认实际队列大小

启动 Livox 驱动后，终端应打印：

```
Lidar[0] storage queue size: 64
```

- 若显示 `10` → 未重新编译或修改未生效
- 若显示 `64` → 修改已生效

### 3.3 确认是 Driver 队列满（新增调试）

已在 `lds.cpp` 的 `PushLidarData` 中，当 queue full 时增加**限速打印**（每 50 次打印一次）：

若看到 `[livox_ros_driver2] driver queue full, drops=` → 说明是 **driver 的 LidarDataQueue** 满。

若从未出现该打印但仍有 "the queue is full" → 说明来自 **Livox SDK**。

### 3.4 针对 Driver 队列满的缓解

- 已做：`CalculatePacketQueueSize` 返回 64（原 10）
- 若仍满：可改为 128 或 256（在 `comm.cpp`）
- 或：降低 `publish_freq` 到 5.0（减少每次发布的数据量，但会增加延迟）

### 3.5 针对 SDK 队列满的缓解

SDK 内部队列不可配置，只能：

- **降低雷达数据率**：在 LivoxViewer 中调低扫描模式/分辨率（若支持）
- **提高回调速度**：减少 `PushLidarData` 及下游处理时间（如降低 voxel 等）
- **降低 publish_freq**：让每次发布聚合更多包，减少 publish 次数，可能减轻 CPU 负载（需实测）

## 4. 快速测试

```bash
# 1. 重新编译
cd ~/Desktop/RadarStation && colcon build --packages-select livox_ros_driver2 && source install/setup.bash

# 2. 仅启动 Livox（不启动 lidar 处理），观察是否有 queue full
ros2 launch livox_ros_driver2 msg_MID360_launch.py publish_freq:=5.0

# 3. 若 5Hz 无 queue full，再试 10Hz
ros2 launch livox_ros_driver2 msg_MID360_launch.py publish_freq:=10.0
```

若**仅 Livox 驱动**时就有 "queue is full" → 更可能是 SDK 或网络/CPU 问题。  
若**加上 lidar 处理**后才出现 → 下游（localization 等）是瓶颈。
