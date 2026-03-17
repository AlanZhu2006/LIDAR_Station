# 雷达建图指南

使用 FAST-LIO 为雷达站建立场地地图。

---

## 1. 安装依赖

```bash
# 安装 PCL 和 Eigen
sudo apt install -y ros-humble-pcl-ros libpcl-dev libeigen3-dev
```

---

## 2. 编译建图工作空间

```bash
cd ~/Desktop/RadarStation/mapping_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 3. 配置雷达网络

```bash
# 配置主机 IP（与雷达同网段）
sudo ip addr add 192.168.1.5/24 dev enp4s0

# 测试连接
ping 192.168.1.114
```

> **注意**: 确认你的雷达 IP。默认可能是 `192.168.1.114` 或 `192.168.1.182`

---

## 4. 建图流程

### Step 1: 启动雷达驱动

**终端 1**:
```bash
cd ~/Desktop/RadarStation
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Step 2: 启动 FAST-LIO 建图

**终端 2**:
```bash
cd ~/Desktop/RadarStation/mapping_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

### Step 3: 可视化（可选）

**终端 3**:
```bash
rviz2
# 添加 PointCloud2，选择 /cloud_registered 话题
# Fixed Frame 设为 camera_init
```

### Step 4: 走一圈场地

- 拿着雷达在场地上走动
- 确保覆盖所有区域
- 保持平稳移动，避免剧烈晃动

### Step 5: 保存地图

按 `Ctrl+C` 停止 FAST-LIO，点云自动保存到：
```
~/Desktop/RadarStation/mapping_ws/src/FAST_LIO/PCD/scans.pcd
```

---

## 5. 转换地图格式

### 查看点云
```bash
pcl_viewer ~/Desktop/RadarStation/mapping_ws/src/FAST_LIO/PCD/scans.pcd
```

### 复制到项目 config 目录
```bash
cp ~/Desktop/RadarStation/mapping_ws/src/FAST_LIO/PCD/scans.pcd \
   ~/Desktop/RadarStation/config/RM2024.pcd
```

---

## 6. 建图注意事项

| 事项 | 说明 |
|------|------|
| 空场建图 | 场地上**不要有机器人**，只录静态环境 |
| 覆盖完整 | 确保走遍场地所有角落 |
| 平稳移动 | 避免快速旋转，防止 SLAM 丢失 |
| 雷达位置 | 建图时雷达高度和角度要和比赛时一致 |

---

## 7. 文件说明

| 文件 | 位置 | 说明 |
|------|------|------|
| FAST-LIO 配置 | `mapping_ws/src/FAST_LIO/config/mid360.yaml` | 雷达参数配置 |
| 保存的点云 | `mapping_ws/src/FAST_LIO/PCD/scans.pcd` | 建图输出 |
| 项目地图 | `config/RM2024.pcd` | 雷达站使用的地图 |

---

## 8. 常见问题

### Q: FAST-LIO 启动失败
```bash
# 确保 livox 驱动使用 CustomMsg 格式
# 检查 src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
# xfer_format = 1  # CustomMsg 格式
```

### Q: 点云飘移/不稳定
- 检查 IMU 是否正常
- 减慢移动速度
- 确保雷达固定稳固

### Q: 找不到 /livox/lidar 话题
```bash
# 检查雷达是否正常发布
ros2 topic list | grep livox
ros2 topic hz /livox/lidar
```

---

*最后更新: 2026-02-28*
