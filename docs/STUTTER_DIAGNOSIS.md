# "一顿一顿" 根因排查指南

## 可能原因（按优先级）

### 1. dynamic_cloud 的 GetDynamicCloud 阻塞（最可能）

`dynamic_cloud` 对**每个点**做 kd-tree 最近邻搜索，点云 1 万点时约 50–150ms/帧。

- **现象**：与 /livox/lidar 同频（10Hz）卡顿
- **验证**：运行最小测试（见下），若最小测试流畅而完整流程卡顿，则基本可确认

### 2. GICP 更新间隔

TF 由 GICP 更新，`accumulate_time=3` 时约每 0.3 秒更新一次。

- **现象**：约每 0.3 秒一次明显跳变
- **验证**：看 Lidar 终端 GICP 日志时间间隔

### 3. 多节点共享线程池

localization、dynamic_cloud、cluster、kalman 在同一容器，共享 MultiThreadedExecutor。任一回调阻塞都会拖慢其他回调。

### 4. RViz 渲染

点云量大时，RViz 渲染可能掉帧。

- **验证**：降低点云密度（如 voxel 0.5）或关闭部分显示，看是否改善

---

## 排查步骤

### 步骤 1：最小测试（仅 Livox + Localization + RViz，无 dynamic_cloud）

```bash
./scripts/test_minimal_stutter.sh
```

或手动：
```bash
# 终端 1: Livox
# 终端 2: ros2 launch dynamic_cloud lidar_minimal.launch.py map_file:=mapping_ws/test.pcd
# 终端 3: rviz2 -d .../pointcloud_lidar.rviz
```

**结论**：
- 若最小测试仍卡顿 → 问题在 livox / localization / RViz
- 若最小测试流畅 → 问题在 dynamic_cloud 或 cluster

### 步骤 2：监控发布频率

```bash
# /livox/lidar 应为 ~10 Hz
ros2 topic hz /livox/lidar

# 观察是否有明显波动或掉帧
```

### 步骤 3：查看 Lidar 终端

关注：
- `GICP time=XXms`：单次 GICP 耗时，>100ms 说明较重
- `GICP accumulating`：GICP 触发间隔是否稳定

### 步骤 4：启用性能日志（可选）

在 `scripts/start_fusion.sh` 中给 lidar 加上环境变量（若已实现）：
```bash
ROS_LOG_LEVEL=debug  # 或使用脚本中的 profile 参数
```

---

## 若确认是 dynamic_cloud 导致

1. **对点云先降采样再 GetDynamicCloud**：在 kd-tree 前用 VoxelGrid 降到约 2000–5000 点
2. **降低 dynamic_cloud 处理频率**：每 N 帧处理一次，其余帧直接转发或跳过
3. **将 dynamic_cloud 移到独立进程**：用单独节点和 executor，避免与 localization 争用线程
