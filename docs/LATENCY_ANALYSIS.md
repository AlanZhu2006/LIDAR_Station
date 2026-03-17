# 延迟排查与原因分析

## 1. 数据流与延迟来源

```
Camera (30 FPS) ──► camera_image ──► NYUSH detect ──► SlidingWindowFilter(3) ──► resolve_result ──┐
                                                                                                    ├──► Kalman ──► livox/lidar_kalman
Livox (5 Hz) ──► /livox/lidar ──► Localization(累积6帧) ──► GICP ──► TF ──► Dynamic(累积3帧) ──► cluster ──┘
```

## 2. 主要延迟来源（按影响排序）

### 2.1 雷达路径：Localization 累积帧数（最大瓶颈）

| 参数 | 当前值 | 延迟 | 说明 |
|------|--------|------|------|
| accumulate_time | 5 | **~1.0 秒** | 5 Hz × 5 帧 = 1.0 秒才做一次 GICP |
| publish_freq | 5 Hz | - | Livox 发布频率 |

**原因**：GICP 需要累积多帧点云以提高配准稳定性，但 6 帧在 5 Hz 下就是 1.2 秒的固有延迟。

**建议**：
- 当前 start_fusion 已用 `accumulate_time:=5`（约 1.0 秒）
- 可进一步降到 4（约 0.8 秒），但 GICP 可能略不稳
- 若可接受 queue full，提高 `publish_freq` 到 10 Hz，则 5 帧 = 0.5 秒

### 2.2 相机路径：图像分辨率

| 项目 | 当前值 | 影响 |
|------|--------|------|
| 相机输出 | 4096×3000 | 约 12M 像素，resize + 推理耗时大 |
| YOLO 输入 | 640×640 (car) / 192×192 (armor) | 模型内部会 resize |

**原因**：`hik_camera_node.cpp` 将原图 resize 到 4096×3000 再发布，整幅大图参与后续处理，增加传输和推理负担。

**建议**：若检测只需 ROI 或较小分辨率，可在相机端先缩小再发布（需改 hik_camera）。

### 2.3 视觉路径：SlidingWindowFilter

| 参数 | 当前值 | 延迟 |
|------|--------|------|
| window_size | 2（已调） | 需 2 帧才输出 |
| 相机 30 FPS | - | 2 帧 ≈ **66 ms** |

**原因**：为平滑检测结果，需累积多帧才发布。start_fusion 已用 `window_size:=2` 降低延迟。

### 2.4 雷达路径：Dynamic Cloud

| 项目 | 说明 |
|------|------|
| accumulate_time | 3 帧 |
| GetDynamicCloud | KD-tree 最近邻，点越多越慢 |
| TF lookup | 使用 `tf2::TimePointZero` 取最新 TF |

**原因**：点云累积 + KD-tree 搜索，点云密度高时耗时明显。

### 2.5 GICP 计算

- 单次 align 约 50~200 ms（与点数、体素大小相关）
- `voxel_leaf_size` 越大，点数越少，GICP 越快

## 3. 快速诊断命令

融合运行时，另开终端执行：

```bash
# 1. 话题频率
ros2 topic hz /camera_image      # 期望 ~30
ros2 topic hz /livox/lidar      # 期望 5
ros2 topic hz /resolve_result   # 期望 ~30
ros2 topic hz /livox/lidar_cluster
ros2 topic hz /livox/lidar_kalman

# 2. 端到端延迟（时间戳差）
ros2 topic echo /camera_image --field header.stamp -n 1
ros2 topic echo /resolve_result --field header.stamp -n 1
# 比较两者时间戳差值

# 3. 查看 Lidar 终端日志
# - "Dynamic callback time: X ms"：dynamic_cloud 单次回调耗时
# - "Running GICP"：GICP 执行
# - "GICP fitness"：配准质量
```

## 4. 推荐调参（降低延迟）

| 目标 | 参数调整 |
|------|----------|
| 降低雷达固有延迟 | `accumulate_time:=4` 或 `5` |
| 提高雷达数据率 | `publish_freq:=10.0`（需机器能跟上） |
| 降低 GICP 计算 | `voxel_leaf_size:=0.3` |
| 降低视觉平滑延迟 | `window_size:=2`（在 nyush_integration 中传参） |

**一键低延迟启动**：

```bash
./scripts/start_fusion.sh --low-latency
```

（使用 publish_freq=10、voxel=0.3、accumulate=4。若出现 queue full，用默认 `./scripts/start_fusion.sh`）

## 5. 延迟构成估算（当前配置）

| 阶段 | 估算延迟 |
|------|----------|
| 相机采集 | ~33 ms |
| NYUSH 检测 (TensorRT) | ~20–50 ms |
| SlidingWindowFilter | ~100 ms |
| **视觉小计** | **~150–200 ms** |
| Livox 发布 | 5 Hz = 200 ms 间隔 |
| Localization 累积 | **~1000 ms**（5 帧 @ 5 Hz） |
| GICP | ~100 ms |
| Dynamic 累积 | ~600 ms（3 帧 @ 5 Hz） |
| **雷达小计** | **~1700 ms** |
| **整体** | 以雷达路径为主，约 **1.7 秒** |

结论：**Localization 的累积帧数是最大延迟来源**。Lidar 终端会打印 `GICP ... time=Xms`，可观察 GICP 耗时。
