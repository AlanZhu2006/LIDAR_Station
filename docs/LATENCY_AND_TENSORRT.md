# 卡顿与延迟优化 / TensorRT 使用说明

## 1. Detect 是否在用 TensorRT？

**当前融合流程**：使用 `nyush_world_node.py`（Python），加载 NYUSH 的 `car.engine`、`armor.engine`。

### 如何确认

启动 NYUSH 节点后，在终端输出中查找：

```
Backend: engine=True, onnx=False, pt=False, device=cuda:0
```

- **engine=True**：正在用 TensorRT，推理最快
- **engine=False, onnx=True**：回退到 ONNX Runtime（若用 GPU 尚可，若用 CPU 会卡）
- **dnn=True**：回退到 OpenCV DNN，通常较慢

若看到 `Failed to load TensorRT engine ... Falling back to ONNX/OpenCV DNN`，说明 TensorRT 加载失败，已回退到 ONNX。

### 常见 TensorRT 失败原因

1. **只有 tensorrt_dispatch + tensorrt_lean**：若出现 `Unexpected call to stub deserializeEngine`，说明当前是精简/调度包，**无法反序列化 .engine**。需安装完整 `tensorrt` 包。
2. **TensorRT 版本不匹配**：`.engine` 由特定 TensorRT 版本生成，版本不一致会加载失败
3. **CUDA 不可用**：`nvidia-smi` 正常但 Python 里 `torch.cuda.is_available()` 为 False

### 修复 "stub deserializeEngine" 错误

当前若只有 `tensorrt_dispatch` 和 `tensorrt_lean`，需安装**完整** TensorRT：

```bash
# 安装完整 tensorrt（与当前版本一致）
pip3.10 install tensorrt==10.15.1

# 若用系统包，可尝试
sudo apt install python3-tensorrt  # 或 tensorrt
```

安装后确认：

```bash
python3.10 -c "import tensorrt as trt; e=trt.Runtime(trt.Logger(trt.Logger.WARNING)); print('OK:', trt.__version__)"
```

### 重新生成 .engine（若加载失败）

```bash
cd /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
python3 onnx2engine.py
# 会生成 models/car.engine、models/armor.engine
```

## 2. 降低延迟的可行方案

### 2.1 确保 TensorRT 生效

- 保证 `models/car.engine`、`models/armor.engine` 存在
- 启动时确认输出为 `engine=True`

### 2.2 ONNX 回退时用 GPU

若必须用 ONNX，NYUSH 的 `models/common.py` 已改为优先使用 `CUDAExecutionProvider`，避免纯 CPU 推理导致卡顿。

### 2.3 减轻 RViz 负担

- 用 `rqt_image_view` 单独看相机，不在 RViz 里显示 Image
- 或关闭不需要的 Display（如部分点云、Image 等）

### 2.4 降低 Livox 发布频率（缓解 queue full）

**"the queue is full"** 时，把 `publish_freq` 降到 5 Hz：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py publish_freq:=5.0
```

`start_fusion.sh` 已默认使用 5.0。若机器性能足够且无 queue full，可改为 10.0。

### 2.5 点云密度 / GICP 参数（lidar.launch.py）

- **voxel_leaf_size**：体素降采样大小（米），默认 0.25。增大（如 0.3、0.4）可减少 GICP 计算量，但可能略降精度。
- **accumulate_time**：GICP 累积帧数，默认 6。减少（如 5）可降低延迟，但配准可能略不稳。

```bash
ros2 launch dynamic_cloud lidar.launch.py map_file:=mapping_ws/test.pcd voxel_leaf_size:=0.3 accumulate_time:=5
```

### 2.5 TDT C++ TensorRT 检测器（可选，需改流程）

项目中有 TDT 的 C++ TensorRT 检测器（`detect.cpp`），推理通常比 Python 更快，但：

- 默认未编译：`BUILD_TDT_TENSORRT_DETECTOR=OFF`
- 需要自己的模型路径（`config/detect_params.yaml` 中的 yolo/armor/classify）
- 当前融合流程用的是 NYUSH 的 map 标定，与 TDT detect 的 resolve 流程不同

若要用 TDT C++ 检测器：

```bash
cd ~/Desktop/RadarStation
colcon build --packages-select tdt_vision --cmake-args -DBUILD_TDT_TENSORRT_DETECTOR=ON
```

然后需修改 launch，用 `radar_detect_node` 替代 `nyush_world_node`，并保证 resolve 流程与地图标定一致。

## 3. 快速诊断

```bash
# 1. 检查 TensorRT
python3.10 -c "import tensorrt_dispatch as trt; print('TensorRT OK:', trt.__version__)"

# 2. 检查 CUDA
python3.10 -c "import torch; print('CUDA:', torch.cuda.is_available())"

# 3. 检查 .engine 是否存在
ls -la /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/*.engine

# 4. 话题频率（融合运行时）
ros2 topic hz /camera_image
ros2 topic hz /resolve_result
ros2 topic hz /livox/lidar_cluster
```
