<div align="center">

# NYU Robotics — RoboMaster Radar Station

**Multi-sensor fusion radar station for the RoboMaster competition.**  
Combines 3D LiDAR localization, camera-based object detection, and Kalman-filtered target tracking in a unified ROS 2 pipeline.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](./LICENSE)
[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![CUDA](https://img.shields.io/badge/CUDA-11%2B-green)](https://developer.nvidia.com/cuda-toolkit)

</div>

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Repository Structure](#repository-structure)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Workflow: Mapping to Fusion](#workflow-mapping-to-fusion)
- [Configuration Reference](#configuration-reference)
- [Key ROS 2 Topics](#key-ros-2-topics)
- [Verification & Debugging](#verification--debugging)
- [Known Limitations](#known-limitations)
- [Documentation Index](#documentation-index)
- [Changelog](#changelog)
- [Acknowledgements](#acknowledgements)

---

## Overview

This repository contains the full software stack for the **NYU Robotics RoboMaster Radar Station**. The system detects and tracks opponent robots in real time by fusing data from a Livox Mid-360 LiDAR and a Hikvision camera.

**Primary pipeline:**

| Stage | Component |
|---|---|
| 3D Mapping & Localization | FAST-LIO (Livox Mid-360) |
| Camera Detection | NYUSH ROS 2 Vision |
| LiDAR–Camera Coordinate Alignment | `calibrate_testmap_lidar_alignment.py` |
| Sensor Fusion & Tracking | Kalman Filter node |
| Visualization | RViz 2 + 2D debug map |

**Based on:** [T-DT 2024 Radar](https://github.com/TDTsolutions/Radar_2024) and the NYUSH Robotics Radar Station project.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         RADAR STATION                            │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Livox Mid-360 ──► livox_ros_driver2 ──► /livox/lidar           │
│       │                                       │                  │
│       │                               dynamic_cloud              │
│       │                               (localization,             │
│       │                                dynamic pts,              │
│       │                                clustering)               │
│       │                                    │                     │
│       │                               kalman_filter ──► /livox/lidar_kalman
│                                                │        /kalman_detect
│  Hikvision Cam ──► hik_camera_node ──► /camera_image            │
│       │                                       │                  │
│       │                               nyush_world_node           │
│       │                               (detection +              │
│       │                                homography +             │
│       │                                rm_frame align)          │
│       │                                    │                     │
│       │                            /resolve_result               │
│       │                                    │                     │
│       └────────────── fusion ─────────────►│                    │
│                   kalman camera_match()                          │
│                                                                  │
│  debug_map ──► /map_2d  (2D visualization)                       │
└──────────────────────────────────────────────────────────────────┘
```

### Coordinate Frames

| Frame | Description |
|---|---|
| `rm_frame` | Global RoboMaster field frame (origin at field corner) |
| `testmap_metric` | NYUSH camera homography output, in metres |
| `camera_init` | FAST-LIO SLAM origin |

The `testmap_metric → rm_frame` alignment is stored in `config/local/testmap_to_rm_frame.yaml` and applied at runtime by `nyush_world_node.py`.

---

## Hardware Requirements

| Component | Model |
|---|---|
| LiDAR | Livox Mid-360 (IP: `192.168.1.114` or `192.168.1.182`) |
| Camera | Hikvision MV-CA016-10UC (or compatible) |
| Host PC | Ubuntu 22.04, x86-64, NVIDIA GPU |

---

## Software Requirements

| Dependency | Version |
|---|---|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Python | 3.10 |
| CUDA | ≥ 11.x |
| TensorRT | 8.x – 10.x |
| OpenCV | 4.x |
| PCL | 1.12.x |
| Livox SDK2 | latest |
| Hikvision MVS SDK | latest (install to `/opt/MVS/`) |

---

## Repository Structure

```
RadarStation/
├── config/
│   ├── RM2024.pcd                        # Competition field map (backup)
│   └── local/
│       ├── testmap_to_rm_frame.yaml      # LiDAR–camera alignment (runtime)
│       ├── testmap_to_rm_frame_topdown.png
│       └── testmap_to_rm_frame_preview.png
├── docs/                                 # Detailed technical documentation
│   ├── CURRENT_BUG.md
│   ├── RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md
│   ├── RADAR_STATION_INTEGRATED_WORKFLOW.md
│   └── ...
├── mapping_ws/
│   └── test.pcd                          # Current field map (active)
├── model/                                # TensorRT / ONNX model files
├── scripts/
│   ├── start_fusion.sh                   # Main entry point
│   ├── start_mapping.sh                  # FAST-LIO mapping
│   ├── rotate_pcd.py                     # Tilt correction utility
│   ├── check_latency.sh
│   ├── check_radar_setup.sh
│   └── wait_for_image_topics.sh
├── src/
│   ├── hik_camera/                       # Hikvision camera driver
│   ├── lidar/                            # LiDAR utilities
│   ├── livox_ros_driver2/                # Livox official ROS 2 driver
│   ├── livox_driver/
│   ├── fusion/
│   │   └── debug_map/                    # 2D map visualization node
│   ├── tdt_vision/                       # Vision + fusion pipeline
│   │   └── detect/scripts/
│   │       ├── nyush_world_node.py       # Camera → rm_frame
│   │       └── calibrate_testmap_lidar_alignment.py
│   ├── interface/                        # Custom ROS 2 message types
│   └── utils/
├── watchDog/                             # Process watchdog
├── MAPPING_GUIDE.md
├── SETUP_GUIDE.md
└── README.md
```

---

## Installation

### 1. System and ROS 2 Dependencies

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
  build-essential cmake git python3-pip \
  ros-humble-pcl-ros libpcl-dev libeigen3-dev
```

### 2. Python Dependencies

```bash
sudo pip3 install 'numpy<2' IPython onnxruntime-gpu open3d
```

### 3. Livox SDK2

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git ~/Livox-SDK2
cd ~/Livox-SDK2 && mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
```

### 4. Hikvision MVS SDK

Download from the Hikvision website and install to `/opt/MVS/`.

### 5. TensorRT Headers (if missing)

```bash
cd /tmp
git clone --depth 1 --branch v10.0.0 https://github.com/NVIDIA/TensorRT.git tensorrt_headers
git clone --depth 1 https://github.com/onnx/onnx-tensorrt.git onnx_tensorrt
cp onnx_tensorrt/NvOnnxParser.h tensorrt_headers/include/
```

### 6. Build the Workspace

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

To build a single package:

```bash
colcon build --packages-select <package_name> --symlink-install
# e.g.: tdt_vision, debug_map, dynamic_cloud, kalman_filter
```

---

## Quick Start

### Full System (LiDAR + Camera Fusion)

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

`SELF_COLOR` sets your robot's team color (`R` = Red, `B` = Blue).

### Common Launch Variants

```bash
# Enable camera–LiDAR match debug logging
DEBUG_CAMERA_MATCH=true SELF_COLOR=R ./scripts/start_fusion.sh

# Allow stationary targets (useful for calibration)
PUBLISH_STATIONARY_TARGETS=true SELF_COLOR=R ./scripts/start_fusion.sh

# Enable tilt correction (only if raw /livox/lidar is visibly tilted)
USE_TILT_CORRECTION=true SELF_COLOR=R ./scripts/start_fusion.sh

# Temporarily widen camera-match radius
CAMERA_DETECT_RADIUS=1.5 SELF_COLOR=R ./scripts/start_fusion.sh
```

### Camera-Only Validation

```bash
cd ~/Desktop/RadarStation
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=R \
  field_width_m:=6.79 \
  field_height_m:=3.82 \
  publish_debug_map:=true \
  apply_testmap_rm_alignment:=true \
  alignment_config_path:=config/local/testmap_to_rm_frame.yaml
```

### LiDAR-Only Validation

```bash
cd ~/Desktop/RadarStation
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 \
ros2 launch dynamic_cloud lidar.launch.py \
  map_file:=mapping_ws/test.pcd \
  ceiling_z_max:=100.0
```

### ROS Bag Replay

```bash
# Terminal 1 – LiDAR processing
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 \
ros2 launch dynamic_cloud lidar.launch.py \
  map_file:=config/RM2024.pcd ceiling_z_max:=100.0

# Terminal 2 – Bag playback
ros2 bag play bag/merged_bag_0.db3 --loop --clock
```

---

## Workflow: Mapping to Fusion

### Step 1 — Build the Field Map

```bash
cd ~/Desktop/RadarStation
./scripts/start_mapping.sh
```

Output: `mapping_ws/test.pcd`

> See [MAPPING_GUIDE.md](./MAPPING_GUIDE.md) for full details. Walk the entire field slowly with the LiDAR; avoid rapid rotations. Map while the field is empty.

### Step 2 — Camera Homography Calibration

Run this in the external NYUSH repository:

```bash
cd /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
python3 calibration.py
```

This produces the camera-to-`my_map.jpg` homography used at runtime.

### Step 3 — Align `testmap` to `rm_frame`

Re-run this step whenever `my_map(m).jpg`, `test.pcd`, or the control-point pairs change:

```bash
cd ~/Desktop/RadarStation
python3 src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py \
  --orientation-mode auto
```

`--orientation-mode auto` tests both normal and `flip_x` orientations and selects the better fit. The result is written to `config/local/testmap_to_rm_frame.yaml`.

**Current alignment quality (local):**

| Metric | Value |
|---|---|
| Selected orientation | `flip_x` |
| RMSE | 0.0361 m |
| Max error | 0.0478 m |
| Inliers | 4 / 4 |

### Step 4 — Run the Full System

```bash
SELF_COLOR=R ./scripts/start_fusion.sh
```

### Step 5 — Verification

```bash
# Check that key topics are active and non-zero
ros2 topic echo --once /resolve_result
ros2 topic echo --once /livox/lidar_kalman
ros2 topic list | grep -E "resolve_result|nyush_map_image|lidar_cluster|lidar_kalman|map_2d"
```

In RViz 2, set **Fixed Frame** to `rm_frame` and verify:

- `/livox/lidar_cluster` shows dynamic targets
- `/nyush_map_image` updates continuously
- `/map_2d` background aligns with point positions

---

## Configuration Reference

### Runtime Parameters (`start_fusion.sh`)

| Parameter | Default | Description |
|---|---|---|
| `SELF_COLOR` | — | Team color: `R` (Red) or `B` (Blue) |
| `USE_TILT_CORRECTION` | `false` | Enable LiDAR tilt compensation |
| `FIELD_WIDTH_M` | `6.79` | `my_map(m).jpg` width in metres |
| `FIELD_HEIGHT_M` | `3.82` | `my_map(m).jpg` height in metres |
| `APPLY_TESTMAP_RM_ALIGNMENT` | `true` | Apply `testmap → rm_frame` transform |
| `WINDOW_SIZE` | `1` | Camera tracking window (low-latency) |
| `MAX_PROCESSING_FPS` | `12.0` | Camera-side FPS cap |
| `CAMERA_DETECT_RADIUS` | `1.0` | LiDAR–camera match radius (m) |
| `PUBLISH_STATIONARY_TARGETS` | `false` | Include stationary targets in Kalman output |
| `DEBUG_CAMERA_MATCH` | `false` | Log per-frame match distances |

### Key Files

| File | Purpose |
|---|---|
| `mapping_ws/test.pcd` | Active LiDAR field map |
| `config/local/testmap_to_rm_frame.yaml` | LiDAR–camera alignment config |
| `scripts/start_fusion.sh` | Main system launcher |
| `src/tdt_vision/detect/scripts/nyush_world_node.py` | Camera → `rm_frame` node |
| `src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py` | Alignment solver |
| `src/fusion/debug_map/debug_map.cpp` | 2D debug map node |

---

## Key ROS 2 Topics

### Inputs

| Topic | Source |
|---|---|
| `/livox/lidar` | Livox ROS 2 driver |
| `/camera_image` | Hikvision camera node |

### Outputs

| Topic | Description |
|---|---|
| `/detect_image` | Annotated camera frame |
| `/nyush_map_image` | Raw `testmap`-space detections |
| `/resolve_result` | Detections projected to `rm_frame` |
| `/livox/lidar_dynamic` | Dynamic point cloud |
| `/livox/lidar_cluster` | Clustered dynamic targets |
| `/livox/lidar_kalman` | Kalman-filtered tracks (XYZRGB) |
| `/kalman_detect` | Fusion output for visualization |
| `/map_2d` | 2D debug overhead map |

---

## Verification & Debugging

### Quick Sanity Check

```bash
# List active nodes and key topics
ros2 node list | grep -E "nyush|hik|kalman|lidar|debug"
ros2 topic list | grep -E "camera_image|detect_image|resolve_result|lidar_cluster|lidar_kalman|map_2d"
```

### System Health Scripts

```bash
./scripts/check_radar_setup.sh     # Full environment check
./scripts/check_latency.sh         # Show pipeline latency
./scripts/wait_for_image_topics.sh # Block until image topics are live
```

### Troubleshooting

**`/resolve_result` is all zeros**

1. Verify `/detect_image` shows bounding boxes.
2. Verify `/nyush_map_image` shows detection points.
3. Check that `SELF_COLOR` matches your team color.
4. Confirm the camera calibration matches the actual camera resolution.
5. If needed, re-run `calibration.py`.

**`/livox/lidar_kalman` remains grey despite camera detections**

LiDAR tracks exist but `/resolve_result` is not landing near them. Run with relaxed parameters to diagnose:

```bash
DEBUG_CAMERA_MATCH=true CAMERA_DETECT_RADIUS=1.5 PUBLISH_STATIONARY_TARGETS=true \
  SELF_COLOR=R ./scripts/start_fusion.sh
```

If this enables color, the issue is likely visual coordinate stability or match radius.

**`/nyush_map_image` detections flicker**

Check: `window_size`, `max_inactive_time`, `armor_conf`. Verify armor plates are stable in the camera image.

**RViz point cloud not visible**

- Set **Fixed Frame** to `rm_frame`.
- Set PointCloud2 QoS Reliability to `Best Effort`.
- Confirm `/livox/lidar` is publishing: `ros2 topic hz /livox/lidar`.

---

## Known Limitations

| Issue | Status |
|---|---|
| Tilt-mount workflow not fully closed | The `rotate_pcd.py` utility exists, but the main launch flow defaults to `test.pcd`. Manual map/cloud version consistency check required when using `USE_TILT_CORRECTION=true`. |
| Tilt sign convention inconsistency | Historical references to both `+50°` and `−50°` remain; needs consolidation. |
| Temporal alignment is not strict pairwise sync | Fusion uses "latest camera result + LiDAR history window + radius/time gate", not hard-synchronized pairs. See `docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md`. |
| Some auxiliary outputs hard-coded to official RM field geometry | Core fusion uses `rm_frame`; certain debug helpers still reference fixed field dimensions. |

---

## Documentation Index

| Document | Purpose |
|---|---|
| [`docs/RADAR_STATION_INTEGRATED_WORKFLOW.md`](docs/RADAR_STATION_INTEGRATED_WORKFLOW.md) | End-to-end system workflow |
| [`docs/CURRENT_BUG.md`](docs/CURRENT_BUG.md) | Active known issues and fix status |
| [`docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md`](docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md) | Temporal alignment analysis |
| [`docs/TESTMAP_LIDAR_ALIGNMENT.md`](docs/TESTMAP_LIDAR_ALIGNMENT.md) | `testmap → rm_frame` alignment deep-dive |
| [`docs/VISION_LIDAR_CALIBRATION_AND_COLLAB_GUIDE.md`](docs/VISION_LIDAR_CALIBRATION_AND_COLLAB_GUIDE.md) | Vision–LiDAR calibration guide |
| [`MAPPING_GUIDE.md`](./MAPPING_GUIDE.md) | FAST-LIO mapping procedure |
| [`SETUP_GUIDE.md`](./SETUP_GUIDE.md) | Quick-start setup checklist |

---

## Changelog

### 2026-03-18

- Audited the full `testmap` pipeline from camera calibration to runtime fusion.
- Identified `testmap_metric → rm_frame` x-axis mirror as the root cause of one-sided alignment errors.
- Added `--orientation-mode auto|normal|flip_x` to `calibrate_testmap_lidar_alignment.py`; solver now stores `runtime_transform.matrix_3x3`.
- `nyush_world_node.py` now validates alignment quality on load and rejects bad YAML files.
- `debug_map.cpp` uses the same runtime transform field and suppresses display on invalid alignment.
- Regenerated `config/local/testmap_to_rm_frame.yaml` (RMSE = 0.0361 m, 4/4 inliers, `flip_x`).
- Added `docs/CURRENT_BUG.md` and `docs/RADAR_LIDAR_TIME_ALIGNMENT_AUDIT.md`.

### 2026-03-17

- Unified `start_fusion.sh` main runtime chain.
- Set `mapping_ws/test.pcd` as the default active map.
- Enabled NYUSH `/detect_image`, `/resolve_result`, `/nyush_map_image` pipeline.
- Added `DEBUG_CAMERA_MATCH`, `CAMERA_DETECT_RADIUS`, `PUBLISH_STATIONARY_TARGETS` launch parameters.

---

## Acknowledgements

- [T-DT 2024 Radar](https://github.com/TDTsolutions/Radar_2024) — base radar station codebase
- [NYUSH Robotics Radar Station](https://github.com/NYUSH-Robotics/NYUSH_Robotics_RM_RadarStation) — vision pipeline and calibration tooling
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) and [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO) — LiDAR-inertial odometry
- ROS 2 community
