# Dependency Checklist (T-DT-2024-Radar)

Use this checklist to resolve runtime/build dependencies before full bring-up.

## 1. Base Environment
- [x] Ubuntu 22.04
- [x] ROS2 Humble installed
- [x] `colcon` + common ROS build tools installed

Verify:
```bash
lsb_release -a
printenv ROS_DISTRO
which colcon
```

## 2. GPU / CUDA / TensorRT
- [x] NVIDIA driver working
- [x] CUDA toolkit available
- [x] TensorRT runtime + `trtexec` available

Verify:
```bash
nvidia-smi
nvcc --version
ls -l /usr/src/tensorrt/bin/trtexec
```

Notes:
- `tdt_vision` detect node auto-generates TensorRT engines from ONNX if missing.
- First run may take significantly longer while `.engine` files are built.

## 3. ROS Package Dependencies (Apt)
Install these first:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-map-server \
  ros-humble-foxglove-bridge \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-pcl-conversions \
  ros-humble-pcl-msgs \
  ros-humble-tf2-geometry-msgs \
  ros-humble-tf2-bullet \
  ros-humble-rclcpp-components \
  ros-humble-rosbag2-cpp \
  ros-humble-rosbag2-storage
```

Check installed:
```bash
ros2 pkg prefix nav2_map_server
ros2 pkg prefix foxglove_bridge
ros2 pkg prefix cv_bridge
```

- [x] `nav2_map_server` found
- [x] `foxglove_bridge` found
- [x] core ROS deps found

## 4. System Libraries
- [x] OpenCV (>=4.5.x)
- [x] PCL
- [ ] Livox SDK / driver-side deps (if using LiDAR path)

Quick check:
```bash
pkg-config --modversion opencv4
pkg-config --modversion pcl_common
```

## 5. Workspace Build Check
From repo root:
```bash
source /opt/ros/humble/setup.zsh
colcon build --packages-select vision_interface rosbag_player tdt_vision --symlink-install
source install/setup.zsh
```

- [x] `vision_interface` builds
- [x] `rosbag_player` builds
- [x] `tdt_vision` builds

## 6. Runtime Input Checklist
- [x] Valid rosbag path exists (`*.db3`)
- [x] `ROSBAG_FILE` set if using watchdog scripts

Verify:
```bash
ls -l /absolute/path/to/bag_directory_or_bag.db3
export ROSBAG_FILE=/absolute/path/to/bag_directory_or_bag.db3
```

## 7. Vision Launch Smoke Test
```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/absolute/path/to/bag_directory_or_bag.db3
```

Expected:
- `radar_detect_node` loaded
- `radar_resolve_node` loaded
- `map_server` configured + activated
- `rosbag_player_node` loaded (if bag path valid)

- [x] launch starts successfully
- [x] no missing-package error
- [x] no rosbag open error

## 8. Functional Topic Checks
In another terminal:
```bash
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 topic list | egrep "camera_image|detect_result|resolve_result|map"
ros2 topic hz /detect_result
```

- [ ] `/camera_image` present
- [ ] `/detect_result` publishing
- [ ] `/resolve_result` publishing
- [ ] `/map` publishing

## 9. Common Failure -> Fix Mapping
1. `package 'nav2_map_server' not found`
- Fix: install `ros-humble-nav2-map-server`.

2. `package 'foxglove_bridge' not found`
- Fix: install `ros-humble-foxglove-bridge`.

3. `No storage could be initialized from the inputs`
- Fix: invalid/missing `rosbag_file` path; pass a real `.db3` file.

4. Foxglove `Bind Error`
- Fix: default websocket port already occupied; stop other foxglove bridge process or change bridge config/port.

5. OpenCV window/GTK errors (headless environment)
- Fix: run with desktop/X server, or disable GUI calls in nodes that open windows.

## 10. Completion Gate
Mark complete only when all are true:
- [x] Build succeeds
- [x] Launch succeeds with real bag
- [ ] Topics publish as expected
- [ ] No dependency-related errors in launch logs

## Current Status Snapshot (2026-02-27)
- Verified installed: `nav2_map_server`, `foxglove_bridge`, `cv_bridge`, `image_transport`, `rosbag2`, PCL packages.
- Verified toolchain: NVIDIA driver + CUDA + TensorRT `trtexec`.
- Verified workspace build: `vision_interface`, `rosbag_player`, `tdt_vision` all build successfully.
- Verified runtime launch with real recovered bag directory:
  - `/home/nyu/Desktop/T-DT-2024-Radar/bag_recovered`
  - `rosbag_player` storage error resolved.
- Important caveat:
  - recovered bag currently has zero message counts after corruption recovery.
  - newly recorded test bag currently contains only `/livox/lidar` (no camera image topic).
- Remaining blockers to close checklist:
  - Start camera pipeline and confirm `/camera_image` or `/compressed_image` is live.
  - Confirm functional topic publishing (`/detect_result`, `/resolve_result`, `/map`).
  - Confirm `/detect_result` has non-zero publish rate with `ros2 topic hz`.
  - Run from graphical desktop session (`DISPLAY` set), not tty.
- Shell note: `ROS_DISTRO` was empty in unsourced shell. Run `source /opt/ros/humble/setup.zsh` (and `source install/setup.zsh`) before ROS commands.
