# Vision, Radar, and LiDAR Calibration / Collaboration Guide

## 1. Scope

This document explains:

1. What must be calibrated in this repo.
2. How to perform the existing five-point camera extrinsic calibration.
3. Whether LiDAR needs calibration in this project.
4. How to run vision and LiDAR collaboratively with the current ROS2 pipeline.

## Current Recommended Path

If you are using the NYUSH radar station project on this machine, the recommended integration path is now:

1. Use NYUSH calibration and NYUSH map transforms as the vision source of truth.
2. Publish NYUSH world-coordinate results directly on `/resolve_result`.
3. Keep TDT LiDAR localization, clustering, and Kalman fusion unchanged.

In that setup, TDT `Resolve` becomes optional legacy compatibility code rather than the main runtime path.

This guide is written for the current codebase in this repository, especially:

- `config/camera_params.yaml`
- `config/out_matrix.yaml`
- `config/RM2024.pcd`
- `src/tdt_vision/launch/calib_rosbag.launch.py`
- `src/tdt_vision/launch/calib_camera.launch.py`
- `src/tdt_vision/launch/run_rosbag.launch.py`
- `src/lidar/dynamic_cloud/launch/lidar.launch.py`

---

## 2. What Needs Calibration

### 2.1 Camera / Vision

The vision side needs two things:

1. Camera intrinsics in `config/camera_params.yaml`
2. Camera extrinsics in `config/out_matrix.yaml`

In this repo, the critical operational step is the camera extrinsic calibration, because it is used to convert image detections into field coordinates through the `Resolve` pipeline.

### 2.2 LiDAR

LiDAR does **not** use a camera-LiDAR joint calibration workflow in this repo.

Instead, LiDAR is aligned to the field map by:

1. Loading the reference map `config/RM2024.pcd`
2. Running GICP in `src/lidar/localization/src/localization.cpp`
3. Publishing the transform from `rm_frame` to `livox_frame`

So for LiDAR, the practical requirement is:

- a correct field map
- a stable LiDAR mounting position
- successful online localization against that map

### 2.3 Vision-LiDAR Fusion

This project is a **decoupled fusion** pipeline:

- Vision provides robot color/ID and image-based position estimate.
- LiDAR provides cluster centers and tracking.
- Fusion matches them by position inside `kalman_filter`.

The default match radius in the current code is:

- `detect_r = 1.0` meter

That value is defined in `src/fusion/kalman_filter/include/filter_plus.h`.

---

## 3. Files and Outputs

### 3.1 Inputs

- `config/camera_params.yaml`: camera intrinsics and distortion
- `config/RM2024.pcd`: field point-cloud map
- `config/RM2024_Points.yaml`: known field geometry used by the parser

### 3.2 Outputs

- `config/out_matrix.yaml`: camera extrinsic result

### 3.3 Key Runtime Topics

- `/camera_image`: live image input
- `/detect_result`: legacy image-space vision output
- `/resolve_result`: vision detections in field/world coordinates
- `/livox/lidar`: raw LiDAR point cloud
- `/livox/lidar_cluster`: LiDAR cluster centers
- `/livox/lidar_kalman`: fused tracked targets with color

---

## 4. Before You Calibrate

Do these first.

### 4.1 Fix the Physical Setup

The camera extrinsic calibration is only valid when all of the following stay unchanged:

- camera position
- camera angle
- camera height
- lens / focus / resolution
- field layout

If any of those change, recalibrate.

### 4.2 Confirm Camera Intrinsics Exist

Check `config/camera_params.yaml`.

The current repo reads:

- `camera_matrix`
- `dist_coeffs`

and also tolerates the fallback key:

- `dist_coeff`

The current file in this repo already uses `dist_coeff`.

### 4.3 Decide Calibration Mode

There are two supported modes:

1. Rosbag calibration
2. Live camera calibration

Use rosbag if you want repeatability. Use live camera if the hardware is already mounted in its final position.

---

## 5. Vision Extrinsic Calibration: Rosbag Workflow

This is the safest path when you want to calibrate carefully and replay the same scene.

### 5.1 Build

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
colcon build --packages-select vision_interface rosbag_player tdt_vision --symlink-install
source install/setup.bash
```

### 5.2 Start Calibration Launch

```bash
ros2 launch tdt_vision calib_rosbag.launch.py rosbag_file:=/absolute/path/to/your_rosbag
```

Notes:

- Pass the rosbag directory or valid rosbag input path used by your local setup.
- The calibration node subscribes to the replayed camera stream.
- `map_server` is launched together with calibration.

### 5.3 Operate the Calibration UI

When the calibration window appears:

1. Press `Enter` to enter calibration mode.
2. Click the five field reference points in order.
3. After each click, use `W`, `A`, `S`, `D` for fine adjustment if needed.
4. Press `N` to save the current point.
5. After the fifth point is saved, the node solves and writes `config/out_matrix.yaml`.

### 5.4 Required Point Order

Use this exact order:

1. `R0/B0` left top
2. `R0/B0` right top
3. own outpost HP bar top point
4. enemy base guide light
5. enemy outpost guide light

This order matches the hardcoded world-point list in `src/tdt_vision/calibrate/src/calibrate.cpp`.

### 5.5 Calibration Success Check

After calibration, verify that `config/out_matrix.yaml` was updated and contains:

- `world_rvec`
- `world_tvec`

Example check:

```bash
sed -n '1,80p' config/out_matrix.yaml
```

### 5.6 Functional Validation

After writing the new extrinsic:

1. Run the vision resolve path.
2. Confirm detections appear in reasonable field positions.
3. Confirm the projected positions are stable across several frames.

Bad signs:

- detections mirrored left/right
- detections shifted by several meters
- colors or IDs rarely matching LiDAR tracks
- fusion cloud points staying in random default colors

---

## 6. Vision Extrinsic Calibration: Live Camera Workflow

Use this when the actual competition camera is already mounted and you want the final extrinsic directly from hardware.

### 6.1 Start Camera Driver

In terminal 1:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

### 6.2 Start Calibration Node

In terminal 2:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision calib_camera.launch.py
```

### 6.3 Perform the Same Five-Point Procedure

The live workflow is identical to rosbag calibration:

1. Press `Enter`
2. Click one calibration point
3. Fine-tune with `WASD`
4. Press `N`
5. Repeat until all five points are stored

### 6.4 Live Calibration Notes

- The camera must already be in its final mounting pose.
- Do not move the tripod, bracket, or lens after calibration.
- The calibration UI uses OpenCV windows, so this must be run in a graphical desktop session.

---

## 7. Optional Chessboard Mode

The calibration node also supports a chessboard path controlled by:

- `usechessboard` in `config/camera_params.yaml`

Current repo default:

```yaml
usechessboard: 0
```

If enabled, the node looks for an `11 x 7` chessboard and solves PnP automatically.

Use this only if you intentionally prepare that target and know the board scale. The existing competition workflow in this repo is the five-point field calibration, not the chessboard path.

---

## 8. Does LiDAR Need Calibration?

### Short Answer

No camera-LiDAR joint calibration is required by this repo.

### What LiDAR Still Needs

LiDAR still depends on:

1. Correct Livox driver setup
2. Correct message format for the task
3. A valid field map
4. Successful GICP alignment
5. A stable LiDAR mount

### 8.1 Driver Mode

For detection and fusion, `xfer_format` must be:

```python
xfer_format = 0
```

For mapping with FAST-LIO, it must be:

```python
xfer_format = 1
```

The repo README points to:

- `src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py`

### 8.2 When LiDAR Setup Must Be Redone

Redo the LiDAR-side setup if:

- the LiDAR mounting pose changes
- the map is rebuilt
- the field geometry changes materially
- GICP fitness becomes poor
- `rm_frame -> livox_frame` alignment is visibly wrong in RViz

### 8.3 Practical Interpretation

For this project, LiDAR "calibration" mostly means:

- map building
- map replacement
- localization verification

not camera-LiDAR extrinsic solving.

---

## 9. Recommended Collaboration Pipeline

The current intended fused flow is:

1. NYUSH vision detects robots and armors in image space.
2. NYUSH map transforms convert detections directly to field coordinates.
3. LiDAR localization aligns the live cloud to `rm_frame`.
4. LiDAR clustering generates target centers.
5. `kalman_filter` matches camera and LiDAR by position.
6. The fused track is published on `/livox/lidar_kalman`.

### 9.1 Data Flow Summary

```text
camera_image
  -> resolve_result

livox/lidar
  -> localization
  -> dynamic_cloud
  -> lidar_cluster

resolve_result + lidar_cluster
  -> kalman_filter
  -> /livox/lidar_kalman
```

---

## 10. Collaborative Use: Rosbag Verification Path

This is the most complete currently-packaged collaboration path in the repo because `run_rosbag.launch.py` already includes:

- detect
- resolve
- rosbag playback
- map server

### 10.1 Start LiDAR Processing

In terminal 1:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

### 10.2 Start Vision Replay + Resolve

In terminal 2:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/absolute/path/to/your_rosbag
```

### 10.3 Visualize

In terminal 3:

```bash
rviz2
```

### 10.4 What to Inspect

Check these topics:

```bash
ros2 topic list | rg "detect_result|resolve_result|lidar_cluster|lidar_kalman"
```

```bash
ros2 topic hz /detect_result
ros2 topic hz /resolve_result
ros2 topic hz /livox/lidar_cluster
ros2 topic hz /livox/lidar_kalman
```

### 10.5 Expected Behavior

- `/resolve_result` positions should land on sensible field locations.
- `/livox/lidar_cluster` should follow moving targets.
- `/livox/lidar_kalman` should inherit red/blue color from camera when matching succeeds.

If matching fails, the track color often falls back to default random coloring from the Kalman track object.

---

## 11. Collaborative Use: Live Camera + Live LiDAR

This is the real deployment path, and the current recommended setup is to let NYUSH publish `/resolve_result` directly.

### 11.1 Bring Up LiDAR

In terminal 1:

```bash
sudo ip addr add 192.168.1.5/24 dev enp4s0
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

In terminal 2:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

### 11.2 Bring Up Camera

In terminal 3:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

### 11.3 Bring Up NYUSH World-Coordinate Vision

In terminal 4:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B
```

### 11.4 Current Behavior

`nyush_integration.launch.py` starts the NYUSH world node directly.

That means:

- `/resolve_result` is the primary vision output used by fusion
- TDT `Resolve` is not required for the recommended NYUSH path
- TDT LiDAR fusion can consume NYUSH positions directly

### 11.5 What You Must Ensure for Live Collaboration

At runtime, verify:

```bash
ros2 topic echo --once /resolve_result
ros2 topic echo --once /livox/lidar_cluster
ros2 topic echo --once /livox/lidar_kalman
```

If `/resolve_result` is missing, the NYUSH world node is not feeding coordinates into fusion.

---

## 12. Practical Validation Checklist

Use this checklist after calibration and before competition:

### 12.1 Vision

- NYUSH calibration arrays for the active field exist and are current
- `/resolve_result` is publishing
- resolved robot positions appear on the correct side of the map

### 12.2 LiDAR

- `/livox/lidar` is publishing
- localization converges
- `/livox/lidar_cluster` is publishing
- tracked targets move consistently in `rm_frame`

### 12.3 Fusion

- `/livox/lidar_kalman` is publishing
- target colors become stable red/blue near real robots
- color does not jump randomly when the target is continuously visible

---

## 13. Troubleshooting

### 13.1 Fusion Fails After Camera Calibration

Possible causes:

- wrong point-click order during five-point calibration
- stale `out_matrix.yaml`
- camera moved after calibration
- field geometry differs from `RM2024_Points.yaml`
- `/resolve_result` missing
- match radius too small for the current error

The current fallback knob is:

- `detect_r` in `src/fusion/kalman_filter/include/filter_plus.h`

Increase it only as a temporary debug measure.

### 13.2 LiDAR Looks Misaligned

Possible causes:

- wrong `config/RM2024.pcd`
- LiDAR mount moved
- poor initial scene coverage
- wrong driver mode
- GICP not converged yet

### 13.3 Calibration Window Does Not Appear

Possible causes:

- no graphical desktop session
- OpenCV GUI backend issue
- no image arriving on `/camera_image` or `compressed_image`

### 13.4 Live Camera Works But Fusion Still Does Not

Most likely in the current repo:

- NYUSH detector is publishing `/detect_result`
- but no `Resolve` node is publishing `/resolve_result`

Check that first before changing thresholds.

---

## 14. Recommended Operating Order

For a new field deployment, use this order:

1. Verify camera intrinsics in `config/camera_params.yaml`
2. Verify field map `config/RM2024.pcd`
3. Fix camera and LiDAR mounts mechanically
4. Calibrate camera extrinsic with the five-point workflow
5. Verify `Resolve` output on the field map
6. Verify LiDAR localization and clustering
7. Verify fused tracks on `/livox/lidar_kalman`
8. Only then tune match thresholds or tracking parameters

This order avoids tuning fusion on top of a bad extrinsic or a bad map alignment.
