# NYUSH Vision + TDT LiDAR/Fusion Usage Guide

## 1. Purpose

This guide explains how to run the full radar-station stack with:

- NYUSH vision as the primary vision system
- TDT LiDAR localization, clustering, and fusion

The intended runtime flow is:

```text
camera_image
  -> NYUSH world node
  -> /resolve_result

livox/lidar
  -> localization
  -> dynamic_cloud
  -> lidar_cluster
  -> kalman_filter

/resolve_result + /livox/lidar_cluster
  -> /livox/lidar_kalman
```

In this setup:

- NYUSH handles camera detection and map-coordinate conversion
- TDT handles LiDAR alignment, target clustering, and fusion
- TDT `Resolve` is not required for the recommended runtime path

---

## 2. What Must Already Be Ready

Before running the full system, confirm these are already prepared.

### 2.1 NYUSH Side

- The NYUSH project exists at:
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation`
- NYUSH models exist:
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/car.engine`
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/armor.engine`
  - optional fallback files:
    - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/car.onnx`
    - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/models/armor.onnx`
- The correct NYUSH map calibration exists for your field:
  - for test field: `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/array_test_custom.npy`
- The correct test map image exists if using `testmap` mode:
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/images/my_map(m).jpg`

TensorRT note:

- on this machine, TensorRT 10 is installed via the split Python package layout (`tensorrt_dispatch` instead of legacy `tensorrt`)
- `tdt_vision` now handles that compatibility automatically inside `nyush_world_node.py`
- if you use `.engine` weights, you still need a working NVIDIA driver and CUDA runtime at launch time

### 2.2 TDT Side

- TDT workspace builds successfully:
  - `colcon build --packages-select tdt_vision --symlink-install`
  - `colcon build --packages-select localization dynamic_cloud cluster kalman_filter --symlink-install`
- LiDAR field map exists:
  - `config/RM2024.pcd`
- Livox driver is installed and functional
- Hik camera driver is installed and functional

### 2.3 Hardware / Physical Setup

- Camera is fixed in its final pose
- LiDAR is fixed in its final pose
- LiDAR Ethernet connection is correct
- Camera is publishing `/camera_image`
- Livox is publishing `/livox/lidar`

---

## 3. One-Time Build Step

Run this after pulling changes or after editing the repo.

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
colcon build --packages-select tdt_vision localization dynamic_cloud cluster kalman_filter --symlink-install
source install/setup.bash
```

What this does:

- builds the NYUSH world-coordinate ROS node in `tdt_vision`
- builds the TDT LiDAR localization pipeline
- builds the TDT fusion node

Expected result:

- build completes without errors
- `install/setup.bash` is generated or updated

---

## 4. Calibration Before Runtime

You should do calibration before trying to run the full fused stack.

For this NYUSH + TDT setup, calibration has two parts:

1. NYUSH vision-to-map calibration
2. TDT LiDAR map/alignment verification

### 4.1 NYUSH Vision Calibration for Your Field

Use the NYUSH calibration workflow when:

- the camera pose changed
- the field changed
- you are switching between battle map and test map
- `array_test_custom.npy` is missing or stale

For the test-field workflow, the important NYUSH files are:

- map image:
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/images/my_map(m).jpg`
- output transform array:
  - `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/array_test_custom.npy`

Start NYUSH calibration:

```bash
cd /home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation
python3 calibration.py
```

When prompted, use the values that match your setup.

Typical test-field example:

```text
camera mode (test/hik/video): hik
robot state (R/B): B
map mode (battle/testmap) [battle]: testmap
```

What those mean:

- `camera mode`
  - `hik`: live Hikvision industrial camera
  - `video`: USB camera or file source
  - `test`: static test image
- `robot state`
  - your own side color in the NYUSH logic
- `map mode:=testmap`
  - use the custom mini-field map instead of battle assets

Then perform the NYUSH calibration procedure:

1. Adjust the camera so the field view is stable and final.
2. Click the corresponding points on the camera image and the map image.
3. Complete all required point groups for each height layer.
4. Save the computed transform.

According to the NYUSH project README, the standard sequence is:

1. Ground layer point pairs
2. R-height point pairs
3. Ring-height point pairs
4. Save the final transform

Expected result:

- `array_test_custom.npy` is generated or updated

Why this matters:

- the NYUSH world node uses these transform matrices directly
- if the calibration is wrong, `/resolve_result` will be wrong, and LiDAR fusion will not match

### 4.2 Quick NYUSH Calibration Sanity Check

After calibration, run the NYUSH world node alone and verify that world coordinates look reasonable.

Terminal 1:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

Terminal 2:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B
```

Check:

```bash
ros2 topic echo --once /resolve_result
```

What you want:

- detections appear on the correct side of the field
- positions do not jump wildly
- enemy IDs roughly match expected robots

### 4.3 TDT LiDAR Map Verification

TDT LiDAR does not use a camera-LiDAR extrinsic calibration workflow here.

Instead, it depends on:

- `config/RM2024.pcd`
- correct Livox data
- successful online localization against the map

Before full bring-up, make sure the map file exists:

```bash
ls -l ~/Desktop/RadarStation/config/RM2024.pcd
```

Then start LiDAR and LiDAR processing only:

Terminal 1:

```bash
sudo ip addr add 192.168.1.5/24 dev enp4s0
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Terminal 2:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

Then verify:

```bash
ros2 topic echo --once /livox/map
ros2 topic echo --once /livox/lidar_cluster
```

In RViz, confirm:

- `Fixed Frame` is `rm_frame`
- the map looks correct
- LiDAR targets are appearing in plausible field positions

If LiDAR localization is wrong, do not continue to fusion tuning yet.

### 4.4 Calibration Order Recommendation

Use this order whenever you move to a new field:

1. Fix the camera and LiDAR mounting positions mechanically.
2. Calibrate NYUSH against the correct map.
3. Verify NYUSH `/resolve_result`.
4. Verify TDT LiDAR map alignment in `rm_frame`.
5. Only then run the full fused stack.

---

## 5. Recommended Runtime Order

Bring the system up in this order:

1. LiDAR driver
2. LiDAR processing and fusion
3. Camera driver
4. NYUSH world-coordinate node
5. Visualization / topic checks

That order makes failures easier to isolate.

---

## 6. Terminal 1: Start Livox Driver

If your Livox network interface is not already configured, set it first.

```bash
sudo ip addr add 192.168.1.5/24 dev enp4s0
```

Then start the driver:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

What this step does:

- brings up the Mid-360 driver
- publishes raw LiDAR data on `/livox/lidar`

What to verify:

```bash
ros2 topic echo --once /livox/lidar
```

If this prints one message, the LiDAR driver is alive.

Common problems:

- wrong Ethernet interface name
- wrong static IP
- Livox not powered
- driver mode mismatch

---

## 7. Terminal 2: Start TDT LiDAR Processing and Fusion

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

What this step starts:

- `localization`
- `dynamic_cloud`
- `cluster`
- `kalman_filter`

What each submodule does:

- `localization`
  - loads `config/RM2024.pcd`
  - aligns the live LiDAR cloud to the field map
  - publishes the transform from `rm_frame` to `livox_frame`
- `dynamic_cloud`
  - processes raw LiDAR data to focus on dynamic targets
- `cluster`
  - groups dynamic points into target centers
  - publishes `/livox/lidar_cluster`
- `kalman_filter`
  - tracks LiDAR targets over time
  - matches them against vision world coordinates from `/resolve_result`
  - publishes `/livox/lidar_kalman`

What to verify:

```bash
ros2 topic echo --once /livox/lidar_cluster
ros2 topic echo --once /livox/lidar_kalman
```

If the field is empty, `/livox/lidar_cluster` may be sparse or empty. That is not necessarily a bug.

More checks:

```bash
ros2 topic hz /livox/lidar_cluster
ros2 topic hz /livox/lidar_kalman
```

---

## 8. Terminal 3: Start Hik Camera Driver

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

What this step does:

- starts the industrial camera driver
- publishes images on `/camera_image`

What to verify:

```bash
ros2 topic echo --once /camera_image
```

Optional image check:

```bash
ros2 run rqt_image_view rqt_image_view
```

In `rqt_image_view`, select:

- `/camera_image`

Common problems:

- MVS SDK not installed correctly
- machine is in a non-graphical TTY session and GUI tools fail
- camera is detected by USB but not opened by the driver

---

## 9. Terminal 4: Start NYUSH World-Coordinate Vision Node

For the test field setup:

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B
```

What this does:

- runs the NYUSH two-stage detector
- uses NYUSH map calibration matrices
- converts image detections directly into field coordinates
- publishes them on `/resolve_result`

Runtime note:

- if TensorRT is installed as `tensorrt_dispatch`, the launch now aliases it automatically for the NYUSH YOLOv5 code
- if `.engine` deserialization fails at runtime, the node now falls back automatically to sibling `.onnx` files through OpenCV DNN
- keep `car.onnx` and `armor.onnx` available in the NYUSH `models/` directory if TensorRT is not fully usable on the machine

What the launch arguments mean:

- `nyush_path`
  - path to the NYUSH radar station project
- `map_mode:=testmap`
  - uses your custom test field map and transform
- `state:=B`
  - tells the NYUSH logic which side is self side

If your own side is red, use:

```bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=R
```

If you want to override the default NYUSH test map files explicitly:

```bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B \
  test_map_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/images/my_map(m).jpg \
  test_array_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/array_test_custom.npy
```

What to verify:

```bash
ros2 topic echo --once /resolve_result
```

Optional topic-rate check:

```bash
ros2 topic hz /resolve_result
```

Optional debug image check:

```bash
ros2 run rqt_image_view rqt_image_view
```

Then select:

- `/detect_image`

---

## 10. Terminal 5: Visualize in RViz

```bash
source /opt/ros/humble/setup.bash
rviz2
```

Recommended `Fixed Frame`:

- `rm_frame`

Recommended displays:

- `PointCloud2` -> `/livox/lidar_cluster`
- `PointCloud2` -> `/livox/lidar_kalman`
- `PointCloud2` -> `/livox/map`

What to look for:

- `/livox/map` matches the field
- `/livox/lidar_cluster` follows moving robots
- `/livox/lidar_kalman` becomes colorized when vision-LiDAR matching succeeds

If fusion succeeds:

- enemy targets in `/livox/lidar_kalman` should stabilize in red or blue

If fusion fails:

- tracks may exist, but colors may appear unstable or default/random

---

## 11. Full Bring-Up Summary

Use these exact terminal groups.

### Terminal 1

```bash
sudo ip addr add 192.168.1.5/24 dev enp4s0
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### Terminal 2

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 ros2 launch dynamic_cloud lidar.launch.py
```

### Terminal 3

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hik_camera hik_camera_node
```

### Terminal 4

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B
```

### Terminal 5

```bash
source /opt/ros/humble/setup.bash
rviz2
```

---

## 12. Topic Checklist

After all terminals are running, check these topics.

### 11.1 Input Topics

```bash
ros2 topic list | rg "camera_image|livox/lidar"
```

You should have:

- `/camera_image`
- `/livox/lidar`

### 11.2 Vision Output

```bash
ros2 topic list | rg "resolve_result|detect_image"
```

You should have:

- `/resolve_result`
- `/detect_image`

### 11.3 LiDAR Output

```bash
ros2 topic list | rg "lidar_cluster|lidar_kalman|livox/map"
```

You should have:

- `/livox/lidar_cluster`
- `/livox/lidar_kalman`
- `/livox/map`

### 11.4 Frequency Checks

```bash
ros2 topic hz /camera_image
ros2 topic hz /resolve_result
ros2 topic hz /livox/lidar_cluster
ros2 topic hz /livox/lidar_kalman
```

You do not need identical rates, but all of them must be alive.

---

## 13. How Fusion Works in Practice

This is the logic behind the runtime behavior.

### 12.1 Vision Side

NYUSH:

- detects robots and armors
- determines robot identity and color
- maps image points onto the field map
- publishes world-coordinate target positions on `/resolve_result`

### 12.2 LiDAR Side

TDT LiDAR:

- aligns the live cloud to the field map
- extracts dynamic targets
- clusters them into likely robot centers

### 12.3 Fusion Side

TDT `kalman_filter`:

- tracks LiDAR targets
- compares LiDAR target positions with NYUSH positions
- assigns the vision identity/color to LiDAR tracks when positions match

The current match radius is controlled in:

- `src/fusion/kalman_filter/include/filter_plus.h`

Current default:

- `detect_r = 1`

That means LiDAR and vision positions must be within about 1 meter for a match.

---

## 14. Field-Specific Scaling

The NYUSH world node converts map pixels into metric coordinates.

Default launch parameters assume:

- field width = `28.0` meters
- field height = `15.0` meters

If your test field is physically different, pass custom scale values:

```bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B \
  field_width_m:=9.6 \
  field_height_m:=5.4
```

Only change these if your custom map represents a field with different real dimensions.

---

## 15. Common Operating Modes

### 14.1 Test Field Mode

Use this when validating on your own mini field:

```bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=B
```

This mode typically uses:

- `images/my_map(m).jpg`
- `array_test_custom.npy`

### 14.2 Official Battle Map Mode

Use this when you want NYUSH to use its battle-field assets:

```bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=battle \
  state:=B
```

This mode uses the NYUSH battle map and battle calibration files.

---

## 16. Troubleshooting

### 15.1 `/resolve_result` Does Not Exist

Check:

- terminal 4 actually started
- `nyush_path` is correct
- NYUSH model files exist
- `/camera_image` is alive

Useful checks:

```bash
ros2 topic echo --once /camera_image
ros2 node list
```

### 15.2 `/livox/lidar_cluster` Does Not Exist

Check:

- Livox driver is running
- terminal 2 is running
- `/livox/lidar` exists

Useful checks:

```bash
ros2 topic echo --once /livox/lidar
ros2 node list
```

### 15.3 `/livox/lidar_kalman` Exists But Colors Are Wrong

Possible causes:

- NYUSH map calibration is wrong
- field width/height scaling is wrong
- LiDAR localization is misaligned
- vision and LiDAR coordinates differ by more than the match radius

Check:

- `/resolve_result`
- `/livox/lidar_cluster`
- RViz alignment

### 15.4 Camera Image Exists But No Vision Targets

Possible causes:

- NYUSH model files missing
- confidence thresholds too strict
- image quality too poor
- wrong test map or bad NYUSH calibration matrix

### 15.5 LiDAR Map Looks Wrong in RViz

Possible causes:

- wrong `config/RM2024.pcd`
- LiDAR mounting changed
- localization has not converged
- wrong network or driver mode

### 15.6 `ModuleNotFoundError: No module named 'tensorrt'`

Meaning:

- the NYUSH YOLOv5 code tried to load `.engine` weights using the legacy `import tensorrt`
- newer TensorRT 10 installs may only provide `tensorrt_dispatch`

Current repo behavior:

- `src/tdt_vision/detect/scripts/nyush_world_node.py` now aliases `tensorrt_dispatch` to `tensorrt` before importing `detect_function`

What to check if it still fails:

- run `python3.10 -c "import tensorrt_dispatch; print(tensorrt_dispatch.__version__)"`
- confirm you rebuilt after pulling changes:
  - `colcon build --packages-select tdt_vision --symlink-install`
- confirm you sourced the rebuilt workspace:
  - `source install/setup.bash`

If the import issue is fixed but launch still fails on TensorRT:

- the next likely problem is engine deserialization or CUDA/GPU runtime availability rather than missing Python bindings
- current repo behavior is to fall back automatically to `car.onnx` and `armor.onnx` if the corresponding `.engine` files fail to initialize

---

## 17. Shutdown Order

Recommended shutdown:

1. Stop NYUSH world node
2. Stop camera driver
3. Stop LiDAR processing
4. Stop Livox driver

This reduces misleading error spam from downstream nodes losing upstream topics.

---

## 18. Minimum Success Criteria

You can consider the whole stack operational when all of the following are true:

1. `/camera_image` is publishing
2. `/livox/lidar` is publishing
3. `/resolve_result` is publishing
4. `/livox/lidar_cluster` is publishing
5. `/livox/lidar_kalman` is publishing
6. RViz shows LiDAR tracks in the correct field frame
7. LiDAR tracks get stable red/blue identity from NYUSH vision

If all seven are true, the NYUSH Vision + TDT LiDAR/Fusion radar station is running correctly.
