# Log Journal

## 2026-02-27 Vision Pipeline Consistency Fixes

### Scope
Normalized the `tdt_vision` bring-up flow so launch scripts, runtime paths, map assets, watchdog scripts, and README instructions are internally consistent and machine-portable.

### Changes Applied
1. Launch files now use runtime rosbag argument instead of hardcoded user path.
- Updated: `src/tdt_vision/launch/run_rosbag.launch.py`
- Updated: `src/tdt_vision/launch/calib_rosbag.launch.py`
- Added launch arg: `rosbag_file`
- `RosbagPlayer` now receives `LaunchConfiguration('rosbag_file')`

2. Map server now resolves map yaml from package share path.
- Updated: `src/tdt_vision/launch/map_server_launch.py`
- Replaced static source-tree path with `get_package_share_directory('tdt_vision')`

3. Map assets are now installed with package and referenced correctly.
- Updated: `src/tdt_vision/CMakeLists.txt`
- Added `maps` to `INSTALL_TO_SHARE`
- Added install rule for `config/RM2024.png` to `share/tdt_vision/config`
- Updated: `src/tdt_vision/maps/map.yaml`
- Image path changed to `../config/RM2024.png`

4. Resolve node typo fix for minimap load.
- Updated: `src/tdt_vision/resolve/src/resolve.cpp`
- Fixed `configa/RM2024.png` -> `config/RM2024.png`

5. Camera parameter key compatibility fixes.
- Updated: `src/tdt_vision/calibrate/src/calibrate.cpp`
- Updated: `src/tdt_vision/radar_utils/radar_utils.cpp`
- Fallback handling added:
  - `dist_coeffs` -> `dist_coeff`
  - `usechessborad` -> `usechessboard`

6. Watchdog scripts aligned with existing launch files and rosbag argument.
- Updated: `watchDog/camera.sh`
- Updated: `watchDog/calib.sh`
- Fixed launch targets:
  - `tdt_vision run_rosbag.launch.py`
  - `tdt_vision calib_rosbag.launch.py`
- Added required env var guard:
  - `ROSBAG_FILE` must be set
- Launch now passes:
  - `rosbag_file:=$ROSBAG_FILE`

7. README command corrections.
- Updated: `README.md`
- Changed invalid usage and added explicit `rosbag_file:=/absolute/path/to/merged_bag_0.db3`
- Corrected calibration command to `ros2 launch ...` form.

### Validation Performed
1. Build check passed:
- `colcon build --packages-select vision_interface rosbag_player tdt_vision --symlink-install`

2. Launch file syntax check passed:
- `python3 -m py_compile src/tdt_vision/launch/run_rosbag.launch.py src/tdt_vision/launch/calib_rosbag.launch.py src/tdt_vision/launch/map_server_launch.py`

3. Runtime smoke test passed for corrected flow behavior:
- `ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/tmp/not_exist.db3`
- Confirmed:
  - `map_server` launches and loads installed map yaml/png path.
  - `resolve` component loads with corrected map path.
  - invalid bag path fails at rosbag player as expected.

### Notes
- Existing unrelated local modifications were preserved.
- TensorRT/CUDA deprecation warnings still appear during build; these are non-blocking for current functionality.

## 2026-02-27 Runtime Follow-up (User Change)

### Change Applied
- User commented out OpenCV GUI initialization in detect node:
  - File: `src/tdt_vision/detect/src/detect.cpp`
  - Line: `cv::namedWindow("detect", cv::WINDOW_NORMAL);`
  - New state: commented out to avoid GTK backend initialization failure in non-GUI/headless launch context.

### Reason
- Launch error observed:
  - `Can't initialize GTK backend in function 'cvInitSystem'`
- The GUI window is not required for core detection output topics.

### Next Required Actions
- Rebuild vision package:
  - `colcon build --packages-select tdt_vision --symlink-install`
- Re-run launch and continue debugging rosbag URI/storage error separately.

## 2026-02-27 Runtime Follow-up (Bag Storage Recovery + Launch Success)

### Symptoms Observed
1. `radar_detect_node` previously failed on startup with:
- `OpenCV ... Can't initialize GTK backend in function 'cvInitSystem'`

2. `rosbag_player_node` failed with:
- `No storage id specified, and no plugin found that could open URI`
- `No storage could be initialized from the inputs.`

### Root Cause Analysis
1. GUI crash root cause:
- `src/tdt_vision/detect/src/detect.cpp` created a HighGUI window (`cv::namedWindow`) in a runtime context where GTK backend init was unavailable.

2. rosbag open failure root cause:
- Input bag was a standalone `.db3` without `metadata.yaml`.
- Additional inspection showed DB corruption:
  - `sqlite3 merged_bag_0.db3 ".tables"` initially failed with `database disk image is malformed`.
- Even after putting DB in a directory, `ros2 bag reindex` failed until DB integrity and filename pattern were corrected.

### Fixes Applied / Executed
1. Disabled detect GUI window initialization:
- `cv::namedWindow("detect", cv::WINDOW_NORMAL);` commented out in:
  - `src/tdt_vision/detect/src/detect.cpp`
- Rebuilt:
  - `colcon build --packages-select tdt_vision --symlink-install`

2. Recovered corrupted bag DB:
- Installed sqlite CLI:
  - `sudo apt install -y sqlite3`
- Performed SQLite recovery:
  - `sqlite3 merged_bag_0.db3 ".recover" | sqlite3 merged_bag_recovered.db3`
- Verified recovered DB tables:
  - `lost_and_found`, `messages`, `metadata`, `schema`, `topics`

3. Rebuilt valid rosbag2 directory + metadata:
- Created directory: `bag_recovered/`
- Copied recovered DB in and renamed to rosbag2 naming pattern:
  - `bag_recovered/bag_recovered_0.db3`
- Reindexed:
  - `ros2 bag reindex bag_recovered`
- Generated:
  - `bag_recovered/metadata.yaml`

4. Relaunch with recovered bag directory:
- `ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/home/nyu/Desktop/T-DT-2024-Radar/bag_recovered`

### Verification Outcome
- `radar_detect_node` loaded successfully.
- `radar_resolve_node` loaded successfully.
- `foxglove_bridge_node` loaded successfully.
- `map_server` configured and activated successfully.
- `rosbag_player_node` storage error resolved when using recovered bag directory with metadata.
- Process completed cleanly in user test.

### Why These Changes Were Necessary
- Commenting out HighGUI initialization prevents non-essential GUI dependency from breaking core detection startup.
- rosbag2 runtime requires valid storage metadata and readable SQLite schema; the original DB was malformed and unusable without recovery + reindex.

## 2026-02-27 Dependency Walkthrough Status (Pre-Reboot Snapshot)

### Completed During Guided Checklist
1. Environment and package baseline confirmed:
- ROS2 Humble, CUDA/TensorRT tools, and required ROS packages are installed.
- `tdt_vision` package rebuild succeeds after detect-node GUI line change.

2. Runtime launch issues triaged:
- GTK/OpenCV startup crash in `radar_detect_node` resolved by commenting:
  - `cv::namedWindow("detect", cv::WINDOW_NORMAL);`
- Launch progressed past detect/resolve/map/foxglove startup.

3. Bag storage failure root-caused and handled:
- Original `merged_bag_0.db3` had no `metadata.yaml`.
- SQLite inspection tool installed (`sqlite3`).
- DB integrity error observed: `database disk image is malformed`.
- SQLite recovery performed:
  - `sqlite3 merged_bag_0.db3 ".recover" | sqlite3 merged_bag_recovered.db3`
- Reindex required rosbag naming pattern:
  - `bag_recovered/bag_recovered_0.db3`
  - `ros2 bag reindex bag_recovered`
- `metadata.yaml` generated successfully.

4. Launch with recovered bag directory:
- `ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/home/nyu/Desktop/T-DT-2024-Radar/bag_recovered`
- Storage plugin error resolved and process reported clean finish.

### New Finding After Bag Recovery
- `ros2 bag info bag_recovered` shows all topic counts are `0`; recovered bag schema exists but data is effectively empty for playback.
- New recording (`bag_new`, `bag_new3`) only captured `/livox/lidar`; no camera image topics captured.

### Camera/Driver Investigation
1. Hardware detection:
- USB device detected: `Hikrobot MV-CA016-10UC` (`2bdf:0001`) via `lsusb`.

2. Device permissions:
- `/dev/bus/usb/001/016` is accessible (`crw-rw-rw-`, group `plugdev`).
- User belongs to `plugdev`.

3. MVS SDK status:
- `/opt/MVS` exists (SDK installed).
- Initial library error fixed by including `/opt/MVS/bin` in `LD_LIBRARY_PATH`.

4. Current blocker:
- MVS GUI still fails with `QXcbConnection: Could not connect to display`.
- Session variables show non-graphical mode:
  - `DISPLAY=`
  - `XDG_SESSION_TYPE=tty`
- Conclusion: machine is in TTY session; GUI-based camera tools cannot run until logging into graphical desktop session.

### Next Planned Action (After Reboot/Login)
1. Boot into graphical login session (`Ubuntu` / `Ubuntu on Xorg`).
2. Verify:
   - `XDG_SESSION_TYPE` is `x11` or `wayland`
   - `DISPLAY` is non-empty (e.g. `:0`)
3. Run MVS with corrected library path:
   - `LD_LIBRARY_PATH=/opt/MVS/bin:/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH /opt/MVS/bin/MVS`
4. Validate live camera stream and publish ROS image topic for radar pipeline (`/camera_image` or `/compressed_image`).

## 2026-02-27 End-of-Day Delta (Post Snapshot)

### Additional Observations
- `ros2 bag info /home/nyu/Desktop/T-DT-2024-Radar/bag_recovered` confirms all tracked topics currently have `Count: 0`.
- New local recording `bag_new3` is valid (`sqlite3`) but only includes `/livox/lidar` (no camera image topic).
- Topic checks during runtime showed missing camera feed path for detect pipeline:
  - `/detect_result` not published.

### Camera Runtime Status
- MVS launch path fixed for libs, but GUI launch still blocked due to tty session:
  - `DISPLAY=`
  - `XDG_SESSION_TYPE=tty`
  - error: `QXcbConnection: Could not connect to display`

### Handoff
- Created concise handoff document:
  - `docs/LAST_MIN_LOG.md`
- Next working session should begin by switching to graphical desktop login, validating MVS stream, then restoring ROS camera topic before final topic-rate verification.
