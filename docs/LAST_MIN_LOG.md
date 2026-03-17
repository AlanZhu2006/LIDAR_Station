# Last Min Log (2026-02-27)

## Current State
- Repo-side vision pipeline fixes are applied and buildable.
- Core launch path (`tdt_vision` + `map_server` + `foxglove`) starts.
- Major remaining blocker is live camera input (Hikrobot camera + GUI session).

## What Was Completed
1. Code/paperwork alignment completed:
- launch files now take `rosbag_file` parameter (no hardcoded user path)
- map server pathing fixed
- map assets installed correctly
- config key mismatch fallback added (`dist_coeff(s)`, `usechessboard` typo fallback)
- watchdog scripts updated to valid launch files + `ROSBAG_FILE`
- README commands corrected

2. Runtime crash fixes:
- detect node GTK crash fixed by commenting:
  - `cv::namedWindow("detect", cv::WINDOW_NORMAL);`

3. Bag/storage investigation:
- original `merged_bag_0.db3` was malformed (`sqlite3` reported disk image malformed)
- recovery + reindex process succeeded structurally (`metadata.yaml` generated in `bag_recovered`)
- but recovered bag has zero playable messages on all topics

4. New bag recording test:
- `bag_new3` valid sqlite bag, but contains only `/livox/lidar`
- no `/camera_image` or `/compressed_image` captured

5. Camera driver/SDK checks:
- hardware detected: Hikrobot `MV-CA016-10UC` (`2bdf:0001`)
- permissions OK on USB device node
- MVS SDK present at `/opt/MVS`
- MVS library path issue identified and corrected (`/opt/MVS/bin` needed)

## Unresolved Problems (Blocking)
1. No GUI session:
- current shell shows:
  - `DISPLAY=`
  - `XDG_SESSION_TYPE=tty`
- MVS GUI fails with `QXcbConnection: Could not connect to display`

2. No camera ROS topic available:
- topic scan showed only `/match_info` (later bag recording confirmed only lidar stream active)
- without `/camera_image` or `/compressed_image`, detect pipeline cannot produce `/detect_result`

3. Test bag quality issue:
- recovered bag structure exists but message counts are zero, so it is not useful for functional vision validation

## First Steps For Tomorrow
1. Log into graphical desktop session (`Ubuntu` / `Ubuntu on Xorg`), not tty.
2. Verify GUI env:
```bash
echo $XDG_SESSION_TYPE
echo $DISPLAY
```
Expected: `x11`/`wayland` and non-empty `DISPLAY`.
3. Start MVS with proper runtime libs:
```bash
LD_LIBRARY_PATH=/opt/MVS/bin:/opt/MVS/lib/64:/opt/MVS/lib/32:$LD_LIBRARY_PATH /opt/MVS/bin/MVS
```
4. Confirm camera can stream in MVS.
5. Start camera ROS publisher and confirm topic exists:
- `/camera_image` or `/compressed_image`
6. Re-run radar vision launch and validate:
- `/detect_result`, `/resolve_result`, `/map` topic behavior.

## Useful Known-Good Commands
```bash
cd /home/nyu/Desktop/RadarStation
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch tdt_vision run_rosbag.launch.py rosbag_file:=/home/nyu/Desktop/RadarStation/bag_recovered
```

```bash
ros2 topic list | egrep "camera_image|compressed_image|detect_result|resolve_result|map|match_info"
```
