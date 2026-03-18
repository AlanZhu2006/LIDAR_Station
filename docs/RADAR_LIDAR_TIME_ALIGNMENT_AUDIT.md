# Radar/LiDAR Time Alignment Audit

Date: 2026-03-18

Scope:
- Active fusion pipeline in this repo (`/home/nyu/Desktop/RadarStation`)
- Related NYUSH desktop repo (`/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation`)
- Replay / alternate driver paths that can invalidate timing tests

## Short Answer

The current fusion path is **not** doing strict pairwise camera/LiDAR synchronization.

What it really does today is:
- keep only the **latest** `/resolve_result`
- keep a short LiDAR track history inside each KF
- accept a match if the camera point is within `detect_r` and within a **1.0 s** time gate of some LiDAR history sample or the KF's smoothed current output

So your statement is only partially right:
- yes, there is a spatial check
- yes, there is a time check
- no, it is **not** an exact "same-time camera report vs same-time LiDAR cluster report" check

Both sides are already temporally smeared before fusion:
- LiDAR side: default dynamic-cloud accumulation merges multiple LiDAR frames
- Camera side: NYUSH sliding window averages multiple detections

## Findings

### 1. LiDAR dynamic cloud merges multiple frames but stamps the result as one frame

Evidence:
- `scripts/start_fusion.sh:137-139` sets default `ACCUM=3`
- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp:264-293` accumulates multiple dynamic clouds, merges them, then publishes the merged cloud with only the **current** `msg->header.stamp`
- `src/lidar/cluster/src/cluster.cpp:133-137` preserves that single stamp into `/livox/lidar_cluster`

Impact:
- The cluster position can contain points from several LiDAR frames while pretending to be a single-time measurement.
- For moving robots, this directly weakens the time-alignment check. The oldest points in a 3-frame window at 10 Hz are about 200 ms older than the newest ones.
- Fusion then compares camera against a temporally blurred LiDAR position, not a true single-time LiDAR position.

Severity: High

### 2. Fusion uses only the latest camera detection, not a synchronized camera/LiDAR pair

Evidence:
- `src/fusion/kalman_filter/src/kalman_filter.cpp:114-119` stores only `latest_detect_msg_`
- `src/fusion/kalman_filter/src/kalman_filter.cpp:221-244` pulls that one cached message when a LiDAR cluster callback arrives
- There is no `message_filters`, no timestamp-indexed queue, and no nearest-message pairing

Impact:
- A good camera sample can be overwritten by a newer one before the matching LiDAR cluster arrives.
- Under load or jitter, the fusion step is matching LiDAR against "whatever camera result arrived most recently", not against the camera result closest to the LiDAR message time.
- This makes time alignment dependent on scheduler timing, callback order, and process load.

Severity: High

### 3. The time gate is very loose: 1.0 second, and it can match against a smoothed KF output

Evidence:
- `src/fusion/kalman_filter/include/filter_plus.h:256-313`
- `TIME_THRESHOLD = 1.0`
- if the latest KF output is closer than the nearest history sample, the code may use `get_output_point()` instead of a true same-time history point
- `get_output_point()` itself is a weighted average of the last 3 LiDAR observations in `src/fusion/kalman_filter/include/filter_plus.h:185-200`

Impact:
- A match can still pass with a very large temporal offset by robotics standards.
- The time check is not a "same frame" or even "same cycle" check. It is closer to "within 1 second and spatially plausible".
- Because the fallback point is itself a short temporal average, the "time check" is partly checking against a smoothed estimate, not a raw LiDAR observation at that timestamp.

Severity: High

### 4. Camera timestamps come from ROS publish time, not sensor capture time

Evidence:
- `src/hik_camera/src/hik_camera_node.cpp:155-216` grabs a frame, converts pixel format, optionally resizes, applies LUT, then stamps with `this->now()`
- `src/hik_camera/src/hik_camera_node.cpp:210-214`

Impact:
- The camera timestamp includes host-side delay from SDK retrieval, conversion, resize, image processing, and scheduler jitter.
- `/resolve_result` inherits that delayed camera stamp in `src/tdt_vision/detect/scripts/nyush_world_node.py:985-1009`.
- This means the fusion time check is using "publish-time-ish" camera stamps, not true exposure time.

Severity: High

### 5. NYUSH output is also temporally smoothed, but still stamped as one image timestamp

Evidence:
- `src/tdt_vision/detect/scripts/nyush_world_node.py:36-62` averages a sliding window of detections
- `scripts/start_fusion.sh:166` defaults `WINDOW_SIZE=2`
- `scripts/start_fusion.sh:186` defaults `MAX_PROCESSING_FPS=12.0`
- `src/tdt_vision/detect/scripts/nyush_world_node.py:985-1009` publishes the averaged result with the current image's `msg.header.stamp`

Impact:
- `/resolve_result` is not a pure single-frame camera position. It is already a short temporal average.
- At the default 12 FPS cap and `window_size=2`, the reported camera position is typically about one processed frame behind the newest detection.
- Combined with LiDAR accumulation, the two sides can both be "time checked" while neither side represents one exact instant.

Severity: High

### 6. NYUSH temporal logic uses wall clock instead of message time

Evidence:
- `src/tdt_vision/detect/scripts/nyush_world_node.py:43-62` uses `time.time()` for inactivity and window retention
- `src/tdt_vision/detect/scripts/nyush_world_node.py:291-309` uses `time.monotonic()` for processing rate limiting

Impact:
- If you replay bags, pause, slow down, or use `use_sim_time`, the camera-side temporal filter no longer follows message timestamps.
- Even without bags, this makes the smoothing/inactivity behavior sensitive to host load rather than only sensor timing.

Severity: Medium

### 7. Dynamic cloud uses the latest TF, not the point cloud's TF at message time

Evidence:
- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp:195-199` calls `lookupTransform("rm_frame", msg->header.frame_id, rclcpp::Time(0), ...)`

Impact:
- `Time(0)` asks for the latest available transform, not the transform at the point cloud timestamp.
- If localization TF is still converging, jumps, or is delayed, a LiDAR cloud can be transformed with a pose from a different time.
- This is especially risky during startup and any re-alignment period.

Severity: Medium

### 8. LiDAR timestamps degrade to host receive time when hardware sync is absent

Evidence:
- `src/livox_ros_driver2/src/comm/pub_handler.cpp:265-275`
- Only GPTP/PTP or GPS timestamps use the sensor-provided timestamp
- Otherwise the driver falls back to `std::chrono::high_resolution_clock::now().time_since_epoch().count()`

Impact:
- If the LiDAR is not actually hardware time-synced, its timestamps are only host receive-time approximations.
- That can drift relative to camera publish time and makes cross-sensor timing dependent on NIC/host latency and clock behavior.

Severity: Medium

### 9. Per-point Livox timing exists but the fusion/localization chain throws it away

Evidence:
- `src/livox_ros_driver2/src/lddc.cpp:319-329` populates a per-point `timestamp` field in the PointCloud2 payload
- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp:181-182` converts incoming data to `pcl::PointCloud<pcl::PointXYZ>`
- `src/fusion/kalman_filter/src/kalman_filter.cpp:160-168` also converts to plain XYZ / XY points

Impact:
- Downstream nodes use only the cloud header stamp, not the per-point times.
- Any intra-scan motion timing is lost before localization, clustering, and fusion.
- This matters most for moving targets and for any future attempt at tighter time alignment.

Severity: Medium

## Trap Paths That Can Invalidate Timing Tests

### 10. The repo contains a second Livox driver that overwrites timestamps with `now()`

Evidence:
- `src/livox_driver/livox_ros2_driver/livox_ros2_driver/lddc.cpp:178-219`
- It first computes a sensor timestamp, then overwrites `cloud.header.stamp = rclcpp::Clock().now()` before publish

Impact:
- If someone launches the wrong Livox package from this repo, LiDAR sensor time is destroyed.
- This is easy to miss because both Livox driver trees exist in the same workspace.

Severity: Medium

### 11. The custom rosbag player rewrites both image and LiDAR timestamps

Evidence:
- `src/utils/rosbag_player/rosbag_player.cpp:52-74`
- It assigns `ros_time = rclcpp::Clock().now()` to both `/livox/lidar` and `camera_image`
- It also sleeps to a fixed ~100 ms loop in `src/utils/rosbag_player/rosbag_player.cpp:87-92`

Impact:
- Any timing test done with this player does not preserve the original sensor timing relationship.
- It is not valid for checking camera/LiDAR time alignment.

Severity: High for testing validity

## Relevant NYUSH Desktop Repo Issues

The desktop NYUSH repo shows the same timing model that the ROS bridge still inherits conceptually:

- `main.py:221-280` and `latency_test.py:230-282` use `time.time()` for sliding-window retention instead of per-frame timestamps.
- `main.py:403-443` has no sensor timestamp propagation at all; it only keeps a global latest image.
- `main.py:822-823` reads `camera_image.copy()` without `frame_lock`, so legacy `main.py` can even read an image while the capture thread is replacing it.
- `latency_test.py:412-456` measures latency from host receipt time (`perf_counter`) rather than sensor capture time.
- `latency_test.py:839-980` confirms the legacy code path is built around "latest frame in memory" timing, not message-pair synchronization.

These files are not the active ROS fusion path, but they matter because they are the origin of the camera-side timing assumptions and are still used for debugging / reference.

## Practical Consequence For The Current Fusion Check

Today, a colored / matched fusion output means:
- the latest cached camera result had a nonzero position
- that camera stamp was within 1.0 s of some LiDAR KF history sample or its smoothed output
- the camera world point was within `camera_detect_radius` of that LiDAR point

It does **not** guarantee:
- same-cycle camera/LiDAR pairing
- same-frame timing
- unsmeared LiDAR position
- unsmeared camera position
- hardware capture-time synchronization

## Recommended Next Fixes

1. Replace the single `latest_detect_msg_` cache with a short timestamp-ordered deque and match LiDAR callbacks to the nearest camera result by stamp.
2. Disable dynamic-cloud accumulation for the time-alignment check, or publish an explicit time interval instead of a single latest stamp.
3. Tighten the fusion time gate far below 1.0 s and log the actual camera-LiDAR delta used for each match.
4. Move NYUSH smoothing to message-time semantics instead of `time.time()` / `time.monotonic()`.
5. If the Hik SDK exposes sensor/frame timestamps, publish those instead of `this->now()`.
6. During testing, do not use `src/utils/rosbag_player/rosbag_player.cpp` for timing validation.
7. Make sure the active Livox path is `src/livox_ros_driver2`, not `src/livox_driver/livox_ros2_driver`.

## Opinion On Your Runtime Symptom

Symptom you described:
- camera side looks correct on location
- LiDAR side sees the dynamic cluster correctly
- but the fused object is not consistently dyed blue/red

--------------------------------------------------------------------------------------------------------------------
My opinion from the time side:

- Yes, the time-side problems in this report are very capable of causing exactly that symptom.
- In the live default path, the most plausible timing causes are Findings 1, 2, 3, 4, and 5.
- The less relevant ones for this exact live symptom are the rosbag player and the alternate Livox driver, unless you are using them during testing.

Why I think the time side is a serious candidate:

- A KF becomes colored only after `camera_match()` pushes entries into `detect_history` in `src/fusion/kalman_filter/include/filter_plus.h:296-310`.
- A KF stays classified as long as that history is non-empty in `src/fusion/kalman_filter/include/filter_plus.h:242-243`.
- The visible point becomes gray again only if you are looking at a different/new KF, or if the old KF got deleted and recreated. KF deletion happens after `last_time > 1.5` in `src/fusion/kalman_filter/src/kalman_filter.cpp:248-252`.

That means "not consistently dyed" is usually not a one-off visualization problem. It usually means one of these is happening:

- the correct camera result is not being paired with the correct LiDAR track often enough
- the LiDAR centroid is temporally smeared / shifted enough that the match sometimes falls outside `detect_r`
- the LiDAR track gets broken and recreated, so the new KF starts gray again

From the current code, the strongest explanation chain is:

1. LiDAR cluster position is already temporally smeared by `ACCUM=3`, but stamped as if it were one instant.
2. Camera world position is also smoothed by the NYUSH sliding window, but stamped as one image instant.
3. Fusion does not pair by nearest camera message; it only uses the latest cached `/resolve_result`.
4. The match is then decided against a radius gate and a very loose 1.0 s time gate.

That combination can absolutely produce this pattern:
- each subsystem looks "correct enough" by itself
- but the camera sample that reaches fusion is not the best-time partner for the current LiDAR KF
- some callbacks match and color the KF
- some callbacks miss
- if cluster motion/jump is large enough, KFs can split/recreate and you see gray again

So if I focus only on the time side, my judgment is:

- Yes, time-side behavior is a plausible root cause.
- More specifically, I think the most likely time-related root cause is not raw clock drift alone, but the combination of:
  - latest-only camera caching
  - multi-frame LiDAR accumulation with single latest stamp
  - multi-frame camera smoothing with single latest stamp
  - publish-time camera stamping instead of capture-time stamping

If this were a pure spatial-calibration failure, I would expect the object to be persistently uncolored or almost never colored. The fact that it is not **consistently** dyed makes me lean more toward intermittent pairing failure, and the current time-handling design is enough to create that.

--------------------------------------------------------------------------------------------------------------------

## Implementation Applied On 2026-03-18

I fixed the active RadarStation runtime path in code.

### 1. Fusion now pairs LiDAR with the nearest queued camera result instead of "latest only"

Changed:
- `src/fusion/kalman_filter/include/kalman_filter.h`
- `src/fusion/kalman_filter/src/kalman_filter.cpp:81-165`
- `src/fusion/kalman_filter/src/kalman_filter.cpp:184-307`

What changed:
- Replaced the single `latest_detect_msg_` cache with a short detect deque.
- On each LiDAR cluster callback, fusion now searches that deque and picks the camera result whose timestamp is nearest to the LiDAR message timestamp.
- Added runtime parameters:
  - `camera_time_match_threshold` default `0.25`
  - `detect_queue_max_age` default `1.0`
  - `detect_queue_max_size` default `20`
  - `use_smoothed_camera_match_point` default `false`

Why this helps:
- It removes callback-order dependence as the main pairing rule.
- A newer camera result no longer automatically overwrites the correct-time camera result for the current LiDAR cluster.

### 2. The time check is tighter by default, and smoothed KF output is no longer used by default

Changed:
- `src/fusion/kalman_filter/include/filter_plus.h:257-319`
- `src/fusion/kalman_filter/src/kalman_filter.cpp:199-243`

What changed:
- Removed the hardcoded `1.0 s` assumption from `camera_match()`.
- `camera_match()` now uses the KF's configured `camera_time_threshold`.
- Matching against `get_output_point()` is now behind `use_smoothed_camera_match_point`, which defaults to `false`.

Why this helps:
- The default path now prefers true timestamped LiDAR history points for the time check.
- This is much closer to a real "same-time enough" alignment test than the previous smoothed fallback behavior.

### 3. Camera image stamps are now taken at frame acquisition, not after processing

Changed:
- `src/hik_camera/src/hik_camera_node.cpp:155-215`

What changed:
- The camera node now grabs `capture_stamp = this->now()` immediately after `MV_CC_GetImageBuffer()` succeeds.
- That stamp is reused when publishing `camera_image`.

Why this helps:
- It removes the extra host-side skew from pixel conversion, resize, LUT, and publish work.
- This is still host-clock time, not true exposure time, but it is a better approximation than publish-time stamping.

### 4. NYUSH smoothing now follows message time, and default live fusion uses single-frame output

Changed:
- `src/tdt_vision/detect/scripts/nyush_world_node.py:36-61`
- `src/tdt_vision/detect/scripts/nyush_world_node.py:78`
- `src/tdt_vision/detect/scripts/nyush_world_node.py:157`
- `src/tdt_vision/detect/scripts/nyush_world_node.py:291-300`
- `src/tdt_vision/detect/scripts/nyush_world_node.py:695-711`
- `src/tdt_vision/detect/scripts/nyush_world_node.py:906-1004`
- `src/tdt_vision/launch/nyush_integration.launch.py:90-94`

What changed:
- `SlidingWindowFilter` now stores and ages tracks using `msg.header.stamp`, not `time.time()`.
- Added a guarded fallback to node clock only if an input image has a zero stamp.
- Default `window_size` is now `1` in the node and launch file.

Why this helps:
- The active `/resolve_result` path now has message-time semantics instead of wall-clock semantics.
- With `window_size=1`, the published camera world position corresponds to one processed image rather than a multi-frame temporal average.

### 5. LiDAR dynamic cloud defaults now avoid multi-frame blur, and TF lookup uses cloud stamp first

Changed:
- `src/lidar/dynamic_cloud/include/dynamic_cloud.h:29`
- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp:19-20`
- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp:195-211`
- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp:295-310`
- `src/lidar/dynamic_cloud/launch/lidar.launch.py`
- `src/lidar/dynamic_cloud/launch/lidar_tilt.launch.py`

What changed:
- Default `accumulate_time` is now `1`.
- Default `publish_accumulated_dynamic_cloud` is now `false`.
- TF lookup now tries `msg->header.stamp` first, then falls back to latest TF with a warning if exact-time TF is unavailable.
- Added timestamps to the published `/livox/lidar_other` output as well.

Why this helps:
- The cluster input used for fusion is single-frame by default again.
- The LiDAR side is less temporally smeared before it reaches the Kalman tracker.

### 6. Live startup defaults were updated so the safer timing path is actually used

Changed:
- `scripts/start_fusion.sh`

What changed:
- Default mode now runs with:
  - `ACCUM=1`
  - `PUBLISH_ACCUMULATED_DYNAMIC_CLOUD=false`
  - `WINDOW_SIZE=1`
  - `CAMERA_TIME_MATCH_THRESHOLD=0.25`
  - `USE_SMOOTHED_CAMERA_MATCH_POINT=false`
- The launch command now passes the new fusion timing parameters through to the LiDAR/fusion launch files.

Why this helps:
- The code fix would not be enough if the runtime script kept launching the old blurred defaults.

## What I Expect This To Change In Your Symptom

For the "sometimes colored, sometimes gray" behavior, this patch should help in the exact places that were most suspicious:

- the camera result used by fusion should now be the nearest-time result, not merely the most recently arrived one
- the LiDAR cluster used by fusion is now single-frame by default instead of 3-frame accumulated
- the camera world result is now single-frame by default instead of sliding-window averaged
- the time gate is now tight by default and no longer quietly relies on a smoothed KF point

So if the intermittent coloring was caused mainly by time pairing failure, this patch should reduce it substantially.

## Verification

I verified the patch by:

- `python3 -m py_compile src/tdt_vision/detect/scripts/nyush_world_node.py src/lidar/dynamic_cloud/launch/lidar.launch.py src/lidar/dynamic_cloud/launch/lidar_tilt.launch.py src/tdt_vision/launch/nyush_integration.launch.py`
- `source /opt/ros/humble/setup.bash && colcon build --packages-select kalman_filter dynamic_cloud hik_camera tdt_vision --symlink-install`

Build result:
- all 4 packages built successfully
- only existing ament header-install warnings remained in `dynamic_cloud` and `kalman_filter`

## Still Not Fixed In This Patch

These timing risks still exist, but they are outside the core live fusion bug I targeted here:

- Livox can still fall back to host receive time if hardware sync is absent.
- Per-point Livox timestamps are still not propagated into fusion logic.
- `src/utils/rosbag_player/rosbag_player.cpp` still rewrites timestamps and should not be used for timing validation.
- The standalone desktop NYUSH scripts under `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation` were not modified in this patch because they are not the active ROS fusion path used by `nyush_world_node.py`.

--------------------------------------------------------------------------------------------------------------------

## Delay / Frequency Risk Audit On 2026-03-18

Question:
- camera-side detect visualization and LiDAR-side visualization both look delayed
- what risks in the code could cause that
- what should be refined so delay is less likely to damage fusion/localization

## Short Answer

Yes, there are still several plausible delay risks on both sides.

The most important distinction is:

- some of the visible delay is **designed throttling / subsampling**
- some of it is **real processing latency / backlog risk**
- some of it can be **viewer lag** rather than pipeline lag

Also, the camera-vs-LiDAR **frequency mismatch itself is not automatically wrong anymore** because fusion now pairs by timestamp. The real danger is:

- stale processed outputs
- queue buildup
- TF lag
- heavy callbacks sharing one executor

## Camera-Side Delay Risks

### 1. `detect_image` is inherently slower than `camera_image`

Evidence:
- `src/hik_camera/src/hik_camera_node.cpp:25` requests `target_fps=30.0`
- `scripts/start_fusion.sh:191` defaults `MAX_PROCESSING_FPS=12.0`
- `src/tdt_vision/detect/scripts/nyush_world_node.py:302-320` rate-limits processing
- `src/tdt_vision/detect/scripts/nyush_world_node.py:906-1036` publishes `detect_image` only after the whole NYUSH pipeline finishes

Impact:
- `camera_image` can be fresh while `detect_image` is older, because `detect_image` is a post-inference debug image, not the raw camera stream.
- With 30 FPS input and 12 FPS max processing, many camera frames are intentionally dropped before detection.

Refinement:
- Treat `/camera_image` as the transport-freshness indicator.
- Treat `/detect_image` as a processed debug product, not as proof of camera capture freshness.
- If GPU headroom exists, raise `MAX_PROCESSING_FPS`.
- If not, reduce `PROCESS_WIDTH` / `PROCESS_HEIGHT` so the processed stream stays fresh.

### 2. Default debug-image publish rate is lower than the actual detector rate

Evidence:
- `scripts/start_fusion.sh:142` default mode uses `DEBUG_EVERY_N=2`
- `scripts/start_fusion.sh:129` low-latency mode uses `DEBUG_EVERY_N=3`
- `src/tdt_vision/detect/scripts/nyush_world_node.py:910-913` only publishes debug images every N processed frames

Impact:
- In default mode, if NYUSH is processing at 12 FPS, `detect_image` may only update at about 6 FPS.
- In low-latency mode, it can be about 4 FPS.
- This can look like delay even if `/resolve_result` itself is fresher.

Refinement:
- If visualization smoothness matters, use `DEBUG_EVERY_N=1`.
- If fusion freshness matters more than debug smoothness, keep it throttled and judge latency from `/resolve_result` and message ages instead.

### 3. Debug-map publishing adds extra CPU load inside the critical image callback

Evidence:
- `scripts/start_fusion.sh:182` defaults `PUBLISH_DEBUG_MAP=true`
- `src/tdt_vision/detect/scripts/nyush_world_node.py:759-803` builds and draws the debug map
- `src/tdt_vision/detect/scripts/nyush_world_node.py:1036` publishes it from the same callback path

Impact:
- Every processed image can also trigger map copy/draw/publish work.
- That extra work competes directly with inference and debug image generation.

Refinement:
- For live fusion runs, prefer `PUBLISH_DEBUG_MAP=false` unless that map view is actively needed.

### 4. Camera-side image copies / conversion still happen multiple times

Evidence:
- `src/hik_camera/src/hik_camera_node.cpp:161-215` does pixel conversion, optional resize, optional LUT, and ROS image publish
- `src/tdt_vision/detect/scripts/nyush_world_node.py:914-917` converts ROS image to OpenCV and resizes again for processing
- `src/tdt_vision/detect/scripts/nyush_world_node.py:1030-1034` converts the debug frame back to ROS Image

Impact:
- Even when timestamps are correct, the debug path still pays for multiple memory copies and image-format conversions.

Refinement:
- If detection quality allows it, publish the camera closer to NYUSH processing resolution so one resize stage disappears.
- If debug image lag is the main complaint, decouple debug rendering/publishing from the critical detection callback in a future patch.

### 5. Viewer lag can be mistaken for pipeline lag

Evidence:
- `scripts/start_fusion.sh:234` launches `rqt_image_view`
- `scripts/view_camera_opencv.py` exists specifically to view `/camera_image` or `/detect_image` without RViz overhead

Impact:
- RViz / `rqt_image_view` can lag on their own, especially when point clouds and images are shown together.

Refinement:
- Compare `/camera_image` or `/detect_image` with `python3 scripts/view_camera_opencv.py /camera_image` or `/detect_image` before blaming the pipeline itself.

## LiDAR-Side Delay Risks

### 6. The Livox driver has an internal raw-packet queue with no visible backlog guard

Evidence:
- `src/livox_ros_driver2/src/comm/pub_handler.h:119` declares `std::deque<RawPacket> raw_packet_queue_`
- `src/livox_ros_driver2/src/comm/pub_handler.cpp:147-151` pushes every raw packet into that deque
- `src/livox_ros_driver2/src/comm/pub_handler.cpp:229-253` processes them on a worker thread

Impact:
- If packet processing falls behind, the queue can grow and the published PointCloud2 messages can become older without an explicit warning from this code path.

Refinement:
- Add queue-length logging / warning / cap in the Livox driver so backlog becomes visible immediately.
- Use `scripts/check_latency.sh` and `scripts/check_latency.py` to watch `/livox/lidar` message age during live runs.

### 7. Localization can process stale scans during startup

Evidence:
- `src/lidar/localization/src/localization.cpp:47` subscribes with queue depth `10`
- `src/lidar/localization/src/localization.cpp:75-82` accumulates frames before aligning
- `src/lidar/localization/src/localization.cpp:222` sets `accumulate_time = 10`

Impact:
- At 10 Hz Livox publish rate, startup alignment may need about 1 second before the first full GICP attempt.
- Because the subscription queue is deeper than 1, slow startup callbacks can work through older scans.
- That can make TF and any TF-dependent visualization look delayed or jumpy during startup.

Refinement:
- If startup delay is a problem, reduce localization `accumulate_time` carefully after checking GICP stability.
- If stale-TF behavior is observed, consider reducing the localization subscription queue depth for live mode.

### 8. `gridMap` in localization is not cleared inside the callback before rebuilding

Evidence:
- `src/lidar/localization/src/localization.cpp:84-107` keeps inserting / updating `gridMap`
- `src/lidar/localization/src/localization.cpp:228` stores `gridMap` as a member
- I do not see a `gridMap.clear()` at the start of each callback

Impact:
- Before alignment finishes, startup preprocessing can keep carrying older farthest-point data across callbacks.
- That is both a latency risk and a stale-data risk during localization bootstrap.

Refinement:
- Clear `gridMap` each callback before rebuilding the current accumulated representation.
- This is one of the more concrete code-level LiDAR delay risks still worth fixing.

### 9. Dynamic cloud still does heavy KD-tree work on the live path

Evidence:
- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp:58-87` does nearest-neighbor KD-tree search over the input cloud
- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp:265-266` runs `GetDynamicCloud(...)` whenever `process_every_n_` says to process
- `src/lidar/dynamic_cloud/include/dynamic_cloud.h:33` defaults `process_every_n_ = 1`

Impact:
- Dense clouds or expensive map comparisons can slow `/livox/lidar_dynamic` enough to make it older than raw `/livox/lidar`.
- This delay then propagates into cluster and Kalman outputs.

Refinement:
- If `/livox/lidar` is fresh but `/livox/lidar_dynamic` is old, reduce point count earlier or increase `voxel_leaf_size`.
- Use `process_every_n=1` when smoothness matters; use higher values only if you knowingly accept lower effective update rate.

### 10. Exact-time TF lookup can block the LiDAR callback when TF is late

Evidence:
- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp:195-203` waits up to `0.05 s` for exact-time TF and then another `0.05 s` for latest-TF fallback

Impact:
- A missing or late TF can stall a dynamic-cloud callback by up to about 100 ms.
- That alone is enough to make a 10 Hz visualization look stale or uneven.

Refinement:
- If these fallback warnings appear often, the real problem is TF freshness from localization, not just dynamic_cloud itself.
- Monitor how often exact-time TF fails before tuning anything else.

### 11. Cluster extraction is another full-frame processing stage

Evidence:
- `src/lidar/cluster/src/cluster.cpp:56-142` runs Euclidean clustering and shape filtering every callback

Impact:
- `/livox/lidar_cluster` can look delayed even when `/livox/lidar_dynamic` is acceptable, because clustering is another full pass over the dynamic cloud.

Refinement:
- If cluster age is the problem, reduce dynamic-cloud point count first.
- Only then tune cluster parameters such as pre-cluster voxel size.

### 12. Multiple heavy callbacks still share the same component container

Evidence:
- `src/lidar/dynamic_cloud/launch/lidar.launch.py` puts `localization`, `dynamic_cloud`, `cluster`, and `kalman_filter` into the same `component_container_mt`
- `docs/STUTTER_DIAGNOSIS.md` already flags this as a contention risk

Impact:
- One heavy callback can delay others even if each node is individually "correct".
- This mainly affects LiDAR-side smoothness and TF freshness.

Refinement:
- If live profiling still shows callback contention, split localization and dynamic/cluster/fusion into separate processes.

## Frequency Mismatch: What Is Fine And What Is Dangerous

Current live defaults are approximately:

- camera publish requested at 30 FPS
- NYUSH processing capped at 12 FPS by default
- Livox publish at 10 Hz
- LiDAR dynamic processing usually 10 Hz in default mode, lower if `process_every_n` is increased

This mismatch is acceptable **if**:

- the processed outputs stay fresh
- queues do not build up
- fusion still finds a recent cross-sensor pair within threshold

It becomes dangerous when:

- `/detect_image` or `/resolve_result` age climbs because the detector cannot keep up
- `/livox/lidar_dynamic` or `/livox/lidar_cluster` age climbs relative to raw `/livox/lidar`
- localization TF is delayed enough that dynamic_cloud frequently falls back to latest TF

## Most Useful Refinements To Reduce Workflow Risk

If the goal is to reduce the chance that delay damages fusion/localization, the highest-signal refinements are:

1. Measure message age continuously, not just frequency.
   - Use `scripts/check_latency.sh` and `scripts/check_latency.py`.
   - Frequency alone can look okay while age is already bad.

2. Disable nonessential debug work during live fusion.
   - Set `PUBLISH_DEBUG_MAP=false`.
   - Only enable `PUBLISH_DEBUG_IMAGE` when needed.

3. If camera-side delay is visible, first raise processing freshness, not debug smoothness.
   - Prefer reducing `PROCESS_WIDTH` / `PROCESS_HEIGHT` or increasing `MAX_PROCESSING_FPS`.
   - Only after that, decide whether `DEBUG_EVERY_N` should be 1.

4. If LiDAR-side delay is visible, compare each stage separately.
   - `/livox/lidar` fresh but `/livox/lidar_dynamic` old: dynamic_cloud is the bottleneck.
   - `/livox/lidar_dynamic` fresh but `/livox/lidar_cluster` old: cluster is the bottleneck.
   - frequent TF fallback warnings: localization / TF freshness is the bottleneck.

5. Add backlog observability where it is currently weak.
   - Livox raw-packet queue size logging
   - localization callback age logging
   - dynamic-cloud TF fallback counters

6. Fix localization bootstrap staleness.
   - `gridMap.clear()` per callback is a concrete refinement worth doing.
   - review whether queue depth `10` is appropriate for live mode

7. Keep viewer overhead separate from data-path judgment.
   - use `scripts/view_camera_opencv.py` for images
   - use `scripts/test_minimal_stutter.sh` to separate RViz lag from LiDAR pipeline lag

## Practical Interpretation Of What You Saw

If both LiDAR visualization and `detect_image` look delayed at the same time, the most likely explanations are:

- the machine is overloaded by combined inference + debug-image/map publishing + point-cloud processing + viewers
- the visual topics you are watching are processed/debug topics, not the freshest raw topics
- LiDAR-side callbacks are contending inside the shared component container

If I rank the current risks by likelihood for visible delay in the live workflow, I would put them roughly like this:

1. `detect_image` is slow by design because it is capped + post-inference + debug-throttled
2. dynamic_cloud KD-tree work and cluster extraction
3. shared LiDAR component-container contention
4. debug map / debug image overhead
5. localization startup staleness (`accumulate_time=10`, deep queue, uncleared `gridMap`)
6. Livox raw-packet queue backlog under load
