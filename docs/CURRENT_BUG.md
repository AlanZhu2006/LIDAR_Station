# Current Bug Notes

This document records the major problems found in the audit of the current
`testmap` + `mapping_ws/test.pcd` workflow, from calibration outputs through
runtime fusion.

Audit date: 2026-03-18

## 1. Default testmap-to-rm_frame alignment was bad; code now guards against it

### Where

- `scripts/start_fusion.sh`
- `src/tdt_vision/detect/scripts/nyush_world_node.py`
- `src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py`
- `config/local/testmap_to_rm_frame.yaml`

### Original problem

The original runtime path enabled `APPLY_TESTMAP_RM_ALIGNMENT=true` by default
and loaded `config/local/testmap_to_rm_frame.yaml` whenever that file existed.
The loaded matrix was not rejected based on fit quality.

The previously saved alignment was clearly poor:

- `rmse_m: 4.206119337491437`
- `max_m: 5.949827959726479`
- `num_pairs: 4`
- `num_inliers: 2`

Rechecking the saved correspondences showed that pairs 1 and 2 fit nearly
exactly, but pairs 3 and 4 are off by about `5.95 m`. That means the transform
is effectively fit to only one side of the map.

### Current transform flow

The important point is that the radar-station code does already account for the
vertical-to-horizontal `testmap` rotation. The current bad fit is not explained
by `my_map.jpg` vs `my_map(m).jpg` being mixed incorrectly.

Current path in code:

1. `calibration.py` in `NYUSH_Robotics_RM_RadarStation` uses `images/my_map.jpg`
   as the clicked calibration map for `map_profile == "testmap"`.
2. The saved metadata also records that the runtime display map is
   `images/my_map(m).jpg`.
3. `nyush_world_node.py` loads:
   - `array_test_custom.npy`
   - runtime map: `images/my_map(m).jpg`
   - calibration map: `images/my_map.jpg`
4. The saved homography in `array_test_custom.npy` maps camera pixels into the
   calibration-map coordinate system, which is the vertical `my_map.jpg`.
5. `nyush_world_node.py` then converts that vertical-map coordinate into the
   horizontal runtime-map coordinate with:

```text
display_x = calib_y
display_y = (calibration_map_width - 1) - calib_x
```

This is a `90 deg` counterclockwise rotation from `my_map.jpg` to
`my_map(m).jpg`. It is a rotation, not a left-right mirror.

Corner examples with the current image sizes (`my_map.jpg = 540x960`,
`my_map(m).jpg = 960x540`):

```text
vertical my_map.jpg      horizontal my_map(m).jpg
(0, 0)                -> (0, 539)
(539, 0)              -> (0, 0)
(0, 959)              -> (959, 539)
(539, 959)            -> (959, 0)
```

So if `/nyush_map_image` shows the radar station on the right side of
`my_map(m).jpg`, that is consistent with the current raw runtime `testmap`
coordinate definition. `/nyush_map_image` is still raw `testmap` space and is
not drawn in LiDAR `rm_frame`.

After that rotation, `nyush_world_node.py` converts the horizontal runtime-map
pixel into raw `testmap` metric coordinates with:

```text
world_x_m = pixel_x * 6.79 / 960
world_y_m = (540 - pixel_y) * 3.82 / 540
```

So the raw `testmap` metric frame used by `/resolve_result` before LiDAR
alignment is defined directly in `my_map(m).jpg` space:

- `+x` goes to the right on `my_map(m).jpg`
- `+y` goes upward on `my_map(m).jpg`

The LiDAR-camera alignment tool is also consistent with that same raw frame:

- `calibrate_testmap_lidar_alignment.py` uses `my_map(m).jpg` directly
- it converts clicked `my_map(m).jpg` pixels into meters with the same rule:

```text
world_x_m = pixel_x * field_width / map_width
world_y_m = (map_height - pixel_y) * field_height / map_height
```

So the current flow is:

```text
camera image
  -> homography to my_map.jpg (vertical calibration map)
  -> rotate 90 deg CCW to my_map(m).jpg display coordinates
  -> scale to raw testmap meters
  -> optional similarity transform to LiDAR rm_frame
```

This means the current "only one side fits" issue starts after the raw
`testmap` metric frame is already established. The problem is in the saved
`testmap_metric -> rm_frame` correspondences, not in the `my_map.jpg` to
`my_map(m).jpg` rotation logic.

More specifically, the saved correspondence ordering shows a handedness
conflict:

- in raw `testmap` meters, pairs 1-2 are on one horizontal side and pairs 3-4
  are on the other
- in saved `rm_frame` meters, that left-right ordering is reversed

Because the calibration tool solves a rotation + translation + uniform scale
model, not a mirrored transform, RANSAC locks onto the pair set that is
internally consistent and throws the opposite side away as outliers.

### Runtime impact before the fix

- `/resolve_result` can be warped into the wrong `rm_frame` coordinates even if
  the raw NYUSH `testmap` projection is correct.
- The default `camera_detect_radius` is only `1.0 m`, so a `4 m` to `6 m`
  alignment error is enough to make camera-LiDAR fusion fail systematically.
- The failure can be misleading because `/nyush_map_image` may still look
  correct while `/resolve_result` is wrong after alignment is applied.

### Why this was a workflow bug

The calibration tool writes the matrix and its error metrics, but the runtime
loader only checks file existence and basic matrix shape. It does not reject
bad calibrations, low-inlier results, or obviously unsafe error values.

### Implemented fix

The code now handles the handedness issue explicitly and refuses unsafe
alignment files at runtime.

Implemented changes:

1. `calibrate_testmap_lidar_alignment.py` now supports:
   - `--orientation-mode normal`
   - `--orientation-mode flip_x`
   - `--orientation-mode auto` (default)
2. In `auto`, the tool solves two hypotheses:
   - direct `testmap_metric -> rm_frame`
   - `x`-flipped `testmap_metric -> rm_frame`
3. The tool saves:
   - `orientation_hypothesis`
   - `similarity_transform` for the post-pretransform similarity fit
   - `runtime_transform.matrix_3x3` for the actual matrix runtime should apply
4. `nyush_world_node.py` now:
   - prefers `runtime_transform.matrix_3x3` when available
   - falls back to legacy `similarity_transform.matrix_3x3` for older YAMLs
   - recomputes alignment error from saved correspondences when available
   - rejects the alignment if it fails quality checks
5. `debug_map.cpp` now:
   - prefers the same runtime matrix field
   - suppresses the warped NYUSH background when the saved alignment metrics are
     obviously unsafe, so the visualization does not keep showing a bad warp

### Runtime acceptance gate

`nyush_world_node.py` now rejects a saved `testmap_to_rm_frame.yaml` if any of
these checks fail:

- fewer than `3` correspondence pairs
- fewer than `max(3, ceil(0.75 * num_pairs))` correspondences within `0.25 m`
- `rmse_m > 0.5`
- `max_m > 1.0`

If the file is rejected, the node falls back to publishing raw `testmap`
metric coordinates on `/resolve_result` instead of applying a bad warp.

### Verified behavior on the current saved YAML

The local `config/local/testmap_to_rm_frame.yaml` has now been re-solved from
the same 4 saved correspondences using the new orientation-aware logic.

Results:

- legacy `normal` hypothesis recorded in `candidate_metrics`:
  - `rmse = 4.206119337491437 m`
  - `max = 5.949827959726479 m`
  - `inliers at 0.25 m = 2 / 4`
  - runtime acceptance result: rejected
- current saved runtime hypothesis:
  - `selected = flip_x`
  - `rmse = 0.036085625341434356 m`
  - `max = 0.047770712207540963 m`
  - `inliers at 0.25 m = 4 / 4`
  - runtime acceptance result: accepted

That means the repo is now in the intended state for this issue:

- the calibration tool can discover the mirrored relation
- runtime consumes the correct matrix field
- runtime rejects obviously unsafe alignment files
- the local alignment YAML now contains the accepted `flip_x` solution

### Current operator note

If the map, PCD, or landmark picks change later, the correct way to refresh the
alignment is still:

```bash
python3 src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py --orientation-mode auto
```

and then save a new `config/local/testmap_to_rm_frame.yaml`.

## 2. Tilted-installation workflow is not wired end to end

### Where

- `scripts/start_mapping.sh`
- `scripts/rotate_pcd.py`
- `scripts/start_fusion.sh`

### Problem

The tilt workflow is described as:

1. save `mapping_ws/test.pcd`
2. run `python3 scripts/rotate_pcd.py`
3. use tilt correction at runtime

But `rotate_pcd.py` writes to `mapping_ws/test_rotated.pcd` by default, while
`start_fusion.sh` still launches runtime localization against
`mapping_ws/test.pcd`.

So the documented tilted-install flow does not actually switch runtime over to
the rotated map file.

### Runtime impact

- The live point cloud may be tilt-corrected while localization still uses the
  unrotated map.
- This can break GICP initialization or make localization unstable.
- The operator can follow the documented steps and still end up with a
  mathematically inconsistent map/live-cloud pairing.

### Why this is a workflow bug

The repo has the helper to create the rotated map, but the main startup path
does not consume that artifact automatically and does not tell the operator to
rename or replace `test.pcd`.

## 3. Tilt sign convention is internally inconsistent

### Where

- `src/lidar/dynamic_cloud/launch/lidar_tilt.launch.py`
- `src/lidar/pointcloud_transform/src/pointcloud_transform_node.cpp`
- `scripts/rotate_pcd.py`
- `README_LIDAR.md`

### Problem

The repo does not use one stable sign convention for tilt correction:

- `lidar_tilt.launch.py` publishes `livox_frame -> livox_frame_ground` using
  `+50 deg`
- `pointcloud_transform_node.cpp` comments say the matching TF should use
  `-50 deg`
- `rotate_pcd.py` also defaults to `--pitch -50`
- `README_LIDAR.md` documents `-0.873 rad` for the tilt TF

### Runtime impact

- Even if the rotated-map file wiring were fixed, tilt mode still cannot be
  trusted without re-deriving the sign convention once and applying it
  consistently.
- This makes the tilt branch fragile and operator-dependent.

### Why this is a workflow bug

This is not just a stale doc issue. The inconsistency exists between the
runtime launch file, the helper script, and the inline source comments.

## 4. Auxiliary outputs still assume official RM field geometry

### Where

- `src/lidar/dynamic_cloud/src/dynamic_cloud.cpp`
- `src/fusion/kalman_filter/src/kalman_filter.cpp`

### Problem

The core fusion path uses `rm_frame`, but several auxiliary paths still hardcode
official-field coordinates:

- `dynamic_cloud.cpp` uses fixed dart/fly/engine region coordinates
- `kalman_filter.cpp` mirrors blue-side output around `28 x 15`
- `kalman_filter.cpp` injects fixed engine coordinates

These values are not derived from the active `testmap` or `mapping_ws/test.pcd`.

### Runtime impact

- `/lidar_detect` is not mathematically generalized to arbitrary custom maps.
- `/radar2sentry` is not a pure function of the active LiDAR map frame.
- Practice-field runs may look correct in core fusion while these side outputs
  remain field-specific and misleading.

## 5. Small moving robot detection regressed after single-frame LiDAR defaults

### Where

- `scripts/start_fusion.sh`
- `src/lidar/dynamic_cloud/launch/lidar.launch.py`
- `src/lidar/dynamic_cloud/launch/lidar_tilt.launch.py`
- `src/lidar/cluster/src/cluster.cpp`
- `src/lidar/cluster/include/cluster.h`
- `scripts/diagnose_detection.sh`

### Problem

The recent runtime change to single-frame dynamic clouds is good for timing,
but it also removes the extra point density that previously helped a small,
low robot survive clustering.

With the new single-frame defaults:

- `accumulate_time = 1`
- `publish_accumulated_dynamic_cloud = false`

the cluster stage was still using a relatively dense target assumption:

- default `min_cluster_size = 8`
- no recovery path for sparse-but-valid small clusters

That combination can explain the observed symptom:

- larger moving objects like people or scooters still produce enough dynamic
  points and continue to cluster
- a shorter robot may now produce too few points in one frame and disappear
  before Kalman/fusion ever sees it

There was also a separate operator-facing bug:

- `scripts/diagnose_detection.sh` recommended increasing
  `kd_tree_threshold_sq` when `/livox/lidar_dynamic` is empty
- that advice is backwards for this codebase because a larger threshold makes
  dynamic extraction more conservative, not less

### Implemented fix

The LiDAR detection path is now more tolerant of small robots without undoing
the single-frame timing fix:

1. The runtime default `MIN_CLUSTER_SIZE` was reduced from `8` to `6`.
2. `cluster.cpp` now has a small-cluster recovery path:
   - it still keeps the normal shape gates for standard clusters
   - it also accepts sparse clusters down to `4` points if they satisfy tighter
     size and height bounds
3. Cluster logs now report:
   - `recovered_small`
   - `rejected_small`
   - `rejected_shape`
4. `diagnose_detection.sh` now points operators in the correct direction:
   - if `/livox/lidar_dynamic` is empty, try a smaller
     `kd_tree_threshold_sq` such as `0.12` or `0.10`

### Expected runtime effect

- Small moving robots should be more likely to produce `/livox/lidar_cluster`
  outputs again under the current single-frame pipeline.
- Humans and scooters should keep working as before.
- If the active map itself is bad, this patch will not fully mask that; however,
  it removes the strict clustering gate that was the clearest repo-level cause
  of the regression.

## Suggested Fix Order

Alignment fix status:

- completed in code
- completed for the current local `testmap_to_rm_frame.yaml`

1. Fix the tilted-installation map flow so the rotated PCD is actually the map
   consumed by runtime when tilt mode is used.
2. Unify tilt sign convention across launch files, helper scripts, and docs.
3. Move hardcoded field geometry out of `dynamic_cloud.cpp` and
   `kalman_filter.cpp` into explicit map/config parameters.
