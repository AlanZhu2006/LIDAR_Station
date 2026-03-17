# Testmap-LiDAR Alignment

## 1. Why This Exists

Before this change, the fusion stack assumed that two independently-produced position streams were already in the same field coordinate frame:

- NYUSH `testmap` world coordinates from `nyush_world_node.py`
- LiDAR `rm_frame` coordinates from `localization -> dynamic_cloud -> cluster`

That assumption was not enforced anywhere.

The old flow was effectively:

1. Camera detections were projected onto the NYUSH `testmap`
2. `nyush_world_node.py` scaled those testmap pixels into meters
3. LiDAR was localized against `mapping_ws/test.pcd`
4. `kalman_filter` compared the two outputs by Euclidean distance

This only worked if:

- the NYUSH testmap metric frame
- and the LiDAR `rm_frame`

already happened to share the same:

- origin
- rotation
- scale
- handedness / axis convention

In practice, that was a hidden contract. When it was wrong, fusion failed silently:

- `/resolve_result` was nonzero
- `/livox/lidar_cluster` existed
- but `/livox/lidar_kalman` stayed gray because camera and LiDAR positions were more than `camera_detect_radius` apart

The new alignment block makes that contract explicit and calibratable.

---

## 2. Design Goal

We wanted a solution that:

1. Works with the current NYUSH `testmap` workflow
2. Works with the actual LiDAR map used by fusion, namely `mapping_ws/test.pcd`
3. Does not require changing the LiDAR localization pipeline
4. Does not require changing the NYUSH camera calibration workflow
5. Produces a stable transform that can be saved and reused
6. Fits into the existing startup flow with minimal operator overhead

The chosen design is:

- an offline manual calibration tool
- that solves a 2D similarity transform
- from `testmap` metric coordinates to LiDAR `rm_frame`
- and applies that transform inside `nyush_world_node.py`

This means:

- NYUSH still does camera-to-testmap calibration as before
- LiDAR still does map localization as before
- but now an explicit bridge transform connects the two

---

## 3. What We Chose

The implementation follows these decisions:

### 3.1 Transform Model

We use a **2D similarity transform**:

- rotation
- translation
- uniform scale

We did **not** use:

- rigid transform only
  because scale mismatch is realistic between image map assets and LiDAR maps
- full affine transform
  because that would allow shear and would be too unconstrained for a real field

Similarity is the best compromise:

- flexible enough to absorb practical map mismatch
- constrained enough to stay physically meaningful

### 3.2 Where The Transform Is Applied

The saved transform is applied in:

- `src/tdt_vision/detect/scripts/nyush_world_node.py`

This was chosen instead of applying it in `kalman_filter` because:

1. `/resolve_result` should represent the true runtime world frame, not a half-baked internal frame
2. all downstream consumers should see corrected positions
3. it keeps fusion logic simple: fusion should compare already-aligned positions

### 3.3 Calibration Method

We chose:

- manual point picking
- on the NYUSH runtime map image
- against a LiDAR top-down projection generated from `mapping_ws/test.pcd`

This was chosen because:

1. it is robust enough for a first working version
2. it avoids dependence on automatic feature extraction
3. it uses the exact runtime assets already present in your stack

### 3.4 Runtime Debug Choice

We intentionally kept:

- `/nyush_map_image`

in the original NYUSH testmap display frame.

This is important.

That topic is still the best way to inspect whether camera-to-testmap calibration itself is correct.

If we had also changed `/nyush_map_image` into LiDAR frame directly, we would lose a valuable debug view of the original NYUSH mapping stage.

So the new behavior is:

- `/nyush_map_image` stays in raw testmap space
- `/resolve_result` becomes aligned into `rm_frame`

This separation is deliberate.

---

## 4. Coordinate Philosophy

The final shared coordinate truth is now:

- **LiDAR `rm_frame`**

This is the best choice because:

1. LiDAR localization is already explicitly tied to the real point-cloud map
2. all LiDAR dynamic points and cluster centroids are already published in `rm_frame`
3. `kalman_filter` already consumes LiDAR output in that frame

So the new transform should be understood as:

```text
testmap metric coordinates  ->  rm_frame coordinates
```

not the other way around.

---

## 5. What Was Added

### 5.1 New Calibration Tool

Added:

- `src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py`

This tool:

1. loads the NYUSH runtime testmap image
2. loads `mapping_ws/test.pcd`
3. rasterizes the PCD into a top-down occupancy-like image
4. lets the operator click corresponding points on both maps
5. estimates a 2D similarity transform in meters
6. saves the result as YAML
7. saves preview images

### 5.2 New Runtime Parameters

Added to `nyush_world_node.py` and `nyush_integration.launch.py`:

- `apply_testmap_rm_alignment`
- `alignment_config_path`

These control whether the saved transform is applied.

### 5.3 New Default Output Location

The alignment file is intended to live under:

- `config/local/testmap_to_rm_frame.yaml`

This path is machine-specific calibration data, so:

- `config/local/`

was added to `.gitignore`.

That choice is intentional because this calibration depends on:

- your current `test.pcd`
- your current NYUSH testmap asset
- your current real-world setup

It is local runtime calibration, not a universal repository constant.

### 5.4 Integration Into Existing Fusion Startup

`scripts/start_fusion.sh` now defaults to:

- `APPLY_TESTMAP_RM_ALIGNMENT=true`
- `TESTMAP_ALIGNMENT_CONFIG=$WS_ROOT/config/local/testmap_to_rm_frame.yaml`

So after calibration, normal fusion startup works without changing your day-to-day command.

---

## 6. Files Changed

### 6.1 New Files

- `src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py`
- `docs/TESTMAP_LIDAR_ALIGNMENT.md`

### 6.2 Modified Files

- `.gitignore`
- `scripts/start_fusion.sh`
- `src/tdt_vision/CMakeLists.txt`
- `src/tdt_vision/launch/nyush_integration.launch.py`
- `src/tdt_vision/detect/scripts/nyush_world_node.py`

---

## 7. How The New Calibration Tool Works

### 7.1 Inputs

By default, the tool uses:

- LiDAR map:
  `mapping_ws/test.pcd`
- NYUSH runtime map image:
  `/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation/images/my_map(m).jpg`

The field size defaults to:

- width: `6.79 m`
- height: `3.82 m`

### 7.2 LiDAR Top-Down Projection

The tool uses `open3d` to load the PCD and creates a 2D raster from the point cloud.

The rasterization process:

1. filters points by z range
2. computes x/y bounds
3. projects points into a 2D grid
4. accumulates density per cell
5. applies log scaling and histogram equalization

This produces a human-clickable top-down map that corresponds to LiDAR `rm_frame`.

### 7.3 Point Picking

The UI shows:

- left: NYUSH runtime testmap
- right: LiDAR top-down map

The operator clicks one corresponding landmark on each side.

Once both clicks exist, they become one pair.

Keys:

- `s`: solve and save
- `u`: undo last point or pair
- `r`: reset
- `q`: quit

### 7.4 Transform Estimation

The tool converts:

- testmap pixels -> testmap world meters
- LiDAR top-down pixels -> rm_frame world meters

Then it estimates a similarity transform using:

- `cv2.estimateAffinePartial2D`

with RANSAC enabled.

The saved matrix is stored as a `3x3` homogeneous transform:

```text
[ x_rm ]   [ a  b  tx ] [ x_testmap ]
[ y_rm ] = [ c  d  ty ] [ y_testmap ]
[  1   ]   [ 0  0   1 ] [    1      ]
```

where `a,b,c,d` encode:

- uniform scale
- rotation

### 7.5 Saved Outputs

The calibration tool saves:

- YAML transform file
- LiDAR top-down PNG
- preview overlay PNG

Default paths:

- `config/local/testmap_to_rm_frame.yaml`
- `config/local/testmap_to_rm_frame_topdown.png`
- `config/local/testmap_to_rm_frame_preview.png`

The preview overlay is particularly useful because it lets you visually inspect whether the solved transform makes geometric sense before running fusion.

---

## 8. Runtime Behavior After This Change

### 8.1 Inside NYUSH World Node

`nyush_world_node.py` still performs the same stages:

1. receives `/camera_image`
2. runs NYUSH detector
3. uses NYUSH perspective matrices to map camera points onto testmap
4. converts testmap pixels into metric coordinates

At that point, the old flow ended.

Now there is one more step:

5. if enabled and available, apply the saved `testmap -> rm_frame` similarity transform
6. publish aligned `/resolve_result`

### 8.2 Debug Map Behavior

`/nyush_map_image` still shows the raw testmap-space result.

Additionally, text is drawn onto the image to indicate:

- whether runtime alignment is enabled
- whether the transform file was loaded
- the loaded similarity summary

This makes it clear that:

- the display map is still raw testmap
- while `/resolve_result` may now be in aligned `rm_frame`

### 8.3 Fusion Behavior

`kalman_filter` remains unchanged.

This is an important design point.

We did **not** complicate the fusion logic.

The new assumption is now valid:

- `/resolve_result` is already in the same world frame as `/livox/lidar_cluster`

So `kalman_filter` can continue matching by:

- time proximity
- spatial proximity

This improves engineering clarity:

- calibration fixes coordinate problems
- fusion does not secretly compensate for bad frames

---

## 9. Operational Workflow Now

With this change, the intended workflow becomes:

1. Build or refresh the LiDAR map:
   `mapping_ws/test.pcd`
2. Run NYUSH camera-to-testmap calibration as before
3. Run the new testmap-to-LiDAR alignment calibration
4. Start fusion normally

So yes, the only new operator step is:

- one extra calibration between testmap and LiDAR map

After that, the fusion launch flow remains the same.

---

## 10. Exact Usage

### 10.1 Build

```bash
cd ~/Desktop/RadarStation
source /opt/ros/humble/setup.bash
colcon build --packages-select tdt_vision --symlink-install
source install/setup.bash
```

### 10.2 Run The Alignment Tool

```bash
python3 ~/Desktop/RadarStation/src/tdt_vision/detect/scripts/calibrate_testmap_lidar_alignment.py
```

### 10.3 Pick Landmarks

Recommended landmarks:

- outer field corners
- outpost corners
- base corners
- ramp or high-ground corners
- other rigid, easy-to-identify map features

Recommendations for good calibration:

- use at least 4 points
- use 6 to 8 points if possible
- spread them across the whole map
- avoid clustering all picks in one local area

### 10.4 Save

Press:

- `s`

The transform YAML and preview images will be written under `config/local/`.

### 10.5 Run Fusion As Usual

```bash
cd ~/Desktop/RadarStation
source install/setup.bash
./scripts/start_fusion.sh
```

The startup script now automatically looks for:

- `config/local/testmap_to_rm_frame.yaml`

and enables runtime alignment by default.

---

## 11. Manual Launch Equivalent

If you do not want to use `start_fusion.sh`, the equivalent NYUSH launch is:

```bash
ros2 launch tdt_vision nyush_integration.launch.py \
  nyush_path:=/home/nyu/Desktop/NYUSH_Robotics_RM_RadarStation \
  map_mode:=testmap \
  state:=R \
  field_width_m:=6.79 \
  field_height_m:=3.82 \
  apply_testmap_rm_alignment:=true \
  alignment_config_path:=/home/nyu/Desktop/RadarStation/config/local/testmap_to_rm_frame.yaml
```

If you want to temporarily disable the new behavior:

```bash
APPLY_TESTMAP_RM_ALIGNMENT=false ./scripts/start_fusion.sh
```

or:

```bash
ros2 launch tdt_vision nyush_integration.launch.py \
  ... \
  apply_testmap_rm_alignment:=false
```

---

## 12. What Success Looks Like

After a good alignment:

1. `/nyush_map_image` still looks correct in testmap space
2. `/resolve_result` is nonzero and stable
3. `/livox/lidar_cluster` contains targets in `rm_frame`
4. `/livox/lidar_kalman` should colorize and identify targets more reliably

In practical terms, the biggest expected improvement is:

- the LiDAR tracks should no longer stay gray solely because camera and LiDAR coordinates disagree

---

## 13. Troubleshooting

### 13.1 `/resolve_result` Is Still Far From LiDAR

Possible causes:

- bad landmark picks
- landmarks too concentrated in one region
- wrong `test.pcd`
- wrong testmap image
- wrong field size

Action:

- recalibrate using better-distributed landmarks
- inspect the saved preview PNG

### 13.2 `/nyush_map_image` Looks Good But Fusion Still Fails

This can now mean:

1. NYUSH camera-to-testmap calibration is fine
2. but testmap-to-LiDAR alignment is still wrong

Action:

- inspect `config/local/testmap_to_rm_frame_preview.png`
- verify your point pairs

### 13.3 Alignment File Not Loaded

`nyush_world_node.py` will warn if:

- `apply_testmap_rm_alignment=true`
- but the YAML path is missing or unreadable

In that case it falls back to raw testmap coordinates and fusion will behave like before.

### 13.4 `/nyush_map_image` And `/resolve_result` Now Seem Different

That is expected.

Remember:

- `/nyush_map_image` is intentionally still raw testmap space
- `/resolve_result` may now be aligned into `rm_frame`

This split is by design.

---

## 14. Why We Did Not Choose Other Approaches

### 14.1 Why Not Apply The Transform In `kalman_filter`

Because that would hide calibration errors inside fusion and make downstream world coordinates ambiguous.

We want:

- one node to own world-coordinate correction
- one clear source of aligned positions

### 14.2 Why Not Make `/nyush_map_image` Also Use LiDAR Frame

Because `/nyush_map_image` is the best debug view for the original NYUSH calibration path.

If we changed it into LiDAR frame directly, it would be harder to debug whether the camera-to-testmap stage itself is wrong.

### 14.3 Why Not Full Automatic Alignment

Automatic registration between:

- a stylized map image
- and a noisy LiDAR top-down raster

is possible, but significantly more brittle.

A manual point-picked first version is much more controllable and realistic for immediate field use.

### 14.4 Why Not Rigid-Only

Because real map assets and map generation pipelines often include small scale mismatch.

Rigid-only would be too optimistic.

### 14.5 Why Not Full Affine

Because shear is not physically meaningful for a real field and would overfit operator error.

Similarity is the right engineering compromise.

---

## 15. Validation We Performed

The following were validated during implementation:

1. Python syntax:
   - `calibrate_testmap_lidar_alignment.py`
   - `nyush_world_node.py`
   - `nyush_integration.launch.py`
2. LiDAR top-down raster generation from the current `mapping_ws/test.pcd`
3. `colcon build --packages-select tdt_vision --symlink-install`
4. launch argument exposure through `nyush_integration.launch.py`

What was **not** performed here:

- actual operator landmark clicking
- runtime field verification against moving targets

That final verification must be done on your machine with your real GUI and real landmarks.

---

## 16. Future Improvements

The current implementation is the correct first block, but it can be extended.

Good next improvements would be:

1. A runtime debug topic that overlays aligned NYUSH positions directly onto the LiDAR top-down map
2. A second calibration mode using typed correspondences instead of GUI clicks
3. More structured residual reporting for calibration quality
4. Optional landmark snapping or zoom UI
5. Semi-automatic landmark extraction
6. A live validation node that compares `/resolve_result` to `/livox/lidar_cluster` and reports residuals continuously

The most useful immediate next step is likely:

- a runtime aligned overlay debug topic

because it would let you visually inspect the final post-alignment fusion geometry in the LiDAR frame directly.

---

## 17. Bottom Line

This change does not replace your existing calibration flow.

It adds the missing bridge between:

- NYUSH testmap coordinates
- and LiDAR map coordinates

The new operational model is:

1. calibrate camera to testmap
2. calibrate testmap to LiDAR map
3. run fusion normally

That extra second calibration is now the only added operator step needed to make the current decoupled fusion architecture explicit and much more reliable.
