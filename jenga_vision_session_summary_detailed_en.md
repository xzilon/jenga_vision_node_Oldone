# ROS2 Jenga Robot Vision — Detailed Session Log

## Project Context

- Team: iRAP, a world-championship-level RoboCup Rescue robotics team
- User: Poorinut ("Gateaux"), first-year Mechatronics Engineering student at KMUTNB
- Goal: computer vision system for a Jenga-block-stacking competition robot
- Robot hardware: X-drive holonomic base, rotating gripper turret, a single fixed RGB camera mounted on the **side** of the robot (not top-down)
- Target block: wooden block painted blue on all visible faces
- Dev progression: started testing on a **laptop webcam**, later switched to the actual competition camera — a **Logitech C270** mounted on a **Raspberry Pi** running the robot's onboard software
- ROS 2 distro: Humble (on the dev laptop, Pop!_OS), workspace path evolved from `~/ros2_ws` to `~/negativeJ3k_ws`
- Package name: `jenga_vision`, primary source file by the end of the session: `camera_node2.py` (an earlier file `camera_node.py` exists from initial ROS2-learning exercises and is not the active one)
- Note: one Pi terminal in later screenshots showed `ros-jazzy` paths instead of Humble — the Pi appears to be running a different ROS2 distro (Jazzy) than the dev laptop (Humble); this was not explicitly flagged as a problem during the session but is worth knowing about for future debugging (message/API compatibility between distros is normally fine for these basic APIs, but worth keeping in mind).

## Teaching / Collaboration Style Used This Session (important for continuity)

- The user explicitly asked to be taught **slowly, step by step**, because it had been about a year since they last touched ROS (and that was ROS1, not ROS2). They wanted room to "recall memory."
- Claude's role throughout: explain a concept, give a skeleton/hints with `TODO` markers, let the user write the actual code, then review what they wrote — pointing out exact bugs (often typos, missing `get_` prefixes, mismatched return-value counts, indentation) rather than rewriting everything.
- The user prefers implementing things themselves and asking for targeted conceptual guidance and feedback rather than being handed full solutions.
- Communicates in Thai mixed with English technical terms; responses in this session were given in Thai (the user's primary language) except where explicitly requested otherwise (this summary is in English per explicit request).
- Recurring pattern: user pastes their code attempt → Claude identifies 1-3 specific bugs with exact line/cause → user fixes and re-pastes → repeat until working → then build/run/test instructions are given → user reports back real hardware results (often via screenshots of terminal + rqt_image_view) → Claude interprets the numeric/visual results and decides next step.
- The user is comfortable asking Claude to pause deep technical rabbit holes (e.g., Canny edge detection, camera_calibration ROS package) when a simpler existing tool/approach solves the same problem — Claude pushed back constructively several times when a proposed solution (e.g. Canny edge detection for corner-finding, or blaming compressed-transport for mask noise) didn't actually address the root cause, and explained why before agreeing to proceed differently.

## Chronological Build Log

### Phase 1 — ROS2 fundamentals from scratch

Starting point: user had no ROS2 workspace at all. Built up from zero:

1. `mkdir -p ~/ros2_ws/src`, verified `$ROS_DISTRO` was `humble`, added `source /opt/ros/humble/setup.bash` to `.bashrc` for persistence.
2. Created the package: `ros2 pkg create --build-type ament_python jenga_vision --dependencies rclpy`
3. Explained core ROS2 concepts (node / topic / message type) before touching code, per user's explicit request to review basics first.
4. Wrote a minimal `CameraNode` skeleton (empty node that logs "Camera node started" and spins) purely to verify the ROS2 node lifecycle worked before adding any OpenCV logic.
5. Registered the entry point in `setup.py`'s `entry_points > console_scripts`, format: `'camera_node = jenga_vision.camera_node:main'`.
6. `colcon build --packages-select jenga_vision`, then `source install/setup.bash`, then `ros2 run jenga_vision camera_node` — confirmed working with a log line.

**Bugs fixed along the way (Phase 1):**
- `self.logger()` → must be `self.get_logger()` (missing `get_` prefix) — this exact mistake recurred **multiple times** throughout the whole session (also later as `self.get_logger.info(...)` missing the parentheses on `get_logger()`, and once as `sef.logger()` — a compound typo of both `self`→`sef` and missing `get_`).
- Attempted to write a `while True` + `cv2.imshow` loop directly inside `__init__` in early OpenCV-camera-node attempts — corrected to the ROS2 idiom: use `self.create_timer(period, callback)` and let `rclpy.spin()` drive the loop, since a blocking `while True` inside `__init__` would prevent `rclpy.spin()` from ever being reached.
- Indentation errors when pasting code (flattened to column 0) — general reminders about Python's whitespace-based block structure.
- `rclpy.shutdown()` being called twice (once inside the timer callback on a "press q to quit" OpenCV window, once again in `main()`'s `finally` block) — fixed by wrapping `main()`'s spin loop in `try/except KeyboardInterrupt` / `finally`, and guarding the final `rclpy.shutdown()` with `if rclpy.ok():`.

### Phase 2 — Camera capture + custom message serialization (no cv_bridge)

- Added `cv2.VideoCapture(0)`, checked `.isOpened()`, wired up a 15 Hz timer callback (`1.0/15.0`).
- First attempted to use `cv_bridge.CvBridge().cv2_to_imgmsg(...)` — this **failed at runtime** with:
  ```
  AttributeError: _ARRAY_API not found
  ...
  KeyError: 16
  ```
  Root cause: the machine had **numpy 2.2.6** installed via pip, but ROS2 Humble's pre-built `cv_bridge` binary (installed via `apt`) was compiled against numpy 1.x's C-API. This is a fundamental ABI incompatibility, not a bug in the user's code.
- Two fixes were discussed: (a) `pip install "numpy<2" --break-system-packages` (simple downgrade, risks breaking other numpy≥2-dependent projects), or (b) **write the `sensor_msgs/Image` conversion by hand**, bypassing `cv_bridge` entirely. The user chose (b).
- Hand-written `frame_to_imgmsg(self, frame)`:
  ```python
  msg = Image()
  msg.header.stamp = self.get_clock().now().to_msg()
  msg.header.frame_id = 'camera_frame'
  msg.height = frame.shape[0]
  msg.width = frame.shape[1]
  msg.encoding = 'bgr8'
  msg.is_bigendian = 0
  msg.step = msg.width * 3
  msg.data = frame.tobytes()
  return msg
  ```
- This raw `Image` publisher caused severe **lag/stutter** in `rqt_image_view` due to bandwidth (640×480×3 bytes/frame × 15fps ≈ 13.5 MB/s uncompressed over DDS).
- Solution: added a second publisher using `sensor_msgs/CompressedImage` with JPEG encoding via `cv2.imencode('.jpg', frame)`:
  ```python
  def frame_to_compressed_imgmsg(self, frame):
      msg = CompressedImage()
      msg.header.stamp = self.get_clock().now().to_msg()
      msg.header.frame_id = 'camera_frame'
      msg.format = 'jpeg'
      success, encoded = cv2.imencode('.jpg', frame)
      if not success:
          self.get_logger().error('failed to encoded frame...')
          return None
      msg.data = encoded.tobytes()
      return msg
  ```
  This substantially reduced perceived stutter.

**ROS2 `image_transport` topic-naming gotcha encountered:** subscribing directly to `/videoframe/compressed` in `rqt_image_view` produced:
```
[image_transport] It looks like you are trying to subscribe directly to a transport-specific image topic '/videoframe/compressed'...
```
and later, when done incorrectly, a hard crash (`invalid allocator`, `RCLError`, process aborted). The fix: `image_transport` expects you to subscribe to the **base topic** (`/videoframe`) and select the transport (`compressed`) either via the GUI dropdown or via a launch parameter:
```bash
ros2 run rqt_image_view rqt_image_view --ros-args -p ~image_transport:=compressed
```

### Phase 3 — HSV color detection

- Standard pipeline: BGR→HSV via `cv2.cvtColor`, `cv2.inRange(hsv, lower_blue, upper_blue)` with initial values `lower_blue=(100,100,50)`, `upper_blue=(130,255,255)`, `cv2.findContours` (RETR_EXTERNAL, CHAIN_APPROX_SIMPLE), pick `max(contours, key=cv2.contourArea)`, then `cv2.boundingRect()` → `(x, y, w, h)`.
- Early bug pattern repeated a few times: inconsistent return-value counts between the "object found" and "object not found" branches of `detection_blue_object`, causing `ValueError` on tuple unpacking in `timer_callback`. Fixed each time by making both branches return the same number of values (using `None` placeholders on the "not found" branch).
- Initial real-world test (laptop webcam, testing on a phone before the wooden block was fully adopted as the test object) produced a suspiciously small bounding box (`w=26, h=25`) — diagnosed as detecting only a small glare/reflection rather than the whole object, not a code bug. Confirmed and resolved once the actual **wooden test block** was used and a **debug visualization of the mask** was added.
- Debug method: rather than `cv2.imshow()` (which fails outright on a headless SSH-only machine, i.e., the Raspberry Pi later in the session, since there is no X display), the mask was **published as its own `CompressedImage` topic** (`/mask/compressed`) so it could be inspected remotely via `rqt_image_view` running on the laptop, over the network, while the node itself ran on the Pi. This is the standard headless-debugging pattern used for the rest of the session.
- `w` value showed jitter of roughly ±2–3 px frame to frame at a fixed physical distance. This was later fixed (see Phase 5).

### Phase 4 — Camera intrinsics (distance estimation attempt #1: known-width method)

- Concept taught: pinhole camera model, `distance_mm = (real_width_mm * focal_length_px) / pixel_width_px`.
- First focal-length estimate used the "known distance, known width" single-point calibration method: placed the block at a **measured** distance of 300 mm, read off `w` from the logs (~79 px averaged), used a real width of 251 (later revealed to be a measurement unit/scale error — likely should have been ~25.1 mm, not 251), giving `focal_length_px ≈ 94.4`.
- Distance predictions using this estimate were wildly wrong at first (**2134 mm predicted vs ~220 mm actual** — off by ~10×), traced directly to the `known_width_mm=251` typo/scale error. After correcting to a small value (`~25`), predictions became much closer but still consistently biased low (e.g., true 270 mm → measured ~222–233 mm, a 14–18% systematic undershoot).

### Phase 5 — Root-causing the accuracy problem (multiple iterations)

This was the most extensive debugging arc of the session. Several hypotheses were raised and tested in sequence:

1. **Jitter in `w` between frames** → traced to `boundingRect`'s min/max-pixel sensitivity to mask edge noise. Fixed with **morphological cleanup**: `cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))`, then `cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)` followed by `cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)`. This visibly cleaned the published mask (near-perfect solid rectangle in `rqt_image_view`) and stabilized `w` dramatically (from a ±2–3 px swing down to bit-exact repeats across many consecutive frames in some tests).
   - Bug encountered while implementing this: user computed `opening`/`closing` correctly but then still fed the **pre-morphology raw `mask`** into `findContours` and into the function's return value — the cleanup had zero effect until both usages were switched to `closing`.

2. **User's own hypothesis: Canny edge detection + diagonal-intersection midpoint finding** — proposed early as a way to reduce jitter. Claude's counter-analysis: once `w` was already bit-stable post-morphology, spatial-noise-reduction techniques like Canny would add complexity without addressing the actual remaining error (which was a scale/calibration problem, not a per-frame noise problem). This was revisited later in a different, legitimate form (see Phase 6).

3. **Genuine camera intrinsics calibration**: the user had a **USB flash drive** containing a prior calibration dataset for the C270 camera, produced by a previous session with ROS2's `camera_calibration` package (`ros2 run camera_calibration cameracalibrator`). Contents included ~110 checkerboard images (`left-0000.png` … `left-0112.png`), an `ost.txt`, and critically a file `C270_Calibration.yaml` with:
   ```yaml
   image_width: 640
   image_height: 480
   camera_matrix:
     data: [922.09704, 0, 351.91185, 0, 919.39416, 282.5806, 0, 0, 1]
   distortion_model: plumb_bob
   distortion_coefficients:
     data: [0.151560, -0.812922, 0.005514, 0.017647, 0.000000]
   ```
   - The USB drive had to be mounted manually on the Raspberry Pi (`lsblk` showed it unmounted as `/dev/sda1`; `sudo mkdir -p /mnt/usb && sudo mount /dev/sda1 /mnt/usb`).
   - A brief false alarm occurred where the user initially seemed to be testing on the laptop webcam while trying to apply the C270-specific calibration file — Claude flagged that camera intrinsics are camera-specific and cannot be transplanted across different physical cameras/lenses. This was resolved once the user clarified they actually were running on the real C270 on the Pi already.
   - The calibration file was copied into the ROS2 package properly (`~/negativeJ3k_ws/src/jenga_vision/config/C270_Calibration.yaml`) instead of being read from the removable USB, since the USB might not always be mounted and mount points aren't guaranteed persistent across reboots.
   - **Important ROS2 packaging lesson learned**: `colcon build` only installs Python files declared via `packages=find_packages(...)` into the install-space `site-packages` directory. Arbitrary data folders like `config/` are **not** copied automatically. This caused a `FileNotFoundError` on first attempt to load the YAML (the code used a relative path derived from `__file__`, which pointed into the install-space `site-packages` folder — a folder with no `config/` subfolder at all).
   - Fixed properly by (a) adding to `setup.py`:
     ```python
     import os
     from glob import glob
     ...
     data_files=[
         ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
         ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
     ],
     ```
     and (b) loading the file in code via the canonical ROS2 API instead of a hand-rolled relative path:
     ```python
     from ament_index_python.packages import get_package_share_directory
     config_path = os.path.join(get_package_share_directory('jenga_vision'), 'config', 'C270_Calibration.yaml')
     ```
   - `camera_matrix['data']` (a flat 9-element list) was reshaped and `[0]` (i.e. `fx`) was extracted and used to replace the old hand-estimated `focal_length_px = 94.4` with the real `fx = 922.097` from the calibration file.

4. **`known_width_mm` recalibration** — even with the real `fx`, distance readings were still off. The user re-measured the block and insisted it really was 15 mm wide (contradicting an earlier, coincidentally-closer-seeming value of 18.2 mm that had been back-solved algebraically from test data). Rather than argue from theory alone, two real data points at two different distances (200 mm and 400 mm, both taken *after* morphological cleanup was already in place) were compared:
   - At 200 mm, measured `w ≈ 87px` → back-solved `known_width ≈ 18.87 mm`
   - At 400 mm, measured `w ≈ 38–44px` (mean ≈ 40.6) → back-solved `known_width ≈ 17.6 mm`
   
   These two independently-derived values were close to each other (~18–19 mm) and consistent across a 2× distance range, which is strong evidence of a genuine, distance-independent scale-calibration issue rather than measurement noise. The working theory was that the physical ruler measurement (15 mm) was taken on the **wrong face/dimension** of the non-square wooden block (e.g. its thickness rather than the face the camera actually sees). `known_width_mm` was set to `18.2` as a working compromise.

5. **Lens distortion / `cv2.undistort()`**: added using the previously-loaded `camera_matrix` and `dist_coeffs`:
   ```python
   undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
   ```
   Applied consistently to *all* downstream uses of the frame (both `detection_blue_object()` input and every published image), rather than only for detection — a deliberate design choice made so that what's seen in `rqt_image_view` matches exactly what the detection pipeline operates on.
   - Result: **at 200 mm true distance, measured distance came out to 196.5–203.4 mm (~±3.5% error)** — the best accuracy achieved with the known-width method. At 400 mm, error grew to roughly +8% to +14% overshoot (432–455 mm measured), with wider frame-to-frame variance — attributed to `w` being small at long range (~38–44 px), so single-pixel noise translates into a larger percentage error. This is an inherent limitation of the known-width/single-dimension approach, not a remaining calibration bug.

6. **A brief detour**: user asked whether compressed-transport (JPEG) was introducing the noise seen in the `rqt_image_view` mask display. Claude clarified this is **not** possible for the distance-calculation bug, because `w`/`x`/`y` are computed from the raw in-memory `closing` numpy array *before* it is ever JPEG-encoded for publishing — the compressed image is purely for human monitoring and has zero causal effect on the detection math. (It was acknowledged that JPEG compression of a binary mask can itself introduce cosmetic block-edge artifacts purely in the *displayed* image, which is a separate, non-functional concern.)

### Phase 6 — Switching to `cv2.solvePnP` for full 3D pose (current, most accurate method)

- The user consulted a senior teammate and brought back a detailed writeup (verbatim content preserved in chat) describing a `solvePnP`-based approach, including handling the case where the camera sees **two faces** of the block at once (silhouette becomes a hexagon, requiring a "plane separation" pre-processing step via either geometric corner analysis or Canny-edge-based plane-boundary detection).
- Claude's assessment: since the camera in this setup only ever sees a **single flat face** of the block (verified from the consistently-rectangular mask shapes seen throughout the whole session), the hexagon-splitting "Plane Separation" step described in the senior's notes is **unnecessary overhead** — `cv2.approxPolyDP` on the single-face contour already yields exactly 4 corners directly, skipping straight to PnP.
- Implementation steps, in order:
  1. `perimeter = cv2.arcLength(largest_contour, True)`; `epsilon = 0.02 * perimeter`; `approx_corners = cv2.approxPolyDP(largest_contour, epsilon, True)`. Verified empirically that `epsilon = 0.02` (2% of perimeter) reliably yields **4** corners for this block/camera setup (confirmed via repeated `Corner founded: 4` log lines).
  2. Wrote `sorting_conner(self, corners)` to consistently order the 4 (unordered, contour-traversal-order) points into a clockwise TL→TR→BR→BL sequence, using the classic trick:
     ```python
     pts = corners.reshape(4, 2)
     Sum = pts.sum(axis=1)
     Top_L = pts[np.argmin(Sum)]
     Bottom_R = pts[np.argmax(Sum)]
     diff = np.diff(pts, axis=1)
     Top_R = pts[np.argmin(diff)]
     Bottom_L = pts[np.argmax(diff)]
     return Top_L, Top_R, Bottom_R, Bottom_L
     ```
     (Note: initial version returned `Top_L, Top_R, Bottom_L, Bottom_R` — non-contiguous/non-clockwise ordering — corrected to `Top_L, Top_R, Bottom_R, Bottom_L` for proper clockwise traversal matching `object_points`' ordering.)
  3. **Crash discovered and fixed**: occasionally `approxPolyDP` returned 5 corners instead of 4 for a given frame (contour noise), causing `sorting_conner`'s `corners.reshape(4, 2)` to throw `ValueError: cannot reshape array of size 10 into shape (4,2)` and crash the whole node (`ExternalShutdownException` / traceback to `_spin_once_impl`). Root cause pinpointed precisely: `sorting_conner()` was being called **before** the `Corner founded: {count}` log line executed, so the crash always happened silently before any diagnostic log for the bad frame could print — this is why the log history only ever showed "4" despite the crash happening on a non-4 frame. Fixed with an explicit guard:
     ```python
     approx_corners = cv2.approxPolyDP(largest_contour, epsilon, True)
     conner_count = len(approx_corners)
     self.get_logger().info(f'Conner founded: {conner_count}')
     if conner_count != 4:
         x, y, w, h = cv2.boundingRect(largest_contour)
         return x, y, w, h, closing, None   # ordered_corners=None signals "skip PnP this frame"
     ordered_corners = self.sorting_conner(approx_corners)
     ```
     `timer_callback` correspondingly checks `if ordered_corners is not None:` before attempting `solvePnP`.
  4. Defined `object_points` with the origin placed at the **center of the block's face** (not a corner), on the `Z=0` plane, e.g.:
     ```python
     half_w = self.known_width_mm / 2
     half_h = self.known_height_mm / 2
     self.object_points = np.array([
         [-half_w,  half_h, 0],   # top-left
         [ half_w,  half_h, 0],   # top-right
         [ half_w, -half_h, 0],   # bottom-right
         [-half_w, -half_h, 0],   # bottom-left
     ], dtype=np.float32)
     ```
     Choosing the centroid as the origin was a deliberate design decision: it means `solvePnP`'s output `tvec` directly gives the 3D position of the block's **center**, with no separate diagonal-intersection/midpoint math needed — the geometry solve does this implicitly. This directly answered the user's earlier stated goal ("find the corners, draw diagonals, find the center point, use that for distance") — `solvePnP` accomplishes the same result more robustly, using all 4 points simultaneously rather than 2 diagonals.
  5. In `timer_callback`:
     ```python
     if ordered_corners is not None:
         image_points = np.array(ordered_corners, dtype=np.float32)
         success, rvec, tvec = cv2.solvePnP(
             self.object_points, image_points, self.camera_matrix, self.dist_coeffs
         )
         if success:
             self.get_logger().info(f'x(tvec0) = {tvec[0]}, y(tvec1) = {tvec[1]}, z(tvec2) = {tvec[2]}')
     ```
- **Real-world validation result (best of the session)**: at a true measured distance of ~310 mm, `solvePnP`'s `tvec[2]` consistently read **310–320 mm**, with a very tight jitter band (never below 310, never above 320) — a ~0–3% error, and clearly outperforming the known-width method's ~10%+ error at comparable/longer ranges.
- Known loose end in the current code at session's end: `known_width_mm` was reset back to `15` (the directly-measured value) for the `solvePnP`-based `object_points`, on the reasoning that PnP's full geometric solve should be more forgiving of/less sensitive to a small measurement discrepancy than the older simple ratio method was — this held up in the 310 mm test. `known_height_mm` is set to `75` (variable name has a typo, `know_height_mm`, missing the `n`) but its provenance (measured directly vs. assumed from "standard Jenga block" dimensions) was **not explicitly confirmed** by the user before the session ended — flagged as a to-do.

## Current State of the Codebase (end of session)

**File:** `~/negativeJ3k_ws/src/jenga_vision/jenga_vision/camera_node2.py`

**Publishers created in `__init__`:**
- `raw_publisher_` → `/videoframe` (`sensor_msgs/Image`, hand-serialized, no cv_bridge)
- `publisher_` → `/videoframe/compressed` (`sensor_msgs/CompressedImage`, JPEG)
- `mask_publisher_` → `/mask/compressed` (`sensor_msgs/CompressedImage`, JPEG, publishes the post-morphology `closing` mask for remote debugging)
- 15 Hz timer (`1.0/15.0` period)

**Loaded once in `__init__` from `config/C270_Calibration.yaml`** (via `get_package_share_directory`):
- `self.camera_matrix` (3×3 numpy array, reshaped from flat YAML data)
- `self.dist_coeffs` (1D numpy array, 5 plumb_bob coefficients)
- `self.focal_length_px = camera_matrix_data[0]` (i.e. fx = 922.097 — still retained/used for the legacy known-width distance calc, which is currently computed in parallel with PnP but not the primary output)
- `self.known_width_mm = 15`
- `self.know_height_mm = 75` (typo in name, not yet renamed to `known_height_mm`)
- `self.object_points` (4×3 float32 array, centroid-origin, Z=0 plane)

**`detection_blue_object(self, frame)` pipeline:**
1. BGR→HSV, `cv2.inRange` with `lower_blue=(100,100,50)`, `upper_blue=(130,255,255)` — **not yet tuned** against real competition-field lighting; still the rough values chosen early in the session on a laptop webcam.
2. Morphological OPEN then CLOSE (5×5 rectangular kernel) → `closing`
3. `cv2.findContours` (RETR_EXTERNAL) on `closing`; if no contours, returns `(None, None, None, None, closing, None)`
4. `cv2.boundingRect` on the largest contour → `x, y, w, h`
5. `cv2.approxPolyDP` (epsilon = 2% of perimeter) → corner count logged every frame
6. If corner count ≠ 4 → returns `(x, y, w, h, closing, None)` (skips PnP for this frame, but boundingRect-based data still returned)
7. Else → `sorting_conner()` → returns `(x, y, w, h, closing, ordered_corners)`

**`timer_callback(self)`:**
1. Reads frame, logs its shape every frame (`(480, 640, 3)`, confirmed to match the calibration file's declared resolution)
2. `cv2.undistort()` immediately after capture — **all** downstream processing and publishing uses this undistorted frame, not the raw one
3. Publishes raw `/videoframe`
4. Calls `detection_blue_object`, publishes the returned mask to `/mask/compressed`
5. If `ordered_corners is not None`: runs `cv2.solvePnP`, logs `tvec` components on success
6. Separately (legacy path, still active): if `w > 0`, computes and logs `distance_mm = (known_width_mm * focal_length_px) / w` — this is the older, less-accurate known-width estimate, currently running **in parallel** with the new PnP-based estimate rather than having been removed
7. Publishes `/videoframe/compressed`

**Known TODOs / not-yet-cleaned-up items in the file:**
- Several commented-out earlier versions of `timer_callback` and `frame_to_imgmsg` usage are left in as comments (harmless but could be deleted for cleanliness)
- `know_height_mm` → should be renamed `known_height_mm` for naming consistency
- The legacy known-width `distance_mm` calculation is still computed/logged every frame alongside the new PnP result; not yet decided whether to keep both, keep PnP only, or use known-width as a PnP sanity-check/fallback
- HSV thresholds are not yet tuned for actual competition-field lighting conditions (this was explicitly deprioritized mid-session in favor of finishing distance calibration first, and was never returned to)
- No temporal smoothing (e.g. exponential moving average) has been implemented yet, despite being discussed as a planned improvement early in the session; morphological cleanup and then PnP ended up being sufficient to reach acceptable accuracy without it being strictly necessary so far — but it may still help for a live IBVS control loop where the robot is physically moving (and thus the target's distance/angle in-frame changes at a nontrivial velocity between reads).

## Planned Coordinate Mapping (for downstream low-level control, discussed but not yet implemented)

The robot's low-level controller expects `x_robot`, `y_robot`, and `yaw` (heading correction) values. Given the side-mounted camera geometry, the proposed mapping (not yet verified against the real physical mounting orientation) is:

- **Camera Z (depth/distance, i.e. `tvec[2]`)** → **Robot X** (forward/backward axis — how far the robot needs to drive in/out)
- **Camera X (horizontal pixel/world offset, i.e. `tvec[0]`)** → **Robot Y** (lateral axis — how far the robot needs to strafe sideways)
- **Yaw** → heading correction, so the robot can rotate to square itself up with the block before driving in

**Important clarification the user gave near the end of the session**, which changes what "yaw" means here: the user does **not** want the in-plane rotation/tilt of the detected rectangle in the 2D image (which is what a naive `minAreaRect`-angle approach would give). They want **the angle between the robot's heading and the physical plane/face of the wooden block** (i.e., "if the robot is not driving in perpendicular to the wood's face, how many degrees off is it") — so that the robot can first rotate to square itself perpendicular to the block's face, and only then drive straight in by the previously-computed distance.

**Approach discussed for this (not yet implemented/tested at session's end):**
Since `object_points` are defined on the `Z=0` plane, the object's own surface-normal vector in its own coordinate frame is trivially `[0, 0, 1]`. Applying the rotation found by `solvePnP` (via `rvec`) to this normal vector transforms it into the camera's coordinate frame, revealing which way the block's face is actually "pointing" relative to the camera's optical axis:

```python
R, _ = cv2.Rodrigues(rvec)
normal_in_camera = R[:, 2]   # 3rd column of R = the object's Z-axis (face normal), expressed in camera coordinates
yaw_error = np.arctan2(normal_in_camera[0], normal_in_camera[2])
yaw_error_deg = np.degrees(yaw_error)
```
- If the block's face is exactly perpendicular to the camera (robot already squared up), `normal_in_camera` points straight along the camera's Z-axis, giving `yaw_error ≈ 0`.
- Any tilt in the face shows up as a deviation of the normal away from Z, mostly along X for a left/right tilt — `arctan2(x, z)` extracts exactly this deviation angle, in the horizontal plane, which matches how a robot's yaw rotation (about its vertical axis) would need to compensate.
- **Not yet implemented, sign convention not yet verified against the robot's actual rotation direction convention** (clockwise-positive vs counter-clockwise-positive needs to be checked once wired up).

## Full Next-Steps List (in suggested priority order)

1. Implement and empirically test the `cv2.Rodrigues(rvec)` + `arctan2` yaw-error calculation above; verify sign convention by manually tilting the block a known direction and confirming the computed sign matches the expected robot rotation direction.
2. Fix the `know_height_mm` → `known_height_mm` typo for naming consistency with `known_width_mm`.
3. **Explicitly verify** `known_height_mm = 75` — confirm whether this was actually measured with a ruler on the real block, or just assumed from "standard Jenga block" dimensions (75mm is the standard Jenga block *length*, not necessarily the height as seen face-on by this camera setup — this exact kind of face/dimension confusion already caused significant debugging time earlier in the session with `known_width_mm`, so it's worth double-checking before trusting it).
4. Decide whether to keep the legacy known-width `distance_mm` calculation running (e.g. as a PnP sanity-check/fallback for frames where corner detection fails) or remove it now that PnP is clearly more accurate.
5. Implement temporal smoothing (e.g. an exponential moving average on `tvec`, or on `w`/distance) if jitter becomes a problem once this is running in a live control loop with the robot physically moving.
6. Design and implement a custom ROS2 message (e.g. a `.msg` file with fields like `x`, `y`, `yaw`, `distance`, `target_valid`) to cleanly publish the final control-relevant outputs to the low-level controller, rather than relying on ad-hoc log lines — this was the original plan discussed very early in the session (before any code was written) but has not yet been revisited.
7. Verify the camera→robot coordinate mapping sign conventions (which physical direction is "positive X robot", etc.) against the real robot once mounted, before wiring this into a live IBVS control loop.
8. Tune the HSV thresholds (`lower_blue`/`upper_blue`, currently `(100,100,50)`–`(130,255,255)`) against actual competition-field lighting conditions — this was explicitly deprioritized mid-session in favor of finishing distance/pose calibration first, using a `cv2.imshow`-based interactive trackbar tuner was suggested as the fast way to do this once revisited.
9. Consider cleaning up the file: remove large blocks of commented-out legacy code (old `timer_callback` versions, unused `frame_to_imgmsg` raw-image publishing path if it ends up unneeded going forward).
