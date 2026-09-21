# Jenga Vision Node 🎯

ROS2 camera node for real-time Jenga block detection and 3D pose estimation. Built for the **iRAP** robotics team's Jenga-block-stacking competition robot.

## Features

- **HSV Color Detection** — Detects blue or red colored blocks using configurable HSV thresholds
- **3D Pose Estimation** — Uses `cv2.solvePnP` with camera calibration data for accurate distance and position measurement
- **Camera Calibration** — Loads intrinsics from a YAML calibration file (Logitech C270)
- **Lens Distortion Correction** — Applies `cv2.undistort()` to all frames before processing
- **Morphological Filtering** — Open + Close operations to clean mask noise
- **Corner Detection** — `cv2.approxPolyDP` for 4-corner extraction with automatic sorting (TL→TR→BR→BL)
- **Low-pass Filtering** — Exponential moving average on position estimates for stable output
- **Dynamic Parameters** — Runtime-tunable HSV thresholds and color mode via ROS2 parameter API

## Hardware

| Component | Detail |
|-----------|--------|
| Camera | Logitech C270 (640×480) |
| Compute | Raspberry Pi |
| Robot | X-drive holonomic base with rotating gripper turret |
| Target | Wooden Jenga block (15mm × 75mm face) painted blue |

## Dependencies

- **ROS2 Humble**
- Python 3
- OpenCV (`cv2`)
- NumPy
- PyYAML
- `sensor_msgs` (Image, CompressedImage)
- `ament_index_python`

## ROS2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/videoframe` | `sensor_msgs/Image` | Raw undistorted camera frame |
| `/videoframe/compressed` | `sensor_msgs/CompressedImage` | JPEG compressed camera frame |
| `/mask/compressed` | `sensor_msgs/CompressedImage` | JPEG compressed detection mask (for debugging) |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `color_mode` | `blue` | Detection color mode (`blue` or `red`) |
| `h_min` | `100` | Blue hue minimum |
| `h_max` | `130` | Blue hue maximum |
| `s_min` | `100` | Saturation minimum |
| `s_max` | `255` | Saturation maximum |
| `v_min` | `50` | Value minimum |
| `v_max` | `255` | Value maximum |

## Usage

### Build (inside your ROS2 workspace)

```bash
colcon build --packages-select jenga_vision
source install/setup.bash
```

### Run

```bash
ros2 run jenga_vision camera_node
```

### View camera feed

```bash
# Compressed image (recommended — low bandwidth)
ros2 run rqt_image_view rqt_image_view --ros-args -p ~image_transport:=compressed

# View detection mask for debugging
# In rqt_image_view, select /mask/compressed topic
```

### Change color mode at runtime

```bash
ros2 param set /camera_node color_mode red
ros2 param set /camera_node color_mode blue
```

## Project Structure

```
jenga_vision_node/
├── vision_node.py                                  # Main ROS2 camera node
├── jenga_vision_session_summary_detailed_en.md     # Detailed development session log
├── .gitignore
└── README.md
```

## Detection Pipeline

```
Camera Frame
    │
    ▼
cv2.undistort()  ──── Camera Matrix + Distortion Coefficients
    │
    ▼
BGR → HSV
    │
    ▼
cv2.inRange()  ──── Blue or Red HSV thresholds
    │
    ▼
Morphological Open + Close  ──── 5×5 rectangular kernel
    │
    ▼
cv2.findContours()
    │
    ▼
Largest Contour → cv2.approxPolyDP()
    │
    ├── corners ≠ 4 → Skip PnP (return bounding box only)
    │
    └── corners == 4
            │
            ▼
        Sort corners (TL, TR, BR, BL)
            │
            ▼
        cv2.solvePnP()  ──── Object Points (15mm × 75mm face)
            │
            ▼
        tvec → X, Y, Z position (mm) with EMA filtering
```

## Team

- **Team:** negative-J3k
- **Developer:** Poorinut ("Gateaux") — Robotics and Automation Engineering, KMUTNB

## License

This project is part of the iRAP team's competition robot system.
