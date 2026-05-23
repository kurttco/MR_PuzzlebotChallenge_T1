# MR_PuzzlebotChallenge_T1
Manchester's Robotics Puzzlebot Challenge - Class TE3002B

# PuzzleBot Closed-Loop Navigation with Line Following and Traffic Light Perception

> ROS 2 package for autonomous navigation of the Manchester Robotics PuzzleBot.
> Covers closed-loop PID waypoint following, vision-based traffic light response,
> and camera-based line following. Developed for the TE3002B (Mobile Robotics)
> course at Tecnologico de Monterrey.

---

## Table of contents

- [What this package does](#what-this-package-does)
- [Hardware](#hardware)
- [Software requirements](#software-requirements)
- [System architecture](#system-architecture)
- [Repository structure](#repository-structure)
- [Project progression](#project-progression)
- [Installation](#installation)
- [Building the workspace](#building-the-workspace)
- [Running the system](#running-the-system)
- [Key configuration parameters](#key-configuration-parameters)
- [HSV calibration workflow](#hsv-calibration-workflow)
- [Generating report plots from a run](#generating-report-plots-from-a-run)
- [General considerations](#general-considerations)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)

---

## What this package does

The PuzzleBot is a small differential-drive robot with a Jetson Nano 2GB running
ROS 2 Humble, an ESP32-based motor driver board (Hackerboard) that exposes raw
wheel velocity setpoints, and an onboard camera.

This package, named `puzzlebot_mc2`, provides:

1. **Closed-loop waypoint navigation (MC2/MC3).** A PID controller drives the
   robot through a sequence of `(x, y)` waypoints defined in a YAML file. Two
   control strategies are available: sequential (rotate then move) and
   simultaneous (rotate and move at the same time).
2. **Traffic light perception (MC4).** A separate node processes the camera
   stream and detects red, yellow and green colored circles. The controller
   subscribes to this detection and modulates its output: immediate full stop on
   red (with a latch that persists until green is confirmed), reduced linear
   velocity on yellow, normal operation on green.
3. **Line following (MC5).** A new vision node detects the center of a black
   line track using connected-component analysis and image moments. The controller
   runs a PID on the normalized pixel error to keep the robot centered between
   the lane lines. Traffic light integration is fully active during line following.

---

## Hardware

| Component | Notes |
|---|---|
| Manchester Robotics PuzzleBot | Differential drive, wheel diameter 9 cm, wheel base 18 cm |
| Jetson Nano 2GB | Runs ROS 2 Humble on Ubuntu 20.04 |
| Hackerboard (ESP32) | Motor driver. Talks to ROS via `micro_ros_agent serial --dev /dev/ttyUSB0` |
| Onboard camera | Streamed through the `ros_deep_learning` pipeline |

Empirical calibration constants (set in `config/robot_params.yaml`):

- `wheel_radius`: 0.045 m
- `wheel_base`: 0.18 m
- `k_lin`: 0.08 m/s per unit command
- `k_ang`: 0.561 rad/s per unit command
- `encoder_scale`: 1.1 (corrects ~10% odometry underestimation)

---

## Software requirements

- Ubuntu 20.04 LTS
- ROS 2 Humble
- `colcon`, `ament_cmake`, `rosidl_default_generators`
- Python 3.8+
- OpenCV 4.x (`python3-opencv`)
- `cv_bridge` (`ros-humble-cv-bridge`)
- NumPy
- `matplotlib` (offline plot script only, not needed at runtime)
- `ros_deep_learning` package for the camera pipeline (must publish `/video_source/raw`)

---

## System architecture

### MC5 (line following + traffic light)

```
                    +--------------------------+
                    |   camera (CSI)           |
                    +------+-------------------+
                           | /video_source/raw
              +------------+-----------+
              v                        v
   +---------------------+   +-------------------------+
   |   line_detector     |   | traffic_light_detector  |
   | (moments + CC)      |   | (HSV + debounce)        |
   +----------+----------+   +------------+------------+
              | /line_error               | /semaphore_state
              | /line_detected            |
              +----------+---------------+
                         v
              +----------------------------------+
              | controller (line_following mode) |
              | PID on pixel error + semaphore   |
              +---------------+------------------+
                              | /cmd_vel
                              v
                   +---------------------+
                   |   cmd_vel_bridge    |
                   +---------+-----------+
                             | Float32 L, R
                             v
                   +---------------------+
                   | firmware (ESP32)    |
                   +---------------------+
```

### Topics (MC5 additions)

| Topic | Type | Direction |
|---|---|---|
| `/line_error` | `std_msgs/Float32` | line_detector -> controller |
| `/line_detected` | `std_msgs/Bool` | line_detector -> controller |
| `/line_debug` | `sensor_msgs/Image` | line_detector (optional debug) |

All MC2/MC3/MC4 topics remain active and unchanged.

---

## Repository structure

```
puzzlebot_mc2/
+-- README.md
+-- package.xml
+-- CMakeLists.txt
+-- msg/
|   +-- Goal.msg
|   +-- SemaphoreState.msg
+-- config/
|   +-- robot_params.yaml
|   +-- controller_params.yaml           MC3 (semaphore off)
|   +-- controller_params_mc4.yaml       MC4 (semaphore on, waypoints)
|   +-- controller_params_mc5.yaml       MC5 (line following mode)    [NEW]
|   +-- vision_params.yaml               HSV ranges + shape filters
|   +-- line_detector_params.yaml        ROI, threshold, blob filters [NEW]
|   +-- waypoints_square.yaml
|   +-- waypoints_custom.yaml
|   +-- waypoints_mc4.yaml
+-- scripts/
|   +-- odometry_node.py
|   +-- cmd_vel_bridge.py
|   +-- controller.py                    updated: adds line_following mode
|   +-- path_generator.py
|   +-- traffic_light_detector.py        updated: adds debounce for all colors
|   +-- line_detector.py                 NEW: connected-component line detector
|   +-- hsv_calibrator.py
|   +-- plot_mc4.py
|   +-- analyze.py
+-- launch/
    +-- mc2_part1_square.launch.py
    +-- mc2_part2_waypoints.launch.py
    +-- mc4_full.launch.py
    +-- mc5_line_following.launch.py      NEW
```

---

## Project progression

### Mini Challenge 2 -- open-loop motion

Timed velocity commands to the wheels using empirically calibrated kinematic
constants (`k_lin`, `k_ang`). No feedback. Validated the base stack:
Jetson + Hackerboard + micro-ROS bridge.

### Mini Challenge 3 -- closed-loop PID navigation

The closed loop was introduced:

- Odometry: integrates encoder velocities into pose (x, y, theta).
- PID controller with FSM (IDLE -> ROTATE_TO_GOAL -> MOVE_TO_GOAL -> HOLD),
  two selectable control strategies.
- Path generator: loads waypoints from YAML, validates reachability, publishes
  goals one at a time.
- Custom message `Goal.msg` based on `geometry_msgs/Pose`.

Key issues solved: QoS mismatch (BEST_EFFORT encoder topics), Hackerboard
resets from current spikes (slew rate limiter), 28 cm overshoot (approach
zone + encoder_scale).

### Mini Challenge 4 -- vision-based traffic light response

- `traffic_light_detector` node: HSV segmentation, morphological cleanup,
  contour filtering by area, circularity, and aspect ratio.
- `SemaphoreState.msg` custom message.
- `hsv_calibrator.py`: interactive OpenCV window with six HSV trackbars.
- Controller integration: `enable_semaphore` parameter. Semaphore override
  applied at publish stage, after the PID.
- Asymmetric red latch: single red reading stops the robot; several consecutive
  green readings needed to release.

### Mini Challenge 5 -- line following

Camera-based lane keeping added alongside the existing navigation stack:

- `line_detector.py` (new): crops a horizontal ROI from the bottom of the
  camera frame, applies adaptive thresholding so dark lines become white blobs,
  runs `cv2.connectedComponentsWithStats`, filters blobs by area and
  bounding-box aspect ratio (rejects flat crosswalk dashes), averages the
  surviving centroids, and publishes a normalized error in [-1, +1].

- `controller.py` (extended): new `control_mode: "line_following"` branch.
  Angular PID drives omega from the pixel error. Linear speed reduces
  automatically when the error is large. Line-lost handling: 0.4 s grace period
  holding the last angular command, then full stop.

- `traffic_light_detector.py` (updated): new `debounce_frames` parameter
  requires N consecutive frames of the same raw color before updating the
  confirmed published state. Eliminates single-frame false positives. Debug
  image now shows a live progress bar of the streak count.

- `mc5_line_following.launch.py` (new): launches odometry, bridge, controller,
  line_detector, and traffic_light_detector. Does not launch path_generator.

---

## Installation

```bash
cd ~/ros2_ws/src
git clone <repo url> puzzlebot_mc2
cd puzzlebot_mc2
chmod +x scripts/*.py
sudo apt update
sudo apt install ros-humble-cv-bridge python3-opencv python3-numpy
pip3 install matplotlib
```

---

## Building the workspace

```bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_mc2 --symlink-install
source install/setup.bash
```

Verify custom messages:

```bash
ros2 interface show puzzlebot_mc2/msg/Goal
ros2 interface show puzzlebot_mc2/msg/SemaphoreState
```

---

## Running the system

Always start the micro-ROS bridge first:

```bash
sudo chmod 666 /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### MC3 -- 2 m square

```bash
ros2 launch puzzlebot_mc2 mc2_part1_square.launch.py
```

### MC3 -- arbitrary waypoint path

```bash
ros2 launch puzzlebot_mc2 mc2_part2_waypoints.launch.py
```

### MC4 -- waypoints + traffic light

```bash
ros2 launch ros_deep_learning video_source.ros2.launch  # terminal 1
ros2 launch puzzlebot_mc2 mc4_full.launch.py            # terminal 2
```

### MC5 -- line following, Part 1 (no traffic light)

Set `enable_semaphore: false` in `config/controller_params_mc5.yaml`, then:

```bash
ros2 launch ros_deep_learning video_source.ros2.launch  # terminal 1
ros2 launch puzzlebot_mc2 mc5_line_following.launch.py  # terminal 2
```

### MC5 -- line following + traffic light, Part 2

Set `enable_semaphore: true` in `config/controller_params_mc5.yaml`, then run
the same launch as above.

Monitor debug images in rqt_image_view:
- `/line_debug` -- detected blobs, centroid line, error value
- `/traffic_light_debug` -- detected contours, confirmed state, debounce bar

---

## Key configuration parameters

### `controller_params_mc5.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `kp_line` | 0.5 | Proportional gain on pixel error -- tune first |
| `ki_line` | 0.0 | Integral -- add only for persistent lateral offset |
| `kd_line` | 0.05 | Derivative -- reduces oscillation on straights |
| `v_line_base` | 0.10 m/s | Forward speed when line is centered |
| `speed_curve_factor` | 0.5 | Speed reduction: v = v_base*(1 - factor*|error|) |
| `line_lost_timeout_s` | 0.4 | Grace period before full stop when line is missing |
| `enable_semaphore` | false | Set true for Part 2 |
| `yellow_speed_scale` | 0.4 | Speed multiplier on yellow |
| `green_frames_to_release` | 3 | Consecutive green frames to release red latch |

### `line_detector_params.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `roi_top_ratio` | 0.55 | Top of ROI as fraction of image height |
| `roi_bottom_ratio` | 0.85 | Bottom of ROI as fraction of image height |
| `threshold_c` | 8 | Adaptive threshold constant -- most impactful, tune first |
| `threshold_block_size` | 31 | Adaptive threshold neighborhood size (must be odd) |
| `min_blob_area` | 300 px | Minimum blob area to be considered a line segment |
| `min_aspect_ratio` | 0.8 | Min height/width -- rejects horizontal crosswalk dashes |

### `vision_params.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `red1_*`, `red2_*` | varies | Two HSV ranges for red (hue wrap-around) |
| `yellow_*`, `green_*` | varies | HSV ranges for yellow and green |
| `min_area_px` | 500 | Minimum contour area |
| `min_circularity` | 0.65 | Shape filter (perfect circle = 1.0) |
| `debounce_frames` | 5 | Frames required before confirming a color state [NEW] |

---

## HSV calibration workflow

```bash
ros2 launch ros_deep_learning video_source.ros2.launch  # terminal 1
ros2 run puzzlebot_mc2 hsv_calibrator.py                # terminal 2
```

Keyboard controls: `r` = red preset, `y` = yellow, `g` = green, `p` = print
values, `q` = quit. Adjust sliders until only the target color appears white in
the mask, then press `p` and copy values into `vision_params.yaml`.

---

## Generating report plots from a run

```bash
python3 ~/ros2_ws/src/puzzlebot_mc2/scripts/plot_mc4.py \
    /tmp/puzzlebot_logs/run_line_following_TIMESTAMP.csv
```

---

## General considerations

### Line following tuning order

1. Verify `/line_debug` shows correct blob detection before moving the robot.
2. Tune `threshold_c` first (most impactful vision parameter).
3. Tune `kp_line` until the robot tracks the line without oscillating.
4. Increase `v_line_base` gradually once steering is stable.
5. Adjust `speed_curve_factor` if the robot exits curves.
6. Enable `enable_semaphore: true` only after Part 1 is working reliably.

### Debounce for traffic light

`debounce_frames` in `vision_params.yaml` controls how many consecutive frames
of the same color must be detected before the confirmed state updates. At 15 Hz,
5 frames is about 0.33 s. Increase to 8 for noisy environments; decrease to 3
if the robot is slow to react to a real card.

### YAML files must be plain ASCII

ROS 2's YAML parser rejects non-ASCII characters. Keep all comments ASCII-only.

### CSV logs accumulate

Each run writes to `/tmp/puzzlebot_logs/`. Clean periodically with
`rm /tmp/puzzlebot_logs/*.csv`.

---

## Troubleshooting

### Robot does not start following the line

The controller requires at least one `/robot_pose` message before activating.
Check that odometry_node is publishing and encoders are connected:

```bash
ros2 topic hz /robot_pose
ros2 topic hz /VelocityEncL
```

### Line detector always reports detected=False

Check `/line_debug`. If nothing is detected: lower `threshold_c` toward 3-5.
If blobs are detected but rejected: lower `min_blob_area` or `min_aspect_ratio`.

### Robot stops at the intersection crosswalk

Increase `line_lost_timeout_s` from 0.4 to 0.6-0.8 s to extend the grace period.

### Traffic light causes false stops

Increase `debounce_frames` in `vision_params.yaml` to 8. Re-run HSV calibration
if the problem persists.

### Hackerboard resets mid-run

Lower `w_slew_rate` to 0.6 in `controller_params_mc5.yaml`.

### Odometry never updates

Check that the micro-ROS agent is running:

```bash
ros2 topic hz /VelocityEncL
ros2 topic hz /VelocityEncR
```

---

## Credits

- **Course**: TE3002B Mobile Robotics, Tecnologico de Monterrey.
- **Industry partner**: Manchester Robotics (PuzzleBot platform and challenge brief).
- **Team**: Equipo 1 -- REPO.

The PuzzleBot platform, the `ros_deep_learning` package, and the
`micro_ros_agent` setup are not part of this repository; they are part of the
official PuzzleBot system image provided by Manchester Robotics.
