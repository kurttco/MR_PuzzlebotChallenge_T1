# MR_PuzzlebotChallenge_T1
Manchester's Robotics Puzzlebot Challenge - Class TE3002B

# PuzzleBot Closed-Loop Navigation with Traffic Light Perception and Line Following

> ROS 2 package for autonomous navigation of the Manchester Robotics
> PuzzleBot. Covers closed-loop PID waypoint following, vision-based
> traffic light response, and camera-guided line following.
> Developed for the TE3002B (Mobile Robotics) course at
> Tecnologico de Monterrey.

---

## Table of contents

- [What this package does](#what-this-package-does)
- [Hardware](#hardware)
- [Software requirements](#software-requirements)
- [System architecture](#system-architecture)
- [Repository structure](#repository-structure)
- [Project progression: from open loop to line following](#project-progression-from-open-loop-to-line-following)
- [Installation](#installation)
- [Building the workspace](#building-the-workspace)
- [Running the system](#running-the-system)
- [Key configuration parameters](#key-configuration-parameters)
- [Line following tuning workflow](#line-following-tuning-workflow)
- [HSV calibration workflow](#hsv-calibration-workflow)
- [Generating report plots from a run](#generating-report-plots-from-a-run)
- [General considerations](#general-considerations)
- [Troubleshooting](#troubleshooting)
- [Credits](#credits)

---

## What this package does

The PuzzleBot is a small differential-drive robot with a Jetson Nano 2GB
running ROS 2 Humble, an ESP32-based motor driver board (Hackerboard) that
exposes raw wheel velocity setpoints, and an onboard camera.

This package, named `puzzlebot_mc2`, provides:

1. **Closed-loop waypoint navigation.** A PID controller drives the robot
   through a sequence of `(x, y)` waypoints defined in a YAML file. Two
   control strategies are available (sequential or simultaneous) and the
   trajectory is validated for reachability before any motion command is
   issued.
2. **Traffic light perception.** A separate node processes the camera
   stream and detects red, yellow and green colored circles. The
   controller subscribes to this detection and modulates its output:
   immediate full stop on red (with a latch that persists until green is
   confirmed), reduced linear velocity on yellow, normal operation on
   green.
3. **Camera-guided line following.** A dedicated line detector node
   processes each camera frame using connected components analysis and
   image moments to locate the track centerline. The controller uses the
   resulting normalized pixel error as the setpoint for an angular PID,
   replacing odometry-based waypoint navigation. Traffic light
   integration remains active in this mode.

The package was developed progressively across four deliverables and
keeps the historical name `puzzlebot_mc2` even though it now implements
the deliverables for Mini Challenges 2 through 5.

---

## Hardware

| Component | Notes |
|---|---|
| Manchester Robotics PuzzleBot | Differential drive, wheel diameter 9 cm, wheel base 18 cm |
| Jetson Nano 2GB | Runs ROS 2 Humble on Ubuntu 20.04 |
| Hackerboard (ESP32) | Motor driver. Talks to ROS via `micro_ros_agent serial --dev /dev/ttyUSB0` |
| Onboard camera | Streamed through the `ros_deep_learning` pipeline |
| Printed track mat | Three parallel black lines on a tan background, including a crosswalk intersection zone |

Empirical calibration constants (already set in `config/robot_params.yaml`):

- `wheel_radius`: 0.045 m
- `wheel_base`: 0.18 m
- `k_lin`: 0.074 m/s (linear-velocity gain of the wheel setpoint command)
- `k_ang`: 0.561 rad/s (angular-velocity gain of the wheel setpoint command)
- `encoder_scale`: 1.1 (corrects a ~10% underestimation in odometry)

> **Important about the firmware**: the firmware on this PuzzleBot does
> not respond to the standard `/cmd_vel` topic. Wheel velocities have to
> be published directly to `/VelocitySetL` and `/VelocitySetR` as
> `Float32`. This package's `cmd_vel_bridge` node performs that
> translation, so the rest of the stack still uses the standard
> `geometry_msgs/Twist` interface.

---

## Software requirements

- Ubuntu 20.04 LTS
- ROS 2 Humble
- `colcon`, `ament_cmake`, `rosidl_default_generators`
- Python 3.8+
- OpenCV 4.x (`python3-opencv`)
- `cv_bridge` (`ros-humble-cv-bridge`)
- NumPy (for the controller's CSV processing)
- `matplotlib` (only for the offline plot script; not used at runtime on
  the robot)
- `ros_deep_learning` package, used for the camera pipeline. It must
  publish to `/video_source/raw`.

---

## System architecture

### Waypoint navigation (MC3 / MC4)

```
                          +--------------------------+
                          |       camera (CSI)       |
                          +-----------+--------------+
                                      | /video_source/raw
                                      v
                          +--------------------------+
                          |  traffic_light_detector  |
                          |  (HSV + circularity)     |
                          +-----------+--------------+
                                      | /semaphore_state
                                      v
  +--------------+  /goals  +-----------------------------+  /cmd_vel
  |path_generator|--------->|  controller (PID + FSM +    |--------+
  +------+-------+          |  semaphore override)        |        |
         ^                  +-------------+---------------+        |
         | /goal_reached                  | /robot_pose             |
         |                                ^                        v
         |                                |             +------------------+
         |                                |             |  cmd_vel_bridge  |
         |                                |             +--------+---------+
         |                                |                      | Float32 L,R
         |                       +--------+--------+             v
         |                       | odometry_node   |   +------------------+
         |                       +--------^--------+   | firmware (ESP32) |
         |                                | EncL, EncR | on Hackerboard   |
         |                                +------------+------------------+
         |
         +---------- full loop closed via odometry ------------------+
```

### Line following (MC5)

```
                          +--------------------------+
                          |       camera (CSI)       |
                          +-----------+--------------+
                                      | /video_source/raw
                          +-----------+--------------+
                          |   traffic_light_detector  |
                          +-----------+--------------+
                                      | /semaphore_state
                          +-----------+--------------+
                          |     line_detector         |
                          |  (threshold + connected   |
                          |   components + moments)   |
                          +-----------+--------------+
                            /line_error  /line_detected
                                      v
                          +-----------------------------+  /cmd_vel
                          |  controller                 |--------+
                          |  mode=line_following        |        |
                          |  (angular PID on px error   |        |
                          |   + semaphore override)     |        |
                          +-----------------------------+        |
                                                                 v
                                                      +------------------+
                                                      |  cmd_vel_bridge  |
                                                      +--------+---------+
                                                               | Float32 L,R
                                                               v
                                                      +------------------+
                                                      | firmware (ESP32) |
                                                      +------------------+
```

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/goals` | `puzzlebot_mc2/Goal` | path_generator -> controller |
| `/goal_reached` | `std_msgs/UInt32` | controller -> path_generator |
| `/robot_pose` | `geometry_msgs/PoseStamped` | odometry_node -> controller |
| `/cmd_vel` | `geometry_msgs/Twist` | controller -> cmd_vel_bridge |
| `/VelocitySetL`, `/VelocitySetR` | `std_msgs/Float32` | cmd_vel_bridge -> firmware |
| `/VelocityEncL`, `/VelocityEncR` | `std_msgs/Float32` | firmware -> odometry_node |
| `/video_source/raw` | `sensor_msgs/Image` | camera -> detectors |
| `/semaphore_state` | `puzzlebot_mc2/SemaphoreState` | traffic_light_detector -> controller |
| `/traffic_light_debug` | `sensor_msgs/Image` | traffic_light_detector (debug) |
| `/line_error` | `std_msgs/Float32` | line_detector -> controller |
| `/line_detected` | `std_msgs/Bool` | line_detector -> controller |
| `/line_debug` | `sensor_msgs/Image` | line_detector (debug) |

### Custom messages

- **`Goal.msg`** -- wraps `geometry_msgs/Pose` plus a unique `id` and a
  `is_last` flag. Required by the assignment to use a custom message
  based on `geometry_msgs/Pose`.
- **`SemaphoreState.msg`** -- carries the detected state
  (UNKNOWN / GREEN / YELLOW / RED), a normalized confidence, and the raw
  pixel area of the winning blob.

---

## Repository structure

```
puzzlebot_mc2/
|-- README.md
|-- package.xml
|-- CMakeLists.txt
|-- msg/
|   |-- Goal.msg
|   +-- SemaphoreState.msg
|-- config/
|   |-- robot_params.yaml
|   |-- controller_params.yaml          <- MC3 settings (semaphore off)
|   |-- controller_params_mc4.yaml      <- MC4 settings (semaphore on)
|   |-- controller_params_mc5.yaml      <- MC5 settings (line following)
|   |-- vision_params.yaml              <- HSV ranges + shape filters
|   |-- line_detector_params.yaml       <- line detection vision params
|   |-- waypoints_mc3_part1.yaml        <- 2 m square
|   |-- waypoints_mc3_part2.yaml        <- multi-waypoint path
|   +-- waypoints_mc4.yaml              <- 6 waypoints inside 2x2 m
|-- scripts/
|   |-- odometry_node.py
|   |-- cmd_vel_bridge.py
|   |-- controller.py                   <- MC5-aware, backwards compatible
|   |-- path_generator.py
|   |-- line_detector.py                <- NEW: camera line detection
|   |-- traffic_light_detector.py
|   |-- hsv_calibrator.py
|   |-- plot_mc4.py
|   +-- analyze.py
+-- launch/
    |-- mc3_part1_square.launch.py
    |-- mc3_part2_waypoints.launch.py
    |-- mc4_full.launch.py
    +-- mc5_line_following.launch.py     <- NEW
```

---

## Project progression: from open loop to line following

### Mini Challenge 2 -- open-loop motion

The starting point was issuing timed velocity commands directly to the
wheels, with the kinematic constants (`k_lin`, `k_ang`) calibrated
empirically. There was no feedback. This iteration validated the basic
stack (Jetson + Hackerboard + micro-ROS bridge) and produced the first
versions of `cmd_vel_bridge` and the wheel-velocity calibration.

### Mini Challenge 3 -- closed-loop PID navigation

The closed loop was introduced with an odometry node, a PID controller
with FSM (IDLE -> ROTATE_TO_GOAL -> MOVE_TO_GOAL -> HOLD), and a path
generator that validates waypoints before any motion. This iteration
also solved three key issues: QoS mismatch between micro-ROS encoder
topics and rclpy defaults (fixed with `qos_profile_sensor_data`),
Hackerboard resets from current spikes (fixed with a slew rate limiter),
and ~28 cm overshoot (fixed with an approach zone and `encoder_scale`).

### Mini Challenge 4 -- vision-based traffic light response

A `traffic_light_detector` node was added using HSV segmentation,
morphological filtering, and circularity-based blob detection. The
controller was extended with an asymmetric red latch: one red frame
latches the robot stopped; multiple consecutive green frames are
required to release it. The semaphore override is applied at the
publish stage, leaving the PID, FSM, and slew rate limiter unchanged.

### Mini Challenge 5 -- camera-guided line following

The camera becomes the primary feedback sensor. A new `line_detector`
node processes each frame using the following pipeline:

1. Crop a configurable horizontal ROI from the bottom of the frame.
2. Convert to grayscale and apply adaptive thresholding (handles uneven
   lighting across the mat without manual threshold calibration per room).
3. Clean the binary mask with morphological open and close operations.
4. Run `cv2.connectedComponentsWithStats` to label each connected white
   blob.
5. Filter blobs by area (rejects noise and floor bleed-through) and by
   bounding box aspect ratio height/width (rejects the horizontal dashed
   crosswalk lines which are much wider than tall in the ROI).
6. Average the centroid X positions of all valid blobs (up to one per
   track line) to get a single horizontal setpoint.
7. Normalize the error: `e = (cx_avg - W/2) / (W/2)`, range [-1, +1].

The controller gains a new `control_mode: "line_following"` branch. In
this mode the angular PID tracks the pixel error directly; linear speed
is set at a constant base value reduced proportionally to error magnitude
(natural slowdown in curves). Semaphore integration is unchanged: the
override is still applied at the publish stage after the PID runs, so
red/yellow/green behavior works identically to MC4.

A two-phase line-lost strategy protects against the crosswalk gap: during
a short grace period the robot holds its last angular command at reduced
speed, allowing the camera to re-acquire the line. After the grace period
the robot stops completely until the line is detected again.

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

Verify the custom messages compiled:

```bash
ros2 interface show puzzlebot_mc2/msg/Goal
ros2 interface show puzzlebot_mc2/msg/SemaphoreState
```

---

## Running the system

Before any run, start the micro-ROS bridge:

```bash
sudo chmod 666 /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### MC3 -- Part 1: 2 m square

```bash
ros2 launch puzzlebot_mc2 mc3_part1_square.launch.py
```

### MC3 -- Part 2: arbitrary waypoint path

```bash
ros2 launch puzzlebot_mc2 mc3_part2_waypoints.launch.py
```

### MC4 -- full system with traffic light perception

Start the camera pipeline first:

```bash
ros2 launch ros_deep_learning video_source.ros2.launch
```

Then:

```bash
ros2 launch puzzlebot_mc2 mc4_full.launch.py
```

### MC5 -- Part 1: line following (no traffic light)

Ensure `enable_semaphore: false` in `config/controller_params_mc5.yaml`.

```bash
# Terminal 1
ros2 launch ros_deep_learning video_source.ros2.launch

# Terminal 2
ros2 launch puzzlebot_mc2 mc5_line_following.launch.py
```

Monitor the line detector visually:

```bash
ros2 run rqt_image_view rqt_image_view
# subscribe to /line_debug
```

Monitor the error signal numerically:

```bash
ros2 topic echo /line_error
ros2 topic echo /line_detected
```

### MC5 -- Part 2: line following with traffic light

Set `enable_semaphore: true` in `config/controller_params_mc5.yaml`, then
launch identically to Part 1:

```bash
ros2 launch ros_deep_learning video_source.ros2.launch
ros2 launch puzzlebot_mc2 mc5_line_following.launch.py
```

The robot follows the line and responds to colored cards held in front of
the camera: red stops it (latched), yellow reduces forward speed by 60%,
green releases the latch and resumes normal speed.

---

## Key configuration parameters

### `robot_params.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `wheel_radius` | 0.045 m | Physical |
| `wheel_base` | 0.18 m | Physical |
| `k_lin`, `k_ang` | 0.074, 0.561 | Wheel-velocity gain constants |
| `encoder_scale` | 1.1 | Compensates ~10% odometry underestimation |
| `max_cmd` | 3.0 | Saturation of the Float32 firmware command |
| `left_gain`, `right_gain` | 1.2, 1.0 | Per-wheel trim for straight-line drift |
| `motor_deadzone` | 0.30 | Minimum command magnitude that moves the wheel |

### `controller_params_mc5.yaml` (line following)

| Parameter | Default | Meaning |
|---|---|---|
| `control_mode` | "line_following" | Activates the new FSM branch |
| `kp_line` | 0.5 | Proportional gain on normalized pixel error |
| `ki_line` | 0.0 | Integral gain (add only for persistent lateral drift) |
| `kd_line` | 0.05 | Derivative gain (raise to damp oscillation) |
| `v_line_base` | 0.10 m/s | Forward speed when line is centered |
| `speed_curve_factor` | 0.5 | Speed reduction factor in curves (0=no reduction, 1=full stop at max error) |
| `line_lost_timeout_s` | 0.4 s | Grace period before full stop when line is lost |
| `enable_semaphore` | false | Set true for Part 2 |
| `yellow_speed_scale` | 0.4 | Forward speed fraction during yellow |
| `green_frames_to_release` | 3 | Consecutive green frames needed to release red latch |
| `v_max` | 0.20 m/s | Hard actuator cap (slew rate applies on top) |
| `omega_max` | 0.45 rad/s | Hard angular actuator cap |

### `line_detector_params.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `roi_top_ratio` | 0.55 | Top of ROI as fraction of image height |
| `roi_bottom_ratio` | 0.85 | Bottom of ROI as fraction of image height |
| `blur_kernel_size` | 5 | Gaussian blur kernel (must be odd) |
| `threshold_block_size` | 31 | Adaptive threshold neighborhood size (must be odd) |
| `threshold_c` | 8 | Adaptive threshold offset -- tune this first |
| `morph_kernel_size` | 5 | Morphological cleanup kernel |
| `min_blob_area` | 300 px | Minimum blob area to be considered a line segment |
| `max_blob_area` | 50000 px | Maximum blob area (rejects full-floor detections) |
| `min_aspect_ratio` | 0.8 | Minimum height/width ratio (rejects horizontal crosswalk dashes) |
| `publish_debug_image` | true | Publish annotated frame on `/line_debug` |
| `publish_rate_hz` | 20.0 | Detection and publish rate |

### `vision_params.yaml` (traffic light detector)

Two groups of parameters:

- **HSV ranges**: `red1_*`, `red2_*` (red uses two ranges for hue wrap-around),
  `yellow_*`, `green_*`. Each color has six values (H/S/V min/max).
- **Shape filters**: `min_area_px`, `min_circularity`,
  `min_aspect_ratio`, `max_aspect_ratio`.

---

## Line following tuning workflow

Tune the vision detector before touching controller gains. A bad detector
cannot be compensated by any PID tuning.

### Step 1 -- validate the detector in isolation

```bash
# Terminal 1
ros2 launch ros_deep_learning video_source.ros2.launch

# Terminal 2
ros2 run puzzlebot_mc2 line_detector.py \
  --ros-args --params-file ~/ros2_ws/src/puzzlebot_mc2/config/line_detector_params.yaml

# Terminal 3
ros2 run rqt_image_view rqt_image_view   # subscribe to /line_debug
ros2 topic echo /line_error
```

The debug image shows a green ROI rectangle, yellow bounding boxes
around each detected blob, a magenta line at the averaged centroid, and
a white line at the image center. A text banner shows
`err: +0.123 | blobs: 3 | det: 1`.

**Good state:** three yellow boxes (one per track line), `/line_detected`
True consistently, magenta line near the white line when the robot is
centered on the track.

### Step 2 -- vision parameter symptoms and fixes

| Symptom | Likely cause | Fix |
|---|---|---|
| `det: 0` always | Threshold too tight | Lower `threshold_c` toward 3-5 |
| Dozens of tiny boxes | Too much noise passing | Raise `threshold_c` to 10-15, raise `min_blob_area` |
| Only 1-2 blobs instead of 3 | Lines merging, or outer lines outside ROI | Adjust `roi_top_ratio` / `roi_bottom_ratio` |
| Crosswalk dashes detected | Aspect ratio filter too loose | Raise `min_aspect_ratio` to 1.2-1.5 |
| Magenta line jumps erratically | Noise blobs included | Raise `min_blob_area`, tighten `min_aspect_ratio` |
| Lines lost in curves | ROI too narrow | Lower `roi_top_ratio` to 0.45-0.50 |

Tuning order: `threshold_c` first, then `min_blob_area`, then `roi_*`,
then `min_aspect_ratio` only if the intersection causes problems.

### Step 3 -- controller gain tuning

| Symptom | Fix |
|---|---|
| Robot slowly drifts off line | `kp_line` too low -- raise by 0.2 steps |
| Robot oscillates left-right | `kp_line` too high -- lower by 0.1 steps |
| Oscillates only on straights | Add `kd_line: 0.08-0.15` |
| Exits track in curves | Raise `speed_curve_factor` or lower `v_line_base` |
| Stops at intersection gap | Raise `line_lost_timeout_s` to 0.6-0.8 s |
| Persistent lateral drift on straights | Add `ki_line: 0.02-0.05` |

Tuning order: `kp_line` -> `v_line_base` -> `speed_curve_factor` ->
`kd_line` if oscillating -> `ki_line` only last.

---

## HSV calibration workflow

The default HSV ranges in `vision_params.yaml` are sensitive to lighting.
Calibrate under the actual lighting of the demo environment.

```bash
# Terminal 1
ros2 launch ros_deep_learning video_source.ros2.launch
# Terminal 2
ros2 run puzzlebot_mc2 hsv_calibrator.py
```

Keyboard controls in the calibration window:
- `r`, `y`, `g` -- load preset for red / yellow / green
- `p` -- print current values in YAML-ready format
- `q` -- quit

Procedure: load the color preset, hold the colored card in front of the
camera, adjust sliders until only that color appears bright in the mask,
press `p`, copy the values into `vision_params.yaml`. Repeat for each
color. For red, two ranges are needed due to hue wrap-around near H=0
and H=180.

---

## Generating report plots from a run

Every controller run with `log_csv: true` writes a CSV to
`/tmp/puzzlebot_logs/`. To generate figures from an MC4 run:

```bash
python3 ~/ros2_ws/src/puzzlebot_mc2/scripts/plot_mc4.py \
    /tmp/puzzlebot_logs/run_sequential_sem_XXXX.csv
```

Output files (in the same folder as the CSV):

- `01_trajectory.png` -- actual path colored by semaphore state
- `02_position_error.png` -- position error vs time
- `03_angle_error.png` -- angle error vs time
- `04_velocity_cmds.png` -- v and omega before and after semaphore override
- `05_semaphore_timeline.png` -- detected state over time with red latch
- `06_dashboard.png` -- combined summary figure
- `metrics.txt` -- total time, blocked time, mean/max error, smoothness

For MC5 line following runs, the `error_pos` column in the CSV stores
`|line_error|` and `error_ang` stores the signed `line_error`, so the
existing plot script can still produce useful error timeline figures.

---

## General considerations

### Lighting for the vision system

Calibrate both the traffic light detector and the line detector under
the same lighting conditions you plan to use for the demonstration.
Moving between rooms or changing from artificial to natural light will
require re-calibration of `threshold_c` (line detector) and HSV ranges
(traffic light detector).

### Camera placement and angle

The line detector relies on the bottom portion of the frame showing the
nearby floor. If the camera is tilted too far up it sees too much ceiling
and wall; too far down it sees only the track immediately under the robot
and cannot anticipate curves. The default ROI (55%-85% of frame height)
works for the standard PuzzleBot camera mount angle.

### Workspace size

The MC4 and MC5 launches assume a clear area around the track. The path
generator (MC3/MC4 only) validates waypoints against a bounding box, but
no such check exists for the line follower -- ensure the track area is
free of obstacles before starting.

### Safety

The slew rate limiter cannot stop the robot instantly. Keep a panic
terminal open during tuning:

```bash
ros2 topic pub --rate 20 /VelocitySetL std_msgs/msg/Float32 "{data: 0.0}" &
ros2 topic pub --rate 20 /VelocitySetR std_msgs/msg/Float32 "{data: 0.0}" &
```

### YAML files must be plain ASCII

ROS 2's `rcl_yaml_param_parser` rejects non-ASCII characters such as
accented letters, arrows, and degree symbols. All YAML files in this
package use plain ASCII. Keep comments ASCII-only if you add them.

### CSV logs accumulate

Each run appends a new file to `/tmp/puzzlebot_logs/`. Clean periodically:

```bash
rm /tmp/puzzlebot_logs/*.csv
```

---

## Troubleshooting

### Odometry never updates, robot only drives the first segment

QoS mismatch. Check that the encoders are publishing:

```bash
ros2 topic hz /VelocityEncL
ros2 topic hz /VelocityEncR
```

If there are no messages, the micro-ROS agent is not running or the
Hackerboard is unreachable. Verify the agent and `/dev/ttyUSB0`.

### Hackerboard reboots mid-run

Current spike from sharp velocity changes. Lower `v_slew_rate` and
`w_slew_rate`. Lowering `w_slew_rate` to 0.6 usually fixes angular
resets.

### Robot overshoots every waypoint (MC3/MC4)

Increase `encoder_scale` in steps of 0.05, then tune `approach_zone`.

### Line follower never starts moving

The controller waits for at least one `/robot_pose` message before
activating. Confirm the odometry node is running and publishing:

```bash
ros2 topic hz /robot_pose
```

If the topic is silent, check that the micro-ROS agent is connected and
encoder topics are active.

### Line detector reports `det: 0` even with the track clearly visible

The adaptive threshold is not isolating the black lines. Decrease
`threshold_c` (try values 3-6). If the floor texture is very similar to
the line color in grayscale, try lowering the ROI to a narrower band
closer to the robot and increase `blur_kernel_size` to 7 or 9.

### Robot stops at the crosswalk intersection

This is expected behavior when the center line gap is longer than
`line_lost_timeout_s`. Raise this parameter to 0.6-1.0 s. The two outer
lines remain continuous through the intersection and should keep
`line_detected` True on most frames; if they do not, lower
`min_blob_area` so narrower blobs are accepted.

### Robot oscillates on straights but tracks curves fine

This is an over-damped curve behavior combined with proportional overshoot
on straights. Add derivative gain: `kd_line: 0.08`. If oscillation
persists, lower `kp_line` by 0.1.

### Traffic light detector reports the wrong color or flickers

Almost always an HSV calibration issue. Re-run `hsv_calibrator.py` under
the actual lighting. If yellow is confused with red or green, narrow the
hue ranges. If false positives come from the background, raise
`min_area_px` and `min_circularity` in `vision_params.yaml`.

### `ros2 interface show puzzlebot_mc2/msg/SemaphoreState` fails after editing

Regenerate the custom message with a fresh build:

```bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_mc2
source install/setup.bash
```

---

## Credits

- **Course**: TE3002B Mobile Robotics, Tecnologico de Monterrey.
- **Industry partner**: Manchester Robotics (the PuzzleBot platform and
  challenge brief).
- **Team**: Equipo 1 -- REPO.

The PuzzleBot platform, the `ros_deep_learning` package, and the
`micro_ros_agent` setup are not part of this repository; they are part
of the official PuzzleBot system image provided by Manchester Robotics.
