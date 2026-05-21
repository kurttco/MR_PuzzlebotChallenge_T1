# MR_PuzzlebotChallenge_T1
Manchester's Robotics Puzzlebot Challenge - Class TE3002B

# PuzzleBot Closed-Loop Navigation with Traffic Light Perception

> ROS 2 package for autonomous navigation of the Manchester Robotics
> PuzzleBot. Covers closed-loop PID waypoint following and vision-based
> traffic light response. Developed for the TE3002B (Mobile Robotics)
> course at Tecnológico de Monterrey.

---

## Table of contents

- [What this package does](#what-this-package-does)
- [Hardware](#hardware)
- [Software requirements](#software-requirements)
- [System architecture](#system-architecture)
- [Repository structure](#repository-structure)
- [Project progression: from open loop to perception](#project-progression-from-open-loop-to-perception)
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

The package was developed progressively across three deliverables and
keeps the historical name `puzzlebot_mc2` even though it now implements
the deliverables for the professor's Mini Challenges 2 through 4.

---

## Hardware

| Component | Notes |
|---|---|
| Manchester Robotics PuzzleBot | Differential drive, wheel diameter 9 cm, wheel base 18 cm |
| Jetson Nano 2GB | Runs ROS 2 Humble on Ubuntu 20.04 |
| Hackerboard (ESP32) | Motor driver. Talks to ROS via `micro_ros_agent serial --dev /dev/ttyUSB0` |
| Onboard camera | Streamed through the `ros_deep_learning` pipeline |

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

The system is composed of independent ROS 2 nodes that communicate
through standard topics. The diagram below shows the data flow.

```
                              ┌──────────────────────────┐
                              │       camera (CSI)       │
                              └─────────────┬────────────┘
                                            │ /video_source/raw
                                            ▼
                              ┌──────────────────────────┐
                              │   traffic_light_detector │
                              │  (HSV + circularity)     │
                              └─────────────┬────────────┘
                                            │ /semaphore_state
                                            ▼
  ┌──────────────┐  /goals    ┌─────────────────────────────┐  /cmd_vel
  │path_generator│───────────▶│  controller (PID + FSM +    │────────┐
  └──────┬───────┘            │  semaphore override)         │        │
         ▲                    └─────────────────┬───────────┘         │
         │ /goal_reached                        │ /robot_pose          │
         │                                      ▲                     ▼
         │                                      │           ┌────────────────┐
         │                                      │           │ cmd_vel_bridge │
         │                                      │           └────────┬───────┘
         │                                      │                    │ Float32 L,R
         │                              ┌───────┴────────┐           ▼
         │                              │ odometry_node  │   ┌──────────────────┐
         │                              └───────▲────────┘   │ firmware (ESP32) │
         │                                      │ EncL, EncR │   on Hackerboard │
         │                                      └────────────┴──────────────────┘
         │
         └─────────────── full loop closed via odometry ──────────────┘
```

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/goals` | `puzzlebot_mc2/Goal` | path_generator → controller |
| `/goal_reached` | `std_msgs/UInt32` | controller → path_generator |
| `/robot_pose` | `geometry_msgs/PoseStamped` | odometry_node → controller |
| `/cmd_vel` | `geometry_msgs/Twist` | controller → cmd_vel_bridge |
| `/VelocitySetL`, `/VelocitySetR` | `std_msgs/Float32` | cmd_vel_bridge → firmware |
| `/VelocityEncL`, `/VelocityEncR` | `std_msgs/Float32` | firmware → odometry_node |
| `/video_source/raw` | `sensor_msgs/Image` | camera → traffic_light_detector |
| `/semaphore_state` | `puzzlebot_mc2/SemaphoreState` | traffic_light_detector → controller |
| `/traffic_light_debug` | `sensor_msgs/Image` | traffic_light_detector (optional, debug) |

### Custom messages

- **`Goal.msg`** — wraps `geometry_msgs/Pose` plus a unique `id` and a
  `is_last` flag. Required by the assignment to use a custom message
  based on `geometry_msgs/Pose`.
- **`SemaphoreState.msg`** — carries the detected state
  (UNKNOWN / GREEN / YELLOW / RED), a normalized confidence, and the raw
  pixel area of the winning blob.

---

## Repository structure

```
puzzlebot_mc2/
├── README.md
├── package.xml
├── CMakeLists.txt
├── msg/
│   ├── Goal.msg
│   └── SemaphoreState.msg
├── config/
│   ├── robot_params.yaml
│   ├── controller_params.yaml          ← MC3 settings (semaphore off)
│   ├── controller_params_mc4.yaml      ← MC4 settings (semaphore on)
│   ├── vision_params.yaml              ← HSV ranges + shape filters
│   ├── waypoints_mc3_part1.yaml        ← 2 m square
│   ├── waypoints_mc3_part2.yaml        ← multi-waypoint path
│   └── waypoints_mc4.yaml              ← 6 waypoints inside 2×2 m
├── scripts/
│   ├── odometry_node.py
│   ├── cmd_vel_bridge.py
│   ├── controller.py                   ← MC4-aware, backwards compatible with MC3
│   ├── path_generator.py
│   ├── traffic_light_detector.py
│   ├── hsv_calibrator.py               ← interactive calibration tool
│   ├── plot_mc4.py                     ← offline analysis (matplotlib)
│   └── analyze.py                      ← optional, MC3 numeric metrics
├── launch/
│   ├── mc3_part1_square.launch.py
│   ├── mc3_part2_waypoints.launch.py
│   └── mc4_full.launch.py
└── docs/
    ├── presentations/                  ← optional, slides used in deliverables
    └── images/                         ← optional, diagrams and screenshots
```

---

## Project progression: from open loop to perception

The package grew across three iterations. Each one closed a gap that the
previous one exposed.

### Mini Challenge 2 — open-loop motion

The starting point was issuing timed velocity commands directly to the
wheels, with the kinematic constants (`k_lin`, `k_ang`) calibrated
empirically. There was no feedback: if the robot was off by 10 percent,
it stayed off by 10 percent. This iteration validated the basic stack
(Jetson + Hackerboard + micro-ROS bridge) and produced the first
versions of `cmd_vel_bridge` and the wheel-velocity calibration.

### Mini Challenge 3 — closed-loop PID navigation

The closed loop was introduced. The main additions in this iteration:

- **Odometry**: a node that integrates the encoder velocities into a
  pose `(x, y, theta)` and publishes `/robot_pose`.
- **PID controller** with an FSM (IDLE → ROTATE_TO_GOAL → MOVE_TO_GOAL
  → HOLD), supporting two control strategies selectable by parameter.
- **Path generator**: an independent node that loads a list of
  waypoints, validates each one for geometric, kinematic, and dynamic
  reachability before any motion, and publishes goals one at a time.
- **Custom message `Goal.msg`** based on `geometry_msgs/Pose`, as
  required by the assignment.

This iteration also revealed three issues that became central to the
final design:

1. **QoS mismatch**: the micro-ROS firmware publishes encoder data with
   `BEST_EFFORT` reliability, while the default `rclpy` subscriber is
   `RELIABLE`. The result was that odometry simply never updated.
   Solved by switching the encoder subscriptions to
   `qos_profile_sensor_data`.
2. **Hackerboard resets** during sharp commands. The root cause was
   current spikes from abrupt velocity changes. Solved by introducing
   a slew rate limiter in the publish stage of the controller, which
   replaces step changes by ramps.
3. **Overshoot of about 28 cm** at the end of each straight segment.
   Two causes were identified: control-loop latency and a 10 percent
   underestimation in odometry. Solved by an `approach_zone` mechanism
   that scales velocity down close to each waypoint, plus an
   `encoder_scale` factor that calibrates out the odometry drift.

By the end of MC3 the robot reliably closed a 2 m square and an
arbitrary multi-waypoint path within a 5 cm tolerance, in both
sequential and simultaneous control modes.

### Mini Challenge 4 — vision-based traffic light response

Perception was added on top of the validated navigation stack, with the
deliberate constraint that the navigation code itself would not be
rewritten. The new components are:

- **`traffic_light_detector` node**: subscribes to `/video_source/raw`,
  converts each frame to HSV, applies color range filters, cleans noise
  with morphological operations, finds contours, and applies a
  circularity and aspect-ratio filter to keep only round, compact
  blobs. The largest valid blob across the three colors determines the
  output state.
- **`SemaphoreState.msg`** custom message that carries the detected
  state plus a confidence value.
- **`hsv_calibrator.py` tool**: a ROS node with an OpenCV window that
  shows the live image, mask and result, with six trackbars for HSV
  range tuning. Used to find the right ranges for the actual lighting
  conditions and color targets.
- **Controller integration**: the existing controller was extended with
  an `enable_semaphore` parameter. When enabled, it subscribes to
  `/semaphore_state` and applies an override **at the publish stage**,
  not inside the PID. This means the PID, the FSM, the approach zone
  and the slew rate limiter all keep working exactly as in MC3; the
  semaphore stage only modulates what reaches the wheels.
- **Asymmetric red latch**: a single red reading is enough to stop the
  robot, but several consecutive green readings are required to release
  the latch. Additionally, if the camera loses sight of the light while
  the latch is active, the robot stays stopped. This enforces the
  deliverable rule of remaining stopped until green is confirmed.
- **`plot_mc4.py`**: an offline analysis script that reads the
  controller's CSV log and produces six figures (trajectory colored by
  semaphore state, position and angle error timelines, velocity
  commands before and after override, semaphore timeline) plus a
  metrics report.

---

## Installation

This assumes a Jetson Nano with the official `puzzlebot` system image,
ROS 2 Humble installed and a `~/ros2_ws` already initialized.

```bash
# clone into the workspace src folder
cd ~/ros2_ws/src
git clone <repo url> puzzlebot_mc2
cd puzzlebot_mc2

# make scripts executable
chmod +x scripts/*.py

# install required system dependencies
sudo apt update
sudo apt install ros-humble-cv-bridge python3-opencv python3-numpy

# matplotlib only needed for the offline plot script
pip3 install matplotlib
```

The `ros_deep_learning` package (for the camera pipeline) is part of the
official PuzzleBot system image. If it is missing, follow the steps in
the Manchester Robotics test manual.

---

## Building the workspace

```bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_mc2 --symlink-install
source install/setup.bash
```

The `--symlink-install` flag means that edits to Python scripts and YAML
files do not require a rebuild — only changes to `.msg`,
`CMakeLists.txt` or `package.xml` do.

Verify the custom messages compiled:

```bash
ros2 interface show puzzlebot_mc2/msg/Goal
ros2 interface show puzzlebot_mc2/msg/SemaphoreState
```

---

## Running the system

Before any of the runs below, start the micro-ROS bridge so the
Hackerboard talks to ROS:

```bash
sudo chmod 666 /dev/ttyUSB0
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```

### MC3 — Part 1: 2 m square

```bash
ros2 launch puzzlebot_mc2 mc3_part1_square.launch.py
```

The robot starts at the origin, drives a 2 m × 2 m square, and stops.
A CSV log is written under `/tmp/puzzlebot_logs/`.

### MC3 — Part 2: arbitrary waypoint path

```bash
ros2 launch puzzlebot_mc2 mc3_part2_waypoints.launch.py
```

The path is defined in `config/waypoints_mc3_part2.yaml`. The path
generator validates reachability before the robot starts moving and
publishes one goal at a time, waiting for confirmation before the next.

### MC4 — full system with traffic light perception

This requires the camera pipeline to be running. Open a terminal:

```bash
ros2 launch ros_deep_learning video_source.ros2.launch
```

Verify the image stream is healthy:

```bash
ros2 topic hz /video_source/raw
```

Then launch the integrated system:

```bash
ros2 launch puzzlebot_mc2 mc4_full.launch.py
```

The robot drives the 6-waypoint path defined in
`config/waypoints_mc4.yaml`. A red, yellow or green circle held in
front of the camera produces the corresponding action. The robot
remains stopped on red until green is sustained for several frames.

To monitor the detector visually, open `rqt_image_view` in another
terminal and subscribe to `/traffic_light_debug` — it shows the
annotated frame with the detected color and the current state.

---

## Key configuration parameters

### `robot_params.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `wheel_radius` | 0.045 m | Physical |
| `wheel_base` | 0.18 m | Physical |
| `k_lin`, `k_ang` | 0.074, 0.561 | Wheel-velocity gain constants |
| `encoder_scale` | 1.1 | Compensates ~10 % odometry underestimation |
| `max_cmd` | 3.0 | Saturation of the Float32 firmware command |

### `controller_params*.yaml`

| Parameter | Default | Meaning |
|---|---|---|
| `control_mode` | "sequential" | "sequential" or "simultaneous" |
| `kp_ang`, `kp_lin` | 0.8, 0.4 | Sequential mode P gains |
| `v_max`, `omega_max` | 0.20, 0.45 | Saturation of the actuator |
| `epsilon_pos`, `epsilon_ang` | 0.05, 0.05 | Goal tolerance |
| `integral_max` | 0.5 | Anti-windup clamp |
| `derivative_filter_alpha` | 0.2 | EMA filter on the derivative term |
| `v_slew_rate`, `w_slew_rate` | 0.18, 0.8 | Anti-spike rate limiter |
| `approach_zone` | 0.25 m | Distance to start velocity scaling |
| `approach_zone_min_scale` | 0.20 | Minimum velocity scale near the goal |
| `enable_semaphore` | false (MC3) / true (MC4) | Vision integration switch |
| `yellow_speed_scale` | 0.4 | Linear-velocity scaling on yellow |
| `green_frames_to_release` | 5 | Consecutive green frames to clear the red latch |
| `semaphore_timeout_s` | 1.0 | Time without messages before treating state as UNKNOWN |

### `vision_params.yaml`

Two groups of parameters:

- **HSV ranges**: `red1_*`, `red2_*` (red has two ranges because of hue
  wrap-around), `yellow_*`, `green_*`. Each color has six values
  (H/S/V min/max).
- **Shape filters**: `min_area_px`, `min_circularity`,
  `min_aspect_ratio`, `max_aspect_ratio`.

---

## HSV calibration workflow

The default HSV ranges in `vision_params.yaml` will rarely work directly
in your room because hue is sensitive to lighting. To calibrate:

```bash
# terminal 1
ros2 launch ros_deep_learning video_source.ros2.launch
# terminal 2
ros2 run puzzlebot_mc2 hsv_calibrator.py
```

A window opens with six trackbars (H/S/V min/max). The keyboard
controls:

- `r`, `y`, `g` — load a preset for red / yellow / green
- `p` — print the current values in a YAML-ready format
- `q` — quit

Procedure for each color: load the preset with the keyboard, hold the
colored card in front of the camera, and adjust the sliders until only
that color appears bright in the mask. Press `p` and copy the printed
values into `vision_params.yaml`. Repeat for the other two colors.

For red specifically, the hue is split across the wrap-around point. If
your red card maps near `H = 0`, set `red1_*` to that range and leave
`red2_*` with a tiny range near 180 (or vice versa). The detector
combines both ranges automatically.

---

## Generating report plots from a run

Every controller run with logging enabled writes a CSV to
`/tmp/puzzlebot_logs/run_<mode>_<timestamp>.csv`. To turn one of those
into report-ready figures:

```bash
python3 ~/ros2_ws/src/puzzlebot_mc2/scripts/plot_mc4.py \
    /tmp/puzzlebot_logs/run_sequential_sem_1715900000.csv
```

This produces, in the same folder as the CSV:

- `01_trajectory.png` — actual path, colored by semaphore state, with
  waypoint markers
- `02_position_error.png` — position error vs time, with shaded regions
  showing when the robot was stopped by red
- `03_angle_error.png` — angle error vs time
- `04_velocity_cmds.png` — `v` and `ω` published vs requested by the
  PID; the gap between the two curves shows where the semaphore
  override took effect
- `05_semaphore_timeline.png` — a colored band of the detected state
  over time, with the red latch overlaid
- `06_dashboard.png` — a single figure that combines the key views
- `metrics.txt` — total time, time blocked by red, mean and max error,
  per-waypoint final error, smoothness score

---

## General considerations

A few things that are easy to overlook and that cost time on the first
run.

### Lighting for the vision system

Hue is much more stable than RGB, but it is not lighting-invariant.
Calibrating HSV ranges under one lighting condition and running the
robot under a different one (a different room, sunlight vs lamps,
shadows) will likely require re-calibration. For the deliverable,
calibrate and record the demonstration video in the same lighting.

### Workspace size

The MC4 launch assumes a 2 × 2 m clear area. The path generator
validates that all waypoints fall inside a bounding box, but it does
not know about physical obstacles. Make sure the area is clear before
starting.

### Camera placement

The traffic light detector treats the largest valid colored circle as
the active state. If two colors are partially visible, the larger one
wins. Keep only one card visible at a time, and present it square-on
to the camera at a reasonable distance (roughly 30 to 80 cm worked
well in development).

### Safety

The slew rate limiter cannot stop the robot instantly. If the
controller is shut down with `Ctrl+C`, ten Twist-zero messages are sent
as a parting shot, but the firmware may still execute the last command
briefly. Keep an emergency cutoff handy.

### YAML files must be plain ASCII

ROS 2's `rcl_yaml_param_parser` rejects non-ASCII characters such as
`≈`, `→`, `°` and accented characters. All YAML files in this package
are written in plain ASCII for that reason. If you add comments, keep
them ASCII-only.

### CSV logs accumulate

Each run appends a new file to `/tmp/puzzlebot_logs/`. The `/tmp` folder
is typically cleared on reboot, but on long sessions the folder can
grow. Clean it periodically with `rm /tmp/puzzlebot_logs/*.csv`.

---

## Troubleshooting

### Odometry never updates, the robot only drives the first segment

This is the QoS mismatch issue. The fix is already in `odometry_node.py`
(uses `qos_profile_sensor_data`), but if you see this happening it
usually means the encoders are not being published at all. Check:

```bash
ros2 topic hz /VelocityEncL
ros2 topic hz /VelocityEncR
```

If those report no messages, the micro-ROS agent is not running or the
Hackerboard is unreachable. Verify the agent and `/dev/ttyUSB0`.

### Hackerboard reboots itself in the middle of a run

This is a current-spike issue. The slew rate parameters
(`v_slew_rate`, `w_slew_rate`) limit the rate of change of the
published command. If resets are still happening with the defaults,
lower both values further. Sharp angular changes are the most common
cause; lowering `w_slew_rate` to around 0.6 usually fixes it.

### The robot overshoots every waypoint

Two parameters fight this: `approach_zone` (size of the slow-down
region) and `encoder_scale` (corrects systematic odometry drift). If
overshoot is bad, try first increasing `encoder_scale` in 0.05 steps
until the robot stops within tolerance, then tune `approach_zone` if
needed.

### Detector reports the wrong color or flickers

Almost always an HSV calibration issue. Re-run the calibrator under
the actual lighting you plan to use. If the issue is yellow being
confused with red or green, narrow the hue ranges. If the issue is
false positives from the background, raise `min_area_px` and
`min_circularity`.

### Detector never sees anything

Three things to check, in order:

1. Is the camera publishing? `ros2 topic hz /video_source/raw`
2. Does the debug image look reasonable? Open `rqt_image_view` on
   `/traffic_light_debug` and check that the banner color updates as
   you move the cards.
3. Are the HSV ranges sensible? Pure black is `H=any, S=low, V=low`.
   Pure white is `H=any, S=low, V=high`. If your ranges accidentally
   require both high S and low V, nothing in a real scene will match.

### `ros2 interface show puzzlebot_mc2/msg/SemaphoreState` fails after editing

The custom message has to be regenerated after any edit. Run a fresh
build:

```bash
cd ~/ros2_ws
colcon build --packages-select puzzlebot_mc2
source install/setup.bash
```

---

## Credits

- **Course**: TE3002B Mobile Robotics, Tecnológico de Monterrey.
- **Industry partner**: Manchester Robotics (the PuzzleBot platform and
  challenge brief).
- **Team**: Equipo 1 — REPO.

The PuzzleBot platform, the `ros_deep_learning` package, and the
`micro_ros_agent` setup are not part of this repository; they are part
of the official PuzzleBot system image provided by Manchester Robotics.
