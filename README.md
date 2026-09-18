# DynamuhSim 🤖

> **Multi-Sensor Perception and Proactive Yielding for Patrol Robots in Narrow, Dynamic Indoor Environments**

DynamuhSim is a simulation and evaluation framework built on **ROS 2 Jazzy** and **Gazebo Sim 8 (Harmonic)**. It provides parametric narrow corridor environments, dynamic human-like moving obstacles with ground-truth streaming, Nav2 navigation stack integration, automated batch trial orchestration, and an offline evaluation framework computing clearance, trajectory quality, and reaction metrics.

---

## 📌 Key Features

- **Parametric Environments & Open Arenas**: Configurable environments including narrow corridors (`0.70m`, `0.90m`, `1.20m`) and an open obstacle arena (`20m x 10m` with `arena`) with static pillars, industrial boxes, and dynamic human pedestrians.
- **Dual Local Controller Support (DWB & MPPI)**: Full support for both DWB (`dwb_core::DWBLocalPlanner`) and Model Predictive Path Integral (`nav2_mppi_controller::MPPIController`) with DiffDrive kinematics, predictive horizon rollouts, and tuned obstacle avoidance critics.
- **Dynamic Obstacle Simulation**: Custom Gazebo Sim C++ System Plugin (`oscillating_obstacle_system`) simulating oscillating human/patrol traffic with exact ground-truth pose publication.
- **Headless & GUI Simulation Modes**: Fast-execution headless Gazebo mode (`gui:=false`) reducing CPU load by >100% and stabilizing Real Time Factor (RTF ~ 1.0) for automated testing.
- **Nav2 Stack Integration**: Tailored costmap and controller configurations for TurtleBot3 Burger, synchronized FastDDS lifecycle managers with race condition mitigation, and action bridge forwarding `/goal_pose` to `/navigate_to_pose`.
- **Automated Trial Orchestrator (`run_baseline_trials.sh`)**: 8-stage lifecycle-managed runner (Gazebo readiness, Nav2 node creation, lifecycle state verification, AMCL localization, ground-truth logging, rosbag recording, navigation goal dispatch, and automatic metrics generation).
- **Comprehensive & Social Evaluation Framework (`evaluation/`)**: Computes trajectory lengths, speed profiles, minimal clearances, reaction latencies, motion smoothness (Linear/Angular Jerk RMS, Jerk Cost), and Proxemic Space Intrusion (PSI) intimate/personal metrics.
- **DynaBARN Benchmark Integration**: Git submodule integration for dynamic indoor navigation benchmarking.

---

## 📁 Repository Structure

```text
DynamuhSim/
├── src/
│   ├── custom_corridor/                 # Main simulation & Nav2 package
│   │   ├── config/                      # Nav2 parameters (nav2_corridor.yaml, nav2_mppi.yaml)
│   │   ├── launch/                      # Launch files (corridor_tb3.launch.py, nav2_corridor.launch.py)
│   │   ├── maps/                        # 2D maps (corridor_070, 090, 120, arena_obstacle)
│   │   ├── rviz/                        # RViz2 visualization setup (corridor.rviz)
│   │   ├── scripts/                     # Helper nodes (goal_pose_bridge.py)
│   │   ├── urdf/                        # TurtleBot3 URDF / Xacro descriptions
│   │   └── worlds/                      # Gazebo SDF worlds (corridor_070, 090, 120, arena_obstacle)
│   ├── custom_corridor_plugins/         # C++ Gazebo Sim System plugins
│   │   └── src/oscillating_obstacle_system.cc
│   └── dynabarn/                        # DynaBARN benchmark submodule
├── evaluation/                          # Modular evaluation package
│   ├── analyzer.py                      # Core trial analysis pipeline
│   ├── motion_metrics.py                # Trajectory, velocity, and jerk metrics
│   ├── obstacle_metrics.py              # Clearance and encounter metrics
│   ├── social_metrics.py                # Proxemics and PSI metrics
│   ├── reporter.py                      # Report and JSON serialization
│   └── types.py                         # Data structures and schemas
├── experiments/                         # Evaluation datasets & baseline configs
├── run_baseline_trials.sh               # Automated batch trial orchestrator
├── record_obstacle.py                   # Ground-truth obstacle pose recorder
├── analyze_baseline.py                  # Single-trial offline metrics CLI facade
├── evaluate_framework.py                # Batch evaluation & algorithm comparison
├── BASELINE_TRIALS.md                   # Quick guide for automated trials
└── README.md
```

---

## 🛠 Prerequisites & Installation

### 1. Requirements
- **OS**: Ubuntu 24.04 LTS (Noble Numbat) or WSL2 Ubuntu 24.04
- **ROS 2**: Jazzy Jalisco (`desktop` or `ros-base`)
- **Simulator**: Gazebo Sim 8 (Harmonic)
- **Middleware**: FastDDS (`rmw_fastrtps_cpp`)

### 2. Clone Repository
Clone the repository recursively to fetch submodules:
```bash
git clone --recurse-submodules https://github.com/CuongBien/DynamuhSim.git ~/nav_ws
cd ~/nav_ws
```

### 3. Install ROS 2 Dependencies
```bash
run: `source setup_env.sh` để setup enviroment cho ROS 2 Jazzy và DynamuhSim workspace

source /opt/ros/jazzy/setup.bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-description \
  ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-rosbag2-storage-mcap \
  python3-numpy \
  python3-scipy
```

### 4. Build Workspace
```bash
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Usage

### Option 1: Manual Run (Gazebo + Nav2 + RViz2)

#### 1. Narrow Corridor (0.90m) with DWB:
**Terminal 1 — Start Gazebo simulation:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# width options: 0.70, 0.90 (default), 1.20, arena
# obstacle options: human (default), object, none
# gui options: true (default), false (headless)
ros2 launch custom_corridor corridor_tb3.launch.py width:=0.90 obstacle:=human
```

**Terminal 2 — Start Nav2 & RViz2:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# controller options: dwb (default), mppi
ros2 launch custom_corridor nav2_corridor.launch.py controller:=dwb
```

#### 2. Open Arena (20m x 10m) with MPPI Controller (Headless):
**Terminal 1 — Start Headless Gazebo simulation (Fast & low CPU):**
```bash
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch custom_corridor corridor_tb3.launch.py width:=arena obstacle:=human gui:=false rviz:=false
```

**Terminal 2 — Start Nav2 with MPPI Controller & Arena Map:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch custom_corridor nav2_corridor.launch.py controller:=mppi map:=arena_obstacle
```

**Terminal 3 — Open RViz2 visualization:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=42

rviz2 -d ~/nav_ws/src/custom_corridor/rviz/corridor.rviz
```

**Terminal 4 — Dispatch Goal Pose:**
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 15.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

---

### Option 2: Automated Batch Trials

Run automated end-to-end trials with automated launch, AMCL pose convergence, bag recording, and offline evaluation:

```bash
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

# Run 1 trial with DWB controller in 0.90m corridor
START_TRIAL=1 CONTROLLER=dwb ./run_baseline_trials.sh 1

# Run 5 trials with MPPI controller in 0.90m corridor
START_TRIAL=1 CONTROLLER=mppi ./run_baseline_trials.sh 5

# Run trials in the open obstacle arena
START_TRIAL=1 WIDTH=arena CONTROLLER=mppi GOAL_X=15.0 ./run_baseline_trials.sh 3
```

**Environment Variables for Configuration:**
| Variable | Default | Description |
|---|---|---|
| `CONTROLLER` | `dwb` | Local trajectory planner plugin (`dwb` or `mppi`) |
| `WIDTH` | `0.90` | Environment selection (`0.70`, `0.90`, `1.20`, `arena`) |
| `OBSTACLE_TYPE` | `human` | Dynamic obstacle type (`human`, `object`, `none`) |
| `ROS_DOMAIN_ID` | `42` | ROS domain used to isolate the batch from other simulators |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | Prevent discovery of ROS sessions on other machines |
| `FASTDDS_BUILTIN_TRANSPORTS` | `UDPv4` | Avoid Fast DDS shared-memory service stalls during bringup |
| `GUI` | `false` | Enable or disable the Gazebo GUI during automated trials |
| `RVIZ` | `false` | Enable or disable RViz2 during automated trials |
| `START_TRIAL` | `3` | Starting trial index |
| `GOAL_X` | `20.0` | Target X coordinate in map frame (meters) |
| `GOAL_Y` | `0.0` | Target Y coordinate in map frame (meters) |
| `GOAL_TOLERANCE`| `0.20` | Goal acceptance tolerance (meters) |
| `TRIAL_TIMEOUT_S`| `180` | Maximum navigation duration (seconds) |
| `READY_TIMEOUT_S`| `240` | Maximum stack bringup waiting time (seconds) |
| `DOMAIN_IDLE_TIMEOUT_S` | `30` | Wait for old DDS endpoints to leave the selected domain |

Each trial outputs:
- `trial_XX/`: MCAP rosbag recording (`/cmd_vel`, `/odom`, `/scan`, `/tf`, `/plan`, `/local_plan`, etc.)
- `obstacle_ground_truth_XX.csv`: Ground-truth moving obstacle trajectory.
- `trial_XX/analysis/report.txt`: Formatted human-readable metrics report.
- `trial_XX/analysis/metrics.json`: Machine-readable sanitized JSON metrics.
- `trial_XX/trial_result.json`: Lifecycle status (`SUCCESS`, `FAILURE`, `TIMEOUT`, `INVALID`).

---

### Option 3: Offline Evaluation & Batch Analysis

Analyze a single trial:
```bash
python3 analyze_baseline.py \
  --bag experiments/corridor_090/baseline_01/trial_04 \
  --ground-truth experiments/corridor_090/baseline_01/obstacle_ground_truth_04.csv \
  --output experiments/corridor_090/baseline_01/trial_04/analysis \
  --goal-x 11.74 --goal-y 0.0 --goal-tolerance 0.15
```

Generate aggregate metrics and comparison across controllers/widths:
```bash
python3 evaluate_framework.py batch \
  --base ~/nav_ws/experiments/corridor_090/baseline_01
```

---

## 📊 Evaluation Metrics

The modular evaluation framework computes 6 categories of performance metrics:

1. **Navigation Task Success**:
   - Status (`SUCCESS`, `FAILURE`, `TIMEOUT`, `COLLISION`).
   - Total navigation duration ($T$) and time-to-goal.
2. **Path & Kinematic Efficiency**:
   - Total trajectory length vs. Euclidean optimal distance.
   - Path curvature and mean/max linear velocity ($v_x$) and angular velocity ($\omega_z$).
3. **Motion Smoothness & Ride Comfort**:
   - **Linear Jerk RMS** ($\text{m/s}^3$): Root-mean-square of acceleration derivative.
   - **Angular Jerk RMS** ($\text{rad/s}^3$): Smoothness of rotational steering adjustments.
   - **Total Jerk Cost**: Integral of squared jerk over the trajectory.
4. **Dynamic Obstacle Clearance**:
   - Continuous 2D Euclidean clearance to moving obstacle.
   - Minimum clearance and collision boundary violations.
   - Encounter start/end detection and time spent in encounter zone.
5. **Reaction & Yielding Latency**:
   - Reaction latency: Time elapsed from obstacle encounter entry to first proactive yielding/deceleration response.
6. **Social Proxemic Space Intrusion (PSI)**:
   - **Intimate Space** ($d < 0.45\text{m}$): Intrusion duration and time ratio.
   - **Personal Space** ($0.45\text{m} \le d < 1.20\text{m}$): Intrusion duration and time ratio.
   - Total count of intimate zone intrusions.

---

## 👤 Author & License

- **Author**: CuongBien ([biencaocuongg@gmail.com](mailto:biencaocuongg@gmail.com))
- **Repository**: [https://github.com/CuongBien/DynamuhSim.git](https://github.com/CuongBien/DynamuhSim.git)
- **License**: Apache 2.0
