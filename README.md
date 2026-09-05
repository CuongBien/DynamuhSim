# DynamuhSim 🤖

> **Multi-Sensor Perception and Proactive Yielding for Patrol Robots in Narrow, Dynamic Indoor Environments**

DynamuhSim is a simulation and evaluation framework built on **ROS 2 Jazzy** and **Gazebo Sim 8 (Harmonic)**. It provides parametric narrow corridor environments, dynamic human-like moving obstacles with ground-truth streaming, Nav2 navigation stack integration, automated batch trial orchestration, and an offline evaluation framework computing clearance, trajectory quality, and reaction metrics.

---

## 📌 Key Features

- **Parametric Narrow Corridors**: Configurable corridor widths (`0.70m`, `0.90m`, `1.20m`) with matching static 2D occupancy grid maps for AMCL localization.
- **Dynamic Obstacle Simulation**: Custom Gazebo Sim C++ System Plugin (`oscillating_obstacle_system`) simulating oscillating human/patrol traffic with exact ground-truth pose publication.
- **Nav2 Stack Integration**: Tailored costmap and controller configurations for TurtleBot3 Burger in confined spaces, including an action bridge forwarding standard `/goal_pose` topics to `/navigate_to_pose`.
- **Automated Trial Orchestrator (`run_baseline_trials.sh`)**: 8-stage lifecycle-managed runner (Gazebo readiness, Nav2 node creation, lifecycle state verification, AMCL localization, ground-truth logging, rosbag recording, navigation goal dispatch, and automatic metrics generation).
- **Comprehensive Evaluation Framework (`evaluate_framework.py` & `analyze_baseline.py`)**: Computes trajectory lengths, speed profiles, minimal clearances, obstacle encounter detection, reaction latencies, and planner frequencies.
- **DynaBARN Benchmark Integration**: Git submodule integration for dynamic indoor navigation benchmarking.

---

## 📁 Repository Structure

```text
DynamuhSim/
├── src/
│   ├── custom_corridor/                 # Main simulation & Nav2 package
│   │   ├── config/                      # Nav2 parameters (nav2_corridor.yaml)
│   │   ├── launch/                      # Launch files (corridor_tb3.launch.py, nav2_corridor.launch.py)
│   │   ├── maps/                        # 2D maps (corridor_070, corridor_090, corridor_120)
│   │   ├── rviz/                        # RViz2 visualization setup (corridor.rviz)
│   │   ├── scripts/                     # Helper nodes (goal_pose_bridge.py)
│   │   ├── urdf/                        # TurtleBot3 URDF / Xacro descriptions
│   │   └── worlds/                      # Gazebo SDF worlds (corridor_070, 090, 120)
│   ├── custom_corridor_plugins/         # C++ Gazebo Sim System plugins
│   │   └── src/oscillating_obstacle_system.cc
│   └── dynabarn/                        # DynaBARN benchmark submodule
├── experiments/                         # Evaluation datasets & baseline configs
├── run_baseline_trials.sh               # Automated batch trial orchestrator
├── record_obstacle.py                   # Ground-truth obstacle pose recorder
├── analyze_baseline.py                  # Single-trial offline metrics engine
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

**Terminal 1 — Start Gazebo simulation:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

# width options: 0.70, 0.90 (default), 1.20
ros2 launch custom_corridor corridor_tb3.launch.py width:=0.90
```

**Terminal 2 — Start Nav2 & RViz2:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

ros2 launch custom_corridor nav2_corridor.launch.py
```

*In RViz2, click **Nav2 Goal** on the top toolbar and set a navigation goal along the corridor.*

---

### Option 2: Automated Batch Trials

Run automated end-to-end trials with automated launch, AMCL pose convergence, bag recording, and offline evaluation:

```bash
cd ~/nav_ws

# Run 1 trial starting at trial index 3
START_TRIAL=3 ./run_baseline_trials.sh 1

# Run 5 consecutive trials (e.g., trial_05 through trial_09)
START_TRIAL=5 ./run_baseline_trials.sh 5
```

**Environment Variables for Configuration:**
| Variable | Default | Description |
|---|---|---|
| `WIDTH` | `0.90` | Corridor width (`0.70`, `0.90`, `1.20`) |
| `START_TRIAL` | `3` | Starting trial index |
| `GOAL_X` | `11.74` | Target X coordinate in map frame (meters) |
| `GOAL_Y` | `0.0` | Target Y coordinate in map frame (meters) |
| `GOAL_TOLERANCE`| `0.15` | Goal acceptance tolerance (meters) |
| `TRIAL_TIMEOUT_S`| `180` | Maximum navigation duration (seconds) |

Each trial outputs:
- `trial_XX/`: MCAP rosbag recording (`/cmd_vel`, `/odom`, `/scan`, `/tf`, `/plan`, `/local_plan`, etc.)
- `obstacle_ground_truth_XX.csv`: Ground-truth moving obstacle trajectory.
- `trial_XX/analysis/report.txt`: Summary of navigation metrics.
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

Generate aggregate metrics over a batch of trials:
```bash
python3 evaluate_framework.py batch \
  --base ~/nav_ws/experiments/corridor_090/baseline_01
```

---

## 📊 Evaluation Metrics

The framework records and analyzes:
1. **Navigation Success Rate (SR)**: Percentage of trials reaching the target goal within tolerance.
2. **Trajectory Length & Duration**: Odometry distance vs. optimal path length, average linear velocity.
3. **Obstacle Clearance**: Continuous Euclidean distance between robot and dynamic obstacles.
4. **Reaction Latency**: Time from obstacle encounter entry to first proactive yielding / deceleration command (`cmd_vel`).
5. **Planner Performance**: Replanning frequency and path update stability.

---

## 👤 Author & License

- **Author**: CuongBien ([biencaocuongg@gmail.com](mailto:biencaocuongg@gmail.com))
- **Repository**: [https://github.com/CuongBien/DynamuhSim.git](https://github.com/CuongBien/DynamuhSim.git)
- **License**: Apache 2.0
