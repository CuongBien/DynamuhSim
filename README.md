# DynamuhSim 🤖

**Luồng hiện tại:** [Dữ liệu và baseline WM](docs/START_HERE_CURRENT.md) · [Runtime Nav2 dùng chung](docs/SHARED_NAV2_RUNTIME.md) · [Registry experiment](docs/experiment_registry.json) · [Capture mới](docs/CAPTURE_RUNTIME.md) · [Audit và artifact](docs/ANALYSIS_AND_ARTIFACTS.md) · [World assets](docs/WORLD_ASSETS.md) · [Archive lịch sử](docs/HISTORICAL_ARTIFACTS.md) · [Report và dependency](docs/REPORT_STORAGE.md) · [Media và test state](docs/REPORT_MEDIA.md) · [Sensor API](docs/SENSOR_CONTROLS.md)

> **Multi-Sensor Perception and Proactive Yielding for Patrol Robots in Narrow, Dynamic Indoor Environments**

DynamuhSim is a simulation and evaluation framework built on **ROS 2 Jazzy** and **Gazebo Sim 8 (Harmonic)**. It provides parametric narrow corridor environments, dynamic human-like moving obstacles with ground-truth streaming, Nav2 navigation stack integration, automated batch trial orchestration, and an offline evaluation framework computing clearance, trajectory quality, and reaction metrics.

---

## 📌 Key Features

- **In-Engine RVO2 / ORCA C++ Social Navigation Plugin (`librvo2_human_system.so`)**: High-throughput reciprocal velocity obstacle collision avoidance running synchronously at 250 Hz in Gazebo Sim 8.
- **Social Compliance & Proxemics**:
  - **Asymmetric Politeness**: Configurable yielding compliance ($0.0\text{--}1.0$) with lateral nudge to the right when meeting robots head-on.
  - **Hall's Anisotropic Proxemics**: Asymmetric personal space modeling ($1.2\text{m}$ front buffer, $0.6\text{m}$ side margin).
  - **Reaction Latency**: 1st-order low-pass filter ($\tau \approx 250\text{--}400\text{ ms}$) capturing human cognitive and physical reaction delay.
  - **Stochastic Recess Yielding FSM**: 60% polite pedestrians step into corridor alcoves, turn to observe the robot, and wait for clearance.
- **Multi-Environment Simulation Suite**:
  - **Narrow Corridors**: Parametric widths (`0.70m`, `0.90m`, `1.20m`) with alcoves.
  - **Open Arenas**: Multi-agent interaction (`arena_rvo2` with 4 distinct colored agents, `arena_obstacle`, and `arena_dataset` with 24 agents).
  - **Hospital Facility**: Realistic $60\text{m} \times 17\text{m}$ hospital with staff, patients, and synchronized medical delivery cart pushers (`easy`: 3 agents, `medium`: 8 agents, `hard`: 16 agents).
  - **School Campus Arena**: 15 classrooms, corridors A–F, stairways, and 8 active student pedestrians.
- **Dual-Layer Physical Perception Model**:
  - **Visual Layer**: Multi-colored 3D skinned meshes (`walk_orange.dae`, `walk_red.dae`, `walk_blue.dae`, `push_trolley.dae`) with zero elevation drift and speed-locked gait animation (zero foot-sliding).
  - **Collision Layer**: Dynamic rigid cylinder proxies ($r=0.25\text{--}0.28\text{m}, h=1.70\text{m}$) providing $100\%$ reliable 2D LiDAR reflection.
- **Dual Local Controller Support (DWB & MPPI)**: Full support for both DWB (`dwb_core::DWBLocalPlanner`) and Model Predictive Path Integral (`nav2_mppi_controller::MPPIController`) with DiffDrive kinematics.
- **Lightweight & High-Throughput**: Consumes $< 1.5\text{ GB}$ VRAM in headless mode (`gui:=false`), ideal for RL / Imitation Learning training pipelines on RTX 4060 (8GB VRAM).
- **Nav2 Stack Integration**: Auto-map resolution and environment-specific AMCL initial pose configuration across all worlds.
- **Comprehensive Evaluation Framework (`evaluation/`)**: Metrics for trajectory length, speed profiles, minimal clearances, jerk smoothness, reaction latency, and Proxemic Space Intrusion (PSI).

---

## 📁 Repository Structure

```text
DynamuhSim/
├── RVO2_SOCIAL_NAVIGATION_GUIDE.md      # Detailed RVO2 / ORCA technical guide & cheat sheet
├── Human_modelize_sync.md               # Technical specification for unified human pipeline
├── BASELINE_TRIALS.md                   # Quick guide for automated trials
├── run_baseline_trials.sh               # Automated batch trial orchestrator
├── record_obstacle.py                   # Ground-truth obstacle pose recorder
├── analyze_baseline.py                  # Single-trial offline metrics CLI facade
├── evaluate_framework.py                # Batch evaluation & algorithm comparison
├── evaluation/                          # Modular evaluation package (PSI, motion, jerk metrics)
├── src/
│   ├── custom_corridor/                 # Main simulation & Nav2 package
│   │   ├── config/                      # Nav2 parameters (nav2_corridor.yaml, nav2_mppi.yaml)
│   │   ├── launch/                      # Launch files (corridor_tb3, nav2_corridor, hospital_arena, nav2_hospital)
│   │   ├── maps/                        # 2D maps (corridor_090, arena_obstacle, school_arena, hospital/)
│   │   ├── models/                      # TurtleBot3 Burger, 3D animated humans (walk_orange, red, blue, trolley)
│   │   ├── rviz/                        # RViz2 visualization setup
│   │   ├── scripts/                     # Helper nodes (goal_pose_bridge.py, hospital_auto_route.py)
│   │   └── worlds/                      # Gazebo SDF worlds (corridors, arena, hospital, school_arena)
│   ├── custom_corridor_plugins/         # C++ Gazebo Sim System plugins
│   │   ├── src/rvo2_human_system.cpp    # In-engine RVO2 / ORCA social navigation plugin
│   │   └── src/rvo2/                    # Core RVO2 / ORCA C++ library
│   └── dynabarn/                        # DynaBARN benchmark submodule
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

### Option 1: Manual Run & Testing

> 💡 *For an exhaustive parameter reference, mathematical details, and benchmark tips, see the [RVO2 Social Navigation Guide](RVO2_SOCIAL_NAVIGATION_GUIDE.md).*

#### Quick Environment Cheat Sheet:
| Environment | Simulation Command (`corridor_tb3` / `hospital_arena`) | Nav2 Command | Goal Pose `(x, y)` |
|---|---|---|---|
| **Corridor (0.90m)** | `ros2 launch custom_corridor corridor_tb3.launch.py width:=0.90` | `ros2 launch custom_corridor nav2_corridor.launch.py map:=corridor_090` | `(20.0, 0.0)` |
| **Arena RVO2 (4 agents)** | `ros2 launch custom_corridor corridor_tb3.launch.py width:=rvo2` | `ros2 launch custom_corridor nav2_corridor.launch.py map:=arena_obstacle` | `(2.0, 0.0)` |
| **School Arena (8 students)** | `ros2 launch custom_corridor corridor_tb3.launch.py width:=school` | `ros2 launch custom_corridor nav2_corridor.launch.py map:=school` | `(10.0, -8.15)` |
| **Hospital Facility (Medium)**| `ros2 launch custom_corridor hospital_arena.launch.py difficulty:=medium` | `ros2 launch custom_corridor nav2_hospital.launch.py map:=medium` | `(8.0, 0.0)` |

---

#### 1. Narrow Corridor (0.90m, 0.70m, 1.20m)
* **Terminal 1 — Gazebo Simulation & TurtleBot3:**
  ```bash
  ros2 launch custom_corridor corridor_tb3.launch.py width:=0.90 obstacle:=human gui:=true rviz:=true
  ```
* **Terminal 2 — Nav2 Stack (DWB / MPPI):**
  ```bash
  ros2 launch custom_corridor nav2_corridor.launch.py map:=corridor_090 controller:=dwb
  ```
* **Terminal 3 — Send Navigation Goal:**
  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 20.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
  ```

#### 2. Open Arena (RVO2 4-Agent, Obstacle, Dataset)
* **Terminal 1 — Gazebo Simulation:**
  ```bash
  ros2 launch custom_corridor corridor_tb3.launch.py width:=rvo2 gui:=true rviz:=true
  ```
* **Terminal 2 — Nav2 Stack:**
  ```bash
  ros2 launch custom_corridor nav2_corridor.launch.py map:=arena_obstacle controller:=dwb
  ```
* **Terminal 3 — Send Navigation Goal:**
  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
  ```

#### 3. School Campus Arena (15 Classrooms, 8 Students)
* **Terminal 1 — Gazebo Simulation:**
  ```bash
  ros2 launch custom_corridor corridor_tb3.launch.py width:=school gui:=true rviz:=true
  ```
* **Terminal 2 — Nav2 Stack:**
  ```bash
  ros2 launch custom_corridor nav2_corridor.launch.py map:=school controller:=dwb
  ```
* **Terminal 3 — Send Navigation Goal:**
  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 10.0, y: -8.15, z: 0.0}, orientation: {w: 1.0}}}}"
  ```

#### 4. Hospital Facility (Easy / Medium / Hard)
* **Terminal 1 — Gazebo Simulation:**
  ```bash
  ros2 launch custom_corridor hospital_arena.launch.py difficulty:=medium gui:=true rviz:=true
  ```
* **Terminal 2 — Nav2 Stack:**
  ```bash
  ros2 launch custom_corridor nav2_hospital.launch.py map:=medium auto_route:=false
  ```
* **Terminal 3 — Send Navigation Goal:**
  ```bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 8.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
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
