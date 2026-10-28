# DynamuhSim 🤖

**Cấu trúc social-navigation 28/10/2026:** 29 stage đã chuyển sang [navigation (13)](experiments/social_navigation/navigation/README.md), [perception (11)](experiments/social_navigation/perception/README.md), [datasets (4)](experiments/social_navigation/datasets/README.md) và [replay (1)](experiments/social_navigation/replay/README.md). Runtime, import, reader và đường dẫn report đã cập nhật; không giữ 29 alias thư mục code ở `benchmark`. [Kiểm chứng](docs/verification/capability_layout_2026-10-28.json).

**Generated và lịch sử:** 1.505 generated output cùng hai experiment chỉ còn báo cáo đã externalize sau kiểm archive/restore. [Input chuẩn](experiments/gazebo_benchmark/configs/fixtures/README.md) · [khôi phục payload](docs/history/closed_experiment_payloads.md) · [phạm vi clean và phần giữ lại](docs/CLEANUP_SCOPE.md).

**Navigation end-to-end 28/10/2026:** [kết quả và hai lỗi điều hướng còn lại](docs/verification/cleanup_navigation_2026-10-28.json).

**Cleanup đã chốt 27/10/2026:** [phạm vi, source giảm và phần cần giữ](docs/CLEANUP_SCOPE.md) · [receipt kiểm chứng cuối](docs/verification/cleanup_closure_2026-10-27.json).

**Nghiệm thu dọn repo 26/10/2026:** [kết quả, entrypoint và giới hạn kiểm chứng](docs/CLEANUP_ACCEPTANCE_2026-10-26.md).


**MPPI code dùng chung:** `experiments/gazebo_benchmark/core/mppi_monitor.py` giữ một implementation callback/goal/measurement cho Step1/2/3; ba runner chọn profile tương ứng. `core/mppi_trial.py` dùng chung việc tạo scenario/world, launch, chờ Nav2 và thu kết quả; tái sử dụng helper `core/sim_env.py`. Teardown chỉ dừng các nhóm tiến trình do trial tạo, có thời gian cho ROS launch đóng các node; batch dùng đúng `--domain_id` đã truyền. `core/geometry.py` là nơi duy nhất tính rectangle–circle clearance cho các suite đã chuyển. Các runner này resolve root theo checkout đang chạy.

**Điểm chạy A3/MPPI:** từ root repo, dùng các runner trong `experiments/gazebo_benchmark/suites/`:

- Kiểm tra cơ bản: `correctness_a3/test_a3_1_velocity_tracking.py`, `test_a3_2_fixed_cmdvel_repeatability.py`, `test_mecanum_smoke.py`.
- Điều hướng Nav2: `correctness_a3/test_a3_3_nav2_goal_navigation.py`; replay: `correctness_a3/test_a3_4_mppi_cmdvel_replay.py`.
- Seed 5 lượt: `correctness_a3/seed_determinism_5runs.py`; MPPI: `correctness_a3/benchmark_mppi_reproducibility.py` và `correctness_a3/test_mppi_mecanum_reproducibility.py`.
- Ablation: `mppi_ablation/benchmark_a3_3_step1_vision.py`, `benchmark_a3_3_step2.py`, `benchmark_a3_3_step3.py`; kiểm tra wall clearance dùng `benchmark_a3_3_wall_clearance.py`; batch Step2/3 vẫn dùng `run_step2_full.py`, `run_step3_full.py`.

Pilot sơ bộ, test seed 2 lượt và các logger cố định H3/401, Step1/304 đã nghỉ dùng. Telemetry, costmap snapshot và báo cáo cũ vẫn ở `experiments/gazebo_benchmark/results/` và `experiments/gazebo_benchmark/reports/`; source lịch sử có tại commit `435722e`.

**Canary scaling:** chạy từ root bằng `python3 experiments/gazebo_benchmark/core/benchmark_runner.py --condition canary` (4 cấu hình uncapped, 3 lượt mỗi cấu hình) hoặc `python3 experiments/gazebo_benchmark/suites/scaling_perf/parallel_scaling_benchmark.py --canary` (1–4 instance, N=10). Hai profile giữ nguyên cấu hình và tệp JSON đầu ra của wrapper canary cũ; không dùng cờ canary thì hành vi mặc định của runner giữ nguyên.

**Phân tích corridor cũ:** trong thư mục `experiments/corridor_090`, chạy `python3 analyze_k6_k12.py` cho K6–K12 hoặc thêm `--summary` cho báo cáo ngắn trước đây của `analyze_trial.py`. Hai chế độ giữ riêng cách tính vận tốc/thời gian dừng và dùng chung phần đọc bag; CLI phân tích có ground truth nằm ở `scripts/analyze_baseline.py`; lệnh cũ ở root vẫn được giữ.

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
├── docs/                      # Technical guides, current workflow and verification
├── scripts/                   # Environment, install, baseline and evaluation entrypoints
├── tools/                     # Shared runtime, capture, audit and artifact commands
├── python/dynamuhsim/         # Shared Python implementation
├── evaluation/                # Offline trial metrics
├── experiments/               # Current suites and preserved historical results
├── src/
│   ├── custom_corridor/       # Worlds, robot models, Nav2, launch files and School workflow
│   └── custom_corridor_plugins/
│       ├── src/rvo2_human_system.cpp  # Gazebo integration and ORCA orchestration
│       ├── src/corridor_fsm.cpp      # Recess yielding and wall clamp
│       ├── src/scenario_presets.cpp # Built-in presets and trolley synchronization
│       ├── include/custom_corridor_plugins/ # Shared types and declarations
│       └── src/rvo2/                # ORCA implementation
└── setup_env.sh               # Compatibility entrypoint to scripts/setup_env.sh
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

> 💡 *For an exhaustive parameter reference, mathematical details, and benchmark tips, see the [RVO2 Social Navigation Guide](docs/RVO2_SOCIAL_NAVIGATION_GUIDE.md).*

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

Các lệnh cũ `analyze_baseline.py`, `evaluate_framework.py`, `run_baseline_trials.sh`,
`install_dependencies.sh` và `source setup_env.sh` ở root chỉ chuyển tiếp sang
implementation trong `scripts/`; không có bản sao logic thứ hai.
