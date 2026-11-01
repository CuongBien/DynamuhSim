# Warehouse Human Runtime Validation

Validation date: 2026-11-01, approximately 13:23–13:33 Asia/Ho_Chi_Minh. Host: `DESKTOP-NVIVQ2U` (WSL2). Source HEAD: `fb49887a3655036563b2f916b201f20ccb557bbc`; branch: `feat/depth-detector-generalization`.

**Gazebo started: YES. Runtime evidence collected: YES. Complete navigation/capture acceptance: NO.** Three fresh headless Warehouse runs executed real Gazebo, ROS 2, Nav2 and the six-human native worker. Each was externally interrupted at 60 wall seconds, with up to 20 seconds reserved for cleanup. All three retain timeout exit code **124** and runner `status=ERROR`; no result was relabeled as a successful episode. Startup and real-time factor limited recorded active motion to approximately 14–16 simulated seconds per run.

This validates mechanisms and short-run symptoms, not a social-naturalness acceptance or a full benchmark. The existing `WAREHOUSE_HUMAN_BEHAVIOR_AUDIT.md` was read; its historical observations are not counted as current runtime evidence. Every repository operation and runtime command used MCP `dynamuhsim-wsl.execute_bash`, `auto_source_ros=false`. No algorithm, production source, branch, historical record, or system package was changed.

## 1. Environment and dependency status

Initial worktree path and branch were verified with `pwd`, `git branch --show-current`, `git status --short`, `git rev-parse HEAD`, and `git ls-remote origin refs/heads/feat/depth-detector-generalization`. Target status was clean and local HEAD equaled the remote branch tip. No applicable ancestor or target/docs `AGENTS.md` was found remotely. The report did not exist and had no pending changes. These checks were repeated before writing.

| Dependency | Current-machine evidence / disposition |
|---|---|
| OS | Linux `6.18.33.2-microsoft-standard-WSL2`, x86_64 |
| ROS 2 | Jazzy; `ROS_VERSION=2`; `ros-jazzy-ros2cli 0.32.12-1noble.20260902.102710` |
| Gazebo | Harmonic, `gz sim --versions` = **8.15.0**; Jazzy gz vendor package `0.0.13-1noble.20260905.073557` |
| Nav2 | `ros-jazzy-navigation2 1.3.13-1noble.20260905.072714`; real AMCL, MPPI controller, planner and navigation lifecycle nodes activated |
| ROS/Gazebo bridge | `ros-jazzy-ros-gz-bridge 1.0.24-1noble.20260905.070149`; sensor, pose, clock and commands observed |
| ROS packages | `custom_corridor`, `custom_corridor_plugins`, `nav2_bringup`, `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro` resolved; actual launches also loaded TurtleBot3 resources, TF, ROS image bridge and rosbag recorder |
| Python | System Python **3.12.3**; imports of `rclpy`, `nav2_msgs`, `yaml`, `numpy`, `PIL`, `gz.transport13`, `gz.msgs10.pose_v_pb2` succeeded |
| Native human environment | Existing main-checkout venv: Python 3.12.3, NumPy **1.26.4**, Gymnasium **0.29.1**, PyYAML **6.0.3**, Shapely **2.2.0**, compiled Python-RVO2 import succeeded; SocNavGym import resolved to the installed editable vendor checkout |
| SocNavGym provenance | `/home/cuongbien/DynamuhSim/experiments/social_navigation/vendor/SocNavGym`, revision `95fbc9cebe4357201145a7d0a28c7736c0efe6cf`, vendor Git status clean when checked |
| Renderer | Preflight and session Ogre logs identify **D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)**; hardware preflight passed |
| Gazebo server plugins | Physics, UserCommands, SceneBroadcaster, Imu, Sensors; robot uses built-in MecanumDrive and JointStatePublisher; actual generated world selects `response_experiment::Adapter`, and captures add `depth_proxy_experiment::Filter` |
| Missing worktree prerequisites initially | Response adapter `.so`, vendor mapping, venv mapping; capture additionally lacked depth filter `.so` and native friction checker |
| Resolution | Temporarily linked the existing main-checkout vendor/venv into previously absent paths. Built only this worktree's response adapter, depth filter and friction checker. Existing GL preflight built its small helper automatically. No system installation, full colcon build, training or benchmark |
| Dynamic linking | `ldd` on the newly built response adapter reported no missing libraries; native launches subsequently exercised the selected plugins |
| Optional ComplexityNav | Docker image `dynamuhsim-complexitynav:round3` exists, ID `sha256:edfd591ea7551813419e511b341089ad956b6d3e4c1bd395a326035865ef1d5d`; this worktree lacks `complexitynav_geometry/generated/{native_contract.py,swept.py,layout.json}`. ComplexityNav was **not selected or executed** |

The MCP process already inherited ROS Jazzy and `/home/cuongbien/DynamuhSim/install` paths even with auto-sourcing disabled. This was inspected, not assumed to be a clean shell. Installed launch helpers, Mecanum model, bridge/config and URDF files used here matched this worktree byte-for-byte. The broader launch directory has an unrelated `depth_person_tracker.launch.py` difference; that launcher was not used. Installed package resources were reused, not claimed to be a new full-project build.

Both session-created dependency symlinks were removed after execution, with exact link-target checks; their external targets were untouched. Focused ignored builds and raw runtime artifacts remain available. A future run must recreate those two mappings. Main-checkout unrelated modifications were preserved. An unrelated `environments/runtime/worker.py` process appeared after the validation runs; it was not signalled or restarted.

## 2. Actual Warehouse runtime and exact launch commands

Maintained entrypoints are `tools/nav2_trial.py` and `tools/capture_episode.py`, as documented in `docs/SHARED_NAV2_RUNTIME.md` and `docs/CAPTURE_RUNTIME.md`. Bare `experiments/social_navigation/warehouse/warehouse.launch.py` does not itself start the human worker.

The selected controller was `navigation/socnavgym_group_nav2/worker.py` → `fixture.make_kernel` → SocNavGym native lifecycle / `ContinuousGroups` → `ResponsiveMotion` per-person ORCA and convex projection → C++ response adapter. Native `SocNavEnv_v2.step` calls `compute_orca_interaction_velocities`, then group `update_speed`; legacy profile leaves these methods in place. Worker requests/steps, moving-group membership, current vendor sources and current source/binary provenance establish the active path. Raw upper-level group solver vectors were not separately instrumented.

**Social Nav components actually active:** SocNavGym human lifecycle/group intention, Python-RVO2 downstream safety solve, per-person projection, ROS Nav2 MPPI/AMCL for the robot. **ComplexityNav inactive.** The existence of its image does not demonstrate its execution. `librvo2_human_system.so` was not selected by these generated worlds.

All commands below ran from the target worktree. The initial supported launch attempt was:

```bash
timeout --signal=INT --kill-after=20s 60s env PYTHONDONTWRITEBYTECODE=1   python3 tools/nav2_trial.py --experiment socnavgym_group_nav2   --scenario walking --seed 42   --report-dir /tmp/warehouse_runtime_validation_20261101_attempt0
```

It returned **1** before Gazebo launch: `FileNotFoundError` for `experiments/social_navigation/navigation/socnavgym_response_fix/build/libresponse_adapter.so`. It recorded `server_shutdown.status=NOT_RUNNING` and no trajectories. That failed attempt is preserved separately.

After dependency preparation, the walking trial used the same command with report directory `/tmp/warehouse_runtime_validation_20261101_walking42`; stdout/stderr were saved to `/tmp/warehouse_runtime_validation_20261101_walking42.console.log`.

For captures, define this shorthand exactly from the worktree root:

```bash
B="$PWD/experiments/social_navigation/benchmark/generated/warehouse_runtime_validation_20261101"
```

A fresh ignored `diagnostic_config.json` was copied from `configs/capture/moderate_v3.json`, changing only `sim_budget_s` from 90.0 to **30.0**, as explicitly supported by the capture API. Seed list, camera, speed, footprint, navigation treatment and 3.2-second label tail were retained. No committed config was edited.

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tools/capture_episode.py prepare   --run-dir "$B/head_on_native_wall45" --config "$B/diagnostic_config.json"   --scenario walking --seed 45 --encounter-profile head_on   --human-motion-profile legacy --human-goal-clock native_wall
PYTHONDONTWRITEBYTECODE=1 python3 tools/capture_episode.py verify   --run-dir "$B/head_on_native_wall45"
timeout --signal=INT --kill-after=20s 60s env PYTHONDONTWRITEBYTECODE=1   python3 tools/capture_episode.py run --run-dir "$B/head_on_native_wall45"

PYTHONDONTWRITEBYTECODE=1 python3 tools/capture_episode.py prepare   --run-dir "$B/crossing_sim45" --config "$B/diagnostic_config.json"   --scenario walking --seed 45 --encounter-profile crossing   --human-motion-profile legacy --human-goal-clock sim_v1
PYTHONDONTWRITEBYTECODE=1 python3 tools/capture_episode.py verify   --run-dir "$B/crossing_sim45"
timeout --signal=INT --kill-after=20s 60s env PYTHONDONTWRITEBYTECODE=1   python3 tools/capture_episode.py run --run-dir "$B/crossing_sim45"
```

Both prepare/verify pairs passed. Capture consoles were redirected respectively to `/tmp/warehouse_runtime_validation_20261101_{headon,crossing}.console.log` and preserved in the raw evidence directory. The runner launched `ros2 launch <stage>/warehouse.launch.py gui:=false` for walking42 and `ros2 launch <repo>/tools/capture_warehouse.launch.py gui:=false` for captures. The actual child command was **`gz sim -r -s <run-runtime>/warehouse.sdf`**, confirmed by owned-process shutdown receipts. The SDF path and exact worker argv are recorded in each `result.json`.

## 3. Scenarios executed and coverage limits

| Requested case | Supported scope / execution |
|---|---|
| A. Single human walking | **Not supported by the selected maintained CLI:** population is fixed at six and adapter validates it. No one-human fixture was invented |
| B. Two humans interacting | **Isolated two-human case not supported** by this CLI. Three actual runs exercised two explicit triplets (IDs 1–3 and 4–6), with human-human distances measured. This is group-interaction coverage, not an isolated two-person experiment |
| C. Human encountering robot | **Executed:** head-on and crossing native fixtures with a live Nav2 robot, plus default walking fixture |
| D. Narrow passage | **No dedicated narrow-passage encounter selector** exists in the inspected CLI. Full Warehouse geometry was active, but these runs do not establish a controlled narrow-passage encounter |

No human count, layout, initial condition outside existing selectors, production launch parameter, or robot velocity script was introduced.

## 4. Runtime configuration and duration

All runs used world `social_warehouse`, 42 factory obstacle boxes, six 0.36 m human-radius proxies and matching visual actors, physics dt **0.0025 s**, native/planning dt **0.1 s**, configured real-time factor **1.0**, robot start **(-28,0)** and navigation goal **(-20,0)**. Humans were frozen until the Nav2 goal was accepted. ROS domain was **91**; each run had a unique `dynamuhsim_group_nav2_<PID>` Gazebo partition. The two capture configs used moderate-v3 MPPI speed limits vx +0.30/-0.15 and vy 0.20 m/s, robot planner/response radius 0.25 m, inflation 0.8 m. The baseline pilot retained its existing unmodified Warehouse Nav2 config.

Default walking group centers were (-25.5,0), (-22.5,0), initial goal (-20.5,0). Supported head-on centers were (-24.5,0.35), (-20.5,-0.35), goals (-28.2,0.35), (-27.5,-0.35). Crossing centers were (-25.1,-3.5), (-24.6,4.5), goals (-25.1,4.5), (-24.6,-3.5). Formation radius was 0.72 m; native interaction goal radius was 1.44 m. Legacy human motion and native lifecycle remained active in all three.

| Run | Gate sim s | Recorded world sim range s | Executed active pose range s | Active duration s | Active worker steps |
|---|---:|---|---|---:|---:|
| walking42 | 13.8575 | 0.0025–28.2825 | 13.8675–28.2825 | 14.4150 | 144 |
| head_on_native_wall45 | 13.6875 | 0.0025–28.7450 | 13.7000–28.7450 | 15.0450 | 151 |
| crossing_sim45 | 13.3700 | 0.0025–29.3075 | 13.3750–29.3075 | 15.9325 | 160 |

Worker steps include the last future endpoint, which can extend 0.1 s beyond the final executed pose. Actual movement metrics use the Gazebo pose timestamps, not that future endpoint. Startup time and synchronous backend compute reduced achieved simulation progress; configured RTF=1 is not an achieved RTF claim.

## 5. Actual observed behavior

The default walking fixture translated all six people by path lengths **5.641–7.055 m** during the active window. Two members had stationary intervals up to **1.375 s**; group metadata included moving and standing states. A stop during native lifecycle is not automatically a deadlock.

Head-on produced a close robot-human encounter: minimum sampled center distance **0.658788 m**. Both groups retargeted once. Each person's cumulative absolute yaw was **8.744–9.295 rad** during about 15 s of executed motion. Humans continued to translate while heading changed; this is telemetry evidence, not a visual judgment of animation quality.

Crossing with `sim_v1` produced **no group-goal changes**, yet cumulative absolute yaw was **13.322–13.911 rad/person**. IDs 4–6 had net yaw changes only approximately **0.006, 0.024, 0.017 rad**, despite large total rotation. This demonstrates substantial back-and-forth turning; IDs 1–3 accumulated about -6.27 rad net yaw. These observations warrant an oscillation/spinning investigation but do not uniquely identify the upstream solver as its cause.

At the 0.1-second motion intervals, no consecutive travel-direction jump exceeded 90 degrees when both interval speeds exceeded 0.1 m/s. Thus high cumulative turning here did not manifest as a single abrupt 90-degree reversal. No translating sample above 0.1 m/s had heading/travel disagreement above 90 degrees. Yielding intent, mutual negotiation and persistent deadlock were not directly labelled or proven in these short interrupted runs.

## 6. Measurements and evidence

Authority distinction: `worker_trajectory.jsonl` contains proposed future endpoints after the downstream safety stages, not raw group intent. `ground_truth.jsonl` contains adapter-applied/interpolated states. `gazebo_poses.jsonl` records actual Gazebo cylinder model poses, which are the source of the movement/clearance measurements below. This is kinematic pose execution, not force-based articulated walking.

| Run | Min HH center / surface m | Min HR center / response-circle gap m | Max observed speed m/s | Max absolute yaw rate rad/s | Max sampled vector acceleration m/s² |
|---|---|---|---:|---:|---:|
| walking42 | 1.160983 / 0.440983 | 1.378966 / 0.768966 | 0.800 | 1.570 | 62.841 |
| head_on_native_wall45 | 1.211237 / 0.491237 | 0.658788 / 0.048788 | 0.800 | 1.570 | 59.701 |
| crossing_sim45 | 1.003075 / 0.283075 | 1.167798 / 0.557798 | 0.800 | 1.570 | 39.061 |

HH surface gaps subtract 2×0.36 m. HR response-circle gaps subtract 0.36+0.25 m; these are model-based center-distance clearances, not an exact distance to the robot mesh, contact-force measurements, swept continuous minima, or a social-comfort certification. No sampled overlap occurred under these circle assumptions. Geometry between samples and all robot links was not independently certified.

The backend projection changed its proposed displacements in **2/144**, **21/151**, and **1/160** active steps respectively. Its predicted clearance is a separate signal from measured pose clearance.

For crossing_sim45, the following metrics are entirely from executed Gazebo trajectories except the last column, which differentiates verified 0.1 s endpoint velocities over fully executed intervals:

| Human ID | Path m | Total absolute yaw rad | Moving heading/travel error p95 / max degrees | Max sampled acceleration / deceleration m/s² | Max interval vector acceleration m/s² |
|---|---:|---:|---|---|---:|
| 1 | 3.984 | 13.523 | 21.12 / 62.13 | 26.291 / 33.924 | 5.859 |
| 2 | 4.744 | 13.322 | 37.36 / 41.94 | 21.730 / 24.108 | 3.616 |
| 3 | 5.167 | 13.389 | 17.84 / 34.15 | 16.766 / 12.891 | 2.525 |
| 4 | 3.983 | 13.707 | 26.84 / 57.22 | 28.029 / 23.130 | 4.206 |
| 5 | 3.745 | 13.652 | 24.76 / 40.29 | 31.973 / 20.719 | 4.799 |
| 6 | 4.210 | 13.911 | 49.56 / 54.59 | 19.038 / 27.065 | 4.060 |

Method: retain the last complete pose message at each simulation timestamp and sort timestamps; duplicate counts were **55/81/86** in the respective active windows. Differentiate XY by actual sample dt, differentiate wrapped yaw similarly, and differentiate interval velocity at segment midpoints for acceleration. Moving heading error uses speed >0.1 m/s; a stop uses speed <0.01 m/s. Sampling dt was 2.5–20 ms for walking/head-on and 7.5–20 ms for crossing; it was **not** a complete 400 Hz physics trace. Acceleration maxima near 0.1 s interpolation boundaries depend on sampling cadence; no physical acceleration limit was specified or tested. The less cadence-sensitive maximum interval vector accelerations across humans were **2.076–3.688**, **3.690–7.463**, and **2.525–5.859 m/s²** by run. No invented threshold turns these measurements into a pass/fail rule.

Same-timestamp Gazebo proxy XY matched interpolation reconstructed from worker `before`/`humans` endpoints to maximum errors **7.4e-15**, **8.7e-15**, **1.1e-14 m**. A 2.5 ms lag fit was worse (up to 2 mm). This directly links the recorded executed poses to the selected current adapter/worker trajectory; it does not expose native pre-ORCA intentions or actor skin deformation.

Example measured trajectory, human 1 in crossing (sim seconds, world meters/radians):

| Time | X | Y | Yaw |
|---:|---:|---:|---:|
| 13.3750 | -24.38000 | -3.50000 | 1.57080 |
| 18.3775 | -24.14465 | -1.16139 | 1.60037 |
| 23.3625 | -24.22515 | -0.84858 | 2.80430 |
| 28.3775 | -24.20326 | -0.77031 | 2.80856 |

Full trajectories for all six IDs and the robot are retained, not replaced by these sparse examples. Live observer PNG/GIF frames also exist; no visual naturalness verdict is based on them.

**Evidence inventory:** let `B` denote the directory in section 2. `B/raw_evidence/attempt0/` preserves the initial failed result. `B/raw_evidence/walking42/` preserves its result, launch log, pose/ground-truth/sensor/control JSONL and observer media; original `/tmp` files remain. Its worker/world/config/ready files remain under `experiments/social_navigation/navigation/socnavgym_group_nav2/generated/warehouse_runtime_validation_20261101_walking42/`. The two captures preserve `plan.json`, `preflight.json`, `capture_freeze.json`, `runtime/` and `episode/` under `B/<case>/`. Capture outputs include MCAP, events, scene, sensor/control logs and paired depth artifacts (285 head-on pairs, 289 crossing pairs). Both rosbag recorders returned 0 and reported MCAP metadata present. These are raw diagnostic artifacts, not audited training episodes.

`B/raw_evidence/analyze.py`, `analysis.log`, and `measurements.json` retain the executed analysis and all metrics. `measurements.json` includes SHA256 of each result, launch log, Gazebo trajectory, adapter ground truth, worker trace, readiness, world and Nav2 config. These artifacts are ignored and intentionally not committed. Key trajectory hashes:

- `walking42/gazebo_poses.jsonl`: `d30234af22d3e13b72ddbd52e1cf9ec5135c9369b59011dd178f418479bdd269`
- `head_on_native_wall45/gazebo_poses.jsonl`: `00841970e8eb649ee224212dbd90d06c54e27e5ea6ba411d39b742d8747bfde1`
- `crossing_sim45/gazebo_poses.jsonl`: `b2c62c533cfbe84973f610e61b60198e1af4223ce5b8f2c5c7bc10e9c5e47ec6`

New response adapter SHA256: `524fe8e945daad94319d7ac1a5f6377a5f36cbf6c505bb580e50e2e73102e984`; its source SHA256: `ffbcda748dc1dba4e06beb79c6e8fa93c77da25fbe2348b10f92062c36279abb`. New depth filter SHA256: `b55c0570daf7c5fc7b907b5deeaa2f080d970b9152e5d5c623af7cfe8e014e87`. The capture freezes additionally hash native sources, plugins, config and selected assets.

## 7. Top three suspected root causes

### 1. Unstable group avoidance intent — LIKELY contributor; unique causal attribution UNVERIFIED

**Expected signals:** unchanged group goals/membership but repeated raw group-preference reversal; group-circle overlap versus actual member envelope; smoothed group velocity, downstream ORCA correction and executed yaw/velocity. **Collected:** goals, membership, executed XY/yaw, final projected endpoints and projection corrections. The native legacy group-intention path was selected and native lifecycle advanced; no alternative group profile or ComplexityNav was installed into this path.

Crossing held both group goals fixed throughout the active trace and nevertheless produced 81.504 rad total absolute yaw over six people. This confirms repeated turning can occur without retargeting or native-wall expiry. It supports the audit's suspected upper group-motion problem, but current logging lacks raw native group vectors, solver radius/velocity inputs and individual-native proposals. Noise, native group smoothing, individual intention and downstream ORCA remain competing explanations. This is **correlation**, not proof that the group envelope or solver initialization causes the observed turns.

**Smallest causal experiment:** replay one short captured robot-input sequence with the same seed/native state through legacy and the existing `group_orca_v1` treatment, recording raw group solver inputs/output, smoothed output, member proposal and final guard output; require baseline reproduction first. Hold the downstream guard fixed and separately ablate only envelope radius and initial measured velocity if assigning a unique mechanism. A closed-loop visual A/B alone cannot isolate these causes because robot trajectories can diverge.

### 2. Host-time versus simulation-time expiry — CONFIRMED current runtime integration mismatch and expiry event

**Expected:** native-wall age includes initialization/warmup and advances on host time; sim_v1 age begins at accepted-navigation time and advances only with Gazebo. **Actual native-wall evidence:** first active head-on request at sim **13.7075**, gate **13.6875**, already reported goal age **24.8217 wall s**. At sim **22.9075**, IDs 4–6 had ages approximately **49.7315 wall s**; at **23.0075**, approximately **50.0012 wall s**. At **23.1075**, their goal changed from (-27.5,-0.35) to (-8.0008583305,15.0623817326), and their wall ages reset to about **0.11549 s**. Only **9.4200 simulated seconds** had elapsed since the start gate.

Before that switch the three member distances to the old goal were **3.2897, 2.5570, 1.8828 m**, each outside native default acceptance **width/2 + goal_radius = 0.36 + 1.44 = 1.80 m**. The loaded vendor's `Human.has_reached_goal` uses `pytime.time()-initial_time > 50`; group acceptance requires all members; native `SocNavEnv_v2.step` directly samples and assigns a new group goal when that predicate succeeds. The goal check precedes collision/response integration. The age threshold crossing, non-arrival geometry and immediate reset establish a current native-wall expiry event, rather than merely noticing two unrelated curves.

**Actual sim_v1 evidence:** crossing retained both original goals while wall ages grew **24.3803 → 63.6698 s**. The existing `goal_clock.verify_trace` reconstructed and passed **320 checks, 0 assignments, 0 expired checks**, activation at gate **13.37**, last logged future-state time **29.4075**. That last endpoint is not asserted executed. This verifies the selected clock domain, not a matched-scene A/B estimate of behavior improvement.

The evidence confirms the expiry mechanism; it does **not** prove expiry causes all spinning. Crossing still rotated heavily without any goal changes. **Smallest controlled follow-up:** repeat one identical head-on measured-input/native-state sequence with a 0 versus 25 s pre-gate host delay, then repeat with sim_v1. Compare first expiry simulation timestamps and goal assignment events, preserving RNG and input sequence. Only the timing treatment should vary; separately measure yaw changes following each event.

### 3. Heading, velocity and acceleration coupling — CONFIRMED executed mismatch/discontinuous-velocity mechanism; naturalness causation LIKELY

**Expected signals:** post-guard XY velocity, independent body yaw, wrapped heading/travel error, velocity jumps and acceleration at planning boundaries. These signals were collected from Gazebo, with endpoint interpolation independently reconstructed.

Executed heading/travel disagreement reached **61.48°** (walking), **65.89°** (head-on), **62.13°** (crossing) at speeds above 0.1 m/s. Yaw rate remained at or below **1.57 rad/s**, and translation speed at or below approximately **0.8 m/s**, while planning-interval vector acceleration reached **7.463 m/s²** in head-on. Shorter-cadence Gazebo finite differences reached **62.841 m/s²** in walking. This is consistent with piecewise-linear XY interpolation having independently limited yaw and discontinuous velocity between intervals.

`ResponsiveMotion.integrate` collects target yaw before downstream ORCA/projection, then applies final XY and a separate yaw-rate clamp. `projection.project` does not accept previous executed velocity as an acceleration constraint. The response adapter independently interpolates XY and shortest-angle yaw. Current executed poses exactly follow those endpoints, establishing the integration mechanism, not just a static possibility. No user-defined acceleration or heading-error limit existed, so these measurements are not declared violations of a fabricated limit. Their causal share in perceived unnatural motion remains **LIKELY**, not experimentally isolated.

**Smallest causal experiment:** one controlled 90-degree preferred-velocity change with known previous final velocity, followed by one lateral safety correction. Log pre/post-ORCA, pre/post-projection and executed pose/yaw at physics rate. Reconstruct the velocity jump and yaw mismatch at the exact interval boundary; keep the identical safe endpoint geometry while testing a separately specified reachable-motion contract. No smoothing or algorithm fix was implemented here.

## 8. Reproducibility and recovery instructions

The initial missing adapter error was resolved; no missing OS package blocked the successful native starts. For a fresh run from this worktree, recover only absent dependencies, preserving existing paths:

```bash
cd /home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/depth_identity_guard/generated/generalization_worktree
pwd
git branch --show-current
git status --short
source /opt/ros/jazzy/setup.bash
source /home/cuongbien/DynamuhSim/install/setup.bash
# These two paths were absent initially and are absent again after link cleanup.
# Do not replace a pre-existing file, directory or symlink.
test ! -e experiments/social_navigation/.venv-socnavgym &&   test ! -L experiments/social_navigation/.venv-socnavgym &&   ln -s /home/cuongbien/DynamuhSim/experiments/social_navigation/.venv-socnavgym     experiments/social_navigation/.venv-socnavgym
test ! -e experiments/social_navigation/vendor &&   test ! -L experiments/social_navigation/vendor &&   ln -s /home/cuongbien/DynamuhSim/experiments/social_navigation/vendor     experiments/social_navigation/vendor

# Focused commands actually used; no full project rebuild.
PYTHONDONTWRITEBYTECODE=1 bash experiments/social_navigation/navigation/socnavgym_response_fix/run.sh build
cmake -S experiments/social_navigation/perception/depth_proxy_ablation   -B experiments/social_navigation/perception/depth_proxy_ablation/generated/build_jazzy   -DCMAKE_BUILD_TYPE=Release
cmake --build experiments/social_navigation/perception/depth_proxy_ablation/generated/build_jazzy -j1
mkdir -p python/dynamuhsim/runtime/native/generated
g++ -std=c++17 -O2 python/dynamuhsim/runtime/native/check_friction.cc   -o python/dynamuhsim/runtime/native/generated/check_friction   $(pkg-config --cflags --libs sdformat14)
```

The filter's wrapper expects a target-local `install/setup.bash`, which this worktree lacks. Instead of creating an install tree or rebuilding everything, its exact CMake build operations were run directly in the already inspected Jazzy environment. Response build and filter build both returned 0; friction compilation returned 0. Build logs are retained in `B/raw_evidence/`.

For new captures, choose a **new** ignored directory and create a new config copy; existing prepare/run directories intentionally refuse overwrite. Recreate the diagnostic config with:

```python
import json
from pathlib import Path
p = Path('configs/capture/moderate_v3.json')
c = json.loads(p.read_text())
c['sim_budget_s'] = 30.0
with Path('NEW_IGNORED_DIRECTORY/diagnostic_config.json').open('x') as f:
    json.dump(c, f, indent=2)
    print(file=f)
```

Use the exact supported prepare/verify/run selectors in section 2 with the new paths. Keep the external 60 s timeout and cleanup allowance. A native start is supported by populated Gazebo pose and ground-truth streams, advancing `/clock`, accepted navigation gate and the actual `gz sim` process receipt; CLI launch success alone is insufficient.

Metric recomputation requires only system NumPy and retained artifacts, not dependency symlinks or a simulation restart:

```bash
B="$PWD/experiments/social_navigation/benchmark/generated/warehouse_runtime_validation_20261101"
PYTHONDONTWRITEBYTECODE=1 python3 "$B/raw_evidence/analyze.py"
```

This writes only its derived `measurements.json` under the ignored artifact directory; raw trajectories remain unchanged. Inspect `measurements.json` for all evidence hashes and per-ID distributions.

## 9. Missing dependencies, execution blockers and limitations

- Initial native launch was blocked by a worktree-local adapter binary; vendor/venv mappings were also absent. Focused preparation resolved these for all three native runs. No system-package installation command is needed on the observed machine.
- Current target paths again lack vendor/venv mappings by deliberate session-link cleanup. Recreate them as above before another run; their existing external installations remain intact.
- Optional ComplexityNav host-generated assets remain absent. Its documented recovery command is `python3 experiments/social_navigation/navigation/common_response/preflight.py` after the appropriate pinned native prerequisites are available. It was not run because ComplexityNav was outside the selected group-path validation; no ComplexityNav behavior is claimed.
- External timeout interrupted all three before action completion and before the configured capture tail. Runner status stayed ERROR with `KeyboardInterrupt` in consoles; result `error` can be absent because KeyboardInterrupt bypasses `except Exception`. Each result also retains `ros_teardown_error` reporting that `rcl_shutdown` had already been called. These outcomes preclude full capture/navigation acceptance.
- Despite the interruption, each owned Gazebo process finished cleanly, ROS launch returned **0**, pause acknowledgement and stable-clock receipts were recorded, worker trace had no terminal errors, adapter faults were empty, and ownership cleanup reported **PASS / no remaining session members**. This is cleanup evidence, not an upgrade of the ERROR outcome.
- No single-human, isolated two-human, controlled narrow-passage or active ComplexityNav trial was performed. No raw upper-group intention telemetry, force/contact stream, full actor/proxy visual comparison, physics-rate acceleration trace or statistically repeated outcome was collected.
- Different scenarios, seeds, configs and clock modes mean these runs are **not** a matched algorithm A/B. Robot MPPI/AMCL randomness and closed-loop trajectory differences limit causal comparisons. No deadlock rate, collision probability, yielding policy or naturalness score is inferred.

## 10. Recommended next experiments and repository delivery

Prioritize the matched-input clock-delay experiment, since a concrete native-wall expiry was observed before geometric arrival. Next collect raw group-intent diagnostics on the unchanged-goal crossing case and isolate envelope/velocity initialization. Then exercise the one-boundary heading/acceleration experiment before considering any algorithm change. Add an explicitly supported one/two-human or narrow-passage diagnostic fixture in separately scoped work if those cases are required; do not silently alter the maintained six-person protocol.

Only this Markdown report is intended for Git staging/commit. Raw logs, media, MCAP and focused binaries remain ignored on the remote machine; no historical result is overwritten. Before writing, target status was clean after removing the two session links and HEAD still matched the remote branch tip. Unrelated main-checkout changes, including concurrently appearing files/processes, were not staged or modified. The report-only commit and non-force push are verified separately by Git after report creation; this report does not contain a self-referential commit hash.
