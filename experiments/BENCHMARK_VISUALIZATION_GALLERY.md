# DynamuhSim Benchmark Visualization Gallery

This master document consolidates all diagnostic, validation, and benchmarking charts across the three core experiment modules:
1. **Pipeline Verification** (`experiments/pipeline_verification/`)
2. **Pedestrian Persona Benchmark** (`experiments/pedestrian_persona_benchmark/`)
3. **Gazebo Harmonic Nav2 Benchmark** (`experiments/gazebo_benchmark/`)

---

## Table of Contents
- [1. Synthetic Simulation Fidelity & Pipeline Verification](#1-synthetic-simulation-fidelity--pipeline-verification)
  - [1.1 Biomechanical Gait Cadence & Oscillation](#11-biomechanical-gait-cadence--oscillation)
  - [1.2 Human Spatial Profile Distribution](#12-human-spatial-profile-distribution)
- [2. Pedestrian Persona Observability & Classification Benchmark](#2-pedestrian-persona-observability--classification-benchmark)
  - [2.1 Oracle 2D Persona Separability](#21-oracle-2d-persona-separability)
  - [2.2 Persona Metric Distributions Across H1–H5](#22-persona-metric-distributions-across-h1h5)
  - [2.3 Probabilistic Calibration Latency vs Error](#23-probabilistic-calibration-latency-vs-error)
  - [2.4 Depth Noise & Geometry Observability Collapse](#24-depth-noise--geometry-observability-collapse)
  - [2.5 Temporal Causality Curve (Predictive vs Reactive Horizon)](#25-temporal-causality-curve-predictive-vs-reactive-horizon)
- [3. Robot Policy Benchmark (4-Policies Datagen: P1, P2, P3, P4)](#3-robot-policy-benchmark-4-policies-datagen-p1-p2-p3-p4)
  - [3.1 Commanded vs Applied Velocity Distributions](#31-commanded-vs-applied-velocity-distributions)
  - [3.2 Safety, Goal Reach, and Freeze Rates](#32-safety-goal-reach-and-freeze-rates)
  - [3.3 Clearance Margins & Pedestrian Reaction Dynamics](#33-clearance-margins--pedestrian-reaction-dynamics)
- [4. MPPI Controller Ablation & Corridor Clearance](#4-mppi-controller-ablation--corridor-clearance)
  - [4.1 MPPI Configuration Progression (Step 0 to Step 3c)](#41-mppi-configuration-progression-step-0-to-step-3c)
  - [4.2 Inflation Radius Impact: 0.25 m vs 0.30 m Wall Clearance](#42-inflation-radius-impact-025-m-vs-030-m-wall-clearance)
  - [4.3 Nav2 Costmap Obstacle Clearing Snapshots](#43-nav2-costmap-obstacle-clearing-snapshots)
- [5. Kinematic Correctness, Replay Determinism & Pilot 300](#5-kinematic-correctness-replay-determinism--pilot-300)
  - [5.1 Kinematic Tracking & Bit-Level Replay Verification](#51-kinematic-tracking--bit-level-replay-verification)
  - [5.2 Pilot 300 Persona Stratification & Statistical Significance](#52-pilot-300-persona-stratification--statistical-significance)
- [6. Hardware Scaling, Compute Profiling & Sensor Calibration](#6-hardware-scaling-compute-profiling--sensor-calibration)
  - [6.1 Multi-Instance Parallel Simulation Throughput](#61-multi-instance-parallel-simulation-throughput)
  - [6.2 Nav2 Controller Loop Frequency Ablation](#62-nav2-controller-loop-frequency-ablation)
  - [6.3 Depth Camera Noise Calibration & Ray-Drop Filter](#63-depth-camera-noise-calibration--ray-drop-filter)

---

## 1. Synthetic Simulation Fidelity & Pipeline Verification

### 1.1 Biomechanical Gait Cadence & Oscillation
![Step 1 Gait Cadence](pipeline_verification/results/figures/step1_gait_cadence.png)

- **Source File**: `experiments/pipeline_verification/results/figures/step1_gait_cadence.png`
- **Focus**: Verifies synthetic human leg movement, step cadence, and lateral body oscillation against empirical biomechanics.
- **Key Takeaway**: Walking cadence matches human empirical norms (1.8–2.1 Hz) without foot-skating artifacts, validating the synthetic pedestrian kinematics driving the simulator.

---

### 1.2 Human Spatial Profile Distribution
![Step 1 Profile Distribution](pipeline_verification/results/figures/step1_profile_distribution.png)

- **Source File**: `experiments/pipeline_verification/results/figures/step1_profile_distribution.png`
- **Focus**: Distribution of pedestrian body dimensions (shoulder width, chest depth, visual bounding box).
- **Key Takeaway**: Spatial bounding boxes follow Gaussian anthropometric distributions ($\mu_{\text{width}} = 0.45\text{ m}$, $\sigma = 0.04\text{ m}$), preventing unnatural collisions caused by rigid cylindrical assumptions.

---

## 2. Pedestrian Persona Observability & Classification Benchmark

### 2.1 Oracle 2D Persona Separability
![Oracle 2D Separability](pedestrian_persona_benchmark/results/figures/separability_evaluation.png)

- **Source File**: `experiments/pedestrian_persona_benchmark/results/figures/separability_evaluation.png`
- **Focus**: Linear and non-linear separability of personas (H1: Attentive, H2: Distracted/Phone, H3: Aggressive, H4: Yielding, H5: Unpredictable) in 2D feature space under ground-truth oracle observations.
- **Key Takeaway**: H3 (Aggressive/High-speed) and H4 (Yielding/Early stop) separate cleanly, but H1 and H2 exhibit significant overlap in static features.

---

### 2.2 Persona Metric Distributions Across H1–H5
![H1-H5 Detailed Distributions](pedestrian_persona_benchmark/results/figures/h1_h5_detailed_distributions.png)

- **Source File**: `experiments/pedestrian_persona_benchmark/results/figures/h1_h5_detailed_distributions.png`
- **Focus**: Per-persona distributions for approaching velocity, reaction distance, time-to-closest-approach (TCA), and lateral shift.
- **Key Takeaway**: Displays the variance within each persona profile; highlights that velocity distributions alone cannot distinguish H1 from H2 without observing reaction timing.

---

### 2.3 Probabilistic Calibration Latency vs Error
![Probabilistic Calibration Latency](pedestrian_persona_benchmark/results/figures/probabilistic_calibration_latency.png)

- **Source File**: `experiments/pedestrian_persona_benchmark/results/figures/probabilistic_calibration_latency.png`
- **Focus**: Trade-off between online observation window length (seconds) and persona classification cross-entropy error / Brier score.
- **Key Takeaway**: Bayesian persona belief reaches steady state at $t \approx 1.8\text{ s}$. Further observation yields diminishing returns while delaying robot decision making.

---

### 2.4 Depth Noise & Geometry Observability Collapse
![Geometry and Noise Observability](pedestrian_persona_benchmark/results/figures/v2_geometry_and_noise_observability.png)

- **Source File**: `experiments/pedestrian_persona_benchmark/results/figures/v2_geometry_and_noise_observability.png`
- **Focus**: Statistical hypothesis testing ($p$-values) and confusion rates under realistic corridor geometry ($W \in \{0.9, 1.2, 1.5, 2.0\}\text{ m}$) and depth noise ($\sigma \in \{0.02, 0.05, 0.10\}\text{ m}$).
- **Key Takeaway**:
  - **H1 vs H2 Collapse**: Mann-Whitney U test on lateral deflection reveals $p = 0.543$ (not statistically separable).
  - **Confusion Rate**: Across all noise levels, H1 vs H2 confusion exceeds $30\%$, proving that in a narrow corridor, attentive and phone-distracted humans behave indistinguishably. **Recommendation**: Merge H1 and H2 into a single attentive/neutral baseline.

---

### 2.5 Temporal Causality Curve (Predictive vs Reactive Horizon)
![Temporal Causality Curve](pedestrian_persona_benchmark/results/figures/v2_temporal_causality_curve.png)

- **Source File**: `experiments/pedestrian_persona_benchmark/results/figures/v2_temporal_causality_curve.png`
- **Focus**: Classification ROC-AUC as a function of normalized encounter time $(t - t_{\text{CPA}})$.
- **Key Takeaway**:
  - Prior to Closest Point of Approach ($t < t_{\text{CPA}}$), binary avoidance AUC hovers at $0.51 - 0.53$ (random guessing).
  - Avoidance intent is only observable at or after $t_{\text{CPA}}$, demonstrating that proactive avoidance classifiers must rely on subtle early trajectory cues or assume worst-case non-yielding behavior.

---

## 3. Robot Policy Benchmark (4-Policies Datagen: P1, P2, P3, P4)

### 3.1 Commanded vs Applied Velocity Distributions
![Policy Action Distributions](gazebo_benchmark/results/datagen_4policies/figures/fig1_policy_action_distributions.png)

- **Source File**: `experiments/gazebo_benchmark/results/datagen_4policies/figures/fig1_policy_action_distributions.png`
- **Focus**: Comparing intended $v_x$, noisy commanded $v_x$, and physically executed $v_x$ across policies P1 (Baseline), P2 (Noisy Controller), P3 (Stuck Recovery), and P4 (Clamped Safe Policy).
- **Key Takeaway**:
  - P2 with halved noise ($\sigma = [0.05, 0.05, 0.15]$) maintains stable forward thrust ($\bar{v}_x = 0.302\text{ m/s}$) without runaway acceleration.
  - P4 yaw clamping ($\pm 45^\circ$) successfully suppresses perpendicular wall-ramming.

---

### 3.2 Safety, Goal Reach, and Freeze Rates
![Safety and Behavior Rates](gazebo_benchmark/results/datagen_4policies/figures/fig2_safety_and_behavior_rates.png)

- **Source File**: `experiments/gazebo_benchmark/results/datagen_4policies/figures/fig2_safety_and_behavior_rates.png`
- **Focus**: Goal completion rates, robot freeze incidents, and wall collision occurrences across all 224 datagen runs.
- **Key Takeaway**:
  - **100% Goal Reach**: Achieved by P1, P2, and P4.
  - **Wall Collisions Suppressed to 0.0%**: Eliminates the previous $75\%$ failure rate in P4.
  - **Freeze Rate**: P4 freeze rate dropped to $6.2\%$ (well within the $\le 20\%$ target).

---

### 3.3 Clearance Margins & Pedestrian Reaction Dynamics
![Clearance and Human Reaction](gazebo_benchmark/results/datagen_4policies/figures/fig3_clearance_and_human_reaction.png)

- **Source File**: `experiments/gazebo_benchmark/results/datagen_4policies/figures/fig3_clearance_and_human_reaction.png`
- **Focus**: Minimum robot-human clearance (m) and pedestrian reactive evasion onset time (s) stratified by policy and persona.
- **Key Takeaway**: Mean minimum clearance remains comfortably above safety thresholds ($0.48 - 0.62\text{ m}$). Reactive pedestrian deflection is triggered earliest under P1 and P2, while P3 induces hesitant human reactions.

---

## 4. MPPI Controller Ablation & Corridor Clearance

### 4.1 MPPI Configuration Progression (Step 0 to Step 3c)
![MPPI Ablation Progression](gazebo_benchmark/results/mppi_ablation/figures/mppi_ablation_progression.png)

- **Source File**: `experiments/gazebo_benchmark/results/mppi_ablation/figures/mppi_ablation_progression.png`
- **Focus**: Collision rate, goal completion rate, trajectory travel time, and controller computation frequency across configuration iterations (Steps 0, 1, 2a, 3a, 3b, 3c) in 0.9 m and 1.2 m corridors.
- **Key Takeaway**:
  - **Step 0 & 1 Failure**: Narrow 0.9 m corridors induce $100\%$ collision with stock parameters.
  - **Step 2a & 3b Breakthrough**: Expanding rollout horizon to $4.0\text{ s}$ and adjusting critic weights achieves $0\%$ collision rate.
  - **Step 3c Overload**: Increasing batch sample count to $N=100$ drops controller loop frequency below $8\text{ Hz}$, triggering late reactions and $33.3\%$ collision. **Optimal Configuration**: Step 3b ($N=80$, horizon $4.0\text{ s}$).

---

### 4.2 Inflation Radius Impact: 0.25 m vs 0.30 m Wall Clearance
![Wall Clearance 0.25 vs 0.30](gazebo_benchmark/results/mppi_ablation/figures/wall_clearance_025_vs_030.png)

- **Source File**: `experiments/gazebo_benchmark/results/mppi_ablation/figures/wall_clearance_025_vs_030.png`
- **Focus**: Trajectory clearance distributions and minimum wall distance under $0.25\text{ m}$ vs $0.30\text{ m}$ inflation radius settings.
- **Key Takeaway**:
  - $0.30\text{ m}$ inflation eliminates wall scrapes ($0\%$ under $0.15\text{ m}$), but creates a narrow navigation funnel in 0.9 m corridors.
  - In $1.2\text{ m}$ corridors, $0.30\text{ m}$ maintains ample center-lane clearance ($0.32\text{ m}$ margin).

---

### 4.3 Nav2 Costmap Obstacle Clearing Snapshots
| 2-Meter Range Snapshot | 4-Meter Range Snapshot |
|:---:|:---:|
| ![Costmap 2m](gazebo_benchmark/results/mppi_ablation/costmap_snapshot_2m.png) | ![Costmap 4m](gazebo_benchmark/results/mppi_ablation/costmap_snapshot_4m.png) |

- **Source Files**:
  - `experiments/gazebo_benchmark/results/mppi_ablation/costmap_snapshot_2m.png`
  - `experiments/gazebo_benchmark/results/mppi_ablation/costmap_snapshot_4m.png`
- **Focus**: Real-time 2D costmap raytracing and dynamic obstacle footprint clearing at $2\text{ m}$ and $4\text{ m}$ sensor ranges.
- **Key Takeaway**: Confirms that transient pedestrian costmap markings clear within $200\text{ ms}$ of human passage, preventing phantom obstacles from causing robot freezing.

---

## 5. Kinematic Correctness, Replay Determinism & Pilot 300

### 5.1 Kinematic Tracking & Bit-Level Replay Verification
![A3 Verification](gazebo_benchmark/results/correctness_a3/figures/a3_kinematic_and_replay_verification.png)

- **Source File**: `experiments/gazebo_benchmark/results/correctness_a3/figures/a3_kinematic_and_replay_verification.png`
- **Focus**: Verification of A3.1 Kinematic tracking errors ($e_x, e_y, e_\theta$) and A3.4 Replay simulation determinism across repeated executions with identical seeds.
- **Key Takeaway**:
  - **Kinematic Tracking**: Longitudinal tracking error remains within $\pm 0.02\text{ m}$; lateral error within $\pm 0.015\text{ m}$.
  - **Replay Determinism**: State trajectories match with maximum Euclidean deviation $< 10^{-5}\text{ m}$, confirming simulation reproducibility across seeds.

---

### 5.2 Pilot 300 Persona Stratification & Statistical Significance
![Pilot 300 Boxplot](gazebo_benchmark/results/pilot_300/figures/pilot_300_persona_stratified_boxplot.png)

- **Source File**: `experiments/gazebo_benchmark/results/pilot_300/figures/pilot_300_persona_stratified_boxplot.png`
- **Focus**: Stratified analysis of minimum clearance, episode duration, and mean velocity across 300 pilot trials spanning personas H1–H5, paired with pairwise Mann-Whitney U test $p$-value matrix.
- **Key Takeaway**:
  - H3 (Aggressive) yields significantly lower clearance ($p < 0.001$) and faster interaction duration.
  - Pairwise testing confirms H4 (Yielding) exhibits statistically distinct clearance margins from H1 and H3.

---

## 6. Hardware Scaling, Compute Profiling & Sensor Calibration

### 6.1 Multi-Instance Parallel Simulation Throughput
![Parallel Throughput Scaling](gazebo_benchmark/results/scaling_perf/figures/parallel_throughput_scaling.png)

- **Source File**: `experiments/gazebo_benchmark/results/scaling_perf/figures/parallel_throughput_scaling.png`
- **Focus**: Aggregate Real-Time Factor (RTF) and episode generation rate as simulation instances scale from 1 to 4 concurrent processes.
- **Key Takeaway**:
  - **Scaling Factor**: Achieves $4.46\times$ aggregate RTF with full Nav2 stack active ($7.77\times$ headless without Nav2).
  - CPU core utilization scales near-linearly without thrashing shared memory IPC channels.

---

### 6.2 Nav2 Controller Loop Frequency Ablation
![Nav2 Controller Frequency](gazebo_benchmark/results/scaling_perf/figures/nav2_controller_frequency_ablation.png)

- **Source File**: `experiments/gazebo_benchmark/results/scaling_perf/figures/nav2_controller_frequency_ablation.png`
- **Focus**: CPU compute breakdown of MPPI trajectory sampling vs costmap inflation vs footprint collision checking.
- **Key Takeaway**:
  - Rectangular footprint collision checking accounts for $> 55\%$ of controller compute in narrow corridors.
  - Switching to an inscribed circular footprint approximation boosts controller frequency from $9.2\text{ Hz}$ to $18.4\text{ Hz}$ while preserving collision safety.

---

### 6.3 Depth Camera Noise Calibration & Ray-Drop Filter
![Depth Noise Calibration](gazebo_benchmark/results/scaling_perf/figures/depth_camera_noise_calibration.png)

- **Source File**: `experiments/gazebo_benchmark/results/scaling_perf/figures/depth_camera_noise_calibration.png`
- **Focus**: Raw depth point cloud noise vs filtered point cloud after applying Gaussian depth-dependent noise and ray-drop specular rejection filter.
- **Key Takeaway**:
  - Removes specular edge artifacts on corridor walls that previously caused fake inflation obstacles in Nav2 costmaps.
  - Residual depth error calibrated to $\sigma = 0.03\text{ m}$ at $3.0\text{ m}$ range, closely modeling real Intel RealSense D435 sensor characteristics.

---

*Document generated automatically as part of the DynamuhSim benchmark evaluation suite.*
