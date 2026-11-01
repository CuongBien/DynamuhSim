# DynamuhSim Gymnasium adapter — proof of loop (NOT yet Gazebo certified)

Branch audited: `test/gazebo-harmonic-benchmark`, commit `9833565fa546d55cb0e553f8388bf4a703c59583`.
The package is **new code supplied for integration**; it is NOT claimed to exist in the upstream repository.

## What is implemented

- Gymnasium API `reset(seed, options)`, `step(Box[3])`, `close`, fixed `action_space` / `observation_space`.
- Single owned ROS2/Gazebo launch per episode, **never more than one active process** owned by this adapter; full stop/start on each reset to avoid persistent human/robot physics state. This is safe-first, NOT a fast in-place reset.
- World `corridor_0.90`, Gazebo Harmonic native physics `max_step_size = 0.004`, policy `0.2s = 50 world-control multi_step ticks` (PAUSED stepping; no wall-clock sleep-as-physics).
- ROS2 Jazzy subscriptions `/clock`, `/scan`, `/odom`, `/imu` with freshness/epoch checks, `TwistStamped /cmd_vel` publisher.
- Observations: `lidar (72,)`, `lidar_mask (72,)`, `goal (3,)`, `ego (3,)`, `prev_action (3,)`, `sensor_age (3,)`. Type float32. No future GT leaks to actor.
- Online **provisional** reward `3 * distance_progress - .1 * sim_elapsed - .1 * squared_action_change + 20 * goal_reached`. `terminated=actual geometric goal`, `truncated=sim time limit`. **No physical collision-ground-truth or social cost** in this proof-of-loop.
- `empty` or `human` scenario: `empty` removes human plugins via existing `obstacle:=none`; `human` uses the existing RVO2 corridor human plugin, **NOT SocNavGym/HuNavSim**. Seed provenance is recorded but RNG determinism for RVO2 not audited.
- Fail-closed `SimulatorFault` for stale sensors, wrong episode, clock mismatch, invalid observation, dead launch or world-control errors; invalid rollouts should be discarded by training supervisor.

## Dependency, setup, and launch

Host MUST have Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic, `custom_corridor` and `custom_corridor_plugins` built, python3 (`rclpy`, numpy, gymnasium, pytest, rosgraph_msgs, sensor_msgs, nav_msgs, geometry_msgs). Run this script in a shell where the ROS2 Python environment is importable; use apt/venv with `--system-site-packages` as appropriate.

```bash
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# Copy this entire dynamuh_gym_adapter directory into ~/nav_ws/
python3 -m pip install gymnasium stable-baselines3 pytest  # use an appropriate venv
cd ~/nav_ws/dynamuh_gym_adapter
python3 -m pytest tests -q

# Real Gazebo smoke acceptance. These are NOT run by package creation.
python3 smoke_gazebo.py --scenario empty --episodes 100 --steps 3 --domain 61 --output empty_acceptance.jsonl
python3 smoke_gazebo.py --scenario human --episodes 100 --steps 3 --domain 61 --output human_acceptance.jsonl
```

**Do not launch Nav2/MPPI concurrently with direct-action PPO**. The base `corridor_tb3.launch.py` publishes Gazebo bridge and robot; this adapter is the sole `/cmd_vel` publisher. Later for supervisory PPO, Nav2 outputs must be remapped to `/nav2/cmd_vel`, with one arbiter handling `/cmd_vel`.

Minimal compatibility smoke:

```python
from env import DynamuhGazeboEnv
import numpy as np
with DynamuhGazeboEnv() as env:
    observation, info = env.reset(seed=11, options={
       'scenario': 'empty', 'start': (-13., 0., 0.), 'goal': (-7., 0.)})
    observation, reward, terminated, truncated, info = env.step(np.zeros(3, np.float32))
```

## Important protocol differences from the requested note

| Field | This deliverable | Note / future step |
|---|---|---|
| Simulator | `corridor_0.90` | WM cohort previously used a separate warehouse world with SocNavGym worker; this is **not** a reproduction of dataset01 training environment. |
| physics | `0.004s` native `corridor_090.sdf` | `0.2s` at 5 Hz equals **50 substeps**, not 10. The WM data-capture driver overrides its warehouse world to `0.0025s`: **80 substeps** for the same policy step. Never reuse 50 unconditionally on the warehouse world. |
| Nav2/MPPI | **OFF** in Phase A direct-velocity Box(3) proof-of-loop | 20 Hz MPPI and Discrete supervisory actions are **Phase B, not implemented**. |
| Reset | strict full process teardown/start, one owned Gazebo session at a time | Fast **in-place** reset is not yet safe-verified, especially for RVO2 / SocNavGym; 100 real resets can be slow. |
| Seed | Exposed in API, logged as unverified | RVO2 plugin seed determinism needs explicit instrumentation and paired replay before claimed. |
| Sensor cadence | Actual robot SDF: LiDAR 5Hz, depth 10Hz, IMU 100Hz, odom publication 30Hz | Dataset history resamples many modalities at 10Hz; sampling rate != native sensor output rate. |
| Observations | 72-bin scan and ego+goal, no depth projection or local BEV | `local_bev (2,64,64)` omitted until online depth projection and validity are ported. `sensor_age` is 3. WM 2s history/3s forecast not connected. |
| Ground truth reward | Progress, elapsed, smoothness, goal only | The full proposed contact / social / deadlock rewards require truthful Gazebo contacts, person poses, and validation; dataset01 RL reward is NaN. |
| Termination | geometric goal or simulation-time limit | Contact-based failure not implemented; no fabricated collision events. |
| Tests | 100 mock resets + 300 mock steps (plus other unit tests) pass locally; real script provided | **No Gazebo/ROS** in creation environment; real 100 resets and 100 human resets must be run on target workstation. |

## Acceptance checklist for 100 *real* Gazebo episodes

All 100 rows per scenario in generated JSONL should have `status=PASS`, no run crashes or hangs, each `epoch` unique/monotonic, positive 0.2±0.008s simulation delta per action, valid non-stale /scan,/odom,/imu, observation space `.contains()` true, finite reward, and no lingering `ros2 launch`/Gazebo process after `close()`. For human scenario additionally verify robot/human reset and human plugin behavior from native model poses; the provided smoke script does **not** yet verify human poses or physical collisions and therefore is not complete scientific acceptance.

## Known issues to verify on the real machine

1. The Gazebo `WorldControl` request/response format and clock tick behavior (the service is `/world/corridor_0.90/control`, `gz.msgs.WorldControl`, `multi_step: 50`), including whether sensors update while the paused world is briefly stepped.
2. `ros_gz_sim create` robot spawning, ROS image/sensor bridge startup, `/odom` in the initial bootstrap phase.
3. RVO2 human worker ownership/reset and deterministic seeds; plugin-generated human motion may differ between runs.
4. Pose-frame agreement: goal geometry uses start/goal displacement transformed to the *initial* odom frame. Audit against Gazebo world-truth and AMCL before publishing research metrics.
5. Unit-test gym shim is used **only if** Gymnasium is missing from the test machine. The runtime `env.py` unconditionally requires *real* Gymnasium.
6. `TwistStamped` delivery while paused is DDS-asynchronous; 40 ms wall time exists for network delivery before stepping and is NOT counted as physics. Audit accepted command vs executed odometry.
7. No Nav2 20 Hz, world-model 2s history, 3s predictions, actual physical collision contacts, social cost, robot occupancy BEV, or PPO optimizer is bundled. This package is an **adapter milestone**.

## Exact source maps

- `src/custom_corridor/worlds/corridor_090.sdf`: `<world name="corridor_0.90">`, `<max_step_size>0.004</max_step_size>`
- `src/custom_corridor/launch/corridor_tb3.launch.py`: `paused`, `gui`, `rviz`, robot spawn, `obstacle` flags.
- `src/custom_corridor/config/mecanum_bridge.yaml`: ROS `TwistStamped` `/cmd_vel`→Gazebo `Twist`.
- `src/custom_corridor/config/nav2_mppi_mecanum.yaml`: controller frequency 20Hz, Omni model (only used later).
- `experiments/social_navigation/benchmark/wm_episode_batch/reports/SCHEMA.md`: history 20x10Hz, depth 240x424, 5Hz native LiDAR, RL reward not valid.
