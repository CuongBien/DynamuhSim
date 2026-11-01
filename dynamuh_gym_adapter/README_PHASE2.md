# DynamuhSim PPO direct velocity — addon (preserve your patched real_backend.py)

**This is NEW Phase 2 code**, not code claimed to exist on the GitHub branch. No ROS/Gazebo execution was possible when packaging. The 100+100 uploaded smoke rows pass the original script, but do not validate nonzero motions, human resets, collisions, or learning improvement.

## Integrate without overwriting local changes

```bash
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# unzip dynamuh_phase2_addon.zip somewhere
cp /path/to/dynamuh_phase2_addon/{phase2_common.py,train_ppo.py,eval_ppo.py,motion_probe.py,audit_logs.py} ~/nav_ws/dynamuh_gym_adapter/
cd ~/nav_ws/dynamuh_gym_adapter
# create venv with ROS distro packages still importable (do NOT pip-install rclpy)
python3 -m venv --system-site-packages ~/venvs/dynamuh-ppo
source ~/venvs/dynamuh-ppo/bin/activate
python3 -m pip install 'gymnasium>=0.29,<2' 'stable-baselines3>=2.3,<3' numpy
python3 -c 'import rclpy,gymnasium,stable_baselines3; print("imports OK")'
```

**One worker:** Do not launch `corridor_tb3` or Nav2 yourself. Each env reset starts/stops its one Gazebo process. No separate EvalCallback. Use an isolated `ROS_DOMAIN_ID`/GZ partition as managed by backend. Cross-validate `ros2 topic info /cmd_vel --verbose` and that only the adapter publishes.

## Phase 1: finish real acceptance

```bash
python3 motion_probe.py --episodes 10 --steps 20 --output motion_probe.jsonl
python3 audit_logs.py empty_acceptance.jsonl human_acceptance.jsonl
```

This proves an actual nonzero command affects odom toward goal and robot odom reset; it does **not** prove physical collision truth or RVO2 human reset. For the human cohort, instrument Gazebo world pose info and RVO2 plugin GT for initial human identity/count/positions/velocities and repeated-seed replay; compare across consecutive episodes. Scan for leftover Gazebo process and human plugin after `env.close()`.

## PPO Phase 2: sanity then training

All commands execute **sequentially** (a second evaluator while training would create 2 Gazebo instances and conflict).

```bash
# Optional: inspect Env but do not assert deterministic same-seed scene before checking RVO2 RNG
python3 -m pytest tests -q
# Very short pipeline proof, untrained checkpoint included
python3 train_ppo.py --scenario empty --timesteps 2048 --run-dir runs/empty_smoke
python3 eval_ppo.py --scenario empty --model runs/empty_smoke/untrained.zip --episodes 10 --output runs/empty_smoke/before.jsonl
python3 eval_ppo.py --scenario empty --model runs/empty_smoke/final.zip --episodes 10 --output runs/empty_smoke/after.jsonl

# Longer actual baseline, measure training and compare fixed-seed evaluations (at least 30 episodes)
python3 train_ppo.py --scenario empty --timesteps 20000 --run-dir runs/empty_20k
python3 eval_ppo.py --scenario empty --model runs/empty_20k/untrained.zip --episodes 30 --output runs/empty_20k/before.jsonl
python3 eval_ppo.py --scenario empty --model runs/empty_20k/final.zip --episodes 30 --output runs/empty_20k/after.jsonl

# Only after basic motion and success sanity in empty corridor
python3 train_ppo.py --scenario human --resume runs/empty_20k/final.zip --timesteps 10000 --run-dir runs/human_finetune
python3 eval_ppo.py --scenario human --model runs/empty_20k/final.zip --episodes 30 --output runs/human_finetune/before.jsonl
python3 eval_ppo.py --scenario human --model runs/human_finetune/final.zip --episodes 30 --output runs/human_finetune/after.jsonl
```

Do not infer progress from these budgets alone. 20k transitions at 5Hz are 4000 simulated seconds plus frequent expensive full Gazebo restarts. Use task curriculum, preserve fixed evaluation seed suites, repeat across multiple model seeds. Keep initial and final **same action/observation spaces, same shield, same scenario goal**, and compare success rate, return, time-to-goal, intervention rate. Episode summary files are in `runs/*/episode_metrics.jsonl` and `runs/*/monitor.csv`.

## Model and reward

- PPO `MultiInputPolicy` for vector-only **Dict**: `lidar(72)`, `lidar_mask(72)`, `goal(3)`, `ego(3)`, `prev_action(3)`, `sensor_age(3)`. No `local_bev` and no WM features yet. SB3 automatically concatenates these vector features.
- `action=Box(3)=[vx,vy,wz]`. `env.py` has per-axis rate limiter; an optional **non-certified** directional LiDAR safety gate blocks unsafe/unknown translation and logs `shield_intervened`.
- `reward = 3*progress - .1*sim_dt - .1*||delta_action||² + 20*goal_success`. This is a **provisional** reward. Collision and social costs in your note are **not implemented**; a claimed physical collision rate must be `NOT_MEASURED` until native contact system is validated. LiDAR near miss is not identical to collision truth.
- `terminated`: geometric goal; `truncated`: simulation-time limit. Backend faults **raise**, they are NOT silently converted into PPO timeouts/rewards.
- `Nav2/MPPI`: OFF in Phase A (direct action). `WM`: OFF. Keep both unchanged during baseline experiments.
- `Shield`: the shield reads the **raw** scan (360 rays) from the ROS backend, treats positive infinity as "no return within SDF max 3.5 m" (Gazebo convention to verify), and blocks on unknown/near hits in a directional cone; this is only a conservative prototype and not proven collision-free for omni mecanum, especially rotated footprint, walls or humans.
- ROS sensors actual rates (SDF): LiDAR 5 Hz; depth 10 Hz (not used); IMU 100 Hz, odom 30 Hz. These differ from WM dataset 10 Hz resampling.

## Remaining **mandatory** improvements before reporting scientific results

1. Add real Gazebo contact truth (robot vs static obstacles and human), and termination/reward for collisions; verify actual contacts with controlled bumper scenarios.
2. Prove RVO2 human reset state and sampling independence or deterministic seed behavior. `DYNAMUH_GYM_SEED` currently only records a value, not necessarily a plugin RNG seed.
3. Add stronger goal reach truth from model poses (odom alone may drift), include run completion and actual contact metrics.
4. Improve throughput with validated in-place reset or persistent worker; one Gazebo restart every episode is functional but expensive.
5. Expand scenario diversity (start/goal/corridor widths/human behavior) before claiming social-nav generalization. Current fixed scenario is for pipeline sanity only.
6. Collect published controller trace and odometry to prove actions reached Gazebo. Confirm timeouts and back-pressure under 5Hz /clock stepping.
7. Do not add `local_bev`, WM or discrete supervisor until Phase 2 direct baseline has trustworthy learning/metrics. If BEV is later added, both policies must use consistent architectures for ablations.
