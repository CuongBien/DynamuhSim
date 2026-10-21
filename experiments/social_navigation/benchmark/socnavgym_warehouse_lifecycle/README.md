# SocNavGym lifecycle in the Gazebo factory

[Measured report and camera GIF](reports/WAREHOUSE_LIFECYCLE_REPORT.md): six persistent human IDs, standing/moving formation and dissolution, measured robot feedback, all42 factory AABBs, proxy interpolation, LiDAR, pause/resume and NVIDIA rendering. Two120s live trials are retained. Seed42 has births only (partial lifecycle coverage); seed43 also shows three releases and a moving group travelling2.17m before dissolution.

This is an opt-in adapter repair, not unmodified upstream behavior. Native social lifecycle is retained; global individual-disc ORCA, bounded turns, continuous formation commits and a swept geometry filter modify movement. Existing robot/Nav2 source, vendor and old fixed-group adapter remain unchanged.

Use the existing ROS Jazzy/install overlay and Python3.12 SocNavGym venv. From the repository root:

```bash
bash experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/run.sh build
python3 experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/test_geometry.py
```

The recorded offline02 protocol matches the current kernel/motion/geometry/config and must match before a live launch. For a separate offline reproduction (three sequential120s traces; new output directory):

```bash
bash experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/run.sh offline \
  --report-dir experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/reports/my_offline
bash experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/run.sh analyze \
  --offline-only --offline-dir experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/reports/my_offline
```

Recorded live protocols, run one at a time with NEW directory names:

```bash
bash experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/run.sh live \
  --gui --report-dir experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/reports/my_live42
bash experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/run.sh live-seed \
  --seed 43 --gui --report-dir experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/reports/my_live43
bash experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/run.sh analyze \
  --live experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/reports/my_live43
```

Each live trial publishes its own bounded robot velocity commands, then holds the robot still. It runs without Nav2 and must have exclusive use of ROS domain90. Gazebo partition is unique. The native worker is restricted to two CPU affinity slots and one BLAS thread; Gazebo uses verified NVIDIA graphics. No CUDA rewrite of ORCA is claimed.

Humans start at world x−25/−23/−21, y±1. Native sampling uses local origin(−24,0), map/sampling window8×4m, goals x[−27.5,−20.5],y[−1.5,1.5]. These sampling limits do not add fake walls. All42 real collision AABBs map to native walls and exact swept checks; the benchmark does not yet cover the entire60×40m floor or bottleneck. Wall observation space is extended on this env instance to42segments; RL observation/training compatibility is not certified.

Clock contract: at simulation time t, worker plans an endpoint valid at t+0.1s using the measured robot velocity; plugin interpolates at physics ticks. Actual SceneBroadcaster poses are independently checked against the plan and against all boxes/humans/robot. Robot is never moved by the human solver. A robot trajectory is only predicted for the next tick; arbitrary incoming motion can make a safe human proposal infeasible, in which case the adapter reports failure. In-place Gazebo reset is unsupported and refused; restart a fresh session. The two recorded sessions both begin with fresh IDs/clock and end cleanly.

Pilot results do not establish general safety or subjective naturalness. Native conversation means standing/inward-facing layout, not speech. Walking mesh animation currently holds when speed is zero; a separate standing animation clip has not been added. CrowdSimPlus/ComplexityNav are not run in this cohort.

Camera GIFs are recorded Gazebo RGB,4× playback. Raw JSONL remain local and have byte-identical gzip archives with SHA256 under each live report; worker future-endpoint traces and exact generated SDF are included too. Analyzer manifests hash the source/media/data. Local preservation checkpoints verify11355 files and the original user index/outside-worktree; on another checkout establish an equivalent checkpoint before audit, rather than changing old evidence. Failed offline01 schema startup and its matching source snapshot remain available.
