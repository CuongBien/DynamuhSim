# Seed46–48 walking and standing regression

This frozen nine-cell cohort checks the existing group_cohesion_v2/sim_v1 walking candidate and unchanged standing sidestep pipeline with the validated bridge shutdown ordering. plan.json declares all cells and thresholds before execution. No runtime/human/Nav2 source changes. Historical failures remain preserved.

Six walking cells use two initial encounter layouts ×three seeds. Three standing cells use the factory conversation groups with legacy standing motion and sim_v1 clock. Standing is not a test of moving cohesion and does not require direct-goal/cohesion traces. Native observations are not speech/audio or articulated social conversation ground truth.

Walking yaw/cohesion/backwards thresholds are fixed numerical regression limits derived from historical seed45 ORCA/sim_v1, with the same original .5×yaw, radius+.25m, backwards+.05 rules. New seeds do not have matched fresh ORCA baselines; never call passing these limits a measured percentage improvement on seeds46–48. MPPI randomness remains unpinned.

All cells require full capture/source/GPU/sensor/TF/costmap/clock/shutdown audit, successful world goal, no recovery/progress failure, near encounters and positive continuous geometric clearances. Standing additionally independently checks unchanged membership, per-step .2m/s sidestep cap, no displacement outside yielding set and existing yaw bound. Maximum total sidestep/group spread is descriptive; no unpublished limit is added after observing results. Geometry is not contact-force truth.

Run once from ROS Jazzy environment with CPU thread caps1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 docs/demos/2026-10-29/seed_regression/run_cohort.py
```

No raw run directory or audit is overwritten. Reports/video analysis are read-only and use a fresh output directory. No dataset/default promotion or WM/RL trainer is introduced in this cohort.
