# Continuous, circle-safe SocNavGym lifecycle adapter

[Measured before/after report](reports/run01/FIX_REPORT.md): the experimental repair passes the three original initial scenes, a reset of an active group, and a separate moving-group regression.5476 adapter updates were executed and checked; primary/reset-prefix and moving-regression trajectories are saved, while the active-reset probe saves its measured comparison summary. There are no measured swept circle-proxy penetrations in the accepted tests, formation commits keep positions continuous, and yaw/translation bounds pass. This is a standalone repair; Gazebo formation is not yet connected.

Changes live in continuous_groups.py, bound only to the experimental env instance. Native vendor files, original robot/Nav2 and existing fixed-group Gazebo adapter remain unchanged. Native lifecycle remains responsible for group membership/type/goals; the adapter preserves positions at commit and replaces final motion with a global individual-disc ORCA pass plus an analytic swept-disc fallback. Standing groups hold position and turn inward gradually. This is an explicitly modified behavior model, not a claim that upstream was fixed.

The fallback rejects invalid initial overlaps and moving-robot use; it currently supports an empty native square room only. Furniture and warehouse geometry are not supported. It does not silently push or teleport people apart. Its common-time shortening did not activate in the3 primary runs; geometric fixtures test its tunneling, wall/robot crossing and invalid-start cases separately.

From the repository root, use a NEW report directory. Existing outputs are kept:

```bash
experiments/social_navigation/.venv-socnavgym/bin/python experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/test_swept.py
python3 experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/run.py \
  --report-dir experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/reports/my_run
experiments/social_navigation/.venv-socnavgym/bin/python experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/probe_active_reset.py \
  --report-dir experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/reports/my_run
experiments/social_navigation/.venv-socnavgym/bin/python experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/probe_moving.py \
  --report-dir experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/reports/my_run
python3 experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/analyze.py \
  --report-dir experiments/social_navigation/benchmark/socnavgym_lifecycle_fix/reports/my_run
```

Runs use the existing Python3.12 venv, two CPU affinity slots, one BLAS/OMP thread, sequentially. No new package or ROS Python change. The primary120s scene uses the existing formation.yaml unchanged and exact initial-state equality against the preserved native baseline. Motion changes mean later trajectories and formation timing need not match native.

An independent analyzer checks closest approach over full linear ticks, unique IDs, speed/yaw bounds, zero instantaneous formation position change, actual group formation/dispersal and non-frozen travel. The extra initial-moving-group fixture has9 independent people and1 moving group of3, seed44; it is a separate regression, not another paired native comparison. Its supplemental robot pose is not recorded, so independent robot-gap claims apply to the3 primary episodes.

Reset uses a fresh epoch with reused numeric IDs. The active-group probe resets at step276 and repeats500 steps exactly, including membership; adapter velocity history is cleared. Observation-space warnings remain and RL training validity is not addressed. A500-step repeat is not proof of general1200-step determinism.

GIFs are actual recorded SocNavGym2D renders, explicitly ADAPTER FIX / NOT GAZEBO, at4× simulation speed. They are not camera recordings or AI-generated scenes. The native_preview_4x.gif filename is inherited from the common measurement runner; the fixed variant is explicit in its image header, result.json and this report.

Validation preserves the user's staged changes/outside worktree and11298 vendor, ROS, map, adapter and earlier baseline files against a local audit checkpoint. A fresh checkout needs an equivalent audit checkpoint; do not rewrite old hashes or results. Next: map warehouse obstacles, accept measured robot motion, then test this variant live in Gazebo.
