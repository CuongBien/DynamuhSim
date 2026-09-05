# Automated baseline trials

Run one or more lifecycle-managed trials from the workspace root:

```bash
./run_baseline_trials.sh 10
```

Defaults preserve the corridor setup and use trials starting at `trial_03`:

- `WIDTH=0.90`
- `GOAL_X=11.74`, `GOAL_Y=0.0`
- `TRIAL_TIMEOUT_S=180`
- `READY_TIMEOUT_S=45`

Override the start number or goal without editing the script:

```bash
START_TRIAL=4 GOAL_X=11.74 GOAL_Y=0 ./run_baseline_trials.sh 10
```

Each valid trial writes:

- `baseline_01/trial_N/` rosbag
- `baseline_01/obstacle_ground_truth_N.csv`
- `baseline_01/trial_N/analysis/` v1 analyzer output
- `baseline_01/trial_N/trial_result.json`

Lifecycle outcomes are `SUCCESS`, `FAILURE`, `TIMEOUT`, or `INVALID`. A trial is not completed by elapsed wall time alone: action result is checked, with timeout only as a fallback. Readiness requires `/clock` and `/bt_navigator` lifecycle state `active`.

Summarize directly from the baseline directory:

```bash
python3 evaluate_framework.py batch \
  --base ~/nav_ws/experiments/corridor_090/baseline_01
```

`collision_contact` is currently a clearance-based proxy (`estimated_clearance <= 0`), not an authoritative contact sensor measurement. `INVALID` trials remain in the trial table but are excluded from aggregates.
