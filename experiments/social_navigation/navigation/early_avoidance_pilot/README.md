# Early avoidance pilot — isolated Nav2 settings

**One valid complete pilot achieved:** avoidance_moderate01, original AMCL/odometry, native standingv3 seed42, native robot friction semantics restored. Nav2 SUCCEEDED, actual goal error14.42cm/0.069rad,87.31sim s, minimum nominal robot–human gap59.25cm, no sampled standing-group hull intrusion, clean NVIDIA shutdown. This is one adaptive pilot, not a success rate or a default configuration promotion. [Report](../../../archive/social_navigation/reports/early_avoidance_pilot/EARLY_AVOIDANCE_REPORT.md).

First read [the correction](../../../archive/social_navigation/reports/early_avoidance_pilot/ERRATA.md): the parent dynamic camera copier rewrote a literal Gazebo friction-frame attribute. Three exploratory runs here used the wrong physics setup and are excluded, with raw outcomes retained. The older completed dataset pilot and src/ robot are unchanged. [Native regression and fix](FRICTION_PREFIX_FIX.md).

## Run in the current Ubuntu24.04/Jazzy workspace

```bash
TASK=experiments/social_navigation/navigation/early_avoidance_pilot
bash "$TASK/run.sh" build
bash "$TASK/run.sh" test
# Re-analyze retained raw data into a new output:
bash "$TASK/run.sh" analyze --capture "$TASK/generated/runs/avoidance_moderate01" --output "$TASK/generated/my_analysis01"
# Audit the locally retained accepted pilot:
bash "$TASK/run.sh" audit --name avoidance_moderate01
```

Uses the existing host Python3.12.3 and sourced ROS/Gazebo vendor SDKs, headless NVIDIA GPU, exclusive domain91 and owned partition cleanup. No new packages, ROS source edits, native human policy changes, system patches or software-render fallback. The experiment runner intentionally opens no GUI; observer PNG/GIF and depth arrays are recorded from the real simulation. Build prerequisite response adapter/filter/worker environments are the existing project dependencies used in the previous pilots.

Moderate profile changes only Nav2 runtime YAML: costmap circle radius0.25m plus padding0.01m, inflation0.8m/scaling3, MPPI vxmax0.30/vxmin−0.15/vymax0.20m/s. Radius-mode CostCritic keeps consider_footprint=false, as required by installed Jazzy. Source footprint/controller settings remain unchanged. Native wheel friction values/frames are restored, not tuned. Standing cap0.2m/s and guard2mm are unchanged. Source camera height remains native low offset; the OFF depth camera has the old simultaneous ON control. No human/group ground truth enters the successful planner; group hulls are offline metrics.

90sim s budget was declared before testing because even8m/0.15mps needs53.3s before detours. It differs from the older45s pilot, so do not compare completion times or label old failures as successes under the new budget. Goal acceptance stays action SUCCEEDED and actual XY≤0.20m/yaw≤0.25rad. The slow corrected-physics profile timed out; it is retained. No automatic retry/tuning loop.

Large raw NPY/MCAP files live locally under generated/runs/. Git contains representative float arrays, real observer images/GIF, manifests, diffs, source snapshots, all outcomes and audit results. A fresh clone lacks full raw data; audit requires the corresponding local evidence. Some auxiliary NPY frames are missing in warmup; all shared OFF frames match raw MCAP bitwise, with exact missing stamps reported.

## Limits and next work

Native groups may disperse while the long episode runs; fixed members later become native individuals.0.2m/s applies to near standing-group responders, not all free walkers, whose original cap remains0.8m/s. The report evaluates actual standing groups and records membership; it does not claim all six people kept conversing for87s. Inflation is geometric, not a semantic group-aware planner. One seed does not certify robust social comfort or dense-crowd behavior.

This pilot is a complete sensor/trajectory record for the next dataset-format check, not train-ready: personas remain unknown, association and future-BEV target still need validation. Keep HuNavSim held out. Next standardize this accepted episode and its configuration/physics provenance into portable sample windows, then a small independent confirmation; no World Model/Isaac or full training batch in this stage.

Cập nhật dọn 27/10/2026: đã gỡ các factory sinh/vá driver slow/fixed/moderate/oracle, helper oracle odometry/localization và test friction đứng riêng bị lặp. Bốn regression trong `test_replay.py` vẫn giữ nguyên, gồm native friction parser. Checker C++, config/namespace helpers và driver moderate đã freeze trong reports còn nguyên. Lượt capture mới dùng [API capture](../../../../docs/CAPTURE_RUNTIME.md); `run.sh live` không còn hỗ trợ. [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/fc3f2d1dd8b41de0538679309d29f43e20da4e3c/experiments/social_navigation/benchmark/early_avoidance_pilot) giữ cách tái lập lịch sử.
