# Live SocNavGym group demonstrations

[Completed continuation and measured gaps](reports/recovery01/RECOVERY_REPORT.md): three new walking trials succeeded after an opt-in AMCL subscriber-QoS fix. Combined with the three standing trials, six scene/variant combinations have valid measurements; five meet actual XY error≤0.20m. The [original startup failure](reports/run01/GROUP_GAZEBO_REPORT.md) is retained. The user confirmed GUI drag/zoom and Pause/Play. Rendering uses NVIDIA; robot/Nav2 parameters are unchanged.

From the repository root, the cohort command below creates a NEW report directory. It runs up to six trials sequentially, stops on infrastructure faults, and retains unsuccessful navigation outcomes. Do not run alongside another experiment session.

```bash
python3 experiments/social_navigation/benchmark/group_gazebo_demo/run.py --report-dir experiments/social_navigation/benchmark/group_gazebo_demo/reports/my_run
```

A separate bounded viewer opens a native walking group, stationary robot, no Nav2, X11 Qt, isolated domain/partition. Source the existing ROS/overlay first:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 experiments/social_navigation/benchmark/group_gazebo_demo/viewer.py --output experiments/social_navigation/benchmark/group_gazebo_demo/generated/my_viewer --hold-seconds 600
```

Use a new output directory. The first viewer was visible but ended with a worker-exited exception before the timer, so it is not yet an accepted unattended demo launcher. No system graphics setting or original robot source was changed. See the separate [offline replay probes](../human_response_replay/README.md) for identical scripted robot inputs.

Reproduce the walking continuation with durable AMCL observation (new output directory):

```bash
QT_QPA_PLATFORM=xcb python3 experiments/social_navigation/benchmark/group_gazebo_demo/run.py \
  --conditions counterflow --amcl-observer-qos transient_local --amcl-late-join-check --gui-first \
  --continuation-of experiments/social_navigation/benchmark/group_gazebo_demo/reports/run01/progress.json \
  --report-dir experiments/social_navigation/benchmark/group_gazebo_demo/reports/my_recovery
```

The generic command above retains historical sensor-data QoS by default; use the explicit transient_local option for retained AMCL pose delivery. No localization reset or forced filter update is introduced. The three-second late-join check is passive and occurs before the start gate. The isolated synthetic mechanism check was also executed (3/3 retained samples): [probe evidence](reports/qos_probe01.json).

Analyze the preserved initial and new recovery attempts together:

```bash
python3 experiments/social_navigation/benchmark/group_gazebo_demo/analyze.py \
  --initial experiments/social_navigation/benchmark/group_gazebo_demo/reports/run01 \
  --recovery experiments/social_navigation/benchmark/group_gazebo_demo/reports/recovery01
```

The analyzer checks original user index/outside-worktree preservation against the local checkpoint and11179 file hashes. This audit checkpoint is local to this checkout; on a different checkout, establish an equivalent audit before running, rather than altering historical evidence. Twelve camera PNG/GIF files decode and have recorded SHA256 hashes. These are Gazebo observer camera previews; the separate replay diagrams remain explicitly labelled offline.
