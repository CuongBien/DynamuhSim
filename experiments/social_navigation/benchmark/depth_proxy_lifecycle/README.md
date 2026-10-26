# Depth filter lifecycle and one moving-robot pilot

**Partial result:** the previously missing third static control passed with an experimental pause-before-stop sequence. The motion sensor prefix and shutdown checks passed, but navigation stopped on the unchanged human avoidance guard at15.2025s. This episode is quarantined, not a successful navigation or training episode. Read [the report](reports/LIFECYCLE_MOTION_REPORT.md).

No source under src/, native simulator policy, robot control, camera height, Nav2 parameters, prior dataset or previous experiment was modified. The original filter binary was reused byte-for-byte. Generated driver differences are [reviewable](reports/motion_driver.diff), with exact implementation snapshots in reports/implementation/.

## Read-only checks on this existing workspace

```bash
TASK=experiments/social_navigation/benchmark/depth_proxy_lifecycle
bash "$TASK/run.sh" audit
bash "$TASK/run.sh" test
# Fresh output directory required for re-analysis:
bash "$TASK/run.sh" analyze --capture "$TASK/generated/runs/motion01" --output "$TASK/generated/reanalysis01"
```

Ubuntu24.04 + ROS2 Jazzy, Python3.12.3, headless D3D12 NVIDIA RTX4060Laptop. ROS/Gazebo host runtime is sourced by run.sh. Heavy raw MCAP and NPY files are ignored locally. Representative arrays, images/GIF, failure fixtures, reports, raw hashes, implementation snapshots and provenance are in Git. An audit requires the local raw evidence; a fresh clone only contains representative evidence.

## Historical acquisition

Các lệnh diagnose/static/motion của cohort đã hoàn tất được gỡ khỏi HEAD, cùng signal-trace C++ helper. Raw evidence, implementation snapshots và outcomes cũ giữ nguyên; phân tích/audit và test infeasible state vẫn còn. Lượt mới dùng [capture API](../../../../docs/CAPTURE_RUNTIME.md), không sinh/vá Python driver.

The first diagnostic01 used a local signal-stack helper preload, exited cleanly and yielded no hung stack; it is not counted as production acceptance. control01 without that helper also exited cleanly. No gdb/strace package, system driver, ptrace policy or global environment was changed.

The new stop helper requests world pause, observes paused WorldStatistics with one stable simulation timestamp, waits0.5s, then calls the existing partition-owned server stop. This is an empirical mitigation, **not a GPU fence or established driver fix**. Three static fixtures and one interrupted dynamic fixture closed cleanly with this helper. The old intermittent hang was not reproduced this turn, so its root cause remains unresolved.

The motion driver is a generated copy of dataset_pilot/trial.py with runtime camera metadata repair, a same-pose ON control camera, the original OFF filter, a passive paired-image recorder and ordered shutdown. Native LiDAR, collision geometry, seed, start/goal, worker and Nav2 configuration remain the existing experiment's settings. It does not publish manual robot commands. The extra camera can change runtime scheduling; this is not a causally matched comparison of navigation trajectories.

## Current next step

Replay the saved failed worker request and evaluate an isolated planner-side early slowdown/group-avoidance treatment before collecting a new complete episode. Preserve the0.2m/s standing cap and2mm guard; do not force humans aside or relabel the failed episode as success. Diagnose/monitor the intermittent shutdown separately, without claiming it resolved. No World Model, Isaac Sim, new framework matrix or large dataset batch in this stage.

Cập nhật dọn 27/10/2026: `motion_prepare.py`, `pair_recording.py`, `quiesce.py` vẫn được capture/freeze sử dụng và giữ nguyên byte. [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/fc3f2d1dd8b41de0538679309d29f43e20da4e3c/experiments/social_navigation/benchmark/depth_proxy_lifecycle) giữ nguồn acquisition/diagnostic lịch sử.
