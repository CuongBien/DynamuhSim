# Depth proxy ablation — isolated sensor experiment

**Limited acceptance: two static scenes, six clean GPU fixtures. The planned third scene is not accepted.** Read [the report](../../../archive/social_navigation/reports/depth_proxy_ablation/DEPTH_PROXY_REPORT.md) and [audit](../../../archive/social_navigation/reports/depth_proxy_ablation/static02/audit.json). No default ROS/Nav2, robot geometry, human behavior, previous dataset or training setting was changed.

The source proxy cylinder is retained for collision and GPU LiDAR. An experiment-only Ogre2 camera listener hides its visual during only the two selected depth cameras, then restores it. Original all-visible depth cameras run alongside them. A separate null-human-visual control proves that actual human mesh remains visible. These are measured GPU depth arrays and deterministic plots, not AI-generated images.

## Run on the existing Ubuntu24.04 + ROS2 Jazzy workspace

Python3.12.3 host environment, Gazebo Harmonic SDK, ROS Jazzy gz-rendering vendor and OgreNext2.3.3. No extra Python venv is required for these ROS sensor captures. Source runtime/SDK must match; apt OgreNext2.3.1 cannot be linked into the active ROS vendor renderer. NVIDIA rendering is mandatory, verified from the actual renderer log for every accepted fixture.

From the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
TASK=experiments/social_navigation/perception/depth_proxy_ablation
# Only if the already-used camera-height actor freezer has not been built:
cmake -S experiments/social_navigation/perception/camera_height_ab \
  -B experiments/social_navigation/perception/camera_height_ab/generated/build
cmake --build experiments/social_navigation/perception/camera_height_ab/generated/build -j1
bash "$TASK/run.sh" build
bash "$TASK/run.sh" test
# New output name required. Start with one scene, not an automatic full benchmark:
bash "$TASK/run.sh" live --output "$TASK/generated/runs/manual01" --snapshot-indices 1
```

The run is headless on GPU: it intentionally does not open a Gazebo GUI. It uses an exclusive domain91 lock and a unique Gazebo partition. Only its own processes are cleaned up. A failed render/shutdown stops the cohort and preserves measurements; do not treat status MEASURED alone as acceptance. No automatic unbounded retry.

Review existing evidence without launching another renderer:

```bash
TASK=experiments/social_navigation/perception/depth_proxy_ablation
bash "$TASK/run.sh" audit --capture "$TASK/generated/accepted_static02" --report "$TASK/reports/static02"
# For fresh analysis use a NEW report directory:
bash "$TASK/run.sh" analyze --capture "$TASK/generated/accepted_static02" --output "$TASK/generated/reanalysis01"
```

The historical scene assembly is complete; its recreation helper has been retired from HEAD. Use the pinned source version below to reproduce that exact assembly.

The recorded assembly validated core physical/acceptance protocol, source/plugin hashes, poses, all three conditions, clean exits and cleanup, then byte-copies and records provenance. Raw run06 and run07 were incomplete runs; only the named complete scenes are accepted. The Git report includes exact representative float32 depth arrays, metrics, attempts, provenance and hashes; large full arrays remain ignored locally.

## Interpretation

Camera heights12/40cm above the floor are comparison variants, not selected defaults. Actors are frozen at recorded standing-group coordinates with the original walk mesh animation at frame0. This tests geometry, not conversation animation or behavior. More visible BEV free area does not establish safe navigation. LiDAR/collision still use a conservative cylinder, while depth sees mesh gaps: this mismatch must remain explicit in data metadata.

Before integration: diagnose the repeatable third-scene shutdown failure and pass the empty-front-view control; then run one isolated moving robot/human sensor pilot and verify per-frame restoration, synchronization and cleanup. Do not launch a new simulator matrix or a training batch from these two static scenes.

Cập nhật dọn 27/10/2026: đã gỡ các công cụ chỉ phục vụ cohort/report cũ: `assemble.py`. [Code trước khi gỡ](https://github.com/CuongBien/DynamuhSim/tree/87b0c6eea9b54794db7b4cb794fae57c78f11c70/experiments/social_navigation/benchmark/depth_proxy_ablation) giữ cách tái lập; report, engine, source freeze và outcome cũ không đổi.
