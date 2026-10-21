# Baseline versus LF-prob+beam-skip, three fixture seeds

Fixed12 trials: seed42/43/44 × baseline/LF-prob+beam-skip × SFM/ORCA+. One trial per cell, no retry/substitution of failed goals. Reverse policy/profile ordering in seed43. Headless,75 sim s budget,15wall s cooldown. Stop on infrastructure faults; keep ABORTED/TIMEOUT as navigation outcomes.

```bash
python3 experiments/social_navigation/benchmark/localization_validation/run.py \
  --report-dir experiments/social_navigation/benchmark/localization_validation/reports/my_run
# Only after all12 trials finish:
python3 experiments/social_navigation/benchmark/localization_validation/analyze.py \
  --report-dir experiments/social_navigation/benchmark/localization_validation/reports/my_run
python3 experiments/social_navigation/benchmark/localization_validation/validate.py \
  --report-dir experiments/social_navigation/benchmark/localization_validation/reports/my_run
```

Run from repository root in Ubuntu/WSL. Use a new directory, existing CrowdSimPlus image and no other experiment session on ROS domain75. The runner sources ROS/overlay via benchmark/run.sh. Validation of user Git preservation uses the local checkpoint created before this cohort; copy its outside/index snapshots if reproducing on another checkout, rather than treating a missing historical checkpoint as a simulation failure.

[Run01 report](reports/run01/MULTISEED_REPORT.md). Images/GIF are observer camera frames recorded from Gazebo; comparison plots use recorded data. No AI image generation. Large logs/diagnostics remain in ignored runtime directories; manifest retains hashes and runtime paths.

Seed42/43 scene bytes stay identical to prior fixtures. The new seed44 uses the same deterministic bottleneck sampler and geometric validation. Only fixed radius/speed/robot metadata comes from the seed42 template; it is not a recorded SocNavGym44 run. `fixture_checks.json` records SHA and route clearance. No native policy parameters, ROS robot source or baseline Nav2 YAML changes.

Report action status separately from actual final XY error (threshold0.20m), AMCL error, passages, signed gaps and overlap duration. A goal is counted as actually reached only when Nav2 reports SUCCEEDED and final actual XY is within0.20m. This is not a full-pose/yaw check or continuous contact guarantee.

This cohort compares the original laser model with LF-prob plus beam skipping, so it does not isolate beam skipping alone. Three fixture seeds are not synchronized AMCL/MPPI/physics random seeds; one run per cell does not establish a population success rate. No automatic promotion to baseline.

Protocol and source hashes are recorded before simulation; analysis runs offline afterward. Native cloud reconstruction uses the independent circular-yaw lift from stability/recluster.py, not hidden-filter replay. Default legacy paths and old pilot outputs are retained.

Validation also hashes each generated `warehouse.sdf`. Within each seed, normalized XML must be identical across all four trials; normalization replaces only the owned socket address and runtime directory. This checks physical world parity independently from the AMCL YAML ablation. Analysis helper SHA256 values and raw world hashes are retained in `manifest.json`.

Run01 đã hoàn tất: baseline SFM2/3, ORCA+3/3; LF-prob+beam-skip SFM3/3, ORCA+2/3. Tổng5/6 mỗi profile, không có báo thành công khi XY thật ngoài0.20m trong cohort này. Hai failure khác nhau: baseline/SFM/seed43 và beam-skip/ORCA+/seed42. Chưa chọn profile mới làm mặc định.

Đã đối chiếu1998/1998 particle clouds, mọi phiên xác nhận fresh Ogre NVIDIA, shutdown sạch và không còn owned processes/container. `media_validation.json` kiểm tra24 file ảnh/GIF, frame count và hashes; đây là kiểm tra file ảnh từ callback camera đã chạy, không phải replay raw camera packets. Camera observer có góc nhìn cố định; vị trí ngoài khung hình vẫn được đo bằng pose Gazebo trong diagnostics.

SFM có human-human overlap lớn nhất2.369cm trong baseline seed44 dù robot tới đích; thời gian vượt1cm khoảng0.316sim s theo mẫu. Beam-skip/SFM seed44 vẫn có overlap0.870cm. Không diễn giải goal success hoặc overlap-duration0 ở threshold1cm thành không có chồng lấn.

Kế hoạch sau cohort: [đánh giá human policy bằng replay chung, sau đó quay lại nhóm SocNavGym](../../NEXT_STEPS.md). Kế hoạch chưa được chạy; giữ baseline AMCL hiện tại. Các dòng “chưa commit” trong report là trạng thái ở thời điểm đo, source hash trong protocol vẫn giữ nguyên.
