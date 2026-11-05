# Kết quả EDA dataset01 — World Model tuần 2

- Dataset: 8 episode được chấp nhận; 2 raw capture cách ly.
- Cửa sổ xuất: 5106; cửa sổ WM observed-BEV hợp lệ: 5094.
- Test coverage: Only standing49 accepted; walking49 quarantined, no held-out walking episode in this batch.
- Train: 5 episode; Validation: 2 episode.
- Tỷ lệ query action có vx < -0.05 m/s (train): 6.68% (đã khử chồng lấn theo timestamp / episode).
- BEV future ở +0.5/+1/+3s: unknown phải được masked, không là free.
- Nhãn vị trí người GT chỉ dành phân tích, chưa có association từ depth/LiDAR.
- Chưa đủ paired intervention để đo action-effect causal/contrafactual.
- Chưa đánh giá mô hình, ADE/FDE hay navigation improvement trong notebook này.

## Việc đề xuất tuần 3
1. Dùng observed-BEV hợp lệ để chạy baseline prediction trên train/validation, không mở test trước khi chốt.
2. Thu cùng snapshot với tiến/chờ/lùi và đối chứng seed, human-policy, geometry.
3. Đo reaction-effect error và so sánh action-aware với action-agnostic WM sau khi có branches.
4. Báo cáo hạn chế seed/scene, không tuyên bố unseen-environment generalization.

## Files tạo ra
- 01_episode_inventory.csv
- 01_split_window_counts.png
- 02_sensor_validity.png
- 02_split_summary.csv
- 03_action_coverage.png
- 03_npz_schema.csv
- 04_action_histograms.png
- 04_sensor_quality.csv
- 05_human_gt_speed.png
- 05_input_integrity.csv
- 06_bev_coverage_by_horizon.png
- 06_task_mask_audit.csv
- 07_action_coverage_by_episode.csv
- 07_train_sample_depth_bev.png
- 08_action_coverage_by_split.csv
- 08_train_sample_gt_human_trajectories.png
- 09_human_gt_motion.csv
- 09_train_sample_executed_action.png
- 10_bev_coverage_occupancy.csv
- 11_bev_coverage_by_split_horizon.csv
- 12_research_readiness_checks.csv
- 13_summary.json
