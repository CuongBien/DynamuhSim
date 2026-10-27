# Warehouse Nav2: runtime hiện tại và benchmark lịch sử

Chạy capture/dataset mới theo [capture runtime](../../../docs/CAPTURE_RUNTIME.md)
và các thí nghiệm Nav2 theo [shared runtime](../../../docs/SHARED_NAV2_RUNTIME.md).

## Trial chẩn đoán còn hỗ trợ

`run.sh trial` giữ runner cho các fixture RVO2, SocNavGym và CrowdSimPlus cũ.
Cần workspace ROS đã build và dependency của adapter tương ứng.

```bash
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend socnavgym --condition counterflow --seed 42 --gui \
  --report-dir experiments/social_navigation/benchmark/generated/manual_trial_01
```

Dùng thư mục mới cho mỗi lượt; trial từ chối ghi đè kết quả đã có trong
`--report-dir`. Không chạy đồng thời hai trial cùng ROS domain (mặc định 75).
Người bắt đầu sau khi Nav2 sẵn sàng và nhận goal; thời gian đo dùng simulation
clock. `TIMEOUT`/`ABORTED` vẫn là kết quả cần giữ khi hệ đo hoạt động đúng.
Headless vẫn thu camera Gazebo để lưu PNG/GIF. Không đổi policy, footprint,
controller hoặc ngưỡng đạt goal để thay kết quả lịch sử.

Worker, geometry/contact, robot-aware ORCA và recorder chẩn đoán còn được
runtime sử dụng. Các kiểm định hình học/ORCA và replay vẫn nằm tại đây;
[hình học](geometry_comparison/README.md), [robot-aware ORCA](avoidance_comparison/README.md)
giải thích phạm vi và lệnh đang hỗ trợ.

## Các cohort đã hoàn thành

Đã nghỉ dùng 8 runner ma trận, 6 bộ phân tích riêng từng đợt và 2 công cụ
chẩn đoán phụ thuộc checkpoint/raw runtime cũ. Không còn mode `*-matrix`.
Source và lệnh tái lập nằm tại
[revision lịch sử đã ghim](https://github.com/CuongBien/DynamuhSim/tree/9833565fa546d55cb0e553f8388bf4a703c59583/experiments/social_navigation/benchmark).
Tái lập cohort cũ cần đúng source, image và raw artifact ghi trong report.

Kết quả đã lưu vẫn giữ nguyên:

- [RVO2/SocNavGym](reports/BENCHMARK_REPORT.md), [chẩn đoán seed 43](diagnostic/reports/run01/DIAGNOSIS_REPORT.md).
- [Hình học contact](geometry_comparison/README.md), [robot-aware ORCA](avoidance_comparison/README.md).
- [CrowdSimPlus](crowdsim_comparison/reports/run01/CROWDSIM_GAZEBO_REPORT.md), [SFM/ORCA+](sfm_comparison/reports/run01/SFM_GAZEBO_REPORT.md).
- [Đích riêng](individual_goals/README.md), [giao cắt](crossing/README.md), [lối hẹp](bottleneck/README.md).
- [AMCL](localization_diagnosis/README.md), [navigation diagnostics](navigation_diagnostics/README.md), [stability](stability/README.md).

Các pilot ít seed chưa đủ xếp hạng thống kê. Clearance vòng tròn, footprint
Nav2 và contact native có quy ước khác nhau; đọc giới hạn trong từng report.
