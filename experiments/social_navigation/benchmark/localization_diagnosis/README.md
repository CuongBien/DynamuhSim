# Chẩn đoán định vị ở lối hẹp từ dữ liệu đã lưu

Phân tích offline năm lượt bottleneck: empty control, SFM/ORCA+ seed42/43.
Không khởi động ROS/Gazebo, đổi tham số Nav2 hoặc sửa dữ liệu benchmark cũ.
[Report và hình ảnh](reports/run01/LOCALIZATION_DIAGNOSIS.md).

## Chạy lại

Tại repository root trong Ubuntu, cần Python 3.12 với NumPy/SciPy/Pillow,
PyYAML và Matplotlib hiện có; native cross-check cần g++ và AMCL Jazzy 1.3.13.

```bash
python3 experiments/social_navigation/benchmark/localization_diagnosis/fetch_reference.py
python3 experiments/social_navigation/benchmark/localization_diagnosis/analyze.py \
  --input experiments/social_navigation/benchmark/bottleneck/reports/run01 \
  --output experiments/social_navigation/benchmark/localization_diagnosis/reports/run02
python3 experiments/social_navigation/benchmark/localization_diagnosis/check_native_score.py \
  --report-dir experiments/social_navigation/benchmark/localization_diagnosis/reports/run02
python3 experiments/social_navigation/benchmark/localization_diagnosis/plot_costmap.py \
  --report-dir experiments/social_navigation/benchmark/localization_diagnosis/reports/run02
python3 experiments/social_navigation/benchmark/localization_diagnosis/validate.py \
  --report-dir experiments/social_navigation/benchmark/localization_diagnosis/reports/run02
```

`fetch_reference.py` chỉ tải source/header official tag1.3.13 vào generated/,
kiểm SHA256 cố định; không thay package cài sẵn. `check_native_score.py` build
một executable offline trong generated/, gọi public sensorUpdate từ thư viện
AMCL đang cài với các pose có prior bằng nhau. Đây không phải replay toàn
particle filter, motion update hoặc resampling.

Raw diagnostics ở `benchmark/generated/` là dữ liệu local ignored, phải còn
đúng các runtime được chỉ trong benchmark report. Analyzer kiểm hash với
run manifest nếu có; không sinh lại hoặc ghi đè input cũ. Dùng output mới
để giữ nguyên report đã lưu. Không cần cài CrowdSimPlus vào Python ROS.

## Phạm vi phép đo

- Ghép timestamp AMCL/odom/LiDAR/TF với actual Gazebo; đo biến thiên vị trí,
  sai lệch, covariance và thời điểm bắt đầu dừng tịnh tiến kéo dài ≥1 s.
- Local costmap đo đúng frame odom, lấy mẫu footprint padding 0,26×0,22 m;
  giá trị là OccupancyGrid 0..100, không phải contact force hay raw cost byte.
- Raycast 42 box và circle proxy người để nhận diện tia có khả năng trúng
  người. Mask dùng ground truth, chỉ phục vụ chẩn đoán, không phải detector.
- Dựng sensor score likelihood_field theo source1.3.13 trên map PGM, đối
  chiếu với thư viện native cài sẵn. Điểm này không phải posterior của PF.
- Không có global costmap/global plan/particle cloud trong dữ liệu gốc;
  không suy diễn các đại lượng đó như đã đo trực tiếp.

Tất cả file mới nằm trong thư mục chẩn đoán; source và cấu hình ROS/Nav2 gốc
được bảo toàn. Kết quả cũ vẫn là 2/4 goal đạt, không bị thay bằng lượt chạy khác.
