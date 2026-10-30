# Detector người từ depth — bản thử nghiệm sau merge

Nhánh gốc `feat/depth-person-detector` được đưa vào layout `perception`, giữ weights depth_linear và IdentityGuard cũ. Đã sửa calibration, đơn vị depth, causal odom, xử lý lỗi inference và ROS packaging. **Chưa đạt gate thay detector mặc định** trên hai pilot đã ghi: coverage proxy hình học chỉ khoảng 8%, trong khi comparator 2D + classifier đạt 83–91%. Không dùng output làm GT/train labels.

[Báo cáo số, receipts và ảnh](verification/depth_detector_integration_2026-10-30/README.md). Khi web server được mở: [localhost](http://localhost:35315/depth_person_detector_v1/index.html).

## Code dùng hiện tại

- `experiments/social_navigation/perception/depth_person_classifier/generalized_detector.py`: implementation duy nhất; bản artifacts chỉ là shim.
- `depth_person_classifier/frozen_classifier.py`: loader gọi lại feature/inference byte-identical, kiểm SHA weights; không train mới.
- `experiments/social_navigation/perception/depth_identity_guard/integrated_pipeline.py`: depth/point-cloud → proposal → classifier → IdentityGuard. Không phụ thuộc Gazebo.
- `depth_identity_guard/ros_support.py`: decode image, kiểm calibration và chọn causal sample, không cần ROS.
- `src/custom_corridor/scripts/depth_person_tracker_node.py`: wrapper ROS; CameraInfo + TF và odom theo timestamp.
- `src/custom_corridor/launch/depth_person_tracker.launch.py`: opt-in; không được include trong launch robot mặc định.

## Chạy node riêng

Trong terminal Ubuntu, từ repository root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src/custom_corridor --packages-select custom_corridor --executor sequential
source install/setup.bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  ros2 launch custom_corridor depth_person_tracker.launch.py use_sim_time:=true
```

Lệnh chỉ mở detector, không mở Gazebo. Cần simulator/bag đang phát `/camera/depth/image_rect_raw`, `/camera/camera_info`, `/odom`, `/tf`, `/tf_static`, `/clock`. Node mặc định nhận odom frame `odom`, child `base_footprint`, lấy TF sang `base_link`; frame khác phải cấu hình đúng, không sửa chuỗi frame để làm hết báo lỗi. CameraInfo phải khớp kích thước/frame của depth; hiện hỗ trợ rectified pinhole depth.

Các topic chỉ dành cho thử nghiệm:

- `/perception/experimental_depth/tracks_json` (`std_msgs/String`): schema `experimental_depth_tracks/1`, timestamp/episode/frame/validity, danh sách ID, pose, velocity, confidence và ambiguity.
- `/perception/experimental_depth/markers`: RViz MarkerArray. Đặt Fixed Frame `odom` rồi thêm MarkerArray topic này.
- `/perception/experimental_depth/poses`: PoseArray để xem; ID/validity chỉ có đầy đủ ở JSON.

`confirmed` chỉ là số lần quan sát của tracker, không phải người/identity GT đã được chứng nhận. `semantic_validated=false`, `training_authorized=false`. Depth stamp lùi tạo episode tracker mới và chờ input mới; stamp trùng được bỏ qua. Odom phải đã có trước anchor depth, tuổi ≤150 ms. Input chưa đủ được ghi observation_valid=false cùng lý do.

## Tái lập so sánh offline

Không cần ROS/Gazebo cho replay. NumPy/SciPy; phần render thêm Matplotlib/Pillow. Dùng output mới, không ghi đè lần FAIL đã đo.

```bash
TASK=experiments/social_navigation/perception/depth_identity_guard
DATA=experiments/social_navigation/datasets/handoff_pilot/generated/pilot01/normalized
OUT="$TASK/generated/my_new_comparison"
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  python3 "$TASK/compare_recordings.py" --normalized-root "$DATA" --output "$OUT"
python3 "$TASK/render_comparison.py" --comparison "$OUT" --normalized-root "$DATA" --output "$TASK/generated/my_new_media"
```

Protocol nằm trong `comparison_protocol.json`. Replay seal sensor outputs của cả hai episode trước khi parse GT. GT chỉ xuất hiện ở evaluator và ảnh đối chiếu; không truyền vào detector/tracker. Comparator `legacy_2d_classifier` là phép nối offline proposal của stage cũ với classifier đóng băng, không tuyên bố đây là một detector ROS đã được triển khai trước đó.

Metric paired/unpaired và visibility proxy chỉ là hình học, chưa phải semantic precision/recall. Bước phát triển sau cần tập development riêng với che khuất/mẫu người đa dạng, rồi freeze và đánh giá sensor set khác. Không mở lại/tuning theo held-out test cũ của classifier; HuNavSim vẫn giữ vai trò test riêng.

## Kiểm tra tích hợp

```bash
python3 -m unittest discover -s tests -p test_depth_person_integration.py -v
python3 tools/handoff.py baseline
```

`tools/check_depth_person_ros.py` là smoke ROS transport dùng CameraInfo/TF/clock/odom có kiểm soát và một frame depth đã ghi. Chạy trong ROS_DOMAIN_ID riêng; nó không điều khiển robot hay chứng minh chất lượng semantic. Lượt đã đo dùng prefix /tmp riêng, xem receipts để biết chính xác đường dẫn và kết quả. Không có live Gazebo perception acceptance mới.

Các frozen pilot, dataset ZIP và baseline bàn giao không đổi. Khi chuyển MuJoCo, tái dùng API point cloud/depth + calibration; ROS wrapper là một adapter tùy chọn. Không mang theo giả định detector 3D đã đạt chất lượng.
