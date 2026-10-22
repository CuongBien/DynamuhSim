# Dataset pilot cho World Model — chuẩn bị dữ liệu

**Một episode Gazebo thật đã thu và đọc lại đạt QA; chưa có bộ train/validation và chưa train World Model.** [Kết quả](reports/PILOT_REPORT.md), [audit độc lập](reports/pilot01/audit.json), [khả năng nguồn/nhãn](CAPABILITIES.md), [contract](contract.json).

Giữ human worker SocNavGym đứng v3 từ mốc nhóm đã commit. Robot Mecanum vẫn do Nav2 MPPI điều khiển; không thêm publisher lệnh robot. Chạy headless, depth/camera dùng NVIDIA D3D12 RTX4060, giới hạn một luồng BLAS. GPU xử lý rendering; ROS, Nav2 và human policy vẫn có phần chạy CPU.

## Cách chạy trên Ubuntu24.04/WSL

Đứng tại repository, đã build ROS Jazzy/install và các dependency thí nghiệm trước:

```bash
bash experiments/social_navigation/benchmark/dataset_pilot/run.sh test
bash experiments/social_navigation/benchmark/dataset_pilot/run.sh live \
  --scenario standing --seed 42 \
  --report-dir experiments/social_navigation/benchmark/dataset_pilot/episodes/pilot02
bash experiments/social_navigation/benchmark/dataset_pilot/run.sh build \
  --episode experiments/social_navigation/benchmark/dataset_pilot/episodes/pilot02
bash experiments/social_navigation/benchmark/dataset_pilot/run.sh audit \
  --episode experiments/social_navigation/benchmark/dataset_pilot/episodes/pilot02 \
  --normalized structured \
  --report experiments/social_navigation/benchmark/dataset_pilot/reports/pilot02/audit.json
```

Thư mục output phải mới. Không chạy hai live song song; harness dùng lock ROS_DOMAIN_ID91 và partition riêng. Không cần Docker hoặc cài thêm Python cho pilot này: hostPython3.12/Jazzy ghi bag, venv SocNavGym chạy worker như trước. Cần MCAP storage plugin có sẵn; thông số Zstd ở storage.yaml. CrowdSimPlus/ComplexityNav vẫn giữ container riêng, chưa chạy lại hoặc thu dữ liệu ở mốc này.

Pilot01 đã thu: dữ liệu chuẩn cuối nằm ở `episodes/pilot01/normalized`, audit ở `reports/pilot01/audit.json`. `structured` là export dở do lỗi broadcasting, `structured02` là bản xuất trung gian; giữ để truy vết, không dùng hai thư mục đó làm bộ dữ liệu chính. Mô phỏng chỉ chạy một lần, không thay failure bằng episode mới. Các thư mục episodes/generated bị Git ignore; báo cáo và ảnh/GIF nhỏ được lưu Git. Khi sao chép dataset, phải sao cả raw_bag, config_snapshot, normalized và result.json; kiểm SHA256 trong manifest.

## Một episode gồm những gì

```text
episodes/pilot01/
  raw_bag/                    # MCAP đầy đủ + metadata.yaml
  config_snapshot/            # world, Nav2 params, camera model/launch/diff, worker, renderer
  result.json                 # goal thật, renderer, nguồn code, cleanup
  events.jsonl                # ready, accepted goal, action end, observation end
  normalized/
    manifest.json             # nguồn/variant/seed/split/SHA256, training_ready=false
    contract.json             # SI, sim-ns, frame, sampling, masks
    calibration.json          # camera K/P/D, optical -> base_link, static TF
    lidar_geometry.json
    depth.npy                 # float32 404x240x424, giữ NaN/Inf như raw
    observations.npz          # cảm biến đầy đủ theo timestamp, không chỉ scan_min
    human_states.jsonl        # pose người Gazebo, chỉ dùng làm nhãn
    samples.npz               # lịch sử cảm biến/hành động 20 bước, tham chiếu depth
    labels.npz                # tương lai 30 bước, action condition, nhóm, persona mask
    bev_observed.npz           # BEV hiện tại từ depth, unknown/free/occupied
    arrays.json               # tên trường/shape/dtype
    events.json
    sample_example.json
    qa.json
    bev_preview.png
```

Lấy raw_bag và config/provenance làm nguồn gốc; NPZ/NPY/JSONL là bản dẫn xuất có thể dựng lại. Không lấy GIF/ảnh observer RGB làm input M2. RGB observer chỉ để demo và kiểm cảnh.

## Quy tắc dùng mẫu

- 10Hz, lịch sử20 điểm `t−1.9s .. t` đại diện cửa sổ `[t−2s,t]`, không có điểm ở t−2s. Không nói 21 điểm. Cảm biến được chọn bằng `max(header_stamp,bag_clock_reception) <= query <= anchor`; có age/valid masks. LiDAR thực tế5Hz được giữ lặp có kiểm tuổi, không bịa thành10Hz mới.
- Tương lai người `t+0.1 .. t+3s`, shape `[N,30,6,2]`. Cùng6ID chỉ bền trong episode. XY ở hệ robot planar tại anchor, lấy actual Gazebo pose; interpolation chỉ cho NHÃN. human_valid biểu thị đủ GT, không có nghĩa người đã được cảm biến nhìn thấy.
- Robot Mecanum dùng `[vx,vy,wz]`. Future actions được lấy ZOH10Hz ở đầu khoảng `t .. t+2.9`, khớp30 khoảng dự đoán. Lệnh đầy đủ ~50Hz còn trong bag; đây là resampling, chưa phải dữ liệu phản thực hoặc expert tối ưu.
- samples.npz không chứa GT pose người, persona hoặc backend ID. Future action trong labels.npz là điều kiện cho nhánh candidate-action khi train M1; không đưa future human GT vào observation. Cần tracker/association từ cảm biến trước khi dùng pipeline dự đoán từng người trên robot.
- M2 đã có depth/K/TF và BEV quan sát hiện tại. Chưa chốt horizon/định nghĩa target BEV tương lai. BEV x[0,6.4),y[−3.2,3.2),cell0.1m; hình học phép chiếu có test, pixel stride4. Unknown giữ−1; không coi pixel Inf/NaN là free. Đây là inverse-sensor projection đơn giản, chưa phải ground truth occupancy hoàn chỉnh.
- H1–H5 giữ trong contract. Persona hiện tại null/index−1, valid=false; không dùng mẫu này tính loss persona M3. Group standing không tự đồng nghĩa H4. Backend/policy/response_variant chỉ là provenance.
- Episode split=pilot_only. Các cửa sổ chồng lấn phải ở cùng split. HuNavSim giữ ngoài train/tuning. Navigation failure vẫn có thể là dữ liệu hợp lệ nếu recorder/cảm biến/nhãn đầy đủ; infrastructure fault phải quarantine.

## Thứ tự các giá trị cảm biến

| Trường | Thứ tự / ý nghĩa |
|---|---|
| imu | qx,qy,qz,qw; angular_velocity x,y,z; linear_acceleration x,y,z, nguồn imu_link |
| odom_local_pose_and_twist | x,y,yaw tương đối odometry tại anchor; linear_velocity x,y,z; angular_velocity x,y,z trong child_frame body |
| action / executed_action_future | vx,vy,wz; base_footprint có cùng trục xoay với base_link, đã kiểm TF cố định |
| scan_ranges_m / scan_ray_valid | 360ray theo lidar_geometry.json; giá trị0 khi invalid, phải dùng raymask |
| depth_index | Chỉ số vào depth.npy; không RGB; depth_invalid giữ NaN/Inf gốc |
| goal_xy_from_amcl | Goalmap đổi về trục robot từ AMCLcausal; có goal_valid và timestamp/availability |
| human_xy_future_m | [sample,30futuresteps,6humanIDs,2XY], chỉ là target |
| persona_index / persona_valid | −1/false ở pilot; không có nhãn persona khả dụng |

Mọi timestamp lưu int64 nanosecond mô phỏng. Covariance IMU/odom và full ROS message giữ trong MCAP; tensor chuẩn hóa không xuất mọi trường ROS. Pose người và group info là nhãn đặc quyền trong mô phỏng, không tự coi là perception trên robot thật.

## Phạm vi sửa camera

Bản launch/SDF runtime riêng bỏ static TF camera_link bị trùng với Mecanum URDF và sửa frame_id ảnh/info thành camera_optical_frame. Không thay vị trí camera, sensor settings, ROS source, robot physics, Nav2 controller hay human policy. Audit xác nhận camera_link chỉ còn một transform [.12,0,.08], trục optical đúng; scan/IMU/odom/depth giữ raw. Base height SDF và URDF chưa được hiệu chuẩn như một hệ3Dworld chung; BEV dùng local base_link, M1 dùng planarXYyaw, không công bố world3D calibration.

## Bước tiếp theo

1. Kiểm cơ chế H1–H5 bằng test robot xa/tiến gần/đối chứng, gồm toàn bộ guard/projection; ưu tiên H3 robot-blind không bị tầng sau làm mất tính mù robot. Chỉ công nhận nhãn khi test hành vi đạt.
2. Chốt sensor tracker/association và định nghĩa target BEV tương lai; khi chưa có, dùng pilot cho QA dữ liệu và giữ mask đúng.
3. Nối recorder contract cho CrowdSimPlus/ComplexityNav tuần tự, pilot một episode/backend; kiểm đa cơ chế và phân bố nhãn trước pilot12 cảnh/seed. Pilot12 chỉ kiểm pipeline/độ phủ, không đủ dataset train chính thức.
4. Sau các gate trên mới freeze split/thu quy mô lớn; HuNavSim đánh giá cuối. Chưa chuyển Isaac Sim hoặc triển khai World Model trong mốc này.
