# ComplexityNav geometry variant: Gazebo pilot

[Báo cáo, camera thật và số đo](reports/GAZEBO_PILOT_REPORT.md): đã chạy4 người ORCA/SFM-intention với robot Mecanum trong nhà xưởng Gazebo. Pilot60s và phép thử ngắt worker đều đạt, có đối chiếu SceneBroadcaster/LiDAR. Người dùng xác nhận thấy4 người và kéo/zoom được. **Chưa có benchmark Nav2 hay mật độ cao cho ComplexityNav.**

Bản offline đã commit `be816c3`; đây là adapter biến thể roadmap + native intent + ORCA safety projection, không phải native SFM/ORCA nguyên bản hoặc RGL đã train. Khởi động bằng image vòng3 có sẵn, ghim content ID; container Python3.8.13 không mạng, chỉ mount code read-only và thư mục runtime/socket riêng. Worker tối đa2 CPU/4GB, BLAS một thread; Gazebo rendering đã đo NVIDIA RTX4060 Laptop qua WSL D3D12. Không sửa robot/Nav2/vendor/ROS Python hoặc rebuild workspace robot.

Từ repo root trong Ubuntu/WSL, build CHỈ plugin thử nghiệm:

```bash
bash experiments/social_navigation/benchmark/complexitynav_gazebo/run.sh build
```

Chạy tuần tự vào report directory MỚI, không ghi đè bằng chứng:

```bash
bash experiments/social_navigation/benchmark/complexitynav_gazebo/run.sh live \
  --fault-after-updates 20 \
  --report-dir experiments/social_navigation/benchmark/complexitynav_gazebo/reports/my_fault
bash experiments/social_navigation/benchmark/complexitynav_gazebo/run.sh live \
  --gui --sim-seconds 60 \
  --report-dir experiments/social_navigation/benchmark/complexitynav_gazebo/reports/my_live
bash experiments/social_navigation/benchmark/complexitynav_gazebo/run.sh analyze \
  --live experiments/social_navigation/benchmark/complexitynav_gazebo/reports/my_live
```

Recorder dùng ROS domain91 và partition Gazebo UUID/PID riêng; không chạy đồng thời phiên dùng domain91. Nó tự phát cmd_vel tiến0.1m/s6s, ngang0.1m/s3s rồi dừng; mode fault luôn giữ robot đứng. Đợi lượt tự kết thúc để lưu camera/log và dừng sạch. `--fault-after-updates` chỉ dành cho test lỗi có chủ ý, không dùng để đánh giá human policy. Không mở Nav2 trong các lệnh này.

Test IPC riêng qua container/socket thật, lưu output mới:

```bash
bash experiments/social_navigation/benchmark/complexitynav_gazebo/run.sh transport --report-name my_transport
```

Transport01 đã đo có source snapshot chính xác; convenience CLI sau đó chỉ thêm lựa chọn tên output và được kiểm --help. Wrapper build/live chưa được chạy lại toàn bộ: các phép đo thực dùng CMake/recorder trực tiếp, command trong report; shell syntax, live --help (kèm preflight/ROS setup) và preflight được kiểm riêng.

Clock: native giữ0.25s, generated world riêng dùng physics step0.0025s để chia hết100 ticks. Worker nhận measured robot world pose/velocity, lập endpoint cho t+0.25s; plugin nội suy ở tick vật lý. Đủ42 boxes giữ nguyên từ factory, cylinder radius0.3m cho cả collision/visual, nominal robot radius0.25m. Visual heading giới hạn1.57rad/s, không sửa human XY/goal của kernel đã đo offline. Actor walk clip giữ theo actual translation; chưa nghiệm thu cơ thể/gait tự nhiên.

Bất kỳ lỗi clock/frame/epoch/nonfinite/infeasible nào đều là terminal: worker gửi fatal, đóng socket, plugin giữ last applied pose và không tự reconnect. Fresh session cần restart launch/container. Hai lượt fault/live đều khởi tạo cùng fixture4 người và IDs do adapter gán; native human.id ở đây là None, không tuyên bố native cung cấp persistent IDs. Reset world ngay trong session không được nghiệm thu/hỗ trợ.

Analyzer đối chiếu pose Gazebo thật với kế hoạch/interpolation, swept disc/AABB, arrival waypoint, LiDAR và odom feedback; không dùng command pose làm actual pose. Gzip archives byte-identical giữ raw JSONL, có SDF/worker/Ogre logs/camera metadata/SHA256. Các PNG/GIF là Gazebo RGB quantized khi lưu, GIF4×; không phải ảnh AI hoặc đồ thị2D. Audit trên máy hiện tại dùng checkpoint source/staging và runtime local; checkout khác cần checkpoint tương đương/build khớp hoặc chạy cohort mới, không viết lại report cũ.

Giới hạn: một fixture4 người, một seed42 và lối đi cục bộ; không test vòng kệ/lối hẹp/dense crowd trong Gazebo ở cohort này (đã có offline). Robot gần nhất vẫn còn khoảng1.07m theo proxy, nên không gọi là stress test tương tác sát người. Không có formation/hội thoại chủ động, RGL hoặc World Model. Bước tiếp theo: commit pilot, chốt bộ cảnh/metrics chung cho ba backend, rồi benchmark tuần tự và chọn mô hình theo năng lực/giới hạn đã đo.
