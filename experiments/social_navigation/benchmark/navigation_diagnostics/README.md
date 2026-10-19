# Ghi global navigation và particle cloud trong Gazebo

Chế độ opt-in `--navigation-diagnostics` mở rộng bộ ghi benchmark, tự bật
`--diagnostics`. Chỉ thêm subscriber; không đổi ROS/Nav2 parameters, robot,
human policy, map hoặc goal. Mặc định các lệnh cũ giữ nguyên.

[Báo cáo hai lượt seed42](reports/run03/NAVIGATION_DIAGNOSTICS.md).
[Chẩn đoán offline trên các ca lỗi cũ](../localization_diagnosis/reports/run01/LOCALIZATION_DIAGNOSIS.md).

## Chạy từng lượt

Tại repository root trong Ubuntu 24.04; Docker image CrowdSimPlus và môi
trường ROS Jazzy hiện có phải sẵn sàng. Dùng thư mục mới, chạy tuần tự:

```bash
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend crowdsimplus --human-policy sfm --goal-layout individual \
  --condition bottleneck --seed 42 --navigation-diagnostics \
  --report-dir experiments/social_navigation/benchmark/navigation_diagnostics/reports/run04
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend crowdsimplus --human-policy orca_plus --goal-layout individual \
  --condition bottleneck --seed 42 --navigation-diagnostics \
  --report-dir experiments/social_navigation/benchmark/navigation_diagnostics/reports/run04
```

Thêm `--gui` nếu cần thao tác Gazebo. Hai lượt đã lưu ở run03 chạy headless,
có ảnh/GIF từ camera Gazebo; không dùng chúng để xác nhận GUI tương tác.
Không chạy hai trial trong cùng ROS domain cùng lúc.

Phân tích offline cần Python hiện có, NumPy/SciPy/Matplotlib/Pillow/PyYAML,
g++ và thư viện AMCL Jazzy 1.3.13. Reference headers được tải/kiểm hash bằng
công cụ đã có của giai đoạn chẩn đoán trước, không cài vào ROS:

```bash
python3 experiments/social_navigation/benchmark/localization_diagnosis/fetch_reference.py
python3 experiments/social_navigation/benchmark/navigation_diagnostics/analyze.py \
  --report-dir experiments/social_navigation/benchmark/navigation_diagnostics/reports/run04
```

`analyze.py` ghép hai policy seed42 trong thư mục được chỉ định; nó đọc raw
captures qua trường runtime trong result JSON. Raw captures vẫn ở
`benchmark/generated/`, local ignored, cần giữ để phân tích lại.
`validate.py --report-dir .../run03` kiểm bằng chứng của giai đoạn triển khai
đã lưu, bao gồm hai warmup lỗi và checkpoint Git local; không phải runner
generic cho các máy không còn checkpoint/raw captures đó.

## Các kênh bổ sung

| Topic | Dữ liệu |
|---|---|
| `/plan` | Global path, header và mọi pose |
| `/global_costmap/costmap` | OccupancyGrid đầy đủ; data int8 nén zlib/base64 không mất dữ liệu |
| `/particle_cloud` | Native nav2_msgs/ParticleCloud: mọi pose và weight |
| `/navigate_to_pose/_action/status`, `/follow_path/_action/status` | Status nhận được, có thể thiếu terminal update lúc shutdown |
| `/spin/_action/status`, `/backup/_action/status` | Có dữ liệu khi behavior action thực sự phát status |
| `/rosout` | Log, node, level và timestamp |

Chụp danh sách publisher/QoS/GID trước goal và trước shutdown. MessageInfo
của bản rclpy hiện có chỉ cung cấp timestamps/sequence number, thiếu GID;
để null, không đoán publisher cho từng message. Hai graph snapshots không
chứng minh publisher identity trong toàn bộ khoảng giữa.

Global grid nén giúp tránh JSON hàng triệu cell. ParticleCloud giữ native
schema nên vẫn tăng CPU/I/O; thời gian callback được ghi trong result.
Capture thụ động vẫn có thể ảnh hưởng scheduling/nhiễu của closed loop.
Mỗi lượt có raw hash, config/scene/image identity và kết quả riêng. Không
thay thế benchmark cũ bằng các lượt chẩn đoán thành công.
