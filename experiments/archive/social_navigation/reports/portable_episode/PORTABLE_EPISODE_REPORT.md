# Nghiệm thu portable episode — 22/10/2026

Đã chuẩn hóa **823 cửa sổ từ một episode mô phỏng đã ghi**, chưa thu thêm episode và chưa train World Model. Raw gốc được giữ nguyên. Đây là nghiệm thu định dạng, tính nhân quả khi chọn quan sát và khả năng đọc dữ liệu độc lập; chưa phải nghiệm thu bộ dữ liệu huấn luyện.

## Nguồn và cấu hình

Nguồn là `early_avoidance_pilot/avoidance_moderate01`: SocNavGym standing v3, seed 42, 6 người, AMCL/odom nguyên bản, robot tới đích sau 87.31 giây mô phỏng trong budget 90 giây. Nav2 runtime dùng bán kính 0.25 m, padding 0.01 m, inflation 0.8 m, scaling 3, giới hạn vx 0.30/−0.15 m/s và vy 0.20 m/s. Giữ nguyên guard/cap của nhóm standing; không dùng oracle.

Camera depth chính dùng mesh người (proxy OFF); LiDAR vẫn dùng proxy bảo thủ. Bản XML runtime giữ đủ bốn thuộc tính literal `gz:expressed_in="base_link"`. SHA256 SDF cuối là `14c15412664eb37dd2f0fb2d61a79b09f98722f204f97d97212f0ec814ad3d9f`. Hash trong `final_artifact_sha256` là hash cuối; metadata camera cũ có thể chứa hash ở bước trung gian.

HEAD ghi lúc capture là `23fc39f`; mã experiment khi đó còn chưa commit. Bản sửa prefix được lưu ở `33427f4`, bằng chứng pilot ở `adb42ad`. Bundle lưu thêm hash và snapshot driver/helpers/native worker thực tế, không giả định HEAD lúc capture đã chứa toàn bộ mã chạy. Xem [provenance](provenance.json) và [audit provenance/timing](provenance_timing_audit.json).

## Dữ liệu đã xuất

Bundle local: `../generated/episode01/`, tổng inventory 539,368,622 byte; 66 file được kiểm hash, không tính chính `bundle_manifest.json`. Dữ liệu lớn nằm trong Gitignore; Git chứa công cụ, báo cáo và một mẫu nhỏ đầy đủ.

```text
episode01/
  raw_bag/                    MCAP và metadata gốc đã copy
  result.json, events.jsonl
  config_snapshot/            cấu hình, nguồn và artifact của lượt chạy
  provenance.json
  normalized/
    depth.npy                 1013 ảnh float32, giữ NaN/Inf
    samples.npz               input, timestamp và validity masks
    labels.npz                ground truth riêng cho supervision
    bev_observed.npz           823 BEV quan sát từ depth
    observations.npz, human_states.jsonl
    calibration.json, lidar_geometry.json
    contract.json, manifest.json, arrays.json, qa.json, ...
  supervision_masks.npz, label_roles.json
  bundle.json, bundle_manifest.json
  tools/                      loader NumPy và exporter/audit đã snapshot
```

| Thành phần | Kết quả |
|---|---|
| Cửa sổ ứng viên / được giữ | 823 / 823; chỉ 1 episode độc lập |
| Sampling | 10 Hz; anchor 12.3–94.5 s |
| History | 20 query từ t−1.9 đến t, cửa sổ 2 s |
| Nhãn người tương lai | 30 bước t+0.1…t+3 s; `[823,30,6,2]` |
| Robot action history / future | `[823,20,3]` / `[823,30,3]`; vx, vy, wz |
| Future executed actions | 30 interval bắt đầu t…t+2.9 s; action đã chạy, không phải thử nghiệm phản thực |
| Goal hợp lệ / bị mask | 629 / 194; goal quá tuổi 1 s bị zero vector và `goal_valid=false` |
| BEV | `[823,64,64]`; −1 unknown, 0 free, 1 occupied |

Không chia ngẫu nhiên các cửa sổ chồng lấn cùng episode sang train/validation. Bundle giữ `split=pilot_only`. HuNavSim tiếp tục dành cho test cuối, không dùng để tune hoặc train.

## Timestamp và hệ trục

Quan sát chỉ được chọn khi đã khả dụng tại query, dùng cả header time và bag/clock reception. Không nội suy input bằng dữ liệu tương lai. LiDAR 5 Hz được giữ theo quan sát cuối khả dụng, không tạo scan mới để giả thành 10 Hz.

Trong export này depth cuối khả dụng già 100 ms so với anchor ở cả 823 mẫu. Đây là kết quả rule chọn quan sát bảo thủ, **không phải phép đo latency wall-clock của camera**. BEV nằm trong base_link tại lúc đo depth, chưa bù chuyển động robot tới anchor. Quỹ đạo người tương lai dùng hệ robot thật tại anchor; người dùng phải tôn trọng hai mốc này khi ghép input/target. Timestamp đều được giữ trong sample.

Độ tuổi history: scan 100–200 ms, IMU 0–10 ms, odom 0–30 ms, action 0–50 ms. Goal bị mask không được điền lại từ ground truth. Xem [thống kê](statistics.json).

## Mask nhiệm vụ và giới hạn

| Nhiệm vụ | Có gì | Chưa đủ gì |
|---|---|---|
| M1 quỹ đạo người | 148,140 điểm XY ground truth hợp lệ | Chưa có sensor tracker/association; mọi mask track-aligned supervision=false |
| M2 BEV | Depth, K/TF và BEV hiện tại từ depth | Target BEV tương lai chưa định nghĩa; mask=false |
| M3 H1–H5 | Schema persona giữ lại | Persona index=−1, valid=false; không suy loại người từ tư thế standing hoặc tên simulator |
| Huấn luyện | Input và supervision đọc riêng | Mọi `episode_training_eligible=false` |

Ground truth có sẵn không chứng minh người được cảm biến nhìn thấy. Group membership/mode là nhãn simulator theo thời gian, ID chỉ có nghĩa trong episode. Bộ này chưa có association, nhãn persona được xác nhận hoặc dữ liệu đa nguồn độc lập; không coi 823 cửa sổ là 823 lần chạy.

## Kiểm chứng đã thực hiện

[Audit raw độc lập](audit02/audit.json) và [chi tiết](audit02/legacy_raw_audit.json) đều PASS: đủ **1013/1013 ảnh depth khớp MCAP từng byte**, gồm 5 ảnh warmup bị thiếu trong bộ NPY phụ của lượt ghi. Nhãn quỹ đạo được dựng lại từ raw với sai số lớn nhất khoảng 1.07e−14 m; future actions và group labels khớp raw. Kiểm riêng 2,285 endpoint depth trên ba BEV, đủ 66 file inventory và các mask chưa hợp lệ. Tám contract test bao gồm late delivery/future poisoning, stale masks, clock reset, NaN/Inf và nhãn persona chưa biết đã đạt.

Đã đọc ở thư mục khác bằng bản hardlink chỉ dùng đọc, chạy Python isolated với loader NumPy, không import ROS/Gazebo và không đưa GT/future labels vào payload input mặc định. Hash bundle sau đọc không đổi. Đây là kiểm di chuyển dữ liệu trong cùng filesystem; chưa thử Windows/Isaac hoặc chạy lại mô phỏng từ snapshot. Để chuyển dữ liệu cần copy toàn bộ bundle; để rerun simulator còn phải remap asset/path và chuẩn bị dependencies.

[Ảnh mẫu](sample_preview.png) được vẽ từ depth/BEV và nhãn GT của dữ liệu đã ghi. **Không dùng AI sinh ảnh; đường tương lai là nhãn thật của simulator, chưa phải dự đoán model.** [Ví dụ input/labels/masks tách riêng](examples/).

## Bước tiếp theo

Xác thực tracker/association từ cảm biến và chốt target BEV tương lai cùng phép căn chỉnh thời gian/hệ trục. Sau đó nghiệm thu nhỏ với episode độc lập trước thu batch đa nguồn. Nhãn H1–H5 chỉ bật sau khi có định nghĩa và bằng chứng hành vi phù hợp. Chưa train, đổi kiến trúc robot hay chuyển Isaac trong lượt này.
