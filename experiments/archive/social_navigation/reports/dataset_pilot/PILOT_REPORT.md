# Dataset pilot01 — kết quả22/10/2026

**Đạt nghiệm thu recorder và chuẩn hóa một episode thực tế. Chưa nghiệm thu bộ train M1/M2/M3.** [Audit](pilot01/audit.json), [QA](pilot01/qa.json), [manifest](pilot01/manifest.json), [field shapes](pilot01/arrays.json), [mẫu dữ liệu](pilot01/sample_example.json).

| Hạng mục | Kết quả đo |
|---|---|
| Cảnh | Nhà xưởng60×40m,6 người,2 nhóm đứng ban đầu, native lifecycle tiếp tục; standing response v3 |
| Robot | Mecanum + Nav2 MPPI/AMCL, goal(−28,0)→(−20,0), không publisher vận tốc ngoài Nav2 |
| Navigation | SUCCEEDED; actual XY14.723cm≤20cm,26.3475sim s active; không suy success rate từ1lượt |
| Recorder | MCAP native Zstd,13,273,778bytes (~12.66MiB), exit0,16 topic đọc lại được |
| Cảm biến | 404 depth float32[240,424],202 full360-ray scans,4039 IMU,1154 odometry; 1322cmd_vel |
| Nhãn | 2456 actual scene-pose messages;6ID bền; group memberships từ2019 common GT messages |
| Mẫu | 213 cửa sổ của1episode,10Hz;20 bước lịch sử,30 bước tương lai;6người |
| Audit | 404/404 ảnh depth bitwise bằng raw; future human XY sai khác tối đa7.11e−15m khi tính độc lập; action exact theo resampling |
| Thời gian | Sim-clock không reset; lịch sử không có header/availability sau query/anchor; 4260/4260 history entries hợp lệ mỗi modality |
| Camera | Static TF duy nhất, optical axes được kiểm; runtime corrections riêng, code robot nguồn giữ nguyên |
| GPU/dừng | ActualOgre renderer NVIDIA RTX4060 D3D12; server clean exit, session cleanup PASS |
| H1–H5 | Chưa xác thực: persona null,maskfalse; không gán đứng=H4 |

Goal accepted10.5225s; action end36.8700s; observation end40.2025s. Anchors12.6–33.8s. Hành động tương lai chỉ lấy trong phase Nav2, không ngoại suy lệnh khi robot dừng. Giữ thêm≥3s observation tail làm nhãn, không điều khiển robot thêm. Topic bag có một số receive-clock nhỏ hơn sensorheader do clock/transport; normalizer lấy max(header,receive-clock), không dùng bag time như metrology latency thật.

## Bằng chứng hình ảnh

![Cảnh Gazebo thật ở giữa episode](pilot01/middle.png)

![BEV dựng từ depth thật](pilot01/bev_preview.png)

Ảnh và [GIF4×](pilot01/camera_preview_4x.gif) từ camera observer Gazebo thật, không AI. BEV là hình tính từ depth ghi thật, không ảnh sinh. Phần lớn BEV unknown do FOV/range/che khuất; không tô các vùng chưa quan sát thành free. RGB observer không tham gia input M2.

## Giới hạn / những gì còn lại

- M1 đã có GT quỹ đạo3s + executed actions3D, nhưng chưa có validated sensor tracker/association. Tất cả6người đều có nhãn; human_valid không phải visibility.
- M2 đã có depth/K/TF + observed BEV. Chưa chốt prediction target BEV tương lai; projection hiện tại không chứng nhận đầy đủ occupancy world3D.
- M3 chưa có supervision H1–H5 hợp lệ. Cần kiểm persona xuyên human policy và guard/projection, đặc biệt H3. Không bịa nhãn từ tên framework.
- Một nguồn/seed/cảnh chưa kiểm đa dạng; các backend ORCA/projection có thể học cùng cơ chế. SocNavGym, CrowdSimPlus, ComplexityNav sẽ là nguồn train theo năng lực; HuNavSim giữ test cuối.
- Dataset chỉ có một episode;213cửa sổ tương quan mạnh. Chưa chia train/validation, chưa train World Model, chưa chuyển Isaac.
- MCAP có warning thiếu message-definition tự mô tả cho action status tùy chọn; local Jazzy đọc CDR thành công. Không ảnh hưởng12 core topics, nhưng portability raw vẫn cần ROS type support hoặc bổ sung definition trong exporter sau.

## Sự cố dẫn xuất được giữ

Export01 có lỗi broadcasting khi tạo ray BEV;8test cuối gồm regression nan/inf depth, optical axes, causality/late delivery, stale masks, label gap, clock reset và native moving-group mode đã đạt. structured là export dở; structured02 là export trung gian. Bản chính ở episodes/pilot01/normalized, được build lại từ cùng rawbag và audit độc lập. Không chạy lại simulator, không thay episode để có kết quả đẹp. [Dấu vết lỗi](pilot01/export_failure01.json).

Code live/recording/camera sửa giữ nguyên sau lượt thu. Wrapper run.sh chỉ thêm nhánh audit offline sau đó; [delta](pilot01/wrapper_delta.diff) được lưu để đối chiếu SHA ở thời điểm capture. Builder và contract cuối được ghim SHA trong manifest.

## Kết luận phạm vi

Đã vượt mốc “bắt đầu thu dữ liệu”: có một raw episode thật và mẫu dẫn xuất đọc lại được. Bước tiếp là xác thực persona và association/BEV contract, rồi pilot từng nguồn còn lại trước batch nhỏ. Không cần dừng mọi việc chờ GUI hoàn hảo; đây là pilot headlessGPU, các lỗi GUI cũ vẫn là hạn chế riêng.
