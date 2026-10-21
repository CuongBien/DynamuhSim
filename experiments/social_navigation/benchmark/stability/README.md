# Lặp chẩn đoán bottleneck sau GPU/cleanup

Cohort định trước: **3 lượt mỗi policy, cùng seed42**, điều kiện bottleneck,
individual goals. Thứ tự vòng1 SFM→ORCA+, vòng2 ORCA+→SFM, vòng3 SFM→ORCA+.
Mỗi lượt budget 75 sim s, nghỉ 15 wall s giữa lượt; chạy headless tuần tự.
Không tune tham số, đổi map/world, physics, robot, native policy hoặc thay
lượt thất bại bằng lượt chạy lại. Bộ ghi navigation luôn bật.

## Chạy lại

Từ repository root trong Ubuntu, dùng thư mục mới:

```bash
python3 experiments/social_navigation/benchmark/stability/run.py \
  --report-dir experiments/social_navigation/benchmark/stability/reports/run02
python3 experiments/social_navigation/benchmark/stability/analyze.py \
  --report-dir experiments/social_navigation/benchmark/stability/reports/run02
```

Runner tự gọi benchmark/run.sh để source ROS Jazzy/overlay. Cần Docker image
CrowdSimPlus và native AMCL reference headers đã chuẩn bị ở các bước trước.
Analyzer chỉ chạy sau khi đã đủ sáu lượt; không trộn tải phân tích vào lượt đo.
Raw inputs ở benchmark/generated, cần giữ đúng runtime để phân tích lại.
`--skip-round-analysis` chỉ gộp lại ba round đã có output phân tích offline.

`protocol.json` được ghi trước trial đầu, có lịch chạy, source hashes, Git
HEAD, budget và cách dừng. ABORTED/TIMEOUT là kết quả navigation được giữ;
lỗi process/protocol, thiếu dữ liệu, GPU sai hoặc còn process của phiên sẽ
dừng các lượt còn lại. Mọi attempt có trong progress.json, không retry ngầm.

Kiểm GPU bằng OpenGL context thật và Ogre2 log mới của từng phiên; kiểm
cleanup bằng guard token và scan process thử nghiệm còn sống. Query nvidia-smi
trước/sau lượt chỉ là snapshot, không phải peak GPU usage hoặc nhiệt CPU.

Seed42 cố định fixture và phần seed có sẵn của worker, không cố định toàn bộ
AMCL/LiDAR/MPPI/physics scheduling. Ba lần mỗi mô hình không đủ xếp hạng an
toàn/naturalness hoặc suy ra failure rate tổng quát. Cohort này giữ riêng với
benchmark gốc 2/4 đạt và hai diagnostic reruns trước cleanup.

Phân tích dùng lại committed navigation_diagnostics/analyze.py: actual Gazebo,
AMCL/odom, global plans/costmap và native libpf clustering. Robot-human đo
footprint 0,24×0,20 m (padding 0,26×0,22), human radius0,36 m, human-human và
robot-machine cùng quy tắc cũ; không phải mọi physics step/contact force.
Mọi overlap nhỏ phải ghi đúng độ lớn, không ẩn bằng threshold 1 cm cũ.

Shutdown warnings được lưu riêng khỏi runtime goal result. Guard PASS nghĩa
không còn process của phiên, không đồng nghĩa tất cả library đã teardown
không warning/segfault. Không sửa code giữa các lượt để cải thiện kết quả.

[Báo cáo loạt đã lưu](reports/run01/STABILITY_REPORT.md). Dựng lại hình failure
cụ thể của **run01** và kiểm audit local sau phân tích:

```bash
python3 experiments/social_navigation/benchmark/stability/plot_failures.py \
  --report-dir experiments/social_navigation/benchmark/stability/reports/run01
python3 experiments/social_navigation/benchmark/stability/validate.py \
  --report-dir experiments/social_navigation/benchmark/stability/reports/run01
```

Hai script trên kiểm các ca đã lưu và checkpoint local; run02 ngẫu nhiên có
thể không có cùng failure, không dùng hardcoded case labels để suy diễn.
Clustering gộp dùng circular-mean lift độc lập từ cloud để tránh seam ±π;
JSON/plots canonical trong các round giữ làm intermediate audit và có README
cảnh báo. Dùng summary/events gộp và `*_circular_clusters.json` cho kết luận.
