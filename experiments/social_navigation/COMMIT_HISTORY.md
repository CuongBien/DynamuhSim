# Các phần social navigation đã được commit

Các commit dưới đây lưu từng phần công việc đã hoàn tất, theo thứ tự phụ thuộc.
Không bao gồm các thay đổi cleanup/di chuyển file hoặc phần staged có sẵn
ngoài `experiments/social_navigation/`. Chưa thực hiện bước chẩn đoán mới.

| Commit | Phần công việc | Bằng chứng |
|---|---|---|
| `5a14b38` | SocNavGym: nhóm có tránh robot, opt-in adapter | [Report/GIF](benchmark/avoidance_comparison/reports/run01/AVOIDANCE_REPORT.md) |
| `e288ff6` | CrowdSimPlus: backend Gazebo độc lập | [Report/GIF](benchmark/crowdsim_comparison/reports/run01/CROWDSIM_GAZEBO_REPORT.md) |
| `2106ec2` | Native SFM: selector và benchmark đối chiếu ORCA+ | [Report/GIF](benchmark/sfm_comparison/reports/run01/SFM_GAZEBO_REPORT.md) |
| `fa9738a` | Endpoint tuần tra riêng cho mỗi người | [Report/GIF](benchmark/individual_goals/reports/run01/INDIVIDUAL_GOALS_REPORT.md) |
| `c970154` | Người đi giao cắt trong Gazebo | [Report/GIF](benchmark/crossing/reports/run01/CROSSING_REPORT.md) |
| `df4841e` | Lối hẹp và bằng chứng failure định vị | [Report/GIF](benchmark/bottleneck/reports/run01/BOTTLENECK_REPORT.md) |

Các report và metadata giữ nguyên provenance khi chạy: HEAD lúc đo là
`954d577` với các thay đổi chưa commit trong working tree. Các dòng trạng thái
“chưa commit” trong báo cáo là trạng thái **ở thời điểm đo**. Việc chia commit
sau đó không có nghĩa đã chạy lại thí nghiệm trên từng commit trung gian.
Một số plumbing kernel chung nằm ở commit backend; commit SFM bổ sung selector
benchmark và kết quả đánh giá riêng.

Đã kiểm tra cú pháp Python/shell của các phiên bản đưa vào từng phần, phạm vi
file của từng commit và sự khớp chính xác giữa source cuối cùng với working
files đã đo. Native config giữ byte-for-byte, kể cả blank line cuối do
ConfigParser tạo. Log thô, runtime/generated và build products không đưa vào
các commit mới; report, cấu hình, số đo, ảnh/GIF và manifest được lưu.

Kết quả quan trọng: crossing có 4/4 goal đạt. Bottleneck có empty control
qua khe, nhưng chỉ 2/4 goal có người đạt; hai failure seed42 có yếu tố AMCL/
planner, ORCA+ có max position error 2,23 m. Không dùng tỷ lệ này để xếp hạng
framework. Native CrowdSimPlus ở đây vẫn là ba người độc lập; ComplexityNav
chưa nối Gazebo. Không triển khai World Model hay chuyển Isaac Sim.

Bước tiếp theo: dùng diagnostics đã lưu để tách lỗi AMCL/scan/costmap ở
bottleneck seed42, trước khi mở rộng benchmark hoặc chọn mô hình người.
