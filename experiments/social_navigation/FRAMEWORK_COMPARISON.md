# Phạm vi đã kiểm tra sau ba vòng

Đây là bảng so tính năng quan sát được ở các version cụ thể, không xếp hạng
hiệu năng, độ an toàn hoặc mức tự nhiên của hành vi người.

| Mục tiêu | SocNavGym vòng 1 | CrowdSimPlus vòng 2 | ComplexityNav vòng 3 |
|---|---|---|---|
| Nhóm đi có membership | Đã quan sát | Chưa có model tương đương trong bản thử | Clustering nhận diện, chưa có luật giữ nhóm |
| Nhóm đứng trò chuyện | Scene bố trí người quanh tâm, hướng vào nhau | Chưa có model tương đương | Static agents, chưa phải hội thoại |
| Hình thành/tan nhóm | Đã ghi sự kiện | Chưa có model tương đương | Phân cụm quỹ đạo, không phải tạo/tan nhóm theo ý định |
| Tránh người, phản ứng robot | Người đi riêng có phản ứng; nhóm chưa chủ động tránh robot | ORCA/SFM; đã thử visibility và choke point | ORCA/SFM/mixed policies; đã thử visibility |
| Crowd/lối hẹp | Chưa benchmark riêng đủ | Có cảnh hallway/bottleneck và case kẹt | Có width/density/directionality/policy mixture |
| Runtime đã chạy | Python 3.12 venv | Python 3.8.13 container | Python 3.8.13 container |
| Điều khiển người trong Gazebo | Adapter thử nghiệm 1 nhóm 3 người | Chưa | Chưa |

Bằng chứng: [vòng 1](reports/ROUND1.md), [vòng 2](round2/reports/ROUND2.md),
[vòng 3](round3/reports/ROUND3.md).

Nhà xưởng 60×40m + Mecanum/Nav2 đã chạy trong Gazebo. SocNavGym hiện có thêm [adapter một nhóm 3 người](gazebo_adapter/README.md), với hai mode đi cùng nhau và đứng trò chuyện. CrowdSimPlus và ComplexityNav vẫn chỉ ở demo độc lập. Adapter chưa đưa formation/dispersal vào Gazebo và đã có [pilot Nav2 ghép cặp RVO2/SocNavGym](benchmark/reports/BENCHMARK_REPORT.md).

[Kiểm định adapter](gazebo_adapter/reports/ADAPTER_REPORT.md) xác nhận simulation clock, pause/resume, robot feedback, pose proxy và phản xạ LiDAR. Giới hạn đáng chú ý: native ORCA nhóm không thêm robot vào phép tính tránh; probe gần/xa cùng seed cho quỹ đạo giống nhau khi chưa tiếp xúc. Cơ chế contact native có thể dừng nhóm khi overlap với robot. Lượt tuần tra dài còn ghi 199/440 trạng thái overlap giữa thành viên, clearance cặp nhỏ nhất −0.354 m. Vì vậy kết quả reactive của người đi riêng trong vòng 1 không chứng minh group walking đã chủ động tránh robot.

Không so trực tiếp tỷ lệ thành công hay clearance giữa ba vòng vì khác môi trường, controller, timestep và điều kiện sampling. Vòng 3 có thêm screening ban đầu; benchmark chung phải áp dụng cùng kiểm tra chất lượng cảnh cho mọi backend.

Pilot gồm 9 goal đạt, nhưng SocNavGym counterflow seed 43 có overlap với footprint Nav2 khoảng 2.0 s, sâu nhất 0.142 m; seed 42 có overlap giữa người trên 1 cm khoảng 0.8 s. RVO2 không có overlap trên 1 cm trong mẫu đo của pilot. Đây chưa phải xếp hạng thống kê. Bước tiếp theo là tái hiện seed 43 với LiDAR/costmap/TF đồng bộ trước khi tăng mật độ. Cần ghi rõ sự khác nhau giữa tránh robot của người đi riêng và của nhóm; chưa sửa hành vi để che giới hạn native. Sau khi lượt đó ổn định mới tăng mật độ và thử bottleneck. Không chạy hai backend điều khiển cùng người.
