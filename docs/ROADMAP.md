# Thứ tự triển khai sau TASK 00

| Task dự kiến | Đầu ra và điều kiện chuyển bước |
| --- | --- |
| TASK 01 · Nền tảng Python | Package `core`, kiểu dữ liệu và cổng v1, cấu hình Hydra, lệnh xuất YAML đã resolve và test đơn vị cho các hợp đồng. |
| TASK 02 · MJCF và hình học tham số | Dựng ba world, robot vi sai và người mocap. Biên dịch headless, đo hình học collision và kiểm chứng chuyển động bánh xe trong fixture. |
| TASK 03 · Runtime episode | Dựng reset/step, limiter, snapshot và contact. Kiểm thử vi sai và mecanum với người mocap đứng yên. |
| TASK 04 · Cảm biến và dataset | Kiểm định depth, LiDAR, odometry và IMU. Ghi episode, manifest và split tách nhãn khỏi input trước khi huấn luyện. |
| TASK 05 · Dự báo | Huấn luyện và chấm baseline cùng Action-Conditioned World Model cho horizon tới 3 giây. Khóa giao thức đánh giá uncertainty. |
| TASK 06 · Điều hướng | Tích hợp Risk Map, PPO no-WM, WM-PPO và biên ROS 2 cho đối chứng Nav2/MPPI trên MuJoCo. Báo kết quả cùng tập đánh giá giữ lại. |
| TASK 07 · Robot thực | Kiểm định giao tiếp, frame, thời gian và điều kiện vận hành. Đo chênh lệch mô phỏng–thực theo cùng nhiệm vụ. |

Tích hợp scripted, ORCA và Social Force theo từng điều kiện độc lập trước khi sinh dataset hành vi. Số task là đề xuất cho việc lập kế hoạch. Chốt tiêu chí nghiệm thu theo [giao thức kiểm định](protocols/SIMULATION_VALIDATION_PROTOCOL.md) và [ma trận baseline](research/BASELINE_MATRIX.md).
