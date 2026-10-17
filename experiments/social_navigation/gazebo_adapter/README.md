# SocNavGym → Gazebo: adapter thí nghiệm, một nhóm 3 người

Đã nối hành vi nhóm SocNavGym vào nhà xưởng Gazebo Harmonic trên Ubuntu 24.04 / ROS 2 Jazzy. Đây là bản tích hợp tối thiểu để quan sát và kiểm tra cảm biến; chưa phải benchmark Nav2 chung giữa các backend.

## Chạy từng bước

Trong terminal Ubuntu, tại repository:

```bash
cd /home/cuongbien/DynamuhSim
bash experiments/social_navigation/gazebo_adapter/run.sh build
bash experiments/social_navigation/gazebo_adapter/run.sh demo --mode walking --gui
```

Dừng bằng Ctrl+C, rồi thử nhóm đứng trò chuyện:

```bash
bash experiments/social_navigation/gazebo_adapter/run.sh demo --mode conversation --gui
```

Thêm `--rviz` để mở RViz. Không truyền `--gui` để chạy server cùng cảm biến và camera quan sát mà không mở cửa sổ Gazebo. `--seed 42` là mặc định. Mỗi lần chạy sinh một thư mục riêng trong `generated/`, giữ config, manifest, SDF và trajectory JSONL; không ghi đè lượt trước.

Cần virtual environment SocNavGym của vòng 1 và workspace ROS đã build trước đó. Nếu thiếu venv, xem `../setup_socnavgym.sh`; không cài dependency vào Python hệ thống. Plugin này build bằng CMake riêng, không cần `colcon build` lại các gói robot.

## Kiểm định có thời hạn

```bash
bash experiments/social_navigation/gazebo_adapter/run.sh verify --mode walking
bash experiments/social_navigation/gazebo_adapter/run.sh verify --mode conversation --gui
bash experiments/social_navigation/gazebo_adapter/run.sh verify --mode walking --observe-sim-seconds 35
experiments/social_navigation/.venv-socnavgym/bin/python experiments/social_navigation/gazebo_adapter/probe_native.py
python3 experiments/social_navigation/gazebo_adapter/audit_preservation.py
```

Verifier kiểm tra `/scan`, `/imu`, `/odom`, `/clock`, lệnh tiến/ngang Mecanum, phản hồi pose/vận tốc robot, pause/resume, ID ổn định, pose model thật trong Gazebo và ray LiDAR so với hình trụ người. Lệnh chuyển động của verifier hiện tính theo **simulation time**. Các kết quả thử đầu tiên sử dụng thời lượng lệnh theo wall time được phân biệt trong báo cáo.

Probe cần ít nhất một lượt walking đã chạy; audit dùng các snapshot local từ vòng 1.

Kết quả ghi trong `reports/`; cùng mode/GUI sẽ thay báo cáo của lượt kiểm định trước. Các demo tương tác luôn có thư mục runtime riêng. Verifier chỉ dừng các process group nó tạo.

## Cách nối và phạm vi

- Worker Python trong `.venv-socnavgym` chạy các hàm hành vi native: `compute_orca_interaction_velocities`, `Human_Human_Interaction.update_speed`, `handle_collision_and_update`. Không sửa vendor SocNavGym.
- Plugin C++ riêng thực thi trong Gazebo `PreUpdate`, trao đổi đồng bộ với worker qua Unix socket riêng mỗi phiên. Mỗi 0.1 giây mô phỏng có đúng một bước người; khi pause không có bước mới. Worker chậm sẽ giảm tốc độ chạy thực, thay vì chạy trước đồng hồ.
- Robot lấy pose từ ECM Gazebo theo tọa độ world; vận tốc đo từ chênh lệch pose, rồi chuyển sang body frame cho đối tượng robot SocNavGym. Worker không điều khiển robot.
- Cả 42 khối collision của nhà xưởng được chuyển thành các hình chữ nhật `Wall` native. Nhóm đi tuần tra giữa `(-24,0)` và `(-15,0)`; hai goal này do kịch bản bên ngoài chỉ định. Đây không phải planner toàn nhà xưởng.
- Ba ID cố định 1–3, group ID 1. `/socnavgym/ground_truth` là JSON trong `std_msgs/msg/String`, chứa timestamp simulation, human world pose, heading, speed native, vận tốc suy ra từ vị trí, group membership và robot feedback.
- Actor dùng asset người sẵn có của project. Proxy collision bán kính 0.36 m, cao 1.7 m; hình trụ xanh trong suốt có chủ đích để GPU LiDAR nhìn thấy hình chuẩn và người dùng kiểm tra nó trực quan. Phép kiểm LiDAR này kiểm hình trụ, chưa đánh giá hình dáng chân/tay sinh học.
- Mặc định ROS domain 74, Gazebo partition riêng. Không bật plugin RVO2 trong world này. Khóa mỗi ROS domain ngăn mở hai phiên adapter cùng domain. Có thể chọn domain/partition qua biến môi trường trước khi chạy.
- Không reset episode ngầm hoặc tái sinh người. Reset simulation giữa phiên, worker mất kết nối hay sai timestep yêu cầu khởi động lại. Plugin có timeout và trạng thái lỗi; kết quả benchmark sau lỗi không hợp lệ.

`--nav` chỉ bật launcher Nav2 hiện có với map nhà xưởng. Chưa xác nhận goal navigation với đám người trong bước này; không xem tùy chọn đó là một benchmark đã đạt.

## Giới hạn cần biết trước khi chọn model

Nhóm đi native không đưa robot vào hàm ORCA nhóm. Probe cùng seed xác nhận robot gần/xa cho quỹ đạo giống nhau trong cảnh không tiếp xúc; cơ chế overlap native mới đặt tốc độ nhóm về 0. Vận chuyển robot feedback đúng không đồng nghĩa với nhóm đã chủ động nhường robot. Người đi riêng và group walking cần được đánh giá tách biệt.

Lượt tuần tra dài còn có overlap nội bộ: 199/440 trạng thái, clearance cặp nhỏ nhất −0.354 m, bắt đầu từ 4.104 s. Kết nối đạt kiểm định không đồng nghĩa với chất lượng hành vi đã đạt. Chưa tăng mật độ với cấu hình này.

Conversation hiện là ba người đứng thành vòng, nhìn vào giữa. Chưa có lời nói hoặc tương tác hội thoại ngữ nghĩa. Formation/dispersal đã quan sát trong demo 2D vòng 1 nhưng **chưa đưa vào adapter** này.

Nhóm chỉ được thử ở lối chính rộng; chưa xác minh qua bottleneck 1.8 m. Native ORCA dùng bán kính đại diện nhóm 1.44 m, có thể quá lớn cho lối đó. Chưa tăng mật độ hay kết luận tránh va chạm an toàn. Worker gọi phép collision Shapely trên 42 khối nên có overhead đáng kể; chạy đúng simulation time không đảm bảo real-time factor 1.

GUI WSLg từng gặp một lỗi khởi tạo driver D3D12; lượt thử lại đã chạy. Log shutdown Gazebo có thể có lỗi SIGTERM/segfault sau khi verifier hoàn tất; báo cáo phân biệt lỗi runtime với lỗi khi đóng.

Xem [báo cáo kết quả](reports/ADAPTER_REPORT.md), [so sánh ba framework](../FRAMEWORK_COMPARISON.md).
