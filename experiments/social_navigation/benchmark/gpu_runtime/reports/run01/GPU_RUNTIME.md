# RTX 4060 đã hoạt động; đã xử lý tải CPU do ROS còn sót

Kiểm tra ngày 19/10/2026 trên WSL Ubuntu 24.04, Gazebo Harmonic/ROS 2 Jazzy.
**Gazebo đã dùng GPU trước thay đổi này**: log Ogre2 gần nhất ghi
`GL_RENDERER = D3D12 (NVIDIA GeForce RTX 4060 Laptop GPU)`.
Không kết luận toàn bộ benchmark trước chạy bằng CPU. Log Ogre1 cũ từng ghi
llvmpipe thuộc lần troubleshooting khác, không dùng làm bằng chứng cho Ogre2.

Nguyên nhân tải CPU nền trực tiếp tìm được là **ba node từ trial cũ đã kết
thúc**, không phải trial đang chạy: image_bridge ~98,1%, goal_pose_bridge
~97,5%, robot_state_publisher ~97,3% theo `ps %CPU` (tỷ lệ tích lũy vòng đời,
100% tương đương một lõi). Cả ba cùng ROS_LOG_DIR/GZ_PARTITION của
`crowdsimplus_sfm_individual_goals_bottleneck_s43_0c7enmwh`; report gốc xác
nhận SUCCEEDED/protocol_valid. Đã xác minh ownership và dừng đúng ba PID
20529/20727/20550; không kill các phiên ROS khác. Sau dọn, không còn ba node
chiếm CPU này. Không đo trực tiếp nhiệt độ CPU nên không đưa số giảm nhiệt.
[Cleanup đã lưu](initial_cleanup.json), [findings](findings.json).

Việc kiểm tra cleanup giai đoạn trước chỉ bao phủ các runtime mới, bỏ sót
ba node từ lượt cũ này. Đây là thiếu sót trong dọn phiên trước. Wall time/RTF
các lượt chẩn đoán trước có thể bị ảnh hưởng tải nền, không dùng để so hiệu
năng giữa policy hoặc công bố mức tăng tốc sau thay đổi.

## Thay đổi và kiểm tra thực tế

1. Thêm probe GLX tạo context OpenGL thật không mở cửa sổ. Trong môi trường
   tiến trình con WSL, chọn `GALLIUM_DRIVER=d3d12`,
   `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, bỏ các cờ ép software/indirect cũ.
   Xác nhận renderer bằng `glGetString`, không chỉ nhìn tên biến môi trường.
   Dừng trước Gazebo nếu gặp software renderer hoặc không tạo được context.
   Cách chọn adapter dựa trên [WSLg official](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg)
   và [Mesa D3D12](https://docs.mesa3d.org/drivers/d3d12.html).
2. Runner benchmark và standalone SocNavGym gắn token phiên riêng cho các
   process con. Trong finally, dọn cả child detached còn sống dù launch cha
   đã thoát; xác minh UID/token trước khi gửi signal, không pkill theo tên.
   Các launch ROS gốc và human policies không sửa.
3. Test bằng child giả đã PASS: cha thoát trước, child tự setsid, child phớt
   lờ SIGINT/SIGTERM cần SIGKILL, process ngoài phiên vẫn sống. Kiểm thêm cờ
   software cũ vẫn chuyển sang context NVIDIA thật. [Checks](checks.json).
4. Chạy **một smoke test empty** với budget chỉ **2 sim s**. Preflight và log
   Ogre2 thực của phiên đều báo RTX 4060; OpenGL 4.6/Mesa 25.2.8, direct=true.
   Có 64 scan, 1.267 IMU, 351 odom, 8 AMCL trong cả warmup+lượt. Robot di
   chuyển 0,53 m, chạy 2,020 sim s rồi TIMEOUT theo budget cố ý. Đây không
   phải goal test, failure benchmark hoặc chứng minh an toàn/naturalness.
   Sau shutdown guard PASS, remaining=[], không còn ROS/Gazebo của phiên.

[Result smoke](smoke/empty_bottleneck_s42.json), [probe](rendering.json),
[cleanup smoke](process_cleanup.json), [Ogre2 phiên thực](ogre2_session.log),
[log GPU trước thay đổi](previous_renderer_evidence.log).

## GPU dùng được cho phần nào?

| Phần đang dùng | Nơi tính |
|---|---|
| Ogre2, camera, observer camera, GPU LiDAR | GPU render qua D3D12/NVIDIA, đã xác nhận phiên thực |
| Physics DART, ROS executors, bridge/copy dữ liệu | CPU ở cấu hình hiện tại |
| AMCL, costmaps, global planner, MPPI Nav2 | CPU ở bản hiện tại |
| Native ORCA+/RVO2 và SFM NumPy | CPU; không có nhánh CUDA trong policy đang chạy |
| Neural network/training khác | Chưa nằm trong workload benchmark hiện tại |

MPPI Nav2 1.3.13 được upstream mô tả là CPU-only, tối ưu vectorization/tensors;
không có flag CUDA để chuyển controller hiện tại lên GPU.
[Source phiên bản tương ứng](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_mppi_controller/README.md).
Native SFM dùng NumPy, ORCA+ gọi rvo2.PyRVOSimulator trong code đã có.
Không cài CUDA PyTorch hoặc đổi backend policy chỉ để tăng chỉ số GPU-util.

GPU không cần đạt 100% để có hiệu quả. Cách xử lý trong phạm vi hiện tại là
xác nhận render thật bằng GPU, tránh fallback CPU, dọn process sau mỗi phiên,
và chạy các lượt tuần tự. Chưa thay kiến trúc robot, controller/human policy,
physics backend, độ phân giải hay tần số cảm biến. Chưa chạy benchmark dài.

Probe hiện dùng GLX/WSLg; không tự thêm `--headless-rendering`/EGL vì đường
render đó cần xác minh riêng. Thư viện/driver toàn hệ thống không bị thay.
Guard không thể chạy finally nếu chính runner bị SIGKILL và không bảo đảm
bắt child chủ động xóa token; đây là giới hạn rõ ràng, không tuyên bố tuyệt đối.

Thay đổi **chưa commit**. Kết quả navigation cũ giữ nguyên. Từ lần chạy sau,
đối chiếu thêm `rendering` và `process_cleanup` trong result JSON trước khi
xem các chỉ số goal/clearance; có thể tiếp tục ma trận lặp sau bước này.
