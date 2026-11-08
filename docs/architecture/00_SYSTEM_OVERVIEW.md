# 00 · Tổng quan hệ thống

NarrowNav nghiên cứu tác động của dự đoán chuyển động người lên an toàn và hiệu quả điều hướng robot trong không gian hẹp. Đơn vị tái lập là một episode với scenario, cấu hình đã resolve, seed, phiên bản phần mềm và định danh riêng.

## Phạm vi

Checkout hiện có kiểu dữ liệu và cổng `core`, bộ nạp Hydra, world MJCF và runtime MuJoCo của TASK 03. Việc tiếp theo là nối controller người và kiểm chứng tương tác trong TASK 04A. Bảng dưới mô tả thiết kế toàn hệ thống, gồm cả các module dự kiến. MuJoCo là engine duy nhất cho các thí nghiệm mô phỏng. [Lộ trình](../ROADMAP.md) · [ADR-0001](../decisions/ADR-0001-mujoco-only.md)

| Lớp | Trách nhiệm thiết kế |
| --- | --- |
| Thí nghiệm và cấu hình | Chọn scenario, hình học, bước thời gian, sensor, chính sách robot, backend người, giới hạn chuyển động và seed. |
| Runtime mô phỏng | Tạo scene MuJoCo, điều phối reset/step, nhận đề xuất lệnh, áp giới hạn chung và cập nhật động lực học. |
| Quan sát và đo lường | Đưa depth, LiDAR, odometry và IMU vào kênh quan sát. Tách trạng thái thật và số đo chuyển động cho kiểm định và nhãn ngoại tuyến. |
| Nghiên cứu dự đoán | Học từ lịch sử cảm biến và hành động robot, dự báo quỹ đạo người, occupancy và uncertainty đến 3 giây. |
| Điều hướng và đánh giá | Chiếu dự báo thành Risk Map để đánh giá WM-PPO cùng PPO no-WM và Nav2/MPPI trong cùng giao thức. ROS 2 là biên giao tiếp cho robot thực. |

Các lớp nghiên cứu sử dụng hợp đồng phiên bản hóa. Robot policy chỉ được truy cập quan sát sẵn có tại thời điểm quyết định. Phân quyền và thứ tự cập nhật được quy định trong [luồng dữ liệu](02_DATA_FLOW.md) và [schema state/action](../schemas/STATE_AND_ACTION_SCHEMA.md).

## Trình tự kiểm chứng

Xác nhận tọa độ, đồng hồ và người di chuyển trước khi đối chứng hành vi. Pilot dataset cần cả hành vi tương tác đã kiểm chứng, cảm biến và bộ ghi episode. Chốt QA và split trước khi huấn luyện dự đoán. Khóa baseline và điều kiện so sánh trước khi đánh giá điều hướng hoặc chuyển giao sang robot thật. [Lộ trình](../ROADMAP.md)
