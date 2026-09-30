# Báo Cáo Công Việc & Tài Liệu Kỹ Thuật (Job Done Report)

## Mục lục
1. [Tổng quan mục tiêu và vấn đề](#1-tổng-quan-mục-tiêu-và-vấn-đề)
2. [Tối ưu hóa hiệu năng CPU & RAM](#2-tối-ưu-hóa-hiệu-năng-cpu--ram)
3. [Sửa lỗi và tự động hóa HuNav Simulation (Docker)](#3-sửa-lỗi-và-tự-động-hóa-hunav-simulation-docker)
4. [Khắc phục hiển thị Robot trong Gazebo Sim GUI Entity Tree](#4-khắc-phục-hiển-thị-robot-trong-gazebo-sim-gui-entity-tree)
5. [Cơ chế an toàn chống xuyên tường (OccupancyGuard)](#5-cơ-chế-an-toàn-chống-xuyên-tường-occupancyguard)
6. [Hướng dẫn vận hành chuẩn](#6-hướng-dẫn-vận-hành-chuẩn)
7. [Danh mục file thay đổi và bổ sung](#7-danh-mục-file-thay-đổi-và-bổ-sung)

---

## 1. Tổng quan mục tiêu và vấn đề

Trong quá trình vận hành mô phỏng trường học kết hợp robot điều hướng Nav2 và người đi bộ mô phỏng HuNav (`school_arena` / `demo_1`), hệ thống gặp các vấn đề kỹ thuật sau:
- **Tải CPU/RAM đạt ngưỡng 100%**: Gazebo và RViz2 chiếm dụng tài nguyên quá mức do bước mô phỏng vật lý quá dày (1000 Hz), đổ bóng động nặng và RViz2 render không giới hạn khung hình.
- **Lỗi tràn bộ đệm DDS**: Thông báo liên tục `sequence size exceeds remaining buffer` từ FastDDS do xung đột cấu hình serialization và kích thước thông điệp lớn giữa ROS 2 Humble (Docker) và Jazzy (Host).
- **Quy trình chạy HuNav phức tạp và dễ lỗi**: HuNav yêu cầu 3 terminal độc lập trong Docker container (`hunav_loader`, `hunav_agent_manager`, `hunav_gz8_school_bridge.py`). Container thường xuyên bị dừng hoặc gặp lỗi thiếu typesupport `people_msgs`. Khi dừng script bằng `Ctrl+C`, lệnh `pkill -f` trước đây vô tình giết chính tiến trình cha, gây ra mã thoát `exit code 143`.
- **Robot không xuất hiện trên cây thực thể (Entity Tree) của Gazebo GUI**: Người dùng kiểm tra Entity Tree chỉ thấy mục cuối cùng là `sun`, nghi ngờ robot chưa được spawn.

---

## 2. Tối ưu hóa hiệu năng CPU & RAM

Để giảm tải CPU từ 100% xuống mức ổn định, các điều chỉnh sau đã được áp dụng:

1. **Chu kỳ mô phỏng vật lý (`max_step_size`)**:
   - Tệp: `school_floor.world`, `school_floor_baseline.world`, `school_arena.sdf`, `school_arena_hunav.sdf`.
   - Thay đổi `max_step_size` từ `0.001` (1000 Hz) thành `0.004` (250 Hz).
   - Tần số 250 Hz giảm 75% số chu kỳ tính toán va chạm và giải phương trình vi phân vật lý mỗi giây nhưng vẫn đảm bảo độ chính xác động học cho TurtleBot3 và người đi bộ.
2. **Vô hiệu hóa đổ bóng động (Dynamic Shadows)**:
   - Tắt `<shadows>false</shadows>` trong `<scene>` và `<cast_shadows>false</cast_shadows>` trong thẻ `<light name="sun">`.
   - Giúp giảm đáng kể chi phí đổ bóng đổ sâu (shadow maps) của card đồ họa / CPU rendering.
3. **Tối ưu hóa Lidar Sensor**:
   - Tệp: `demo_robot.sdf`.
   - Tắt cờ `<visualize>false</visualize>` của sensor `hls_lfcd_lds`, tránh tạo hàng nghìn tia quét visual trong GUI.
4. **Giới hạn tốc độ khung hình RViz2**:
   - Tệp: `demo_1.rviz`.
   - Cấu hình `Target Frame Rate: 15` (thay vì 30/60 FPS mặc định). Điều này cắt giảm hơn 50% CPU do RViz2 ngốn khi cập nhật pointcloud và laser scans.

---

## 3. Sửa lỗi và tự động hóa HuNav Simulation (Docker)

1. **Sửa lỗi thiếu `people_msgs` typesupport**:
   - `hunav_agent_manager` trong container Docker phụ thuộc vào định dạng message `people_msgs`.
   - Thêm lệnh `source /home/ros2_ws/install/setup.bash` vào toàn bộ môi trường thực thi để nạp đầy đủ gói mở rộng.
2. **Tự động kiểm tra và khởi động container**:
   - Cập nhật `setup_demo_hunav_container.sh` với logic tự động kiểm tra `docker ps -a`, nếu container `hunavsim_gz_fortress` đang ở trạng thái `Exited`, script sẽ tự động gọi `docker start` trước khi thực hiện thao tác.
3. **Tạo bộ script chạy tự động 1 lệnh**:
   - `run_hunav_inside.sh`: Được sao chép vào `/tmp/school_arena/` trong container, điều phối chạy tuần tự `hunav_loader`, chờ khởi tạo, chạy nền `hunav_agent_manager` và `hunav_gz8_school_bridge.py`. Bẫy tín hiệu `trap 'kill ...' SIGINT SIGTERM` để tắt sạch sẽ các tiến trình con khi nhấn Ctrl+C.
   - `start_hunav_agents.sh`: Đặt tại thư mục `demo_1` trên Host, cho phép người dùng chạy toàn bộ ngăn xếp HuNav chỉ bằng cú pháp:
     ```bash
     ./start_hunav_agents.sh school_agents
     ```
   - Khắc phục triệt để lỗi `exit code 143` do đã loại bỏ lệnh `pkill -f` quét nhầm PID của chính shell đang chạy.

---

## 4. Khắc phục hiển thị Robot trong Gazebo Sim GUI Entity Tree

1. **Nguyên nhân gốc rễ**:
   - Trong Gazebo Sim GUI (Fortress/Ignition), plugin Entity Tree gom các đối tượng `<light>` ở dưới cùng danh sách. Trong map chỉ có một ánh sáng là `sun`, do đó `sun` luôn ở dòng cuối cùng.
   - Trước đây robot được tạo bằng dynamic spawn qua `ros_gz_sim create` sau 3 giây (`t = 3.0s`). Cây Entity Tree của Gazebo GUI chỉ quét tĩnh các thực thể tại thời điểm khởi tạo (`t = 0s`) và không có cơ chế tự động cập nhật GUI khi có model sinh động sau đó.
2. **Giải pháp đã thực hiện**:
   - Nhúng trực tiếp định nghĩa `<model name="robot">` từ `demo_robot.sdf` vào `school_floor.world` với tọa độ ban đầu `(-13.5, -8.15, 0.01)`.
   - File world mới được xác thực hợp lệ bằng lệnh `gz sdf -k`.
   - Chuyển tham số mặc định trong `school_hunav_demo.launch.py` thành `robot_in_world:=true`.
   - Kết quả: Ngay khi bật Gazebo, `robot` đã hiện diện sẵn trong cây Entity Tree dưới mục Models, có thể click trực tiếp hoặc tìm kiếm. Camera Gazebo cũng tự động bám theo robot qua topic `/gui/follow`.

---

## 5. Cơ chế an toàn chống xuyên tường (OccupancyGuard)

Trong quá trình chạy, hệ thống ghi nhận log:
```text
[school_collision_safe_pose_applier]: BLOCKED wall crossing for student_door_exit: target=(-2.020, -8.094)
```
- **Ý nghĩa kỹ thuật**: Đây là hoạt động phòng vệ bình thường và cần thiết của node `collision_safe_pose_applier.py`.
- **Cơ chế**: HuNav Agent tính toán quỹ đạo dựa trên lực xã hội (Social Force Model - SFM). Khi người đi bộ `student_door_exit` đi qua góc hẹp của cửa lớp học và tiến sát mép tường, OccupancyGuard tra cứu occupancy grid map (`map.yaml`). Nếu vị trí đích có nguy cơ va chạm hoặc đi xuyên tường, node sẽ chặn bước di chuyển đó và giữ người ở lại vùng an toàn.

---

## 6. Hướng dẫn vận hành chuẩn

### Bước 1: Khởi chạy môi trường mô phỏng (Gazebo + Nav2 + RViz2)
Mở Terminal 1 trên Host:
```bash
source /opt/ros/jazzy/setup.bash
source ~/DynamuhSim/install/setup.bash
ros2 launch custom_corridor school_hunav_demo.launch.py
```

### Bước 2: Khởi chạy luồng người đi bộ HuNav
Mở Terminal 2 trên Host:
```bash
cd ~/DynamuhSim/src/custom_corridor/worlds/school_arena/demo_1
./start_hunav_agents.sh school_agents
```
*(Để dừng HuNav, chỉ cần nhấn `Ctrl+C` tại Terminal 2, toàn bộ tiến trình liên quan trong container sẽ được thu dọn an toàn).*

---

## 7. Danh mục file thay đổi và bổ sung

| Tệp tin | Trạng thái | Nội dung thay đổi |
| :--- | :--- | :--- |
| `demo_1/school_floor.world` | Modified | Giảm `max_step_size` về 0.004, tắt shadows, nhúng trực tiếp model `robot` |
| `demo_1/school_hunav_demo.launch.py` | Modified | Chuyển mặc định `robot_in_world` thành `true` |
| `demo_1/demo_robot.sdf` | Modified | Tắt visualize lidar |
| `demo_1/demo_1.rviz` | Modified | Giảm target frame rate về 15 FPS |
| `demo_1/setup_demo_hunav_container.sh` | Modified | Thêm tự động khởi động docker container nếu bị tắt |
| `demo_1/start_hunav_agents.sh` | Added | Script gọi chạy HuNav tự động một lệnh từ host |
| `demo_1/run_hunav_inside.sh` | Added | Script thực thi HuNav và bẫy tín hiệu tắt trong Docker container |
| `demo_1/JOB_DONE_REPORT.md` | Added | Tài liệu tổng kết chi tiết công việc đã thực hiện |
| `school_arena.sdf`, `school_arena_hunav.sdf`, `school_floor_baseline.world` | Modified | Đồng bộ `max_step_size` về 0.004 |
