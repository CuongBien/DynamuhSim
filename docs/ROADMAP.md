# Lộ trình NarrowNav

**Hiện tại · TASK 03. Việc tiếp theo · TASK 04A.** Runtime MuJoCo đã chạy headless với robot vi sai và mecanum. Người trong scenario vẫn đứng tại spawn. `humans=scripted` chọn YAML, adapter hiện chỉ thực thi lệnh bánh robot. Các slot depth, LiDAR, IMU và odometry chưa có mẫu. [Runtime hiện tại](architecture/05_MUJOCO_RUNTIME.md)

| Task dự kiến | Đầu ra và điều kiện chuyển bước |
| --- | --- |
| TASK 01 · Nền tảng Python | Package `core`, kiểu dữ liệu và cổng v1, cấu hình Hydra, lệnh xuất YAML đã resolve và test đơn vị cho các hợp đồng. |
| TASK 02 · MJCF và hình học tham số | Dựng ba world, robot vi sai và người mocap. Biên dịch headless, đo hình học collision và kiểm chứng chuyển động bánh xe trong fixture. |
| TASK 03 · Runtime episode | Dựng reset/step, limiter, snapshot và contact. Kiểm thử vi sai và mecanum với người mocap đứng yên. |
| TASK 04A · Hành vi người | Nối `scripted` vào runtime, kiểm chứng người đi tới đích và cách đo tiếp xúc. Sau đó chạy ORCA và Social Force như các đối chứng độc lập. |
| TASK 04B · Cảm biến và lưu episode | Kiểm định depth, LiDAR, odometry và IMU cho profile thu dữ liệu. Ghi và đọc lại episode theo manifest với các kênh quan sát, lệnh và nhãn tách biệt. |
| TASK 04C · Dataset hành vi | Chạy pilot sau khi đạt 04A và 04B. Kiểm tra các nhánh hành động robot, chất lượng nhãn và split theo nhóm tình huống. Chốt quy mô và tiêu chí QA trước khi sinh dữ liệu huấn luyện. |
| TASK 05 · Dự báo | Huấn luyện và chấm baseline cùng Action-Conditioned World Model cho horizon tới 3 giây. Khóa giao thức đánh giá uncertainty. |
| TASK 06 · Điều hướng | Tích hợp Risk Map, PPO no-WM, WM-PPO và biên ROS 2 cho đối chứng Nav2/MPPI trên MuJoCo. Báo kết quả cùng tập đánh giá giữ lại. |
| TASK 07 · Robot thực | Kiểm định giao tiếp, frame, thời gian và điều kiện vận hành. Đo chênh lệch mô phỏng–thực theo cùng nhiệm vụ. |

## TASK 04A · Việc cần làm ngay

1. Dùng `worlds=corridor`, `scenarios=head_on` và `humans=scripted`. Giữ robot đứng yên, đặt `simulation.observation_sensors=[]` và `simulation.human_control_dt_ns=50000000` để người cùng nhịp 50 ms với robot mặc định. Nối `HumanControllerBackend` vào vòng reset/step, áp giới hạn tốc độ/gia tốc và thực thi vận tốc trong adapter. Fixture đạt khi `human_01` đi rồi dừng trong dung sai của goal, tuân giới hạn chuyển động, không nhảy pose và cho kết quả lặp lại với cùng seed.
2. Kiểm chứng đường đo tiếp xúc và khoảng hở trên các fixture người–robot, người–tường và người–người. Phân biệt tiếp xúc solver với phép đo hình học. Sau đó tích hợp ORCA và Social Force, dùng cùng scene và cơ chế thực thi. Mỗi run chọn một backend cho toàn bộ người.
3. Chạy pilot hành vi với lệnh robot cố định. So sánh backend trên cùng kích thích từ robot. Để đo phản ứng với hành động robot, giữ backend cố định và so hai nhánh tiến/dừng từ cùng điều kiện ban đầu. Mở rộng sang né/nhường hoặc lùi và các bố cục khác sau khi cặp đầu tiên được kiểm chứng. [Giao thức hành vi](protocols/HUMAN_BEHAVIOR_PROTOCOL.md) · [Kiểm định mô phỏng](protocols/SIMULATION_VALIDATION_PROTOCOL.md)

## TASK 04B–04C · Từ schema đến dataset

- Có thể thiết kế schema, kiểm thử writer/reader bằng fixture kỹ thuật và phát triển cảm biến song song với 04A. Đầu ra `smoke.json` chỉ tóm tắt một lần chạy kiểm tra. Pilot thu dữ liệu nghiên cứu bắt đầu ở 04C, sau khi có hành vi tương tác đã kiểm chứng và luồng quan sát/lưu trữ đạt yêu cầu.
- Giữ timestamp và nhịp gốc của từng kênh. Mục tiêu cho mẫu World Model là lịch sử 2 giây, nhãn tới 3 giây trên lưới 10 Hz. Gán split theo nhóm tình huống trước khi cắt cửa sổ và giữ các nhánh cùng trạng thái gốc trong cùng split. Chốt ngân sách episode theo scenario, quy mô và tỷ lệ split trên kết quả pilot. Ngân sách mặc định 20 giây hiện thuộc cấu hình smoke. [Schema episode](schemas/EPISODE_SCHEMA.md) · [Kiểm định mô phỏng](protocols/SIMULATION_VALIDATION_PROTOCOL.md)
