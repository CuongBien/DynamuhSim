# 05 · Runtime MuJoCo

## Biên API

`MuJoCoEngine` thực thi `SimulationBackend`. Gọi `load(config)` với cấu hình Hydra đã compose, rồi `reset(seed=None, scenario=None)` để bắt đầu episode và lấy quan sát đầu tiên. `scenario` là ID kiểm tra tùy chọn. Để đổi scene, gọi `load` với cấu hình khác. `step(RobotAction)` nhận đúng ID robot và timestamp của tick hiện tại. `stop()` gửi lệnh zero qua cùng limiter. Sau khi kết thúc hoặc lỗi vật lý, gọi `reset` trước khi chạy tiếp. `close()` giải phóng model/data và có thể gọi nhiều lần.

`SimulationLifecycle` điều phối các hợp đồng Python. Adapter trong `simulation/mujoco/` biên dịch chuỗi MJCF bằng `MjSpec`, tạo `MjData`, dùng `mj_resetData` và `mj_forward` khi reset, rồi `mj_step` và `mj_forward` cho từng substep. Adapter xuất snapshot bất biến từ pose và vận tốc native, lấy vận tốc tại gốc body và chiếu twist robot theo yaw. Model, data, index native và mảng NumPy nằm trong adapter.

`get_metadata()` chứa header episode, cấu hình đã resolve với seed thực tế, SHA-256 cấu hình và MJCF, hash các template, phiên bản Python/dự án/MuJoCo, Git commit và dirty flag, clock và trạng thái kết thúc. Mỗi reset tạo ID episode mới và khởi tạo lại RNG của adapter từ luồng `engine`. Scene hiện không rút số ngẫu nhiên. Provenance lấy từ Git checkout. Sau lỗi, metadata và snapshot hợp lệ cuối vẫn đọc được. Bản ghi lưu trên đĩa thuộc pipeline sau.

## Timing và giới hạn lệnh

| Đồng hồ | Khóa YAML | Mặc định |
| --- | --- | --- |
| Vật lý | `simulation.physics_dt_ns` | 2 ms |
| Robot | `simulation.control_dt_ns` | 50 ms |
| Người | `simulation.human_control_dt_ns` | 100 ms |
| Quan sát | `simulation.observation_dt_ns` | 50 ms |

Tất cả period là số nguyên dương và chia hết cho bước vật lý. Đồng hồ người và quan sát có thể lệch biên tick robot. Metadata công bố mốc lịch gần nhất của từng đồng hồ. Mỗi step robot tăng đúng một tick và chạy đủ substep. Sai lệch giữa thời gian MuJoCo và lưới nguyên được kiểm bằng dung sai lớn hơn giữa 1 ns và `1e-10 × thời gian mô phỏng`. Người giữ pose spawn ở TASK 03. Lịch người và lịch quan sát hiện ghi nhận mốc đến hạn, còn controller và nguồn lấy mẫu sẽ được nối vào sau.

Bộ điều khiển giới hạn chuẩn Euclid của vận tốc phẳng, biên vận tốc góc, và tốc độ quay bánh theo profile. Sau đó nội suy từ lệnh limited trước với một hệ số đủ thỏa giới hạn gia tốc tuyến tính và góc. Vi sai yêu cầu vận tốc ngang bằng zero. Mecanum chấp nhận đủ ba thành phần twist. Adapter giữ tốc độ bánh mục tiêu trong tick và MuJoCo tính chuyển động từ mô-men, ma sát, trọng lực và tiếp xúc. `executed` lấy dịch chuyển thực chia cho thời lượng tick, dùng frame robot tại đầu khoảng và tổng biến thiên yaw qua các substep. Ưu tiên kết thúc là contact robot nếu cấu hình bật, đạt đích, rồi hết ngân sách tick.

## Mecanum chuyển từ SDF

Profile `robots=mecanum_robot` chuyển hình học từ mẫu Small Mecanum Robot của DynamuhSim Team sang MJCF. Chassis hộp 0,24 × 0,20 × 0,08 m nặng 3,5 kg. Bốn bánh đường kính 0,08 m, rộng 0,03 m, cách trục trước–sau 0,16 m và trái–phải 0,20 m. Mỗi bánh nặng 0,25 kg tính cả con lăn. Các cụm lắp IMU, LiDAR và camera đưa tổng khối lượng lên 4,7 kg. Footprint tròn 0,18 m bao quanh chassis và cụm bánh.

Mỗi hub có hinge chủ động theo +Y và 12 con lăn thụ động có trục nghiêng 45°. Trục con lăn xen kẽ theo bốn hướng trong mẫu SDF. Tiếp xúc con lăn–sàn thay cho ma sát định hướng của SDF. Kích thước và quán tính con lăn là giả định mô hình. Bốn servo giới hạn mô-men điều khiển theo thứ tự trước trái, trước phải, sau trái, sau phải. Camera giữ hướng nhìn +X, độ phân giải 424 × 240 và FOV ngang 87°. MJCF có camera và mốc lắp IMU/LiDAR.

## Quan sát và contact

`get_observation()` chỉ trả các slot cấu hình cho phép. Depth, LiDAR, IMU và odometry hiện là `missing` với mã `capability_unavailable`; không có payload. `get_ground_truth()` trả trạng thái robot và người riêng cho kiểm định và labeling.

Contact lấy từ cặp geom native đang có ràng buộc solver với `dist <= 0` tại cuối mỗi substep. Registry ánh xạ geom về ID actor hoặc tường. Nhiều điểm contact cùng cặp ở cùng mốc được gộp thành một sự kiện. Mô hình hiện quan sát được robot–tường và robot–người. Người mocap đứng yên và contact mask của họ chỉ ghép với robot, nên solver hiện không cung cấp người–tường hoặc người–người. `get_clearance()` đo khoảng hở footprint riêng từ hình học tĩnh đã compile và pose ground truth. Đó là số đo phẳng bảo thủ, không đại diện độ xuyên của solver.

## Kiểm chứng

Bộ kiểm tra gồm 1.000 substep, lệnh zero, tiến/quay/đi ngang, giới hạn bánh và gia tốc, contact native, lỗi trạng thái không hữu hạn, nhịp bất đồng bộ, reset lặp lại và giải phóng tài nguyên. Replay so sánh các giá trị liên tục với dung sai tuyệt đối `1e-9` trong cùng môi trường. Hai smoke run mặc định chạy 40 tick, tương ứng 1.000 bước và 2 giây mô phỏng. [Lệnh chạy](../../README.md)
