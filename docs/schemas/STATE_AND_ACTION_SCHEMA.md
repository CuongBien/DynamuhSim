# State và action

Runtime hiện áp lệnh robot và lệnh người từ backend `scripted` trong profile active. Profile stationary giữ lệnh người bằng zero. Các slot cảm biến hiện trả `missing`. [Runtime hiện tại](../architecture/05_MUJOCO_RUNTIME.md)

Hợp đồng logic `schema_version = 1` cho dữ liệu qua cổng `core`. Kiểu lưu trên đĩa sẽ được chọn khi có task ghi episode. Các trường dưới đây là bắt buộc trừ khi được ghi rõ là tùy chọn.

## Hệ tọa độ, đơn vị và định danh

- `world` là frame tay phải. XY là mặt phẳng điều hướng, +Z hướng lên. Yaw bằng 0 dọc +X, tăng theo chiều từ +X tới +Y khi nhìn từ +Z, và được biểu diễn trong `[-π, π)`.
- Dùng mét, giây và radian theo SI. Vận tốc tuyến tính tính theo m/s, vận tốc góc theo rad/s. `time_ns` là thời gian mô phỏng tính bằng số nguyên nano giây từ reset. Giá trị giây được suy ra bằng cách chia cho 1.000.000.000. Giờ thực chỉ dùng cho metadata hoặc log.
- `frame_id` xác định hệ tọa độ của đại lượng. `world` là ID cố định. `robot_base` là frame điều hướng tại điểm tham chiếu của robot, với +X tiến, +Y trái và +Z lên. Mỗi sensor có frame, trục và hiệu chuẩn khai báo riêng. Mọi phép biến đổi qua frame khác đi qua hiệu chuẩn hoặc pose được cấp cho đúng vai trò.
- `entity_id` là chuỗi nghiệp vụ duy nhất trong episode và ổn định qua reset của cùng scenario. `scenario_id`, ID entity, ID sensor và ID frame khai báo dùng dạng `[a-z][a-z0-9_]*`. `episode_id` nhận diện một lần chạy cụ thể. `episode_seed` là số nguyên không dấu 64 bit, cố định trong episode. [Schema episode](EPISODE_SCHEMA.md)
- Mỗi snapshot và batch lệnh mang `stamp: SnapshotMetadata` gồm `schema_version`, `episode_id`, `episode_seed` và `timestamp: Timestamp` chứa `tick` cùng `time_ns`. Từng state/lệnh của entity mang `entity_id`, timestamp và frame của các đại lượng vật lý. ID native của engine chỉ tồn tại trong adapter.

## Kiểu dữ liệu qua biên

| Kiểu | Trường và quy ước |
| --- | --- |
| `RobotState` | `entity_id`, `pose: Pose2D` trong `world`, `velocity: Twist2D` tức thời tại điểm tham chiếu trong `robot_base` và `timestamp`. |
| `HumanState` | `entity_id`, `pose: Pose2D`, `velocity: Velocity2D` tức thời, `goal: Point2D` và `timestamp`. Các đại lượng vật lý đều trong `world`. |
| `RobotAction` | `entity_id`, `velocity: Twist2D` trong `robot_base` và `timestamp`. Profile vi sai yêu cầu `linear_y = 0` cho requested và limited, còn executed có thể có thành phần ngang. Profile mecanum nhận cả hai thành phần tuyến tính. |
| `HumanMotion` | `entity_id`, `velocity: Velocity2D` trong `world` và `timestamp`. `HumanMotionRequest` dùng cùng biểu diễn ở giai đoạn đề xuất. `HumanMotionResult` nhóm ba giai đoạn cho một người. |
| `GroundTruthState` | Metadata snapshot, đúng một `RobotState` và danh sách `HumanState` theo thứ tự tăng dần của `entity_id`. Là snapshot vật lý dùng cho mô hình hành vi người, chẩn đoán, kiểm định và nhãn ngoại tuyến. |
| `Observation` | Metadata snapshot và danh sách slot theo `sensor_id`. Mỗi slot có `status` là `valid`, `missing`, `invalid` hoặc `stale`. Slot `valid` chứa mẫu mới nhất đã sẵn sàng. Các trạng thái còn lại không có payload và có mã lý do. |
| `HumanContext` | Metadata snapshot, ID, pose và vận tốc phẳng của robot trong `world`, trạng thái người, bán kính robot và bán kính dùng chung của profile người. Mỗi vật cản có ID và danh sách polygon trong `world` với ít nhất ba đỉnh XY mỗi polygon, không lặp đỉnh đóng. Runtime dựng cùng một phép chiếu cho mọi backend. |
| `PlanarWorld` | Kiểu giá trị v1 cho tầng lắp ghép bridge 2D. `floors` là các vòng polygon sàn, `solids` là danh sách `Obstacle` collision tĩnh. Mỗi đỉnh là `Point2D` trong `world`, đơn vị m. Mặt sàn ở Z = 0. `projection_error_m` là cận xấp xỉ footprint tĩnh, đơn vị m. Tách khỏi snapshot `HumanContext` và `Observation`. |
| `HumanBackendConfig.planar_world` | Tùy chọn v1, giá trị `PlanarWorld` từ collision đã load. Các backend JuPedSim yêu cầu trường này để tạo geometry native. External actor và state rebase theo [hợp đồng 04U](../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md). |

`Pose2D` chứa `x`, `y`, `yaw`, `frame_id`. `Velocity2D` chứa `linear_x`, `linear_y`, `frame_id`. `Twist2D` thêm `angular_z`. `RobotState` và `HumanState` cung cấp `to_dict()` và `from_dict()` cho round-trip JSON.

Mỗi `SensorObservation` có `sensor_id`, ID nguồn `source`, `frame_id`, `sample_index` tăng từ 0 theo thứ tự capture của từng sensor, `capture_time_ns`, `available_time_ns` và trạng thái `valid` hoặc `invalid`. Mẫu valid dùng `SensorPayload` với `shape`, giá trị phẳng theo thứ tự hàng và mặt nạ tùy chọn có cùng kích thước. Nguồn sensor khai báo đơn vị và ý nghĩa từng kênh. Mẫu invalid chỉ có metadata và mã lý do. Cấu hình sensor về sau quy định lịch lấy mẫu trên lưới bước vật lý, độ trễ, tuổi tối đa, chính sách dữ liệu thiếu và kênh bắt buộc. Profile kiểm định chuyển động có thể dùng danh sách sensor rỗng và robot policy dùng lệnh cố định.

Payload hợp lệ chỉ có số hữu hạn. Dùng mặt nạ và giá trị thay thế đã khai báo cho điểm đo không hợp lệ, ví dụ điểm depth ngoài miền. Kênh odometry dùng phép đo hoặc mô hình ước lượng đã khai báo, với đặc tính lỗi được kiểm định riêng. Pose vật lý tuyệt đối chỉ thuộc kênh ground truth.

## Ba giai đoạn của chuyển động

`SimulationStepResult` giữ ba `MotionBatch` là `requested`, `limited` và `executed` có cùng ID, tick và mốc đầu khoảng. Mốc cuối lấy từ stamp của `Observation` và `GroundTruthState`. Mỗi batch có đúng một `RobotAction` và đúng một `HumanMotion` cho mỗi ID người của episode, sắp theo ID.

| Giai đoạn | Quyền sở hữu và ý nghĩa |
| --- | --- |
| `requested` | Đề xuất của robot policy và backend người tại `t_k`. Giá trị hữu hạn vượt giới hạn profile được ghi lại nguyên dạng để phân tích. |
| `limited` | Bộ điều phối áp giới hạn tốc độ trước, rồi giới hạn mức thay đổi lệnh theo period của actor. Robot giới hạn chuẩn Euclid của vận tốc phẳng, vận tốc góc và tốc độ bánh theo profile. Người dùng period người và giữ lệnh giữa hai callback. Mốc tính thay đổi là lệnh limited trước đó, lấy zero ở đầu episode. Adapter dùng limited làm mục tiêu trong khoảng. |
| `executed` | Bộ điều phối tính từ dịch chuyển vật lý trên khoảng chia cho thời lượng. Người dùng sai phân XY trong `world`. Robot dùng sai phân XY trong `world` chiếu về `robot_base` tại đầu khoảng và sai phân yaw liên tục chia cho thời lượng. Adapter cung cấp độ biến thiên yaw liên tục qua substep để qua biên ±π chính xác. |

`executed` là chuyển động trung bình, còn twist/vận tốc trong state là giá trị tức thời tại snapshot. Tiếp xúc, trượt và giới hạn cơ cấu có thể tạo chênh lệch giữa `limited` và `executed`. `executed` được ghi cho kiểm định và nhãn, với quyền truy cập tương ứng ground truth.

## Cổng và vòng đời

| Cổng | Đầu vào và đầu ra |
| --- | --- |
| `SimulatorAdapter.reset` | Nhận metadata episode, đặc tả scene đã resolve với trạng thái spawn, cùng seed engine và các `SensorSeed`. Trả `AdapterSnapshot` ở tick 0 với ground truth, mẫu sensor thu tại reset nếu có và registry ID frame. |
| `SimulatorAdapter.advance` | Nhận batch limited tại `t_k`. Trả `AdapterStepResult` với snapshot ở `t_{k+1}`, mẫu sensor mới thu, tiếp xúc trong khoảng và biến thiên yaw liên tục của robot. Adapter chuyển kết quả sang kiểu giá trị của `core`. |
| `RobotPolicy.reset` và `propose` | Reset nhận `RobotMission` với ID và mục tiêu được phép công bố, cấu hình policy và seed luồng. `propose(Observation)` trả một `RobotAction` cho robot. |
| `HumanControllerBackend.reset` và `propose` | Reset nhận cấu hình backend, seed luồng và danh sách ID người. `HumanBackendConfig.max_speed_mps` có mặc định tùy chọn cho caller cũ. Lifecycle truyền trần m/s của profile cho ORCA. `propose(HumanContext)` trả batch `HumanMotion` theo ID. |

Các cổng dùng `Protocol` tổng quát theo kiểu cấu hình scene, policy hoặc backend. Module triển khai cung cấp kiểu cụ thể ở biên lắp ghép.

- `reset` cấp episode mới, xóa lệnh trước đó, hàng đợi sensor và trạng thái nội bộ của policy/backend, rồi xác nhận snapshot và quan sát ở `tick = 0`, `time_ns = 0`. Scenario v1 có đúng một robot và danh sách người cố định, có thể rỗng. Trạng thái ban đầu thỏa điều kiện kết thúc làm reset thất bại.
- `step` chỉ hợp lệ sau reset thành công và khi episode còn mở. `control_dt_ns` và `physics_dt_ns` là số nguyên dương, với `control_dt_ns` chia hết cho `physics_dt_ns`. Một step tiến đúng một tick điều khiển. `time_ns = tick × control_dt_ns` ở mọi snapshot. Mẫu tại biên phải được lấy sau khi cập nhật vật lý tới mốc capture đó.
- Bộ điều phối giữ mẫu đang chờ. `capture_time_ns ≤ available_time_ns ≤ Observation.time_ns` cho mẫu công bố. Ở mỗi slot, chọn mẫu có `sample_index` lớn nhất đã sẵn sàng. Khi chưa có mẫu, trả `missing`. Khi mẫu báo lỗi, trả `invalid`. Khi mẫu valid quá tuổi tối đa tính từ capture, trả `stale`. Không dùng dữ liệu từ tương lai hoặc ground truth để bù.
- Kiểm tra giá trị hữu hạn, kích thước đúng, ID đúng và duy nhất, frame đã đăng ký, tick/time đúng mốc, tính nhất quán metadata và ràng buộc cấu trúc của lệnh như `vy = 0` ở robot vi sai. Kiểm sensor theo schema của nó. Kiểm dữ liệu ở cả hai phía của biên. Trả snapshot độc lập với bộ nhớ có thể thay đổi trong engine hoặc controller.
- Dữ liệu đầu vào sai dừng step trước khi tiến vật lý. Nếu callback đã chạy hoặc quá trình tiến vật lý/kiểm đầu ra thất bại, bỏ kết quả step và đóng episode theo trạng thái `failed`. Không retry tick trên instance đang ở trạng thái dở dang. Bắt đầu lại bằng reset với episode ID mới.

Lifecycle kiểm snapshot reset/advance theo stamp đầy đủ và roster scene. `HumanContext.from_ground_truth` xoay vận tốc tuyến tính robot từ `robot_base` sang `world` cho nhánh backend người. `validate_human_requests` kiểm batch đối với roster và timestamp. Actor đã đến đích vẫn gửi yêu cầu zero.

Mở rộng tương thích schema v1. `SimulationStepResult.clearance` tùy chọn chứa bốn giá trị khoảng hở footprint cuối tick theo m. Giá trị `None` của một cặp nghĩa là không có đủ actor cho cặp đó. `human_motion_blocked_substeps` mặc định 0 và đếm substep trong tick mà guard giữ root người. Đây là kênh kiểm định, tách khỏi `contacts` và `Observation`. [Hợp đồng runtime người](../protocols/HUMAN_MODELS_RUNTIME_CONTRACT.md)

Cách ghi trạng thái kết thúc và phân tuyến dữ liệu được quy định trong [schema episode](EPISODE_SCHEMA.md) và [luồng dữ liệu](../architecture/02_DATA_FLOW.md).
