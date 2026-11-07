# Episode

Hợp đồng logic `schema_version = 1`. Mỗi episode ghi một lần thử reset và các step thành công liên tục của đúng một scenario. Định dạng file và cơ chế ghi được chọn trong task có persistence.

Trong TASK 01, `EpisodeConfig` giữ `scenario_id`, `episode_seed`, `control_dt_ns`, `physics_dt_ns`, `duration_ns` và phiên bản schema. Bước điều khiển phải chia hết cho bước vật lý, thời lượng phải chia hết cho bước điều khiển. `max_ticks` và `timestamp(tick)` được suy ra từ các trường này.

`EpisodeMetadata` là header dùng trước reset, gồm `run_id`, `episode_ordinal`, `config`, `config_sha256`, `git_commit` và `git_dirty`. Nó tạo `episode_id` và `stamp(tick)` cho các bản ghi. Bảng bên dưới quy định manifest hoàn chỉnh khi có runtime và persistence, gồm thông tin tài nguyên, phiên bản và kết quả kết thúc được thu thập trong run.

## Manifest

| Nhóm | Trường bắt buộc |
| --- | --- |
| Định danh | `schema_version`, `run_id`, `episode_id`, `episode_ordinal`, `scenario_id`, `episode_seed`. `run_id` duy nhất cho mỗi lần chạy và có dạng `[a-z0-9][a-z0-9_-]*`. `episode_ordinal` là số nguyên không âm và duy nhất trong run. `episode_id` có dạng `<run_id>:<episode_ordinal>`, với ordinal ở hệ thập phân và không thêm số 0 ở đầu. |
| Cấu hình | Cấu hình đã resolve hoặc tham chiếu tới bản sao bất biến cùng `config_sha256` tính trên byte của bản sao. Ghi ID và tham số của backend người, robot policy, sensor, frame, profile chuyển động, lịch bước và tiêu chí dừng. |
| Nguồn gốc | Commit đầy đủ, trạng thái working tree clean/dirty, phiên bản Python, MuJoCo, phần mềm dự án, backend, robot policy và dependency runtime liên quan. Ghi nền tảng chạy và chế độ backend/device cảm biến nếu có. Các run dùng làm bằng chứng chính thức cần checkout sạch. |
| Tài nguyên | Danh sách đường dẫn tương đối và SHA-256 của từng MJCF/asset thực tế đã nạp, kể cả tài nguyên qua include. |
| Vòng đời | `status`, `end_reason`, `last_valid_tick`, số step đã công bố. `last_valid_tick` để trống nếu reset thất bại. Với lỗi, ghi giai đoạn lỗi, tick đang xử lý và mã lỗi. |

Seed episode do kế hoạch thực nghiệm cấp và được lưu cùng manifest. Từ seed đó, dẫn xuất luồng `spawn`, `sensor`, `robot_policy`, `human_backend` và `engine` theo công thức sau. Lấy 8 byte đầu của SHA-256 theo thứ tự big-endian trên chuỗi ASCII `narrow_nav/v1/<episode_seed>/<stream>/<scope>`. Seed ở hệ thập phân, không thêm số 0 ở đầu. Scope là `global`, `entity.<entity_id>` hoặc `sensor.<sensor_id>`. Ghi stream, scope, seed đã dùng và loại bộ sinh số ngẫu nhiên cho mỗi thành phần. Backend có thể thêm lần rút số trong luồng của nó mà vẫn giữ các luồng khác độc lập.

Khi dùng skin Quaternius, `SimulationMetadata.mjcf_source_sha256` bao gồm manifest, mảng rig/animation và tệp texture nếu có của từng bundle đã nạp. `model_sha256` tính trên chuỗi MJCF lắp ghép. Chế độ render được ghi trong `resolved_config`.

## Bản ghi và căn chỉnh thời gian

| Bản ghi | Nội dung |
| --- | --- |
| `reset` | `tick = 0`, `time_ns = 0`, `Observation` ban đầu, `GroundTruthState` ban đầu và danh sách mẫu sensor đã sẵn sàng tại reset. Không có lệnh hoặc khoảng step. |
| `step` | `step_index = k`, `start_time_ns = t_k`, `end_time_ns = t_{k+1}`. Ghi ba batch chuyển động mang tick `k`, `Observation` và `GroundTruthState` mang tick `k + 1`, các mẫu sensor mới sẵn sàng trong `(t_k, t_{k+1}]`, sự kiện tiếp xúc và cờ kết thúc. |
| Sự kiện tiếp xúc | Danh sách sự kiện adapter quan sát sau các substep trong `(t_k, t_{k+1}]`. Mỗi sự kiện mang `time_ns`, cặp ID đã chuẩn hóa (entity hoặc vật cản trong registry scenario) và `kind` nếu phân loại được. Các loại là `robot_wall`, `robot_human`, `human_wall` và `human_human`. Giữ sự kiện thoáng qua ở giữa hai control tick. |

Một manifest nối các kênh theo episode ID. Bản ghi step bắt đầu ở 0 và liên tục. Mốc cuối của step trước là mốc đầu của step sau. Mẫu sensor được ghi đúng một lần theo `(sensor_id, sample_index)` trong kênh sensor khi lần đầu sẵn sàng ở một biên công bố. Khi lưu, slot hợp lệ của `Observation` tham chiếu mẫu tương ứng và có thể tái sử dụng mẫu đó ở các tick sau. Các mẫu đang chờ sau mốc đóng episode không được công bố.

Tách kênh quan sát, lệnh `requested/limited` và kênh kiểm định gồm ground truth, `executed`, tiếp xúc. Đánh giá hoặc tạo nhãn có thể join bằng episode ID và tick. Reader cho suy luận chỉ lấy dữ liệu có quyền và có thời điểm sẵn sàng không muộn hơn tick quyết định. Tách split theo episode và nhóm scenario/seed đã khai báo trước khi tạo cửa sổ thời gian để tránh chia sẻ các đoạn chồng lấn giữa tập.

## Kết thúc và lỗi

| `status` | Quy tắc |
| --- | --- |
| `terminated` | Điều kiện kết thúc của nhiệm vụ xảy ra, ví dụ đạt đích hoặc va chạm được cấu hình là điều kiện dừng. Step cuối có `terminated = true`, `truncated = false`. |
| `truncated` | Dừng ở giới hạn ngoại sinh đã cấu hình, ví dụ hết ngân sách tick. Step cuối có `terminated = false`, `truncated = true`. Khi cùng tick thỏa điều kiện nhiệm vụ, ưu tiên `terminated`. Thứ tự các lý do trong cùng nhóm do protocol cấu hình và ghi trong manifest. |
| `failed` | Reset, callback, tiến vật lý, kiểm dữ liệu hoặc ghi kết quả thất bại. Giữ prefix hợp lệ, không tạo step giả hoặc cờ kết thúc nhiệm vụ. Lỗi reset có thể tạo manifest mà không có bản ghi reset. |

Hai cờ kết thúc đều false ở step thường. Một episode đang mở chưa phải kết quả hoàn tất. Tick chỉ thuộc prefix hợp lệ khi các kênh của nó đã được công bố đồng bộ. Step mang cờ kết thúc và manifest tương ứng được công bố cùng nhau. Nếu không thể hoàn tất lần ghi cuối, lần thử chưa hoàn tất hoặc được ghi nhận là failed nếu có thể lưu trạng thái lỗi. Dừng khi kết thúc hoặc lỗi. Reset tiếp theo luôn tạo episode ID mới, kể cả khi lặp lại scenario và seed.

Kiểm các bất biến frame, ID, shape và thời gian theo [schema state/action](STATE_AND_ACTION_SCHEMA.md). Giao thức so sánh dùng [kiểm định mô phỏng](../protocols/SIMULATION_VALIDATION_PROTOCOL.md) để xử lý episode lỗi hoặc chưa hoàn tất.
