# 01 · Ranh giới module

Runtime có bộ điều phối episode, điều khiển drivetrain, adapter MuJoCo và bộ dựng MJCF. `src/narrow_nav/humans/` chứa registry và backend `scripted`. Thư mục `humans/` ở gốc checkout chứa demo hình ảnh Quaternius. Bảng dưới mô tả quyền sở hữu, gồm cả các phần dự kiến. [API runtime](05_MUJOCO_RUNTIME.md) · [Lộ trình](../ROADMAP.md)

## Nền tảng mô phỏng

| Module và vị trí | Sở hữu | Hợp đồng qua biên |
| --- | --- | --- |
| **core** · `src/narrow_nav/core/` | Kiểu state, action, observation, context và dữ liệu episode. Kiểm tra dữ liệu dùng chung. Khai báo cổng `SimulatorAdapter`, `RobotPolicy`, `HumanControllerBackend`. | Kiểu giá trị độc lập với engine. Giao diện adapter dùng kiểu `core` ở hai phía. Core không nạp YAML, asset hay ghi output. |
| **configs** · `configs/` | YAML có phiên bản trong `simulation/`, `worlds/`, `corridor_width/`, `scenarios/`, `humans/`, `robots/`, `sensors/` và `experiments/`. | `simulation/configuration.py` compose các nhóm Hydra, resolve tham chiếu và kiểm tra ID, kích thước, bounds, giới hạn, seed và đồng hồ. Runtime ghi fingerprint tài nguyên. |
| **sim/mujoco** · `sim/mujoco/` | MJCF tham chiếu trong `worlds/`, actor trong `models/` và cấu hình MJCF chung trong `templates/`. | Bộ dựng nạp template từ đường dẫn đã resolve. Adapter quản lý việc biên dịch và ánh xạ đối tượng native sang ID nghiệp vụ. [Mô hình MuJoCo](04_MUJOCO_MODELING.md) |
| **simulation** · `src/narrow_nav/simulation/` | Vòng đời episode, đồng hồ điều khiển, routing quan sát/ground truth, dựng `HumanContext`, giới hạn chuyển động chung, dừng episode và công bố bản ghi. | Điều phối gọi các cổng của `core`. Adapter tại `simulation/mujoco/` nắm model/data native, ánh xạ lệnh sang cơ cấu, chạy substep, đọc trạng thái, tiếp xúc và sensor, rồi trả các snapshot đã tách khỏi bộ nhớ engine. |
| **humans** · `src/narrow_nav/humans/` | Các backend ORCA, Social Force, scripted và backend mở rộng, cùng trạng thái nội bộ riêng. | `HumanControllerBackend.reset` nhận config backend, seed luồng và ID người. `propose(HumanContext)` trả đúng một `HumanMotion` cho mỗi ID người của episode. Backend chỉ đề xuất lệnh. Limiter chung và cập nhật vị trí vật lý thuộc runtime. |
| **visualization** · `src/narrow_nav/visualization/` | Bundle Quaternius, nội suy pose và khai báo MJCF skin. | Nhận pose và thời gian, trả bone transforms và tài nguyên render. Adapter đồng bộ skin với mô phỏng. |
| **tests** · `tests/` | `unit/` kiểm hợp đồng và logic độc lập, `integration/` kiểm đường chạy qua cổng và config, `simulation/` kiểm MuJoCo, `regression/` giữ fixture và số đo đã khóa. | Test gọi public interface. Test chuyên biệt của adapter có thể kiểm MuJoCo trực tiếp. Mã sản phẩm không import test. |

Entry point dự kiến tại `scripts/simulation/` resolve cấu hình, tạo adapter, robot policy và human backend, rồi truyền chúng qua các cổng của `core` vào bộ điều phối. `experiments/` lưu định nghĩa thực nghiệm và kế hoạch chạy. `runs/<run_id>/` chứa output của từng lần chạy. [Quy tắc phụ thuộc](03_DEPENDENCY_RULES.md)

Entry point `simulation.record_depth` ghi episode D435i từ API runtime. Adapter áp nhiễu ngay sau render. Runtime công bố mẫu đo trong `Observation` và frame sạch qua cổng kiểm định riêng. Recorder lưu hai kênh riêng dưới run. [Thu depth D435i](../MUJOCO_D435I_DEPTH.md)

## Ranh giới tương lai

| Module dự kiến | Nhận | Trả và giới hạn |
| --- | --- | --- |
| **dataset** · `src/narrow_nav/dataset/` | Episode đã công bố với các kênh có quyền truy cập riêng. | Manifest, split, cửa sổ đầu vào và nhãn. Tách ground truth khỏi feature. Không đọc engine. |
| **perception** · `src/narrow_nav/perception/` | `Observation`, hiệu chuẩn và transform được phép công bố. | Feature có timestamp/frame và mặt nạ hợp lệ. Không lấy nhãn ground truth khi suy luận. |
| **prediction** · `src/narrow_nav/prediction/` | Lịch sử quan sát/feature và chuỗi hành động robot được cấp trước khi suy luận. Huấn luyện nhận nhãn qua dataset. | Dự báo quỹ đạo, occupancy và uncertainty theo thời điểm tới 3 giây. Không dùng hành động thực thi tương lai làm input lúc suy luận. |
| **risk** · `src/narrow_nav/risk/` | Dự báo, uncertainty, frame và thông tin đường đi hợp lệ. | Risk Map có thời điểm gốc, frame và bước thời gian. Không đọc sensor native hay sửa mô phỏng. |
| **navigation** · `src/narrow_nav/navigation/` | Quan sát, nhiệm vụ và Risk Map nếu biến thể được cấu hình dùng. | `RobotAction` yêu cầu. Chỉ biết cổng và dữ liệu công bố. |
| **evaluation** · `src/narrow_nav/evaluation/` | Manifest, kênh quan sát/lệnh và ground truth qua cổng kiểm định. | Metric, báo cáo cặp đối chứng và điều kiện loại mẫu. Không đưa nhãn vào vòng điều khiển. |
| **ROS 2 integration** · `ros2_ws/src/` | Sensor và lệnh qua hợp đồng công bố, ánh xạ frame/thời gian riêng ở biên. | Bridge đổi message sang kiểu dự án và ngược lại. `core` và runtime MuJoCo không phụ thuộc ROS 2. |

Hợp đồng phiên bản đầu tiên cho runtime nằm trong [state/action](../schemas/STATE_AND_ACTION_SCHEMA.md) và [episode](../schemas/EPISODE_SCHEMA.md). Module tương lai sẽ bổ sung schema chuyên ngành khi được triển khai.
