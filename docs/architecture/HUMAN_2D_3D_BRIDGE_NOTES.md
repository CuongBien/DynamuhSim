# Biên điều khiển người 2D và MuJoCo 3D

TASK 02 nối backend `scripted` để kiểm toàn bộ đường chạy. Entry point smoke chọn backend từ Hydra và truyền vào `MuJoCoEngine`. `SimulationLifecycle` nhận đúng một backend đã tạo và ghi ID, phiên bản, seed cùng cấu hình episode. `MuJoCoEngine()` tiếp tục hỗ trợ profile stationary cho caller hiện có. [ADR-0004](../decisions/ADR-0004-scripted-mocap-runtime.md) · [Hợp đồng runtime](../protocols/HUMAN_MODELS_RUNTIME_CONTRACT.md)

## Phép chiếu và nhịp điều khiển

- Pose và vận tốc tuyến tính robot, state và goal người, bán kính và obstacle polygons đi vào `HumanContext` từ snapshot MuJoCo hợp lệ ở đầu khoảng. Xoay vận tốc robot từ `robot_base` sang `world`. Polygon hộp lấy từ mặt collision XY. Trụ dùng đa giác ngoại tiếp 16 đỉnh cho context, còn clearance dùng tiết diện tròn thật. Robot policy chỉ nhận `Observation`.
- `scripted` đề xuất đường thẳng tới goal với tốc độ cấu hình. Khi gần đích, giảm tốc mục tiêu để chừa một nhịp giữ lệnh và quãng đường phanh. Khi vào dung sai, backend giữ actor trong roster với yêu cầu zero tới hết episode. Dung sai mặc định 0,10 m thuộc fixture kỹ thuật. Limiter dùng `max_speed_mps`, `max_acceleration_mps2` và period người. Ghi requested gốc, limited và executed riêng.
- Mặc định physics 2 ms, robot 50 ms, người 100 ms. Callback chạy tại 0, 100, 200 ms, với cả requested và limited được giữ ở tick robot xen giữa. Profile active yêu cầu period người là bội của period robot. GT 10 Hz là lịch xuất riêng. Bridge JuPedSim sau này cần khóa số nguyên bước nội bộ chia hết period người và kiểm đồng bộ mỗi lần nối với snapshot MuJoCo. [Timing](../protocols/HUMAN_MODELS_TIMING_GEOMETRY.md)

## Thực thi và quan sát

Adapter ánh xạ ID người đến physical root mocap. Với mỗi substep, tích phân `limited × dt_physics` trong XY, giữ cao độ gốc và giữ yaw theo hướng dịch chuyển trong tick. Root giữ yaw cũ khi đứng yên hoặc khi substep bị chặn. Quaternius bám pose native, chọn idle hoặc walk theo độ lớn vận tốc executed của tick và reset lại animator mỗi episode. Các bone chỉ ảnh hưởng hiển thị.

Kiểm đoạn đề xuất trước khi ghi root. Với solids, khoảng cách từ trung điểm trừ nửa chiều dài đoạn là cận dưới bảo thủ của khoảng cách trên toàn đoạn. Kiểm vị trí cuối và tâm giữa đối với sàn. Với hai người, kiểm khoảng cách nhỏ nhất của quỹ đạo tương đối trong cùng substep. Nếu một đoạn không qua kiểm định, giữ các root người trong substep đó và ghi `human_motion_blocked_substeps`. Cơ chế này có thể dừng cả các actor ở xa khi một actor bị chặn. Nó giới hạn chuyển động thực thi và chưa giải đường vòng hay điều tiết đám đông. Sàn ghép vẫn dùng profile footprint nêu trong hợp đồng hình học.

Mỗi substep chạy `mj_step` và `mj_forward`. Cặp solver đang có ràng buộc với `dist <= 0` trước và sau forward được hợp và gắn timestamp cuối substep. Robot–tường và robot–người là solver-observable. Người–tường và người–người công bố `not_observable_solver` theo mask hiện tại. Khoảng hở hình học của bốn nhóm là số đo có dấu ở snapshot cuối tick, bằng m, khác với sự kiện solver. Guard chỉ ghi số bước đã chặn, không suy ra force, penetration native hay solver contact. Root mocap có thể tác động lên robot qua contact. Solver không quyết định chuyển động ngược lại cho root.

Executed của người là sai phân vị trí native giữa hai snapshot chia thời lượng tick robot. Vận tốc state người lấy từ hai vị trí native cuối hai đầu substep gần nhất. Lifecycle từ chối displacement lớn hơn `max_speed_mps × dt_robot + 1e-9 m` và penetration người–tường/người–người vượt 1 nm. Adapter đối chiếu root với pose đã áp và kiểm NaN/Inf, warning và clock. Tick lỗi giữ prefix đã công bố.

## Trước khi gắn JuPedSim

MuJoCo tiếp tục sở hữu pose thực thi, physics, sensor và contact. Backend 2D dùng snapshot hiện tại để đề xuất lệnh, rồi đối chiếu đầu vào tiếp theo với chuyển động native đo được. Bridge cần xác minh API từng model khi đồng bộ trạng thái sau limiter hoặc guard, và mô tả cách đưa robot MuJoCo vào tương tác dưới dạng external. Cơ chế robot 2D tự chọn vận tốc cần một điều kiện được định nghĩa riêng. Chưa có bằng chứng cho phép dùng một phép cập nhật native chung cho SFM, AVM và CSM.

Fixture scripted thẳng chỉ mở cổng kiểm định đường truyền lệnh. Geometry đầy đủ, policy nhường/đi vòng, nhãn, persistence và sai khác đã ghi vẫn là điều kiện cho benchmark. [Sổ sai khác](../research/HUMAN_MODEL_DEVIATIONS.md) · [Báo cáo TASK 02](../reports/human_models/TASK_02_REPORT.md)
