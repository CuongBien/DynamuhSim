# Kiểm chứng API JuPedSim 1.4.2

Kiểm tra trên `/home/thaonhi/nav_ws/.venv/bin/python`, CPython 3.12.3, JuPedSim 1.4.2, Shapely 2.2.0 và MuJoCo 4.0.0. Shapely đã thuộc extra `human-jupedsim` qua dependency JuPedSim và được pin trong lockfile. Tài liệu stable được dùng để đối chiếu, hành vi dưới đây dựa trên bản cài thực. [API stable](https://www.jupedsim.org/stable/api/jupedsim/index.html) · [Mã Simulation 1.4.2](https://www.jupedsim.org/v1.4.2/_modules/jupedsim/simulation.html) · [Mã Agent 1.4.2](https://www.jupedsim.org/v1.4.2/_modules/jupedsim/agent.html) · [Mã state SFM 1.4.2](https://www.jupedsim.org/v1.4.2/_modules/jupedsim/models/social_force.html)

| API | Bằng chứng tại 1.4.2 | Áp dụng cho bridge |
| --- | --- | --- |
| `Simulation(model=…, geometry=…, dt=0.01)` | Factory tạo native CFM v1 với polygon và lỗ từ MJCF. Kiểm cả năm world thật. | Dùng chung factory, chọn một component trước khi tạo. |
| `iterate(count=1)`, `iteration_count()`, `delta_time()` | `iterate(10)` tăng đúng mười. `switch_geometry` và truy vấn position không tiến thời gian. 1.000 bước native khớp 10 giây. | Clock dùng nanosecond nguyên và xác nhận số bước ở hai đầu callback. |
| `Agent.position`, `Agent.orientation` | Chỉ có getter trong wrapper đang cài. Gán trực tiếp ném `AttributeError`. | Dùng để đo dự báo. Không rebase pose người hoặc ghim pose agent robot. |
| `Agent.target` | Setter có hiệu lực ở Direct Steering. Theo API, operational model vẫn chọn chuyển động. | Cấp điểm đích cho agent không thể ép agent bám pose/vận tốc đã đo. |
| `Agent.velocity` | Không có property chung. Việc gán thuộc tính Python mới có thể thành công mà không gọi native. SFM có state velocity riêng với setter đã thử trực tiếp. AVM và CSM V2 thiếu property velocity tương đương trên state Python. | SFM setter đơn lẻ không đáp ứng ghép pose/vận tốc chung. |
| `Simulation.switch_geometry(geometry)` | Nhận Shapely polygon trên bản cài, đổi lỗ nhiều lần theo pose robot thật. Từ chối lỗ phủ agent hoặc waypoint. | Probe footprint lấy mẫu khả thi trong miền hình học hạn chế. |
| API external robot | Kiểm tra public surface của `Simulation` và `Agent` không thấy cổng update pose/vận tốc external. Lỗ hình học không mang vận tốc. | `BLOCKED_CAPABILITY` cho tương tác robot động và rebase pose người. |

Các lớp `SocialForceModel`, `AnticipationVelocityModel` và `CollisionFreeSpeedModelV2` đã được xác nhận tên/type ở TASK 00 và kiểm tra lại các bề mặt state trong TASK 04. Không tiến dynamics của ba model trong TASK 04. Test kỹ thuật dùng CFM v1 để đo geometry và clock độc lập với model nghiên cứu.

## Robot đứng yên và robot di chuyển

- CFM v1 vòng qua lỗ footprint tĩnh, đạt dung sai kỹ thuật tới đích, khoảng hở footprint nhỏ nhất **0,075477 m**. Lỗ giữ bán kính vật lý robot. Bán kính người vào tham số agent native một lần.
- Trong probe robot động 1 giây, lỗ cập nhật ở 10 callback từ pose thật. Ở cuối probe, người 2D có vị trí khác đối chứng không có lỗ **0,509246 m**. Robot chạy với cùng lệnh và seed cho cùng chuỗi pose khi lặp lại với probe tắt. Sai số giữ mẫu đo ở lưới robot 50 ms, gồm đa giác ngoại tiếp, đạt tối đa **0,034422 m**. Giá trị này chỉ là sai số hình học lấy mẫu, không đo chất lượng anticipation hay contact động.
- Lỗ đè lên agent hoặc goal waypoint bị từ chối ngay. Bridge nghiên cứu dùng gate `require_dynamic_robot_support()` để báo `BLOCKED_CAPABILITY`. Không tạo robot tự hành trong JuPedSim. Một phương án coupling thay thế cần chứng minh cách giữ pose robot theo MuJoCo, đo ảnh hưởng của vận tốc, xử lý vùng chồng lấn và duy trì lịch cập nhật trước khi mở ba backend. [Báo cáo TASK 04](../reports/human_models/TASK_04_REPORT.md)

## TASK 04R · SFM native

- Python wrapper và lớp `jupedsim.native.Agent` đều chỉ đọc pose/orientation. Thuộc tính Python `Agent.velocity` gán thêm không làm thay đổi model-state velocity. Setter native `SocialForceModelState.velocity` ghi được.
- Sau `mark_agent_for_removal`, lookup ID cũ còn thành công và thêm lại ở cùng XY bị từ chối. Sau một `iterate`, lookup cũ lỗi và có thể thêm lại với ID mới. Việc xóa rồi chèn không tạo được update cùng boundary.
- Tạo lại simulation nhận được XY và vận tốc đo. Native SFM vẫn cập nhật pose/vận tốc robot proxy, kể cả khi Direct Steering đặt target bằng pose hiện tại và desired speed bằng 0. Kiểm một bước ở cùng vị trí cho thấy đổi vận tốc proxy không đổi chuyển động người khi hai footprint rời nhau. Trong overlap hữu hạn, vận tốc tiếp tuyến đi vào lực ma sát. Chèn proxy cùng tâm hoặc cách tâm 0,10 m bị từ chối với bán kính proxy 0,18 m, còn khoảng tâm 0,35 m được chấp nhận dù hai footprint chồng lấn.
- Lỗ footprint tĩnh ngoài agent/stage hợp lệ. Thử che agent và waypoint bằng native SFM đều lỗi, sau đó chuyển tới vùng trống được ở cùng iteration. Đây là thử API hình học. Coupling chính xác và rebase liên tục đang `BLOCKED_UPSTREAM_API`. [Nguồn SFM 1.4.2](https://raw.githubusercontent.com/PedestrianDynamics/jupedsim/v1.4.2/libsimulator/src/SocialForceModel.cpp) · [Kết quả 04R](../reports/human_models/TASK_04R_REPORT.md) · [Đề xuất upstream](../decisions/ADR-0007-jupedsim-external-actor.md)

## TASK 04U · Ma trận theo bản cài

| Năng lực | Stock 1.4.2 | 04S | 04U |
| --- | --- | --- | --- |
| SFM / AVM / CSM V2 native 2D | Có | Có | Có |
| Legacy `register_external_sfm_actor` / `apply_sfm_coupled_state` | Thiếu | SFM | SFM |
| Universal external actor / rebase | Thiếu | Thiếu | SFM, AVM, CSM V2 |
| AVM dùng vận tốc external trong anticipation | Thiếu cổng cấp | Legacy từ chối AVM | Có trong phương trình gốc |
| CSM V2 dùng vận tốc external trong spacing | Không có thành phần này trong phương trình | Như stock | Như stock. Actor tham gia qua XY/bán kính. |
| Native human velocity write qua batch | Thiếu | SFM | SFM/AVM. CSM V2 dùng XY và hướng, không có state vận tốc. |

[Hợp đồng 04U](JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md) · [Báo cáo 04U](../reports/human_models/TASK_04U_REPORT.md)
