# JuPedSim AnticipationVelocityModel

**Closeout TASK 06 · `PARTIAL`.** Sweep trên wheel 04U xác nhận AVM phản ứng với vận tốc robot trước tiếp xúc. Giao cắt người–người đối xứng và ca tranh cửa 0,60 m vẫn chưa hoàn thành. [Số đo và phân loại](../reports/human_models/TASK_06_CLOSEOUT_REPORT.md)

**Cập nhật 04U.** Wheel cô lập `1.4.2+task04u` cung cấp external actor và human rebase cho AVM. Registry mở `humans=avm_jupedsim` trên build này. Phần TASK 06 ghi trạng thái lịch sử của stock và 04S. [Hợp đồng 04U](JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md) · [Kiểm định MuJoCo](../reports/human_models/TASK_04U_REPORT.md)

**TASK 06 · `BLOCKED_CAPABILITY`.** Hai lớp native có trong JuPedSim 1.4.2 và chạy được với hình học chiếu từ MuJoCo. Cổng robot động và đồng bộ người của TASK 04 chưa đạt cho AVM. `avm_jupedsim` là ID dành cho backend sau khi qua cổng. [Báo cáo TASK 06](../reports/human_models/TASK_06_REPORT.md)

## API và tham số đã xác minh

Chữ ký của `jupedsim.AnticipationVelocityModel` và `jupedsim.AnticipationVelocityModelAgentParameters` khớp trên wheel stock `1.4.2` và build cô lập `1.4.2+task04s`. Fixture kỹ thuật dùng mặc định của đúng phiên bản, truyền rõ `rng_seed=11` và bước native 10 ms. Các giá trị dưới đây là tham số của phép thử 2D, chưa phải tham số scenario hiệu chuẩn. [Wrapper 1.4.2](https://www.jupedsim.org/v1.4.2/_modules/jupedsim/models/anticipation_velocity_model.html) · [Log chữ ký và ABI](../../runs/task_06/preflight.log)

| Phạm vi | Tham số API | Mặc định đã xác minh | Đơn vị hoặc vai trò |
| --- | --- | ---: | --- |
| Model | `pushout_strength` | 0,3 | Hệ số đẩy khỏi tường. |
| Model | `rng_seed` | Giá trị ngẫu nhiên lúc nạp module | Seed unsigned 64 bit. Fixture truyền 11. Khi tích hợp cần dùng seed luồng `human_backend`. |
| Agent | `desired_speed` | 1,2 | m/s |
| Agent | `radius` | 0,2 | m. Fixture 2D dùng mặc định. Probe blocker dùng bán kính người của scene MuJoCo. |
| Agent | `time_gap` | 1,06 | s |
| Agent | `anticipation_time` | 1,0 | s. Nhánh đối chứng đặt 0,0 tại t = 1 s. |
| Agent | `reaction_time` | 0,3 | s |
| Agent | `wall_buffer_distance` | 0,1 | m |
| Agent | `strength_neighbor_repulsion` | 8,0 | Hệ số lực đẩy hướng giữa người. |
| Agent | `range_neighbor_repulsion` | 0,1 | m |

## Dữ liệu mà mô hình sử dụng

AVM lọc láng giềng bị tường che khuất. Phép tính native đọc vị trí, hướng và vận tốc nội bộ ở bước hiện tại. Nó ngoại suy ảnh hưởng láng giềng bằng `anticipation_time`, rồi cập nhật vị trí, vận tốc và hướng của bước kế tiếp. Fixture giữ cùng seed, hình học, hành trình và lịch sử trước nhánh, sau đó chỉ đổi `anticipation_time`. Cả hai build tái lập đáp ứng trước khi hai footprint chạm nhau. Kiểm tra này đo tương tác người–người trong 2D. [Bước cập nhật native](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/AnticipationVelocityModel.cpp#L23-L102) · [Dự đoán ảnh hưởng láng giềng](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/AnticipationVelocityModel.cpp#L251-L289)

`Agent.position` chỉ đọc và AVM không công bố `velocity` qua wrapper hay binding của model state. Stock thiếu cổng external actor và rebase. Hai cổng thử nghiệm 04S từ chối AVM ngay tại boundary 0 trên bước 10 ms. Thay robot bằng lỗ hình học chỉ cấp vật cản tĩnh ở mỗi lần lấy mẫu, thiếu hướng và vận tốc láng giềng mà phương trình AVM đọc. Vì vậy hai kết quả 2D không được dùng để chấm phản ứng với robot thật. [Test cổng native](../../tests/integration/test_avm_bridge_capability.py) · [Thiết kế 04S](../decisions/ADR-0007-jupedsim-external-actor.md)

## Cổng trước khi đăng ký backend

Kiểm định API AVM có thể đưa robot đo được vào tập láng giềng ở đúng XY, hướng, bán kính và vận tốc, giữ robot ngoài roster được tích phân. Kiểm định rebase người từ GT sau limiter và guard, giữ ID, journey, stage, state định hướng và RNG theo hợp đồng. Hai phép kiểm cần chạy với phương trình AVM hiện hành và thời điểm dữ liệu không muộn hơn callback. Sau đó mới nối backend qua limiter, xuất manifest đã resolve và chạy ca MuJoCo ở cùng scene, seed, lịch lệnh robot và khóa đối chứng. [Luồng bridge](../architecture/JUPEDSIM_MUJOCO_2D_3D_BRIDGE.md) · [Hợp đồng runtime](../protocols/HUMAN_MODELS_RUNTIME_CONTRACT.md)
