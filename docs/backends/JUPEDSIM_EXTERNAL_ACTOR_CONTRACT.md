# Hợp đồng external actor JuPedSim và MuJoCo

TASK 04U dùng API native mới trong bản cô lập `1.4.2+task04u`, trên nguồn JuPedSim `v1.4.2`. Ba backend độc lập chọn đúng `SocialForceModel`, `AnticipationVelocityModel` hoặc `CollisionFreeSpeedModelV2`. MuJoCo giữ quyền điều khiển robot, pose thực thi của người và dữ liệu ground truth. [Báo cáo 04U](../reports/human_models/TASK_04U_REPORT.md) · [Bản nguồn](https://github.com/PedestrianDynamics/jupedsim/tree/v1.4.2/libsimulator/src)

## API native

| Thao tác | Hợp đồng |
| --- | --- |
| `register_external_actor(radius, anticipation_time=1.0)` | Trước iteration đầu. Radius là bán kính footprint (m), dương và hữu hạn. Trả ID native của actor. Lớp Python giữ ánh xạ sang `HumanContext.robot_entity_id`. Thời gian anticipation (s) thuộc miền [0, 5] và chỉ được AVM đọc. YAML AVM khóa giá trị mặc định 1 s. |
| `apply_coupled_state(time_ns, steps, external=…, pedestrians=…)` | Mỗi sample là `(native_id, (x, y), (vx, vy), yaw)`. Tọa độ, vận tốc và yaw lần lượt là m, m/s và rad trong `world`. Timestamp là ns từ reset, bằng `iteration × 10_000_000`. Window gồm 1–10 bước native 10 ms, phải dùng hết trước khi cập nhật window kế. Hàm ghi state mà không tiến clock. |
| `external_actor(id)` | Đọc `(id, xy, vxy, yaw, radius, time_ns)` của mẫu được giữ. XY, hướng và bán kính lấy từ actor native. SFM/AVM đọc vận tốc từ state model, CSM V2 trả mẫu vận tốc để chẩn đoán. Yaw round-trip theo `atan2` và được so modulo 2π. |
| `coupled_human_velocity(id)` | Đọc vận tốc native SFM/AVM. CSM V2 trả `None` vì phương trình bậc một không có state vận tốc. Vận tốc thực thi luôn lấy riêng từ MuJoCo. |

Batch yêu cầu đủ hai roster, ID hợp lệ không lặp, state hữu hạn, tọa độ hợp lệ, footprint không cắt tường và tâm không trùng nhau trong cặp có người. Native chuẩn bị bản sao và index người trước khi cập nhật object đang sống. Batch bị từ chối giữ trạng thái và clock. Actor có overlap hữu hạn được giữ để đo hạn chế model và contact, với các trường hợp không hợp lệ được xử lý theo lifecycle. Episode giữ roster cố định và không thay geometry giữa các callback.

Mỗi iteration lập index người cho stage/strategic/tactical. Chỉ bản sao dùng cho operational nhận external actor từ mẫu giữ. Actor nằm ngoài roster tích phân và ngoài truy vấn roster công khai, không có journey hoặc stage. Model nhận láng giềng là đúng loại variant native của nó. Ba tệp C++ chứa phương trình model khớp archive upstream từng byte. [Bản vá 04U](../../experiments/social_navigation/patches/jupedsim_v1_4_2_task04u.patch)

## State theo model

| Model | Actor robot | Rebase người |
| --- | --- | --- |
| SFM | XY, bán kính và vận tốc. Native chỉ dùng vận tốc tương đối ở ma sát tiếp tuyến khi overlap. | XY và vận tốc. Giữ hướng nội bộ theo 04S. |
| AVM | XY, hướng, bán kính, vận tốc và anticipation horizon của actor. Native dùng hướng cho alignment, vận tốc và horizon để dự đoán ảnh hưởng láng giềng. | XY, vận tốc và hướng đo. Giữ tham số, RNG native, ID, journey và stage. |
| CSM V2 | XY và bán kính trong repulsion/spacing. Hướng và vận tốc robot được nhận và lưu cho chẩn đoán. Phương trình V2 không dùng vận tốc robot. | XY và hướng đo. Tốc độ tính lại từ spacing và hướng ở bước tiếp theo. |

Hướng, đích, waypoint và route của robot do MuJoCo và policy sở hữu. Tại tₖ, backend lấy sample hiện tại từ `HumanContext`, cập nhật native, rồi dự báo người qua mười bước. Requested là chênh lệch XY dự báo chia 0,1 s. Limiter chung và guard áp lệnh qua tick robot 50 ms và substep MuJoCo 2 ms. Executed là sai phân pose từ MuJoCo. Rebase lần sau cập nhật state của bộ dự báo nội bộ, không đặt lại actor vật lý. Điều này giữ phương trình native nhưng tạo profile đóng vòng có thể khác quỹ đạo JuPedSim tự chạy. Profile SFM 50 ms của 05R vẫn có năm bước native mỗi callback. GT nghiên cứu 10 Hz lấy từ các snapshot đúng mốc 100 ms, riêng với trace kiểm định 50 ms và contact substep. [Schema state/action](../schemas/STATE_AND_ACTION_SCHEMA.md) · [Kiến trúc bridge](../architecture/JUPEDSIM_MUJOCO_2D_3D_BRIDGE.md)

Một nhánh advance/stop có thể phân kỳ do robot đổi vị trí giữa callback. Phép thử tại cùng XY và hướng, chỉ đổi vận tốc robot, mới cô lập ảnh hưởng vận tốc native. SFM và CSM V2 không cho đáp ứng vận tốc trước overlap trong phép thử này. Kết quả HRI phải giữ clearance có dấu, solver contact quan sát được, guard và sai lệch giữ mẫu thành các kênh riêng.
