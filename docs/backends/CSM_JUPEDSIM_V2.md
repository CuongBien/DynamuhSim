# JuPedSim CollisionFreeSpeedModelV2

`humans=csm_jupedsim` chọn `jupedsim.CollisionFreeSpeedModelV2` với `CollisionFreeSpeedModelV2AgentParameters` trên wheel cô lập `1.4.2+task04u`. Dùng bridge và limiter chung. Mỗi episode có một simulation và một backend, với geometry chiếu từ collision MuJoCo, radius lấy từ scene và waypoint native. [Cấu hình](../../configs/humans/csm_jupedsim.yaml) · [Nghiệm thu TASK 07](../reports/human_models/TASK_07_REPORT.md)

| Tham số agent | API 1.4.2 / giá trị fixture |
| --- | --- |
| `desired_speed` | 1,2 m/s |
| `time_gap` | 1,0 s |
| `radius` | Mặc định API 0,2 m. Fixture dùng 0,25 m từ scene. |
| `strength_neighbor_repulsion` / `range_neighbor_repulsion` | 8,0 / 0,1 m |
| `strength_geometry_repulsion` / `range_geometry_repulsion` | 5,0 / 0,02 m |

V2 dùng repulsion và spacing theo vị trí, bán kính và hình học. Tại mỗi callback, bridge rebase XY/hướng của người từ GT và cấp actor robot với XY/radius trong tập láng giềng. V2 không có state vận tốc native để ghi lại. Vận tốc robot được lưu cho chẩn đoán và không tham gia phép tính V2. Khi robot đổi vị trí giữa callback, đề xuất người vẫn có thể đổi. Native dự báo 10 bước 10 ms, MuJoCo thực thi lệnh sau limiter và là nguồn GT/contact. [Hợp đồng external actor](JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md) · [Mã V2 v1.4.2](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/CollisionFreeSpeedModelV2.cpp)

TASK 08C tái xác nhận CSM V2 native, trace MuJoCo và replay cùng seed. Ca tranh cửa 0,60 m vẫn chưa qua cửa khi dời đích robot để kéo dài lượt chẩn đoán từ 5,05 lên 10,25 s. Đây là kết quả hành vi ở điều kiện tranh lối. [Kiểm định TASK 08C](../reports/human_models/PROGRESS.md#kiểm-định-task-08c)

Đối chứng vận tốc tại cùng XY/yaw cho Δ dự báo bằng 0. Đây là tính chất của V2 trong fixture, không phải bằng chứng cho hành vi chủ động nhường. Cửa 0,60 m chỉ qua tuần tự với hai footprint đã dùng. Lượt tranh cửa và giao cắt người–người giữ nguyên kết quả chưa hoàn thành. [Số đo native](../../runs/task_06_07/native-sensitivity.json) · [Trace](../../runs/task_06_07/acceptance-evidence.json)
