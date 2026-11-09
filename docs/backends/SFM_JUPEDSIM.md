# JuPedSim SocialForceModel

**Cập nhật 04U.** Registry hỗ trợ SFM trên wheel cô lập 04U qua API dùng chung và tiếp tục hỗ trợ build 04S qua legacy API. [Hợp đồng 04U](JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md) · [Regression 05R](../reports/human_models/TASK_04U_REPORT.md)

**Trạng thái 05R · IMPLEMENTATION_PASS và SYNC_PASS trên build 04S.** Registry nhận `sfm_jupedsim` trong interpreter cô lập `1.4.2+task04s`. Wheel stock vẫn bị chặn do thiếu API. TASK 05 đọc API của wheel và chạy lại fixture CFM v1 của bridge. TASK 04R thử SFM thật qua profile chẩn đoán dựng lại trạng thái, tách khỏi registry nghiên cứu. [Báo cáo TASK 04](../reports/human_models/TASK_04_REPORT.md) · [Báo cáo TASK 05](../reports/human_models/TASK_05_REPORT.md)

## API và tham số đã xác minh

Bản cài JuPedSim **1.4.2** trên CPython **3.12.3** xuất chính xác `jupedsim.SocialForceModel` và `jupedsim.SocialForceModelAgentParameters`. `inspect.signature` trên wheel xác nhận các mặc định dưới đây. Đây là số tham chiếu API, chưa hiệu chuẩn thành tham số scenario. [Mã Social Force 1.4.2](https://www.jupedsim.org/v1.4.2/_modules/jupedsim/models/social_force.html)

| Phạm vi | Tên tham số | Mặc định API | Đơn vị |
| --- | --- | ---: | --- |
| Model | `body_force` | 120000 | kg/s² |
| Model | `friction` | 240000 | kg/(m·s) |
| Agent | `mass` | 80,0 | kg |
| Agent | `desired_speed` | 0,8 | m/s |
| Agent | `reaction_time` | 0,5 | s |
| Agent | `agent_scale` | 2000 | N |
| Agent | `obstacle_scale` | 2000 | N |
| Agent | `force_distance` | 0,08 | m |
| Agent | `radius` | 0,30 | m |

Khi mở cổng, bán kính lấy từ scene MuJoCo đã resolve để giữ cùng footprint. Khởi tạo agent qua `add_waypoint_stage`, `JourneyDescription`, `add_journey` và `add_agent` trong một simulation cho mỗi episode. Bridge dùng chung sở hữu hình học và nhịp dự báo. MuJoCo tiếp tục sở hữu pose thực thi và contact. Các tên snake_case là API hiện hành, bên cạnh một số alias cũ trong chữ ký.

`SocialForceModelState.velocity` có setter. `Agent.position` và `Agent.orientation` chỉ có getter, và `Agent` thiếu property vận tốc chung. Thay vận tốc nội bộ không ghim được pose robot ngoài hoặc đưa agent người về pose MuJoCo sau sai lệch. [Mã Agent 1.4.2](https://www.jupedsim.org/v1.4.2/_modules/jupedsim/agent.html) · [Kiểm chứng bridge](JUPEDSIM_API_COMPATIBILITY.md)

## Biểu diễn chính sách người

| Nhãn nguồn | Phạm vi diễn giải khi dùng SFM nguyên gốc |
| --- | --- |
| `neutral` | Điều kiện mục tiêu và tham số lực đã resolve. Mức giữ tốc độ khi có tương tác cần đo trên từng Card. |
| `cooperative` | Card yêu cầu hành vi như lách phải hoặc dừng trước vạch để nhường. API SFM không có tham số cho quy tắc ưu tiên hay nhường theo vạch. |
| `non_cooperative` | Card có điều kiện giữ hướng, không giảm tốc, tăng tốc hoặc lạng hướng tùy tình huống. API cho chỉnh tốc độ mong muốn và hệ số lực, nhưng chưa có bằng chứng một bộ hệ số tái hiện các hành vi đó. |

Phản ứng lực mô tả tương tác vận động. Nhãn về ý định nhường đường cần một phép đo hoặc điều kiện thí nghiệm riêng. Tách mọi logic bổ sung thành profile extension có provenance nếu được đặc tả. Mọi biến thể tham số sau này cần ghi trong cấu hình scene đã resolve. [Card FA](../research/HUMAN_MODEL_FINAL.md#L156) · [Card ND](../research/HUMAN_MODEL_FINAL.md#L185) · [Card IN](../research/HUMAN_MODEL_FINAL.md#L236)

## Kết quả sửa cổng 04R

Các nhánh advance/stop/yield/reverse dùng cùng scene và seed, tách từ 0,70 s. Cùng XY và ngoài vùng overlap, đổi vận tốc robot không đổi đề xuất SFM trong một bước. Chuyển động robot qua nhiều bước làm thay đổi khoảng cách/hướng lân cận và đề xuất người. Chênh lệch nhiều bước bao gồm sai số do proxy tự tích phân. Ca hẹp ghi guard và kết thúc ở overlap thật theo footprint. Các số đo thuộc `sfm_reconstruction_probe`. Backend `sfm_jupedsim`, YAML, episode nghiệm thu và viewer vẫn chờ cổng upstream. [Báo cáo và lệnh chạy](../reports/human_models/TASK_04R_REPORT.md)

## TASK 05R

Chọn Hydra `humans=sfm_jupedsim` với interpreter cô lập của 04S. Registry kiểm distribution, API native và nguồn import trong prefix, rồi ghi SHA-256 của extension trong backend version. Collision projection từ adapter đi qua `HumanBackendConfig.planar_world`. Robot là external actor giữ mẫu, người được rebase vào cùng native agent trước mỗi callback. Cả 50 và 100 ms đều dùng lưới native 10 ms. MuJoCo, limiter và guard giữ quyền thực thi.

Bốn nhánh Easy đạt hard constraints. Critical và Hard vẫn có contact. Nhịp 50 ms giảm sai lệch do hold nhưng chưa giải quyết contact của các ca đo. [Báo cáo 05R và lệnh chạy](../reports/human_models/TASK_05R_REPORT.md)
