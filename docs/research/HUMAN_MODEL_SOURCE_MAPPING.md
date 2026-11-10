# Ánh xạ đặc tả Human Model Final

Các mốc dòng trỏ tới bản nguồn đã kiểm SHA-256 `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`. Cột sở hữu ghi vị trí triển khai theo lộ trình. Cấu hình minh họa trong nguồn có tên khác một số Card và nhiều trường trống. Tên Card dưới đây là ID nghiên cứu giữ nguyên. Runtime v1 kiểm `scenario_id` dạng snake_case, nên cấu hình benchmark phải khai báo ánh xạ tường minh và lưu ID nghiên cứu trong kế hoạch. Quyết định biểu diễn bền vững và fingerprint thuộc cổng viết kế hoạch, chưa sinh cấu hình benchmark trong TASK 01. [Nguồn nghiên cứu](HUMAN_MODEL_FINAL.md) · [Sổ sai khác](HUMAN_MODEL_DEVIATIONS.md)

## Thuật toán và ngân sách

| Nguồn | Cấu hình và module sở hữu | Test và báo cáo sở hữu |
| --- | --- | --- |
| [Môi trường và thuật toán, dòng 1–10](HUMAN_MODEL_FINAL.md#L1) · ORCA thuần | `configs/humans/`, backend tại `src/narrow_nav/humans/` dùng `rvo2.PyRVOSimulator` | TASK 00 đã kiểm binding. TASK 03 kiểm ORCA đối chứng, reset và thuật toán. |
| [SFM, dòng 7](HUMAN_MODEL_FINAL.md#L7) | `configs/humans/`, bridge TASK 04 và `jupedsim.SocialForceModel` | TASK 00 đã kiểm API. TASK 05 kiểm tham số, goal và tương tác có MuJoCo. |
| [AVM, dòng 8](HUMAN_MODEL_FINAL.md#L8) | Cùng biên JuPedSim, model `AnticipationVelocityModel`, seed và cấu hình riêng | TASK 00 kiểm API. TASK 06 kiểm tương tác và tái lập. |
| [CSM, dòng 9](HUMAN_MODEL_FINAL.md#L9) | Cùng biên JuPedSim, model `CollisionFreeSpeedModelV2` | TASK 00 kiểm API. TASK 07 kiểm tham số và tương tác. |
| [Bảng episode, dòng 12–18](HUMAN_MODEL_FINAL.md#L12) | `configs/experiments/` và `experiments/` sở hữu plan, key ghép cặp và ngân sách | TASK 08 test số key duy nhất 48/540/900 và kiểm kế hoạch tĩnh trước khi chạy. |

## Bốn nhóm metric

| Nguồn | Cấu hình và module sở hữu | Test và báo cáo sở hữu |
| --- | --- | --- |
| [I · An toàn và thành công, dòng 36–47](HUMAN_MODEL_FINAL.md#L36) | Runtime ghi lifecycle, contact và trajectory. `evaluation/` sở hữu S/SR, C/CR và loại contact, TO, FP, ST, T, PL, SPL. Config thí nghiệm giữ dung sai goal, freeze window và chính sách phân loại. | Fixture đạt đích, timeout, contact và lỗi runtime. TASK 08 kiểm công thức/mẫu số và lưu cả số lần thử lỗi hạ tầng. |
| [II · Chuyển động và xã hội, dòng 51–62](HUMAN_MODEL_FINAL.md#L51) | `evaluation/` đọc GT, executed và clearance theo timestamp để đo V, A, jerk, CD, PSC, DH, TTC và AT. Cấu hình metric chứa cửa sổ và đơn vị. | TASK 08 dùng quỹ đạo giải tích, ca không va chạm, thiếu contact solver và nhiều tần số lấy mẫu. |
| [III · Quy mô và độ phủ, dòng 66–79](HUMAN_MODEL_FINAL.md#L66) | `experiments/` lập plan. `evaluation/` kiểm trajectory count, duration, mật độ, encounter, coverage, edge case và lỗi mô phỏng theo manifest. | TASK 08 kiểm cardinality, dedup, coverage theo strata và giữ lỗi hạ tầng trong mẫu số lần thử. |
| [IV · Gán nhãn và học từ dữ liệu, dòng 83–91](HUMAN_MODEL_FINAL.md#L83) | `evaluation/` và quy trình dữ liệu riêng sở hữu learned/surveyed realism, legibility và visibility. Điều kiện đối chiếu cần dữ liệu tham chiếu và giao thức đánh giá được công bố. | Chưa có dữ liệu đánh giá realism hoặc visibility đủ điều kiện. Báo trạng thái thiếu bằng chứng theo metric. |
| [Gate và báo phân phối, dòng 95–122](HUMAN_MODEL_FINAL.md#L95) · [Gate đồng bộ, dòng 300–303](HUMAN_MODEL_FINAL.md#L300) | `evaluation/` giữ gate dự án S=1, C=0, TO=0, FP=0. Điểm liên tục phục vụ phân phối và outlier. | TASK 08 test phân biệt hoàn thành nhiệm vụ, thất bại tương tác và `INVALID_INFRA`. Ngưỡng mang provenance dự án. |

## Sáu Card kịch bản

`configs/worlds/`, `configs/scenarios/` và `sim/mujoco/` sở hữu geometry, spawn, goal và tham chiếu scene. Cấu hình experiment sở hữu policy robot, strata người và lựa chọn điều kiện. `evaluation/` sở hữu điều kiện labeling. Các fixture repo hiện có dùng để kiểm kỹ thuật trước khi khớp Card.

| ID giữ nguyên và nguồn | Geometry, biến và label phải khớp Card | Test và báo cáo sở hữu |
| --- | --- | --- |
| [`FA_Corridor_Passing`](HUMAN_MODEL_FINAL.md#L143) | Hành lang thẳng, 2,1/1,2/0,9 m, 1–2 người, đối đầu; Advance/Stop/Yield/Turn/Reverse. Label góc, khoảng cách khởi đầu và hướng đối diện. | TASK 01 sàng bề rộng, fixture `head_on`. TASK 08 kiểm label, deadlock, reciprocal dance và nhánh robot. |
| [`ND_Doorway_Passing`](HUMAN_MODEL_FINAL.md#L172) | Hai phòng nối cửa 1,2/0,9/0,6 m, một người hoặc nhóm hai. Label hai bên cùng trong vùng tâm cửa và có quỹ đạo qua cửa. Policy Reverse còn xung đột giữa Card, YAML mẫu và plan. | TASK 01 kiểm cổ chai và ca đường bị chặn. TASK 08 kiểm đường tới cửa, điểm chờ, tương tác và danh sách nhánh đã duyệt. |
| [`BC_Corner_Turn`](HUMAN_MODEL_FINAL.md#L197) | Góc chữ L, 2,1/1,2/0,9 m, một người đi ngược chiều, LOS bị chắn rồi lộ diện. Advance/Stop/Yield. | Test góc khuất hiện có kiểm LOS hình học. TASK 08 kiểm thời điểm lộ diện và label, cảm biến cần gate riêng. |
| [`IN_Crossing_Paths`](HUMAN_MODEL_FINAL.md#L223) | Giao cắt X/T, 2,1/1,2/0,9 m, 1–2 người. Label góc 90° ±20° và lệch thời gian tới tâm ≤2 s. Card chỉ nêu Advance/Stop/Yield. | TASK 08 kiểm cặp thời điểm/góc và duyệt Reverse của nhánh B trước khi lập đủ ma trận. |
| [`OT_Same_Direction`](HUMAN_MODEL_FINAL.md#L249) | Hành lang ≥10 m, 2,1 / {1,2 hoặc 1,6} / 0,9 m. Một người, hai subtype ai vượt ai. Label hướng lệch ≤20° và khoảng cách ≤4 m. Advance/Follow/Yield. | TASK 08 kiểm coverage từng subtype và điều kiện chọn một Critical. |
| [`MG_Small_Group`](HUMAN_MODEL_FINAL.md#L275) | Arena/warehouse/hospital, dải 1,6–2,1 m, nhóm 3–4 hoặc 5–9. Label ≥3 người đồng thời trong bán kính 4 m. Advance/Yield/Follow; lưu liên kết nhóm và người độc lập. | TASK 08 kiểm strata, labeling, cohesion và route. Ba mức geometry và cardinality theo mức còn `BLOCKED_SPEC`. |

## Card thực thi

| Nguồn | Chủ sở hữu và cổng kiểm định |
| --- | --- |
| [Vòng 0, dòng 320–331](HUMAN_MODEL_FINAL.md#L320) | TASK 08/plan chọn Easy đúng từng Card sau khi làm rõ, hai seed đầu của A. Cổng headless MuJoCo ghi lỗi hạ tầng, không xuyên/teleport và GT 10 Hz riêng. 48 key được thiết kế để tái dùng. |
| [Vòng 1, dòng 335–354](HUMAN_MODEL_FINAL.md#L335) | TASK 08/plan giữ 360 A + 180 B. Vòng 1 bổ sung 312 A + 180 B sau khi tái dùng 48 lần thử Vòng 0. Báo phân phối, outlier và kết quả đối chứng robot. |
| [Vòng 2, dòng 358–370](HUMAN_MODEL_FINAL.md#L358) | TASK 08/plan thêm 360 key Critical của nhánh A, 15 seed mới cho mỗi cặp 4 × 6. Có 20 seed cho mỗi Critical của A sau gộp. Báo coverage và khoảng tin cậy 95%. |
| [YAML minh họa, dòng 374–417](HUMAN_MODEL_FINAL.md#L374) | TASK 08 sở hữu cấu hình thực. Alias SC1/SC2, nhánh policy, ngưỡng và 10 Hz phải đối chiếu Card, ADR, schema và ledger trước khi publish. |

Key dùng để dedup dự kiến gồm ID/revision backend, ID Card và fingerprint geometry/scene đã resolve, profile vật lý, policy người/mật độ, policy robot và tham số, cùng episode seed. Phase tag, run ID và episode ID không đổi bản chất điều kiện. Chỉ tái dùng khi các thành phần thực nghiệm và provenance khớp. Nếu Vòng 0 được chạy lại độc lập với cả Vòng 1, số lần chạy thực tế là 588 tới Vòng 1 và 948 tới Vòng 2. Báo riêng số key duy nhất và số lần thử. [Sổ sai khác](HUMAN_MODEL_DEVIATIONS.md)
