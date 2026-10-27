# Kiểm chứng seed 43 — 22/10/2026

**Đã hoàn tất một lượt ghi mới và kiểm chứng code cố định. Dữ liệu đạt integrity; navigation và gate tracking kết hợp chưa đạt.** Không thay baseline, không bật training hoặc sửa ROS/Nav2/native worker. Lượt thu này thực sự chạy Gazebo trên NVIDIA; ảnh/GIF và arrays lấy từ recording, không dùng AI sinh ảnh.

## Thiết kế trước khi chạy

[Protocol](../protocol.json) chọn seed 43 thay 42, cùng cảnh standing với sáu người/hai nhóm ban đầu, start (−28,0,0), goal (−20,0,0), budget 90 sim s và 540 wall s cho pha điều hướng; một attempt. Giữ cùng depth mesh OFF proxy, ON diagnostic, LiDAR proxy, robot và Nav2 moderate radius 0.25 m + padding 0.01 m, inflation 0.8 m/scaling 3, vx max/min 0.30/−0.15, vy max 0.20 m/s.

Driver copy từ parent đã đạt, chỉ đổi đường dẫn runtime để toàn bộ dữ liệu mới nằm trong stage; CLI truyền seed 43. [Manifest driver](driver/manifest.json) lưu inference source/config hashes **trước khi capture**; không thay threshold theo episode mới. [Source extension](frozen_seed43.json) thêm dữ liệu mới vào cùng engine/config. HEAD capture là 5e5de7e và wrapper khi đó chưa commit; driver/helper SHA là bằng chứng implementation thực tế.

Đây là recording/seed mới độc lập với development seed 42, cùng layout/simulator/profile; không phải test khác layout/backend. Không sử dụng HuNavSim. Seed 43 dành internal_validation_only, không đưa vào train/tuning của bản frozen này.

## Kiểm tra mô phỏng và kết quả thu

[Capture audit](capture/capture_audit.json) PASS: nguồn code không đổi, worker/metadata seed 43, không adapter fault, recorder exit 0, bốn literal wheel friction frame được native SDK nhận, actual SDF/Nav2/live params khớp cấu hình đã chọn. [World parity](world_parity.json) xác nhận hai runtime world giống nhau khi chuẩn hóa duy nhất socket do mỗi phiên sở hữu.

GPU preflight và actual Ogre log đều xác nhận **D3D12 NVIDIA GeForce RTX 4060 Laptop GPU**, không software renderer. Giữ cơ chế pause→stats xác nhận dừng→settle→stop, callback balanced và cleanup không còn process của phiên. Không mở cửa sổ GUI ở lượt này; observer camera vẫn ghi ảnh/GIF. Không tuyên bố physics/Nav2 toàn bộ chạy GPU.

| Kết quả | Đo được |
|---|---|
| Recorder/cảm biến | 1051 depth float32 [240,424], 16 topic đọc được |
| Cặp depth auxiliary ON/OFF | 1042 cặp, 902 active; 688 active có ≥100 pixel thay đổi |
| Navigation | **TIMEOUT**, active 90.025 sim s / 231.44 wall s |
| Actual goal | Chưa tới; XY 3.7013 m, yaw 2.9787 rad |
| Observation tail | 3.3975 sim s sau action end |
| Native human trajectory | Khác seed 42 từ native step 39 trong đối chiếu; không chỉ đổi metadata |

Quỹ đạo người được so theo native step, có 911 bước chung, max XY difference 28.57 m. Robot inputs cũng khác trong hệ tương tác khép kín, nên không quy toàn bộ chênh lệch cho riêng RNG. Sáu vị trí nhóm ban đầu vẫn theo fixture giống nhau. Kết quả TIMEOUT giữ nguyên; dữ liệu nhận thức hợp lệ không biến nó thành navigation success.

## Chuẩn hóa và integrity

Exporter legacy byte-identical đã đọc MCAP sau khi recorder đóng. Có **802/851 cửa sổ**, 49 bị loại bởi rule dữ liệu; đây là cửa sổ chồng lấn của **một episode**. Nhãn quỹ đạo người/history/action giữ contract cũ; human/persona GT không đi vào observation. QA format PASS giữ `actual_goal_reached=false`.

[Audit raw độc lập](raw_audit.json) PASS: **1051/1051 ảnh depth khớp MCAP từng byte**, future human XY sai số lớn nhất khoảng 9.77e−15 m, future executed actions khớp resampling. Mỗi modality có 16040 history elements hợp lệ; goal_valid 538/802, số còn lại masked theo contract. NaN/Inf và timestamp gốc giữ nguyên. Raw là authority, không thay bằng tập paired NPY phụ có thể thiếu callback.

[Audit metric](audit01/audit.json) kiểm độc lập 7932 khoảng cách pairing qua bốn ablation, replay 100 frame prefix với chỉ odom đã khả dụng, kiểm thời gian/one-to-one/mask và tính lại gate từ config. Raw hashes sau dẫn xuất không đổi. Audit PASS là integrity/reproducibility; không thay gate FAIL hay nghiệm thu semantic.

## Kết quả so sánh code cố định

| Biến thể | Paired rows | Unpaired rows | Tỉ lệ ghép hình học | Fragmentation proxy | Short-gap pairing changes |
|---|---:|---:|---:|---:|---:|
| Baseline | 1983 | 900 | 68.78% | 9 | 1 |
| Shape only | 1983 | 266 | 88.17% | 9 | 1 |
| Tracking only | 1983 | 915 | 68.43% | 9 | 1 |
| Combined | 1983 | 268 | 88.09% | 9 | 1 |

[Xem số liệu](new_seed_evaluation.json). Bộ lọc giữ 100% paired rows của baseline, combined giảm 632 unpaired rows (70.22%). Sai lệch của phần ghép được median 0.1182 m / p95 0.2424 m, không thay đổi giữa các variant. Đây là bằng chứng bộ lọc hình dạng giảm ứng viên không ghép trong episode mesh mới, không phải accuracy/precision/recall người.

Cơ chế dormant trong combined có hai lần nối lại ID nhưng không lần nào ghép được GT ngay lúc nối. Không coi đó là hai lần re-ID người thành công. Fragmentation không giảm và vẫn có một lần đổi đối ứng GT, nên **gate combined FAIL** theo tiêu chí đã cố định yêu cầu fragmentation phải giảm. Không nới tiêu chí sau khi thấy dữ liệu. Lỗi với proxy cũ của stage trước vẫn giữ nguyên; seed mới mesh không xóa failure đó.

Fragmentation proxy vẫn là tổng max(0, số candidate ID từng ghép mỗi GT người −1), không phải metric IDF1/MOTA theo visibility. Các unpaired rows có thể là vật thể hoặc người không được ghép theo gate; thiếu nhãn visibility/semantic nên không gọi tất cả là false positives.

## Case cần xử lý tiếp

[Diagnostic LABEL ONLY](identity_case_LABEL_ONLY.json) lưu trường hợp track đổi pairing từ **GT5 tại 13.5 s sang GT6 tại 13.9 s**, cùng hiện tượng ở baseline và combined. Case có các quan sát/timestamp/extent quanh sự kiện; không đưa GT vào tracker. Đây là thay đổi ghép hình học, chưa phải nhãn identity-switch đã kiểm thủ công theo semantic/visibility. Active matching của combined giữ nguyên baseline, nên kết quả gợi ý cần kiểm điều kiện nhận association sau thiếu quan sát và lúc mơ hồ; chưa kết luận root cause chỉ từ một gate GT.

Nếu dùng case này để phát triển bản tiếp theo, seed 43 trở thành regression/development cho bản đó; phải dành một seed chưa xem để kiểm chứng bản mới. Không tune rồi báo seed 43 vẫn là validation độc lập.

## Artifact và quyết định

```text
generated/runs/depth_seed43_01/  raw_bag, result/events, normalized,
                               paired_depth, observer media, logs
 generated/runtime/depth_seed43_01/  world/SDF/Nav2, worker trajectory,
                                    GPU log, shutdown/cleanup evidence
 generated/drivers/             actual runtime driver, manifest, diff
 generated/sensor01/            bốn sensor-only candidate streams
 generated/evaluation01/        GT-side pairing, evaluator output
reports/                       compact summaries, hashes, audits,
                               driver/config snapshots, ảnh/GIF
```

Code, reports và bằng chứng nhỏ được commit; raw/NPY/JSONL lớn nằm local trong generated bị Gitignore, không được backup bằng Git. [Cách chạy/tái kiểm chứng](../README.md). [Ảnh comparison](seed43_comparison.png) chọn frame giữa cố định 525 tại 52.5 s, không chọn theo chất lượng. [GIF observer](capture/camera_preview_4x.gif) từ Gazebo thật, playback 4×, không ảnh tổng hợp AI. GT overlay chỉ là nhãn đánh giá, không phải World Model prediction.

[Acceptance](acceptance.json): data/raw/metric integrity PASS; navigation FAIL_TIMEOUT; combined geometry FAIL; semantic detector/association/H1–H5/training đều false. Chưa thu batch. Bước tiếp hợp lý là xử lý active association mơ hồ/thiếu quan sát bằng sensor-only và regression từ case đã lưu, sau đó đóng băng code và kiểm seed mới. Navigation timeout là vấn đề riêng được giữ lại, chưa sửa trong lượt perception này.
