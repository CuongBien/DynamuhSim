# Nghiệm thu track cảm biến / BEV tương lai — 22/10/2026

**Nghiệm thu được hình học và timestamp của sidecar BEV; chưa nghiệm thu person detector/association để train.** Toàn bộ lượt này chạy offline từ portable episode đã có, không chạy Gazebo, sửa ROS/Nav2, đổi hành vi người, train hoặc mở framework khác. BLAS/OMP được giới hạn một luồng; không có lượt mô phỏng GPU mới.

## Dữ liệu và phân tách vai trò

Nguồn: `portable_episode/generated/episode01`, manifest SHA256 `205981f83555d141481823feea00ae21295015c42f1f946c42ce0d27fa1922b4`, 1 episode SocNavGym standing v3 seed 42 với 6 người. Có 823 cửa sổ chồng lấn; split giữ pilot_only. Các nguồn và mask của bundle đã sealed không bị ghi lại.

Nhánh `track_episode.py` chỉ đọc depth, calibration, odom và timestamp/query mẫu; không đọc scene người, nhãn tương lai, persona hoặc simulator state. Tách `build_labels.py` đọc simulator pose ở phía LABEL để đo ghép hình học và dựng target. Các file NPZ input không có human GT/backend ID/future feature. Không dùng ground truth sửa track hoặc lựa chọn ứng viên.

## Baseline cảm biến đã chạy

Depth pinhole stride 4, range 0.28–5 m, điểm trong vùng cao 0.04–1.8 m; cụm XY cell 8 cm, nối lân cận, loại cụm quá rộng hoặc thiếu điểm/chiều cao. Đây là **cụm hình học không biết loại đối tượng**, có thể gồm chân người và vật thể khác. Không suy H1–H5 từ dáng đứng hoặc vận tốc.

Track dùng vị trí trong odom và constant-velocity prediction để ghép một-một; 3 lần quan sát mới đánh dấu confirmed, mất quá 0.5 s tạo ID mới. Không đưa prediction vào một timestep đang thiếu detection. Các tham số được lưu trước khi đối chiếu GT; không sweep hoặc tune theo kết quả ghép của episode này.

Đã xử lý 1013 frame, tạo 56 candidate ID, 3413 candidate-frame rows; 3315 row đạt điều kiện confirmed của baseline. 823 mẫu có tổng 2720 slot đang được quan sát ở cuối history, 49,004 timestep-slot hợp lệ trong history. Tối đa 16 slot/mẫu; không bị truncation. Slot/timestep thiếu giữ zero và `observed_valid=false`, không backfill từ tương lai.

## Đối chiếu GT và giới hạn

Ghép hình học một-một ở **đúng timestamp đo depth**, gate 0.60 m, margin so với lựa chọn khác 0.15 m. GT dùng tọa độ center của model; candidate là centroid của bề mặt nhìn thấy, nên sai số gồm bias bề mặt. Đây là diagnostic, chưa phải xác nhận semantic người.

| Chỉ số riêng của pilot | Kết quả |
|---|---:|
| Confirmed candidate rows | 3315 |
| Rows ghép hình học được | 2577 (77.74%) |
| Rows chưa ghép được | 738 |
| Sai lệch trung vị / p95 trong phần ghép được | 0.1362 / 0.2725 m |
| Đổi GT identity trên cùng track giữa hai lần ghép cách nhau ≤0.5 s | 0 |
| Số candidate ID từng ghép với mỗi GT người 1…6 | 5 / 1 / 3 / 3 / 3 / 4 |
| Slot cửa sổ có ≥10 history observations và mọi lần đều ghép cùng GT tại cuối history | 1854 |

77.74% là **tỉ lệ ghép trong các ứng viên**, không phải accuracy/precision/recall của detector. Không có nhãn visibility/occlusion chuẩn để tính recall: camera wedge XY chỉ là điều kiện góc/khoảng cách, không chứng minh người được nhìn thấy. Chỉ số đổi identity=0 cũng không phủ nhận đứt track hoặc nhầm người; có người mang tới 5 candidate ID. Các ứng viên vật thể, centroid lệch, occlusion và phân mảnh chưa được giải quyết. Vì vậy `semantic_person_valid`, `association_accepted` và training eligibility **giữ false**, kể cả 1854 slot khớp hình học history. H1–H5 tiếp tục chưa biết.

## Target BEV đã chốt cho pilot

[Contract](../contract.json): 3 mốc **+0.5 / +1 / +3 s**, grid 64×64, cell 0.1 m, x [0,6.4), y [−3.2,3.2), cao occupied [0.03,1.8] m. Mốc này là cấu hình pilot có version, không thay 30 bước nhãn quỹ đạo người 3 s của M1 hoặc tự sửa spec nghiên cứu cũ.

Target là **future observed depth occupancy**: chọn ảnh gần query trong tolerance 50 ms, chiếu điểm depth sang base_link tại lúc đo, rồi dùng SE(2) ego pose của simulator chuyển endpoint và camera origin về **khung robot cố định tại anchor**. Simulator ego poses nằm trong LABEL; không đưa vào input. Dùng ray hợp lệ để suy free, giữ unknown khi không có quan sát, không clear sau endpoint hoặc từ NaN/Inf. Occupied thắng free khi nhiều ray cùng ô. Đây là phép chiếu planar; chưa nghiệm thu world 3D/roll-pitch calibration.

Đã dựng **2469/2469 target frame**, sai lệch depth measurement so với future query bằng 0 ns ở lượt này. Có 7,658,022 unknown cells, 2,322,466 free cells và 132,536 occupied cells. Quan sát được khoảng 24.60% / 24.52% / 23.71% grid tại ba horizon. `cell_observed_valid` mask vùng unknown khỏi loss và kết hợp frame_valid. Đây không phải full-world occupancy, social cost hoặc nhãn phản thực cho một action robot khác; vùng nhìn thấy cũng phụ thuộc chuyển động camera/robot đã thực hiện.

BEV input cũng được đưa về anchor **ước lượng bằng odom nhân quả**, thay vì coi depth 100 ms cũ đã đồng thời với anchor. Pose dùng ZOH mới nhất đã được giao tại query, giữ timestamp source/anchor, không lấy odom tương lai. Odom có sai số và độ cũ nên input có thể lệch với nhãn dùng true anchor; không tuyên bố căn chỉnh hoàn hảo hay dùng oracle sửa input. Camera native thấp vẫn chỉ quan sát một phần người và môi trường; BEV có dạng các tia/khoảng trống quan sát, không phải footprint đầy đủ của người hoặc costmap an toàn.

## Artifact và cách dùng

```text
generated/sensor01/
  sensor_tracks.jsonl          candidate rows mỗi frame, không có GT
  track_inputs.npz             [823,20,16,4] XY/vXY + validity/IDs/timestamps
  current_bev_inputs.npz       [823,64,64], odom source/anchor timestamps
  sensor_manifest.json
 generated/labels01/
  association_diagnostics.jsonl   GT-side pairing, không cho vào input
  association_hints.npz           GT ID / history diagnostics, accepted=false
  future_bev_labels.npz           [823,3,64,64] + per-cell mask / timestamps / GT ego poses
  association_summary.json, target_summary.json, label_manifest.json
```

Giữ các source hashes, source bundle manifest và inventory trong audit. Các sidecar lớn bị Gitignore; Git chứa code, báo cáo, hình và [mẫu giữa cố định 411](examples/). NPZ đọc bằng NumPy `allow_pickle=False`; tracking cần thêm SciPy, preview cần Matplotlib. Không cần ROS/Gazebo để chạy offline. [Lệnh chạy](../README.md).

## Kiểm chứng và điều kiện còn thiếu

[Audit cuối](audit02/audit.json) PASS: đủ 66 file bundle gốc khớp hash; kiểm chọn frame bằng brute-force cho 2469 target; đối chiếu độc lập 6503 occupied endpoints bằng công thức pinhole/SE(2) riêng và 2577 khoảng cách pairing; replay 100 frame sensor prefix chỉ với odom đã khả dụng cho kết quả ID y hệt. Kiểm một-một, masks, unknown, timestamp và không có GT trong input. [12 test](contract_tests.txt) gồm late/future odom poisoning, clock reset, mất/đứt track, ego translation, ambiguity, optical axes, frame rotation, unknown rays và thời gian hỗ trợ nhãn.

[Hình](sample_preview.png) lấy từ dữ liệu ghi thật, cố định sample giữa 411, không chọn mẫu theo độ đẹp. Hàng dưới là nhãn depth ở tương lai, **không phải model prediction hoặc ảnh AI sinh**. Dấu ứng viên so với GT ở góc phải chỉ để chẩn đoán; GT không được nạp vào tracker.

Bước kế cần nghiệm thu detector chỉ từ depth phân biệt người/vật thể và xử lý occlusion/đứt ID, với tiêu chí semantic/visibility và episode độc lập trước khi bật mask association. Cần đánh giá sai lệch input odom–label GT, calibration và camera coverage trước khi dùng loss BEV. HuNavSim giữ cho test cuối, không tune/train; chưa thu batch đa nguồn hoặc train World Model. Chỉ số geometry PASS không thay thế nghiệm thu perception.
