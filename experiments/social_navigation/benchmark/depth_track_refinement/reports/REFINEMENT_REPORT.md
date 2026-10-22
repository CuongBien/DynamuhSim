# Kết quả depth track refinement — 22/10/2026

**Nghiệm thu được phép so sánh và cải thiện hình học trên depth mesh; chưa nghiệm thu detector tổng quát hoặc tích hợp.** Validation ban đầu với depth proxy cũ FAIL và được giữ nguyên. Baseline cũ, portable episode, mask train và ROS/Nav2 không bị thay. Lượt này chỉ đọc dữ liệu đã ghi, không chạy Gazebo/GPU hoặc train World Model.

## Thay đổi và cách kiểm chứng

Baseline trước tạo cả ứng viên người và bề mặt cột/kệ. Trên episode phát triển, các track không ghép được có nhiều bề mặt cao khoảng 1.7 m, hẹp hoặc ít thay đổi theo chiều cao. Thử nghiệm thêm prior hình học: vertical span ≥1.55 m và XY minor standard deviation ≤0.025 m, hoặc tâm các lớp cao gần cố định (shift ≤0.04 m, ít nhất 4/5 lớp có dữ liệu). Không gán semantic người cho các cụm còn lại. Các kích thước là giả định hình học của pilot, chưa phải quy luật phổ quát phân biệt người/vật thể.

Tracker mới giữ đường ghép active của baseline, thêm bộ nhớ dormant tối đa 1.5 s cho track đã có ≥5 hits. Sau gap >0.5 s chỉ nối ID nếu vị trí, descriptor hình dạng và điều kiện mutual-unique đủ rõ; gap được ghi. Khi mất detection, không phát observation giả hoặc backfill history. Vị trí/vận tốc lấy từ depth + causal odom; GT chỉ ở evaluator. Hình dạng người có thể giống cột hoặc đổi do góc nhìn, nên mọi semantic/association acceptance vẫn false.

So sánh bốn ablation để tách tác động lọc hình dạng và giữ ID. [Config](../config.json), snapshot baseline và source hash được cố định ở [frozen01](frozen01.json) **trước** replay/evaluate hai lượt đầu. Kết quả validation thất bại không được dùng để sửa threshold. [frozen02](frozen02.json) bổ sung một recording cùng depth mesh sau khi chẩn đoán; engine/config byte-identical, có parent hash. Đây là follow-up diagnostic, không phải validation mới thay cho failure.

## Nguồn dữ liệu

| Vai trò | Recording | Depth frames | Tình trạng nguồn |
|---|---|---:|---|
| Development | avoidance_moderate01 | 1013 | Mesh depth, proxy OFF, điều hướng đã đạt |
| Validation định trước | dataset_pilot/pilot01 | 404 | Camera cũ còn nhìn proxy hình trụ; điều hướng đã đạt |
| Matched-depth diagnostic bổ sung | avoidance_fixed01 | 1037 | Mesh depth, proxy OFF; điều hướng **TIMEOUT** vẫn giữ |

Tổng 2454 ảnh của 3 lần ghi riêng, cùng SocNavGym seed 42 và nhóm cảnh; không phải test khác seed/cảnh/backend. Không sử dụng HuNavSim. Thời điểm warmup/tail cũng được tracker xử lý; metric candidate-frame không tương đương số mẫu train hay số episode độc lập.

Lượt bổ sung được copy raw/result/events từng byte sang thư mục experiment mới rồi chạy exporter legacy không đổi. [Audit raw](mesh_repeat_raw_audit.json) xác nhận 1037/1037 ảnh khớp MCAP, nhãn/action đọc lại được. Có 802 cửa sổ, 49 ứng viên cửa sổ bị loại bởi normalizer vì quy tắc dữ liệu; QA format PASS giữ `actual_goal_reached=false`. Không đưa episode này thành navigation success hoặc cho phép train.

## Kết quả theo ablation

| Nguồn / biến thể | Paired rows | Unpaired rows | Tỉ lệ ghép hình học | Phân mảnh proxy |
|---|---:|---:|---:|---:|
| Development baseline | 2577 | 738 | 77.74% | 13 |
| Development shape_only | 2577 | 164 | 94.02% | 13 |
| Development tracking_only | 2579 | 758 | 77.28% | 10 |
| Development combined | 2579 | 173 | 93.71% | 10 |
| Mesh repeat baseline | 2797 | 902 | 75.62% | 10 |
| Mesh repeat shape_only | 2797 | 304 | 90.20% | 10 |
| Mesh repeat tracking_only | 2801 | 908 | 75.52% | 8 |
| Mesh repeat combined | 2801 | 308 | 90.09% | 8 |
| Legacy proxy baseline | 997 | 94 | 91.38% | 3 |
| Legacy proxy shape_only | 605 | 44 | 93.22% | 3 |
| Legacy proxy tracking_only | 995 | 100 | 90.87% | 3 |
| Legacy proxy combined | 603 | 50 | 92.34% | 3 |

[Development](development/evaluation.json), [validation FAIL](validation/evaluation.json), [mesh repeat](mesh_repeat/evaluation.json). Combined giữ hơn 100% paired rows của baseline trên hai lượt mesh, giảm unpaired 738→173 và 902→308; fragmentation proxy giảm 13→10 và 10→8. Đây là bằng chứng cải thiện baseline hình học trong phạm vi hai recording này, không phải semantic precision/recall.

Trên proxy validation, tỉ lệ ghép có vẻ tăng nhưng paired rows giảm **997→603, chỉ giữ 60.48%**. Gate cố định yêu cầu giữ ≥95% paired rows, giảm unpaired và fragmentation, không tăng short-gap identity changes, không có known-wrong reacquisition; validation không đạt. Không chọn chỉ số tỷ lệ để che số lượt ghép bị mất.

Fragmentation proxy = tổng max(0, số candidate ID từng ghép với mỗi GT người −1), không phải IDF1/MOTA hoặc fragmentation chuẩn theo visibility. Short-gap paired identity changes đo trong ≤0.5 s đều 0; không chứng minh giữ đúng identity lúc khuất. Combined có 6 dormant reacquisitions ở development, 4 ở mesh repeat và 2 ở proxy; chỉ 2 / 2 / 0 sự kiện ghép được GT ngay khi nối lại, không có known-wrong trong phần kiểm được. Các sự kiện chưa ghép không được coi là re-ID thành công.

## Vì sao không dùng bộ lọc mới cho mọi dữ liệu

[Profile evidence](depth_profile_evidence.json) cho thấy camera cũ không có depth proxy filter, trong world vẫn có 6 proxy cylinder radius 0.36 m, cao 1.7 m. Hai recording mesh có plugin depth filter và camera `proxy_low_off`, kèm camera ON đối chứng. Plugin ẩn proxy visual chỉ khi camera OFF render; không xóa collision hoặc LiDAR proxy. Nguồn SDF/filter được kiểm read-only, không serialize XML hoặc sửa runtime.

[Phân tích loại trực tiếp](rejection_effect.json): prior mới loại **383 baseline rows vốn ghép hình học với người** trong proxy validation (người 1/4/5/6); hai lượt mesh không có baseline-paired row bị loại trực tiếp. Phần mất thêm có thể đến từ confirm/reset/association sau khi loại detection, nên không quy toàn bộ delta cho 383 sự kiện đó. [Regression test](contract_tests.txt) có cylinder proxy giống người và xác nhận prior cũng loại nó: geometry đơn thuần không biết cylinder là người hay cột. Không dùng GT hoặc tên simulator để cứu detection trong nhánh input.

Sự khác biệt profile là lý do cụ thể làm phép lọc không chuyển nguyên trạng sang dữ liệu cũ. Không xoá dữ liệu proxy, không đổi failure thành pass, không tự đổi mục tiêu sau kiểm tra. Dữ liệu cũ vẫn có ích cho thử domain shift và các nhãn khác đã audit; chưa trộn nó với mesh để tuyên bố detector tổng quát.

## Audit và artifact đã lưu

[Audit](audit01/audit.json) PASS cho integrity/reproducibility: engine/config không đổi giữa các dataset, baseline development tái lập chính xác JSON của stage trước, kiểm độc lập 24,708 khoảng cách pairing qua 12 bảng kết quả, replay 300 frame prefix không cho odom tương lai tham gia, timestamps nhân quả và pairing một-một. Đây không phải PASS semantic validation. [13 test](contract_tests.txt) đạt, gồm ego motion, late/future odom, gap/TTL, không ép ID khi mơ hồ, shape không khớp, cylinder domain limitation và clock reset.

```text
code: refinement.py, replay.py, evaluate.py, config.json
      freeze.py, extend_freeze.py, extra_repeat.py
      prepare_repeat.py, diagnose_profiles.py, audit.py, preview.py
      test_refinement.py, snapshots/, run.sh
reports/: frozen receipts, summaries/evaluations, PASS/FAIL acceptance
          profile/rejection evidence, raw audit, inventories, test log, PNG
 generated/: source-copy raw/normalized, per-frame tracks and GT-side pairs
```

Raw/NPY/JSONL lớn ở generated bị Gitignore; code/reports/hashes/hình được lưu Git. Dữ liệu nguyên bản và thay đổi của người dùng được giữ qua checkpoint. [Lệnh tái chạy](../README.md). Hình [toàn bộ profile](profile_comparison.png) và [matched mesh](matched_mesh_preview.png) dùng frame giữa cố định, không chọn theo chất lượng. Đây là depth đã ghi và candidate output thực, GT overlay chỉ ở report; không ảnh AI hoặc World Model prediction.

## Quyết định và việc còn lại

Giữ baseline cũ; refinement chỉ là experiment cho nghiên cứu, không tự áp dụng cho camera profile khác/không rõ. Chưa nghiệm thu semantic detector, association dùng train, H1–H5 hoặc World Model. Việc đúng tiếp theo là một episode mới cùng depth mesh nhưng seed/layout khác, có tiêu chí visibility/semantic và kiểm qua cả người–vật thể/occlusion trước batch. Nhãn từ simulator có thể hỗ trợ đánh giá, nhưng không được đưa vào input hoặc suy persona từ hình dáng. HuNavSim tiếp tục giữ test cuối.
