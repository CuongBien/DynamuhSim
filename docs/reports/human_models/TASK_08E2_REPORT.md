# TASK 08E-2 · Benchmark Human Navigation Vòng 2

**`READY_FOR_HUMAN_MODEL_CLOSEOUT`.** 900/900 slot duy nhất có kết quả hợp lệ về hạ tầng, gồm 540 lượt giữ lại và 360 lượt Critical mới. Audit xác nhận 900 attempt, 0 lỗi hạ tầng, 0 gián đoạn, 0 retry và 5.403 tệp lịch sử giữ nguyên SHA-256. Có 543 lượt đạt nhiệm vụ an toàn. Đủ bằng chứng để chuyển sang bước nghiệm thu 08F. [Tổng hợp kiểm toán](../../../runs/task_08e2/report/audit_summary.json)

## Thiết kế, cổng tái sử dụng và provenance

Thiết kế đã duyệt khớp công thức Vòng 2. Nhánh A có 720 slot Advance, gồm 120 Easy, 480 Critical và 120 Hard. Nhánh B giữ 180 lượt Stop, Yield/Turn và Reverse từ Vòng 1. Vòng 2 bổ sung đúng 360 lượt Critical Advance ở seed 1005–1019. Mỗi cặp backend × Card có đủ 20 seed Critical Advance, gồm năm seed giữ lại và 15 seed mới. OT Critical giữ hành lang rộng 1,60 m, một người và quy tắc seed chẵn/lẻ đã duyệt, cho 10 lượt mỗi subtype ở mỗi backend. [Phase 2 manifest](../../../runs/task_08e2/round_2.json) · [Ma trận và binding](../../benchmarks/SCENARIO_MATRIX.md) · [Deviation ledger](../../research/HUMAN_MODEL_DEVIATIONS.md)

Workspace thực tế là `/home/haduckien/nav_ws`. Nhánh làm việc `mujoco-hummodel`, HEAD `b50d0844904e7b90cc6371dafbf601878bfa960a`, có thay đổi từ các task trước. Để khớp provenance, runner tiếp tục run gốc từ snapshot sạch `codex/task-08d-snapshot`, commit `0d86e376fff8ec54c6c9dd3b08330e283b2ad042`, code SHA-256 `6cb053a26ec7d0bd8c55465532b9fc016c624b36b258f5590caee2aa34efdf7a`. SHA-256 nguồn nghiên cứu là `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`. [Run lock](../../../runs/task_08d/benchmark_study/run.json) · [Nguồn nghiên cứu](../../research/HUMAN_MODEL_FINAL.md) · [Source mapping](../../research/HUMAN_MODEL_SOURCE_MAPPING.md)

Runtime dùng Python 3.12.3, MuJoCo 4.0.0, RVO2 0.0.0 và JuPedSim `1.4.2+task04u`. Native JuPedSim khớp entry trong wheel 08B, SHA-256 wheel `76997cc89c2cbfd44fb7279925cfa2ac884a06802b14bd2c3f1bed8d8eebd94c` và extension `5ebdd4593786734392fefa0c3748f4431d3de53fb857614f6a36a8919b23ec65`. RVO2 binary có SHA-256 `af2aac265a6fee5bd01fdb1dbeb14df557b3f9c36a43aff0ee0deb9b9012ccf3`. Bindings chọn một backend cho toàn bộ người trong mỗi episode. Bridge 04U kiểm readback native của robot external và roster người ở từng callback. MuJoCo là nguồn pose, vận tốc và contact. Lịch robot vòng hở chỉ dùng timestamp hiện tại. [Provenance wheel](../../../runs/task_08b/jupedsim_04u/provenance.log) · [Nghiệm thu 04U](TASK_04U_REPORT.md) · [Hợp đồng QA](../../benchmarks/DATA_QA_PROTOCOL.md)

Preflight đối chiếu 900 slot với compiler, cấu hình, geometry và cổng native. Cả 540 slot cũ có execution key khớp với attempt hợp lệ và đúng phase gốc, trong đó có 48 slot Vòng 0. Có 0 slot thiếu, 0 mismatch, 0 trùng và 360 slot mới chưa có attempt. Key của runner khóa binding, backend/version, config, MJCF, asset và runtime. Audit đối chiếu thêm lịch hành động, seed streams, pose reset, số người, policy, subtype, nhịp bước và quy tắc đánh giá. Đã lưu SHA-256 của 5.403 tệp từ trước Vòng 2, gồm ba tệp lock/phase và 5.400 tệp attempt. [Preflight](../../../runs/task_08e2/preflight.json) · [Execution key từng slot](../../../runs/task_08e2/preflight_slots.csv) · [Hash tệp đã bảo toàn](../../../runs/task_08e2/retained_files_sha256.json) · [Dry-run](../../../runs/task_08e2/dry_run.log)

Trạng thái tích hợp và hành vi giữ riêng. AVM vẫn `PARTIAL`, CSM V2 giữ `backend_PASS_with_scenario_failures`, SFM giữ `INTERACTION_FAIL` và ORCA giữ các fixture failure đã ghi. Nghiệm thu 08B và 08C nằm trong PROGRESS, còn kết quả Vòng 0 nằm ở run gốc và báo cáo 08D-QA. [Bàn giao 08B/08C](PROGRESS.md) · [Bằng chứng 08C](../../../runs/task_08c/acceptance-evidence.json) · [Vòng 0 và 08D-QA](TASK_08D_QA_REPORT.md) · [Vòng 1](TASK_08E1_REPORT.md)

## Kết quả nhiệm vụ và an toàn

| Phạm vi | Lượt hợp lệ | Robot tới đích | Đạt nhiệm vụ và an toàn | Contact solver | Đạt nhãn Card |
| --- | ---: | ---: | ---: | ---: | ---: |
| Vòng 0 | 48 | 42 | 41 | 6 | 44 |
| Vòng 1 lũy kế | 540 | 374 | 321 | 166 | 484 |
| Vòng 2, lượt mới | 360 | 249 | 222 | 111 | 331 |
| Vòng 2 lũy kế | 900 | 623 | 543 | 277 | 815 |

SR đo tỷ lệ engine ghi nhận robot tới đích. CR đo tỷ lệ episode có ít nhất một đợt contact solver robot–người hoặc robot–tường. Tỷ lệ đạt an toàn còn yêu cầu hard constraints và khoảng hở hợp lệ. Mỗi ô ghi tử số/mẫu số, tỷ lệ phần trăm và khoảng tin cậy Wilson 95%. Mẫu số thô giữ các lượt trượt nhãn và các thất bại hành vi.

| Backend | SR, % [KTC] | CR, % [KTC] | Đạt nhiệm vụ và an toàn, % [KTC] | Nhãn Card đạt |
| --- | --- | --- | --- | ---: |
| ORCA | 186/225 · 82,7 [77,2–87,1] | 39/225 · 17,3 [12,9–22,8] | 126/225 · 56,0 [49,5–62,3] | 218/225 |
| SFM | 165/225 · 73,3 [67,2–78,7] | 60/225 · 26,7 [21,3–32,8] | 158/225 · 70,2 [63,9–75,8] | 180/225 |
| AVM | 91/225 · 40,4 [34,2–47,0] | 134/225 · 59,6 [53,0–65,8] | 82/225 · 36,4 [30,4–42,9] | 197/225 |
| CSM V2 | 181/225 · 80,4 [74,8–85,1] | 44/225 · 19,6 [14,9–25,2] | 177/225 · 78,7 [72,9–83,5] | 220/225 |
| Chung | 623/900 · 69,2 [66,1–72,2] | 277/900 · 30,8 [27,8–33,9] | 543/900 · 60,3 [57,1–63,5] | 815/900 |

Trong 357 lượt trượt an toàn, 277 lượt dừng do contact robot–người, còn 80 lượt tới đích nhưng có khoảng hở hình học robot–người âm. Nhịp contact solver và phép đo hình học được tổng hợp riêng. [Kết quả từng slot](../../../runs/task_08e2/report/per_episode.csv) · [Theo backend, Card, level, seed, hành động, mật độ và subtype](../../../runs/task_08e2/report/coverage_breakdown.csv) · [Metric và mẫu số](../../benchmarks/METRICS_AND_THRESHOLDS.md)

Có 85 nhãn `false` và 0 nhãn `null`. Trên 815 lượt đạt nhãn, SR là 595/815 (73,0%, KTC 69,9–75,9%), CR là 220/815 (27,0%, KTC 24,1–30,1%) và đạt nhiệm vụ an toàn là 518/815 (63,6%, KTC 60,2–66,8%). Timeout và FP robot đều bằng 0/900. [Tỷ lệ thô và theo nhãn](../../../runs/task_08e2/report/coverage_breakdown.csv)

Ở 480 lượt Critical Advance, đạt nhiệm vụ an toàn theo backend lần lượt là ORCA 73/120 (60,8%, KTC 51,9–69,1%), SFM 98/120 (81,7%, KTC 73,8–87,6%), AVM 24/120 (20,0%, KTC 13,8–28,0%) và CSM V2 100/120 (83,3%, KTC 75,7–88,9%). Theo Card, FA đạt 40/80, ND 53/80, BC 38/80, IN 63/80, OT 80/80 và MG 21/80. Theo seed, số lượt đạt an toàn dao động 13–16 trên 24 slot Critical Advance. Seed 1017 có 13/24. Phạm vi thống kê là 20 seed, với nhiễu spawn và subtype theo đúng thiết kế. [Critical 20 seed cho 24 cặp backend × Card](../../../runs/task_08e2/report/critical_seed_coverage.csv) · [Phân tầng theo seed](../../../runs/task_08e2/report/coverage_breakdown.csv)

## So sánh ghép cặp và bất định

Có 225 điều kiện chung giữa bốn backend, tương ứng 1.350 cặp ứng viên. Tất cả khớp khóa ghép và trạng thái reset. Có 1.151 cặp đủ hai nhãn để tính hiệu ứng nhiệm vụ. Sau khi yêu cầu thêm cả hai lượt đạt hard constraints, còn 532 cặp để phân tích chuyển động, với mẫu số riêng cho mỗi metric. Có 199 cặp bị loại bởi nhãn và thêm 619 cặp trượt điều kiện chuyển động. Số nhãn trượt trái/phải có thể cùng xuất hiện trong một cặp. Hiệu ứng là trái trừ phải. Bootstrap 2.000 lần theo cụm Card/seed với trọng số đều giữa các cụm. [Từng cặp và lý do loại](../../../runs/task_08e2/report/paired_backend_audit.csv)

| Cặp | Cặp nhiệm vụ / 225 | Cặp chuyển động / 225 | Nhãn trượt trái / phải | Chênh lệch đạt an toàn, điểm % [KTC 95%] |
| --- | ---: | ---: | ---: | ---: |
| AVM – CSM V2 | 197 | 82 | 28 / 5 | −46,6 [−54,7 đến −37,9] |
| AVM – ORCA | 196 | 67 | 28 / 7 | −25,9 [−35,5 đến −16,4] |
| AVM – SFM | 180 | 66 | 28 / 45 | −48,2 [−57,2 đến −39,2] |
| CSM V2 – ORCA | 218 | 120 | 5 / 7 | +21,6 [+14,3 đến +29,1] |
| CSM V2 – SFM | 180 | 105 | 5 / 45 | +1,0 [−10,5 đến +12,7] |
| ORCA – SFM | 180 | 92 | 7 / 45 | −15,7 [−28,0 đến −3,2] |

Riêng Critical Advance, CSM V2 cao hơn ORCA 22,5 điểm % về đạt an toàn trên 120 cặp có nhãn (KTC 95% +15,0 đến +30,0). Trong 72 cặp Critical Advance cùng đạt an toàn của hai backend, trung bình đỉnh jerk người của CSM V2 cao hơn 29,33 m/s³ (KTC +10,55 đến +47,93 m/s³). Chênh lệch đạt an toàn giữa CSM V2 và SFM là +2,0 điểm % (KTC −11,0 đến +14,0) trên 100 cặp Critical Advance có nhãn, với 20 cặp ND bị loại. Bảng đầy đủ tách cả hai nhánh và riêng seed mới, có mẫu số, hiệu ứng và KTC cho từng metric. [Kết quả ghép cặp](../../../runs/task_08e2/report/paired_backend_summary.csv)

180 cặp Advance–intervention của Vòng 1 được tính lại từ trace, GT, contact và ba tầng lệnh. Cả 180 cặp khớp common prefix và xác nhận can thiệp. Hai bảng đối chiếu khớp từng byte với 08E-1, giữ 150 cặp nhiệm vụ và 83 cặp chuyển động người. [Log reconciliation](../../../runs/task_08e2/action_regression.log) · [Bảng hành động Vòng 1](../../../runs/task_08e1/report/action_contrasts.csv)

## Kiểm định riêng Card ND

ND Critical dùng cửa 0,90 m, vách dày 0,25 m và hai tuyến danh nghĩa ngược chiều qua cùng khẩu độ. Hình học biên dịch cho dư bề rộng 0,04 m nếu hai đĩa đặt cạnh nhau. Tất cả tuyến danh nghĩa qua kiểm tra hình học trước chạy. Nhãn yêu cầu robot và ít nhất một người đều cắt khẩu độ, đồng thời từng có mặt trong bán kính 3 m quanh tâm cửa. Cả 180 lượt ND có mẫu đồng thời trong vùng 3 m. Tại các tick 50 ms, không lượt nào có tâm robot và tâm người cùng nằm trong vùng khẩu độ dày 0,25 m. Audit lưu riêng thời điểm vào hai vùng và thời điểm qua mặt phẳng cửa nội suy từ trace. Contact solver vẫn giữ mốc 2 ms. [Geometry và tuyến đã duyệt](../../benchmarks/SCENARIO_MATRIX.md#task-08b-research-lock) · [Thời điểm qua cửa, contact, clearance và tiến độ từng lượt](../../../runs/task_08e2/report/nd_doorway_qa.csv)

| Backend | Đạt nhãn / 15 mới | Đạt nhãn / 20 Critical Advance | An toàn / 15 mới | Quan sát từ các lượt mới |
| --- | ---: | ---: | ---: | --- |
| ORCA | 15 | 20 | 10 | Robot qua mặt phẳng cửa ở 5,62 s. Người qua ở 7,95–8,09 s. Năm lượt robot tới đích nhưng có khoảng hở robot–người âm, solver không ghi contact. |
| SFM | 0 | 0 | 15 | Robot qua ở 5,62 s. Tâm người vào vùng khẩu độ ở 10 lượt, nhưng cả 15 lượt không cắt mặt phẳng cửa. Người còn cách đích khoảng 6,45 m lúc robot kết thúc. |
| AVM | 1 | 1 | 1 | Người qua cửa ở 3,10–3,93 s. Trong 14 lượt robot chưa qua, solver ghi contact và dừng lượt. Lượt còn lại cả hai qua, robot đạt nhiệm vụ an toàn. |
| CSM V2 | 15 | 20 | 15 | Người qua ở 3,10–3,46 s, trước robot ở 5,62 s. Cả 15 lượt người có 5,70–6,30 s không cải thiện khoảng cách tốt nhất tới đích và còn cách đích 2,18–2,54 m khi lượt kết thúc. |
| Chung | 31 | 41 | 41 | 29 nhãn `false` và 0 nhãn `null` trong 60 lượt mới. |

ND Hard lịch sử giữ 0/20 đạt nhãn. Hình học 0,60 m có dư 0,10 m cho riêng đĩa lớn nhất và thiếu 0,26 m cho hai đĩa đặt cạnh nhau. Kết quả thô còn nguyên, gồm 5/20 lượt robot đạt nhiệm vụ an toàn. ND Hard thiếu toàn bộ đối chứng theo nhãn. ND Critical Advance thiếu cặp hợp lệ có SFM và chỉ seed 1014 đạt nhãn AVM. Một cụm seed chỉ cho phép đối chiếu mô tả, với KTC không xác định theo protocol. Cặp ORCA–CSM V2 có 20/20 seed đủ nhãn và 12/20 đủ điều kiện đo chuyển động. [Coverage Critical theo backend và seed](../../../runs/task_08e2/report/critical_seed_coverage.csv) · [Bằng chứng Vòng 1](TASK_08E1_REPORT.md)

Ở ND, các quỹ đạo đã quan sát đủ để chấm điều kiện qua cửa. Các nhãn trượt phản ánh việc một hoặc cả hai actor chưa qua trong thời lượng thực tế. Lịch Advance và điều kiện dừng khi robot chạm đích hoặc contact giới hạn thời gian quan sát tiến độ người. Điều này giới hạn suy luận về khả năng cuối cùng của người và mức ảnh hưởng riêng của thiết kế kịch bản. Đề xuất **FOLLOW-UP EXPERIMENT SEPARATE PROTOCOL** cho ND nếu cần phân tách nguyên nhân. Giữ cùng geometry rồi đối chứng thêm độ lệch thời điểm tiếp cận, lịch nhường và cửa sổ quan sát sau khi robot tới đích.

## QA, chuyển động người và số đo thiếu

- Audit đọc lại 194.317 tick điều khiển 50 ms, 97.855 mẫu GT 10 Hz và đồng hồ 4.857.925 substep vật lý 2 ms. Riêng 360 lượt mới có 76.143 tick và 38.351 mẫu GT. Human control giữ nhịp 100 ms. Từng kênh được đối chiếu ID, frame, clock, roster, spawn, goal và giá trị hữu hạn. Sai phân pose khớp executed ở cả robot và người, với sai số lớn nhất ở người là 0 m/s. Bước dịch XY lớn nhất quan sát được trong 50 ms là 0,02735 m ở robot và 0,06616 m ở người. [QA và provenance từng episode](../../../runs/task_08e2/report/episode_audit.csv)
- Khoảng hở người–tường và người–người không âm ở các tick áp dụng. Tâm actor nằm trong miền sàn. Kênh solver ghi 3.413 mẫu robot–người, gộp thành 280 đợt ở 277 lượt. Contact robot–tường là 0. Guard hoạt động 134.561 substep ở 235 lượt. Human–wall và human–human trong kênh solver giữ `null` theo capability. [Audit summary](../../../runs/task_08e2/report/audit_summary.json) · [Dữ liệu gốc theo attempt](../../../runs/task_08d/benchmark_study/attempts/)
- Có 2.320 quỹ đạo trong khoảng 2,70 giờ mô phỏng, 695 encounter theo ngưỡng PSC và 544/1.420 mục tiêu người được quan sát là đạt. Theo episode, có 338 `GOALS_REACHED`, 98 `GOALS_INCOMPLETE`, 107 `NO_PROGRESS` và 357 `SAFETY_FAILURE`. CSM V2 có 53 lượt `NO_PROGRESS` dù robot đạt an toàn, gồm 27 lượt giữ từ Vòng 1 và 26 lượt mới. `NO_PROGRESS` đo thời gian không cải thiện khoảng cách tới đích, nên một người vẫn có thể di chuyển trong khoảng đó. [Hành vi và tiến độ từng actor](../../../runs/task_08e2/report/episode_audit.csv)
- Bảng histogram dùng tập đủ nhãn và hard constraints, có bin, số đo hợp lệ, thiếu và bị loại cho tốc độ, gia tốc, jerk, chiều dài đường, thời gian dừng, clearance, PSC, TTC và thời gian đạt đích. SPL thiếu chiều dài tham chiếu ở cả 900 lượt. TTC hữu hạn ở 842 lượt, còn 58 mang trạng thái `no_predicted_contact`. Khoảng hở người–người không áp dụng ở 740 lượt chỉ có một người. Surveyed/learned realism, legibility, PI, cohesion, crossing distance riêng và dữ liệu cảm biến là `NOT_AVAILABLE`. [Phân phối và histogram](../../../runs/task_08e2/report/motion_distributions.csv) · [Phân phối theo từng điều kiện](../../../runs/task_08e2/report/quality_distributions.csv)
- Audit outlier giữ 571 slot duy nhất, gồm 562 lượt trượt hoặc mục tiêu người chưa hoàn tất và các cực trị. Clearance nhỏ nhất là −0,0800 m ở SFM ND Critical Reverse seed 1001. Đỉnh jerk người lớn nhất là 650,82 m/s³ ở ORCA MG Critical Advance seed 1000, sau một tick guard đưa vận tốc người về 0. Trong tập đạt an toàn, TTC dương thấp nhất là 0,00239 s ở SFM IN Critical Advance seed 1011. TTC là ngoại suy tức thời, giữ riêng với sự kiện contact đã đo. [Các lượt trượt và outlier](../../../runs/task_08e2/report/outlier_audit.csv)

## Lệnh chạy và tái lập

Lệnh campaign dưới đây ghi lại lần chạy từ trạng thái đã có đúng 540 slot của Vòng 1. Mỗi batch tối đa 45 lượt, cùng một manifest và run directory. Audit batch chạy đọc dữ liệu sau khi CLI kết thúc thành công.

```bash
repo=/home/haduckien/nav_ws
source_run="$repo/runs/task_08d"
checkout="$source_run/checkout"
out="$repo/runs/task_08e2"
P="$repo/runs/task_08b/jupedsim_04u/venv/bin/python"
cd "$checkout"
headless() { env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$checkout/src" "$P" -B "$@"; }
audit_python() { env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$checkout/src:$checkout" "$P" -B "$@"; }
headless -m scripts.benchmarks.plan plan --phase 2 --bindings configs/benchmarks/bindings.yaml --output "$out/round_2.json"
headless -m scripts.benchmarks.plan validate --manifest "$out/round_2.json" --runtime
headless -m scripts.benchmarks.plan dry-run --manifest "$out/round_2.json"
audit_python "$out/preflight.py" "$out"
for backend in avm_jupedsim csm_jupedsim orca sfm_jupedsim; do
  for count in 45 90; do
    headless -m scripts.benchmarks.plan run --manifest "$out/round_2.json" --output-dir "$source_run/benchmark_study" --execute --resume --only-new --backend "$backend" --max-episodes 45
    audit_python "$out/audit.py" "$source_run" "$out" batch "$backend" "$count"
  done
done
headless -m scripts.benchmarks.plan report --run-dir "$source_run/benchmark_study" --output-dir "$out/report"
audit_python "$out/audit.py" "$source_run" "$out" final
headless -m pytest -q -rs --basetemp "$out/test_fixtures"
```

Suite trên snapshot dùng native runtime và đạt 251 test, exit 0. Bốn skip là hai ca stock/04S rejection, một ca cần wheel fingerprint 04R stock và một ca cần interpreter stock. Suite bao gồm compiler, metrics/reporting, replay ORCA native, replay MuJoCo của ba model JuPedSim và kiểm contact thật. [Runbook Vòng 2](../../benchmarks/ROUND_2_RUNBOOK.md) · [Pytest](../../../runs/task_08e2/pytest.log) · [Log chạy](../../../runs/task_08e2/run.log) · [QA theo batch](../../../runs/task_08e2/batch_qa.log)

Lần kiểm spawn bổ sung đầu tiên dùng tọa độ YAML làm chuẩn và đánh dấu 40 lượt IN Easy/Hard đã giữ lại. World builder dịch pose theo chiều dài nhánh giao cắt ở hai mức này. Đối chiếu lại với pose/goal sau khi dựng scene xác nhận 40/40 lượt khớp GT và dấu vân tay MJCF. Chỉ thay phép đối chiếu trong audit offline. [Kết quả kiểm ban đầu](../../../runs/task_08e2/audit_initial_spawn_check.json) · [Mã audit](../../../runs/task_08e2/audit.py)

Plan, validate, dry-run, preflight đã hiệu chỉnh, cả tám batch chạy/QA, reporter và audit cuối đều exit 0. Manifest Phase 2 khớp bản duyệt 08B từng byte và có SHA-256 `6bc32a2d7c1d8d2ceac926208b4ad8b3d34c08d9e49b39a2f8c1fe937b01b3f6`. Reporter chạy lại trên cùng raw run vào thư mục tạm và cả 10 tệp chuẩn khớp byte. Từng attempt giữ manifest provenance, config resolve, GT, trace, contact và ba tầng lệnh. [Manifest thực thi](../../../runs/task_08d/benchmark_study/phase_2.json) · [QA tất cả attempt](../../../runs/task_08e2/report/qa.csv) · [Reporting regression](../../../runs/task_08e2/report_regression.log) · [Audit cuối](../../../runs/task_08e2/audit.log)

## Bàn giao 08F

Benchmark đã đủ 900 slot để nghiệm thu giai đoạn Human Models trên snapshot và runtime đã kiểm. Phạm vi suy luận giới hạn ở sáu Card, các policy mở vòng và profile nhiễu đã khóa. Hạn chế còn mở gồm ND thiếu nhãn, lỗi tương tác ở cả bốn backend và số đo realism/cảm biến chưa có. Phạm vi hiện tại là benchmark mô phỏng Human Navigation, với cổng Sensor/Dataset và đánh giá dữ liệu huấn luyện World Model thuộc giai đoạn sau.

Đề xuất **TASK 08F** để khóa benchmark và báo cáo thống kê, đóng gói manifest, snapshot, dependency và bằng chứng tái lập, rồi bàn giao sang Sensor/Dataset. Tệp mới ở cây tài liệu là báo cáo này. Raw run được nối tiếp và toàn bộ công cụ QA, log, bảng số liệu Vòng 2 nằm dưới `runs/`.
