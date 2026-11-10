# TASK 08E-1 · Vòng 1 so sánh mô hình người

**`READY_FOR_ROUND_2` trên snapshot và runtime đã xác minh.** Đủ **540/540 slot duy nhất hợp lệ về hạ tầng**, gồm 48 lượt tái sử dụng từ Vòng 0 và 492 lượt mới. Có 321 lượt đạt nhiệm vụ robot và an toàn, 219 lượt trượt an toàn. Cổng này xác nhận dữ liệu đủ điều kiện tiếp tục nghiên cứu. [Kết quả audit](../../../runs/task_08e1/report/audit_summary.json)

## Thiết kế và provenance

Nhánh A gồm 360 slot Advance, với bốn backend, sáu Card, ba level và năm seed 1000–1004. Nhánh B gồm 180 slot Critical trên FA, ND và IN, với cùng bốn backend, ba hành động Stop, Yield/Turn và Reverse, cùng năm seed. Mỗi slot B trỏ tới đúng Advance cùng backend, Card, level, geometry và seed. FA và ND dùng lệnh dạt ngang ở Yield/Turn, IN giảm tốc dọc. [Phase 1 manifest](../../../runs/task_08e1/round_1.json) · [Binding 08B](../../../configs/benchmarks/bindings.yaml) · [Đặc tả và deviation ledger](../../research/HUMAN_MODEL_DEVIATIONS.md)

Vòng 1 tiếp tục run gốc trên checkout sạch `codex/task-08d-snapshot`, commit `0d86e376fff8ec54c6c9dd3b08330e283b2ad042`, code digest SHA-256 `6cb053a26ec7d0bd8c55465532b9fc016c624b36b258f5590caee2aa34efdf7a`. Dùng Python 3.12.3, MuJoCo 4.0.0, RVO2 0.0.0 và JuPedSim `1.4.2+task04u`. Native extension đang import khớp từng byte với entry trong wheel 08B và có SHA-256 `5ebdd4593786734392fefa0c3748f4431d3de53fb857614f6a36a8919b23ec65`. `PYTHONPATH` trỏ vào checkout này để ưu tiên đúng mã đã khóa trước editable install của workspace chính. Dry-run xác minh 540/540 slot, geometry, nhánh lệnh và điều kiện tính nhãn. [Run lock](../../../runs/task_08d/benchmark_study/run.json) · [Precheck](../../../runs/task_08e1/precheck.log) · [Dry-run](../../../runs/task_08e1/dry_run.log) · [Provenance wheel](../../../runs/task_08b/jupedsim_04u/provenance.log) · [Nghiệm thu 04U](TASK_04U_REPORT.md)

Binding 08B duyệt bốn backend cho đối chứng theo cổng tích hợp. AVM vẫn `PARTIAL`, CSM V2 vẫn `backend_PASS_with_scenario_failures`, SFM vẫn `INTERACTION_FAIL` và ORCA giữ các fixture failure đã ghi. Kết quả dưới đây giữ nguyên các lỗi tương tác và đánh giá chất lượng theo từng trục. [Bàn giao 08A–08C](PROGRESS.md) · [Cổng tái sử dụng 08D-QA](TASK_08D_QA_REPORT.md)

## Hạ tầng, GT và vật lý

- Kiểm lại từng `execution_key`, phiên bản backend, seed streams, cấu hình đã resolve, geometry, MJCF và asset. SHA-256 của từng tệp thuộc 48 attempt Vòng 0 khớp bản trước Vòng 1. Run có đúng 540 attempt, 540 episode ID, 0 crash, 0 gián đoạn và 0 retry. [Danh sách hash được bảo toàn](../../../runs/task_08e1/round_0_protected.json) · [Audit theo episode](../../../runs/task_08e1/report/episode_audit.csv)
- Đối chiếu 59.504 mẫu GT 10 Hz, 118.174 tick điều khiển 50 ms và đồng hồ 2.954.350 substep vật lý 2 ms. Chuyển động executed robot và người khớp sai phân pose theo frame. Sai số lớn nhất khi đối chiếu executed người là 0 m/s. Khoảng hở người–tường, người–người và robot–tường không âm ở các mẫu áp dụng của cả 540 lượt. Guard can thiệp 77.846 substep trong 133 lượt. [Audit kênh dữ liệu](../../../runs/task_08e1/audit.py) · [Tổng hợp QA](../../../runs/task_08e1/report/audit_summary.json)
- Có 166 lượt kết thúc vì contact robot–người, tương ứng 2.194 mẫu solver trong 169 đợt. 374 lượt kết thúc ở đích, trong đó 53 lượt vẫn trượt do khoảng hở robot–người âm dù solver không ghi contact. Hai kênh đo giữ riêng. Timeout và sự kiện freezing theo ngưỡng 5 s đều bằng 0. [Hard constraints và số đo từng lượt](../../../runs/task_08e1/report/per_episode.csv)

## Nhiệm vụ robot và độ phủ Card

SR là tỷ lệ engine ghi nhận tới đích. CR là tỷ lệ lượt có ít nhất một đợt contact solver robot–người hoặc robot–tường. Đạt an toàn yêu cầu cả mục tiêu và hard constraints. Các ô ghi số lượt đạt / số lượt có dữ liệu, tỷ lệ và khoảng tin cậy Wilson 95%. Mẫu thô giữ cả lượt trượt nhãn.

| Backend | SR, % [KTC] | CR, % [KTC] | Đạt nhiệm vụ và an toàn, % [KTC] | Nhãn Card đạt |
| --- | --- | --- | --- | ---: |
| ORCA | 111/135 · 82,2 [74,9–87,8] | 24/135 · 17,8 [12,2–25,1] | 70/135 · 51,9 [43,5–60,1] | 128/135 |
| SFM | 90/135 · 66,7 [58,4–74,1] | 45/135 · 33,3 [25,9–41,6] | 85/135 · 63,0 [54,6–70,6] | 105/135 |
| AVM | 67/135 · 49,6 [41,3–58,0] | 68/135 · 50,4 [42,0–58,7] | 64/135 · 47,4 [39,2–55,8] | 121/135 |
| CSM V2 | 106/135 · 78,5 [70,9–84,6] | 29/135 · 21,5 [15,4–29,1] | 102/135 · 75,6 [67,7–82,0] | 130/135 |
| Chung | 374/540 · 69,3 [65,2–73,0] | 166/540 · 30,7 [27,0–34,8] | 321/540 · 59,4 [55,3–63,5] | 484/540 |

Advance riêng có 201/360 lượt đạt an toàn. Theo backend, ORCA 50/90, SFM 65/90, AVM 29/90, CSM V2 57/90. Trong 484 lượt đạt nhãn, SR là 361/484 (74,6%, KTC 70,5–78,3%), CR là 123/484 (25,4%, KTC 21,7–29,5%) và đạt nhiệm vụ an toàn là 311/484 (64,3%, KTC 59,9–68,4%). [Phân tầng backend × Card × level × geometry, thô và theo nhãn](../../../runs/task_08e1/report/coverage_breakdown.csv) · [Từng điều kiện và hành động](../../../runs/task_08e1/report/per_condition.csv)

Ở nhánh A, FA Hard và BC Hard (0,9 m) có contact ở 20/20 lượt mỗi Card. MG Hard (1,6 m) cũng có 0/20 đạt an toàn, gồm 15 contact và 5 lượt tới đích nhưng trượt clearance. OT Easy và Critical đạt 20/20 mỗi level. ND Hard (0,6 m) có 5/20 đạt an toàn nhưng 0/20 đạt nhãn, nên không có so sánh theo nhãn ở tầng này. [Kết quả và độ phủ theo geometry](../../../runs/task_08e1/report/coverage_breakdown.csv)

Cả 56 nhãn `false` đều ở ND, gồm 11 Easy, 25 Critical và 20 Hard. Không có nhãn `null`. Đối chiếu quỹ đạo với mặt phẳng và khẩu độ cửa cho thấy 35 lượt chỉ người qua cửa, 15 lượt chỉ robot qua và sáu lượt cả hai chưa qua. Tất cả có mẫu robot và người đồng thời trong bán kính 3 m quanh cửa. Trong 15 lượt chỉ robot qua, 10 lượt đạt nhiệm vụ an toàn. Các lượt thiếu nhãn vẫn vào thống kê thô. [Định nghĩa nhãn](../../../runs/task_08d/checkout/src/narrow_nav/evaluation/qa.py) · [Các episode ND](../../../runs/task_08e1/report/episode_audit.csv)

## Chuyển động người và chất lượng xã hội

Phân loại người gồm 230 `GOALS_REACHED`, 45 `GOALS_INCOMPLETE`, 46 `NO_PROGRESS` và 219 `SAFETY_FAILURE`. CSM V2 có 27 lượt người `NO_PROGRESS` dù robot đạt nhiệm vụ an toàn. Chấm mục tiêu người dừng ở thời điểm kết thúc episode. [Hành vi và GT từng lượt](../../../runs/task_08e1/report/episode_audit.csv)

Histogram Advance dùng các lượt cùng đạt nhãn và hard constraints, có 50 ORCA, 55 SFM, 29 AVM và 57 CSM V2. Chẳng hạn trung bình PSC theo thứ tự đó là 4,43%, 1,89%, 0,10% và 1,26%. Mẫu TTC hữu hạn tương ứng là 40, 54, 21 và 53. Đây là các phân phối theo mẫu thực tế còn đủ điều kiện của từng backend. So sánh cùng điều kiện dùng bảng cặp bên dưới. CSV phân phối bao gồm cả hai nhánh, ghi riêng số đo hợp lệ, thiếu và bị loại cho velocity, acceleration, jerk, clearance, PSC, TTC, stall time (ST) và path length (PL). [Histogram Advance](../../../runs/task_08e1/report/branch_a_histograms.svg) · [Phân phối theo backend và nhánh](../../../runs/task_08e1/report/motion_distributions.csv) · [Metric và ngưỡng](../../benchmarks/METRICS_AND_THRESHOLDS.md)

SPL thiếu chiều dài tham chiếu ở 540/540 lượt. TTC có giá trị hữu hạn ở 517/540 lượt, còn 23 mang trạng thái `no_predicted_contact`. Solver người–người/người–tường ngoài khả năng quan sát của cấu hình. Run tắt cảm biến. Realism khảo sát/học, legibility, PI, cohesion và chỉ số khoảng cách crossing riêng chưa có số đo theo giao thức. Các số đo thiếu giữ giá trị rỗng trong CSV và `null` trong dữ liệu JSON tương ứng.

Các ca biên được giữ nguyên. SFM ND Critical Reverse seed 1001 có clearance robot–người thấp nhất −0,0800 m và contact. ORCA MG Critical Advance seed 1000 có jerk người đỉnh 650,82 m/s³ ở 5,85 s, ngay sau tick có 20 substep guard và tốc độ người về 0. ORCA IN Hard Advance seed 1003 đạt cổng an toàn nhưng TTC dương nhỏ nhất chỉ 0,00288 s. TTC là ngoại suy vận tốc tức thời, vì vậy ca này vẫn giữ kết quả đo solver và clearance gốc. [Danh sách failure và outlier](../../../runs/task_08e1/report/outlier_audit.csv)

## So sánh cặp giữa backend

Đã đối chiếu 810 cặp ứng viên trên 135 tổ hợp Card, level, seed, geometry và action. Cả 810 khớp khóa ghép và reset. Hai nhãn phải đạt để tính chênh lệch nhiệm vụ. Phân tích chuyển động yêu cầu thêm cả hai đạt hard constraints. Cột nhãn là số lần trượt của từng vế, có thể cùng xuất hiện trong một cặp. Chênh lệch dưới đây tính trái trừ phải theo điểm phần trăm, lấy trung bình đều theo cụm Card/seed và bootstrap 2.000 lần.

| Cặp backend | Cặp nhiệm vụ | Cặp chuyển động | Nhãn trượt trái / phải | Loại thêm vì hard fail | Chênh lệch đạt an toàn, điểm % [KTC 95%] |
| --- | ---: | ---: | ---: | ---: | ---: |
| AVM – CSM V2 | 121/135 | 64/135 | 14 / 5 | 57 | −21,4 [−28,1; −14,7] |
| AVM – ORCA | 120/135 | 49/135 | 14 / 7 | 71 | +5,0 [−10,0; +21,1] |
| AVM – SFM | 105/135 | 49/135 | 14 / 30 | 56 | −28,7 [−38,0; −19,3] |
| CSM V2 – ORCA | 128/135 | 65/135 | 5 / 7 | 63 | +22,9 [+10,7; +36,2] |
| CSM V2 – SFM | 105/135 | 62/135 | 5 / 30 | 43 | −4,0 [−16,0; +8,7] |
| ORCA – SFM | 105/135 | 61/135 | 7 / 30 | 44 | −14,7 [−24,0; −6,0] |

Tổng cộng 684 cặp đủ nhãn và 350 cặp đủ điều kiện đo chuyển động. Ví dụ riêng Advance, ORCA có tốc độ trung bình người thấp hơn SFM 0,077 m/s trên 41 cặp, KTC 95% từ −0,095 đến −0,059 m/s. Bảng CSV ghi từng vế, lý do loại, mẫu số theo metric và kết quả tách A/B. [Audit 810 cặp](../../../runs/task_08e1/report/paired_backend_audit.csv) · [Ước lượng và KTC theo metric](../../../runs/task_08e1/report/paired_backend_summary.csv)

## Tương phản hành động robot

Cả 180 cặp có cùng điều kiện đầu và prefix khớp tới mốc đổi lệnh ở 4,0 s cho FA, 2,5 s cho ND và 2,0 s cho IN. Đối chiếu trace, GT, contact và ba kênh lệnh trước can thiệp. Từ mốc can thiệp, cả 180 cặp đều đổi requested, limited và executed so với Advance. Stop quan sát tốc độ executed dưới 0,02 m/s ở 60/60 cặp. Reverse quan sát `vx` executed xuống khoảng −0,210 đến −0,208 m/s ở cả 60 cặp. Yield/Turn có độ lệch đỉnh vận tốc executed so với Advance 0,322–0,358 m/s. [Audit hành động và prefix](../../../runs/task_08e1/report/action_contrasts.csv)

| So với Advance | Prefix và can thiệp | Cặp nhiệm vụ | Cặp chuyển động | Trung vị khoảng cách giữa quỹ đạo người trong cửa sổ chung |
| --- | ---: | ---: | ---: | ---: |
| Stop | 60/60 | 50/60 | 26/60 | 0,002 m |
| Yield/Turn | 60/60 | 50/60 | 31/60 | 0,030 m |
| Reverse | 60/60 | 50/60 | 26/60 | 0,002 m |

Khoảng cách trong bảng lấy trung bình theo mẫu đồng thời của cùng người ở mỗi cặp, rồi lấy trung vị giữa các cặp đủ điều kiện. Ví dụ CSM V2 tại FA có độ lệch trung bình quỹ đạo người 0,407 m với Yield/Turn (5/5 cặp, KTC 95% 0,357–0,461 m) và tốc độ người tăng 0,372 m/s trong cửa sổ chung (KTC 0,282–0,464 m/s). Cửa sổ chung quan sát được dài 1,65–3,00 s. Với mỗi hành động có 10 cặp thiếu nhãn Advance, năm cặp trong số đó cũng thiếu nhãn ở vế can thiệp. Thêm 24 cặp Stop, 19 Yield/Turn và 24 Reverse trượt điều kiện chuyển động vì ít nhất một vế trượt hard constraints. Các tầng thiếu cặp giữ số mẫu 0 và giá trị hiệu ứng rỗng, gồm toàn bộ tương phản ND của AVM và SFM. [Hiệu ứng và KTC theo backend × Card × hành động](../../../runs/task_08e1/report/action_contrast_summary.csv)

## Tái lập và tệp kết quả

```bash
repo=/home/haduckien/nav_ws
source_run="$repo/runs/task_08d"
checkout="$source_run/checkout"
out="$repo/runs/task_08e1"
P="$repo/runs/task_08b/jupedsim_04u/venv/bin/python"
cd "$checkout"
headless() { env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$checkout/src" "$P" -B "$@"; }
headless -m scripts.benchmarks.plan plan --phase 1 --bindings configs/benchmarks/bindings.yaml --output "$out/round_1.json"
headless -m scripts.benchmarks.plan dry-run --manifest "$out/round_1.json"
headless -m scripts.benchmarks.plan run --manifest "$out/round_1.json" --output-dir "$source_run/benchmark_study" --execute --resume --only-new
headless -m scripts.benchmarks.plan report --run-dir "$source_run/benchmark_study" --output-dir "$out/report"
env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$checkout/src:$checkout" "$P" -B "$out/audit.py" "$source_run" "$out"
MPLCONFIGDIR=/tmp/narrownav_matplotlib /usr/bin/python3 -B "$out/plot_histograms.py" "$out/report"
headless -m pytest -q -rs --basetemp "$out/test_fixtures" tests/unit/test_benchmark_08a.py tests/unit/test_module_dependencies.py tests/integration/test_simulation_contract.py tests/simulation/test_collision_monitor.py
headless -m pytest -q -rs --basetemp "$out/native_test_fixtures" tests/integration/test_jupedsim_universal.py
```

Nhóm kiểm thử benchmark, dependency, simulation và contact đạt 18/18. Nhóm bridge native đạt 20 test, skip hai ca kiểm tra từ chối trên stock/04S. Các CSV kiểm toán và histogram là output offline. Artifact gốc vẫn theo cấu trúc run chuẩn, gồm config đã resolve, manifest, GT, contact và ba tầng lệnh. Runner dùng lịch lệnh robot vòng hở theo tick. Audit đối chiếu toàn bộ requested với lịch, còn GT cấp cho đo lường, ghi dữ liệu và tính executed. [Log chạy](../../../runs/task_08e1/run.log) · [Log test](../../../runs/task_08e1/pytest.log) · [Log test native](../../../runs/task_08e1/pytest_native.log) · [Manifest đã thực thi](../../../runs/task_08d/benchmark_study/phase_1.json) · [Các attempt gốc](../../../runs/task_08d/benchmark_study/attempts/) · [Giao thức QA](../../benchmarks/DATA_QA_PROTOCOL.md)
