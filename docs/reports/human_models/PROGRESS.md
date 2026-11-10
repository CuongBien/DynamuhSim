# Tiến độ Human Models

Cập nhật ngày 10/11/2026 trên nhánh `mujoco-hummodel`, checkout `b50d0844`.

| Task | Trạng thái | Bằng chứng |
| --- | --- | --- |
| TASK 00 · Runtime và dependency | PASS | [Báo cáo TASK 00](TASK_00_REPORT.md) |
| TASK 01 · Preflight | PASS cho biên hiện có và test hợp đồng. 147 test đạt. | [Báo cáo TASK 01](TASK_01_REPORT.md) |
| TASK 02 · Scripted tích hợp | PASS trên fixture MuJoCo headless | [Báo cáo TASK 02](TASK_02_REPORT.md) |
| TASK 03 · ORCA | PASS trên fixture native RVO2 và MuJoCo, có giữ ca contact và bế tắc | [Báo cáo TASK 03](TASK_03_REPORT.md) |
| TASK 04 · Bridge JuPedSim | Phép chiếu, clock và đối chiếu GT đạt trên fixture CFM v1. Robot động `BLOCKED_CAPABILITY`. | [Báo cáo TASK 04](TASK_04_REPORT.md) |
| TASK 05 · JuPedSim SFM | `BLOCKED_CAPABILITY` theo cổng robot động TASK 04. API SFM 1.4.2 đã xác minh. | [Báo cáo TASK 05](TASK_05_REPORT.md) |
| TASK 04R · Đồng bộ robot động | `BLOCKED_UPSTREAM_API`. Đã chạy probe SFM native với MuJoCo và giữ ca sai lệch, guard và overlap. | [Báo cáo TASK 04R](TASK_04R_REPORT.md) |
| TASK 04S · External actor và rebase native | PASS trên build thử nghiệm `1.4.2+task04s`. Stock wheel vẫn thiếu API. | [Báo cáo TASK 04S](TASK_04S_REPORT.md) |
| TASK 05R · Backend SFM | Implementation và sync PASS. Interaction FAIL ở Critical/Hard. | [Báo cáo TASK 05R](TASK_05R_REPORT.md) |
| TASK 06 · JuPedSim AVM trước 04U | `BLOCKED_CAPABILITY` cho robot động và rebase người. Hai fixture người–người AVM native đã kiểm trên stock và build 04S. | [Báo cáo TASK 06](TASK_06_REPORT.md) |
| TASK 04U · Bridge JuPedSim dùng chung | `INFRASTRUCTURE_PASS` và `MODEL_INTEGRATION_PASS` cho SFM, AVM, CSM V2 trên wheel cô lập 04U. Kết quả kịch bản tách riêng. | [Báo cáo TASK 04U](TASK_04U_REPORT.md) |
| TASK 06 closeout · Hành vi AVM | `PARTIAL`. Độ nhạy vận tốc xác nhận. Giao cắt người–người đối xứng và ca tranh cửa chưa đạt. | [Closeout TASK 06](TASK_06_CLOSEOUT_REPORT.md) |
| TASK 07 · Nghiệm thu backend CSM V2 | `PASS` theo tiêu chí backend native, sync và dữ liệu truy nguyên. Giữ timeout giao cắt đối xứng và FAIL tranh cửa. | [Báo cáo TASK 07](TASK_07_REPORT.md) |
| TASK 08A · Hạ tầng benchmark | `PARTIAL`. Manifest danh nghĩa 48/540/900 và runner, metric, QA đã kiểm bằng fixture nhỏ. Chưa có binding nghiên cứu được duyệt. | [Báo cáo TASK 08A](TASK_08A_REPORT.md) · [Runbook](../../benchmarks/BENCHMARK_REPRODUCIBILITY.md) |
| TASK 08B · Khóa đặc tả nghiên cứu | Sáu Card có binding. Dry-run đạt 48/540/900 slot với bốn backend native. Cổng kiểm định tương tác Vòng 0 vẫn cần lượt chạy chính thức. | [Ma trận đã khóa](../../benchmarks/SCENARIO_MATRIX.md#task-08b-research-lock) · [Runbook Vòng 0](../../benchmarks/ROUND_0_RUNBOOK.md) |
| TASK 08C · AVM và kiểm định bốn backend | Tích hợp và replay MuJoCo đạt cả bốn backend. AVM đã sửa quá đích và còn `PARTIAL` ở giao cắt đối xứng và tranh cửa. CSM V2 giữ nghiệm thu backend và các ca lỗi hành vi. | [Kiểm định TASK 08C](#kiểm-định-task-08c) |

Dùng `.venv/bin/python` cho scripted, thêm native `pyrvo2` để chạy ORCA. Dùng wheel cô lập 04U cho cả ba backend JuPedSim. Bản 04S vẫn tái lập SFM qua legacy API. [Hợp đồng external actor](../../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md)

### Kiểm định TASK 08B

Chạy từ root repo với `P=runs/task_08b/jupedsim_04u/venv/bin/python` trên Python 3.12.3, MuJoCo 4.0.0, RVO2 0.0.0, JuPedSim `1.4.2+task04u` và Shapely 2.1.2. Archive JuPedSim khớp SHA-256 trong recipe 04U. Giữ log và manifest dưới `runs/task_08b/`.

```bash
P=runs/task_08b/jupedsim_04u/venv/bin/python
for phase in 0 1 2; do
  "$P" -B -m scripts.benchmarks.plan plan --phase "$phase" \
    --bindings configs/benchmarks/bindings.yaml --output "runs/task_08b/round_$phase.json"
  env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m scripts.benchmarks.plan dry-run \
    --manifest "runs/task_08b/round_$phase.json"
done
```

- Ba lần `plan` và `dry-run` đều exit 0, không có `BLOCKED_SPEC` hay lỗi chuẩn bị scene. Slot mới lần lượt là 48/492/360.
- Thử kỹ thuật robot-only đạt đích 30/30 lịch. Probe reset và 0,20 s vật lý đạt 120/120 tổ hợp binding/backend, với fingerprint cấu hình, MJCF và asset khớp ở reset. Tái lập hai probe với `env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$PWD" "$P" -B runs/task_08b/robot_probe.py` hoặc `runs/task_08b/native_probe.py` trên checkout này. Các lượt kỹ thuật nằm ngoài mẫu số nghiên cứu.
- Smoke runner ORCA 0,5 s kết thúc `TIMEOUT` hợp lệ và xuất báo cáo theo [lệnh smoke](../../benchmarks/BENCHMARK_REPRODUCIBILITY.md). Lượt kiểm dùng `runs/task_08b/smoke.json`, `smoke_final/` và `smoke_report_final/` trong cùng thư mục.
- Phép chiếu đường ngắm trên tuyến danh nghĩa BC lần đầu mở khi cả hai ở trong 2 m với mọi seed. Với Critical, khoảng cách lớn nhất tới tâm tại mốc mở đường ngắm nằm trong 1,075–1,20 m. Label episode tiếp tục lấy từ quỹ đạo GT đã thực thi.
- `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m pytest -q -rs` đạt 250 test. Bốn skip dành cho cổng wheel stock, 04R và 04S. `.venv/bin/ruff check scripts/benchmarks src/narrow_nav/evaluation src/narrow_nav/simulation/world_geometry.py tests/unit/test_benchmark_08a.py` và `git diff --check` đều đạt.

Vòng 0 là lượt nghiên cứu đầu tiên sau khi khóa binding. Chạy theo cổng checkout sạch và kiểm QA trước khi tái sử dụng 48 slot. Trạng thái hành vi từ TASK 05R, 06 và 07 được giữ trong danh mục backend.

### Kiểm định TASK 08C

#### AVM và CSM V2

- **AVM quá đích.** Ở cửa tuần tự 0,60 m, profile cũ đi vào dung sai 0,10 m ở 5,65 s khi lệnh limited còn khoảng 1,20 m/s, sau đó đi ra và cuối lượt cách đích 0,166 m. Profile `braking_envelope` dùng lại giới hạn vận tốc đến đích để chừa quãng đường giữ lệnh và hãm theo trần gia tốc chung. Khi đã trong dung sai, requested bằng zero. Ba ca cửa tuần tự giữ đích tới cuối lượt, cách đích 0,049 / 0,056 / 0,042 m theo thứ tự khẩu độ 1,20 / 0,90 / 0,60 m. Có thể tái hiện profile cũ bằng `humans.goal_approach=native`. Endpoint dự báo native vẫn được lưu nguyên. [AVM](../../backends/AVM_JUPEDSIM.md) · [Hợp đồng bridge](../../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md)
- **Giao cắt đối xứng.** Với seed 11, AVM native 2D chưa tới đích và khoảng cách tâm nhỏ nhất là 0,499693 m so với tổng bán kính 0,500000 m. MuJoCo lặp tình trạng kẹt ở 100 và 50 ms, kể cả khi nâng trần gia tốc riêng trong probe. Ở profile 100 ms, cuối lượt dự báo khoảng cách tâm 0,499803 m, GT còn 0,500002 m và guard đã chặn 2.743 substep. Seed 47 phá đối xứng trong phép thử, còn 11/23/53/89 không qua. Dời spawn dọc tuyến của một người 0,03 m cho cả hai qua và dừng trong dung sai ở cả năm seed. Đây là đối chứng chẩn đoán. Tăng anticipation lên 1,4 s đạt 5/5 ở 1,2 m/s nhưng chỉ 1/5 ở 0,8 m/s, nên profile nghiên cứu giữ 1,0 s. [Fixture](../../../tests/integration/test_jupedsim_universal.py)
- **Tranh cửa 0,60 m.** Dư đi riêng +0,10 m và dư đi song song −0,26 m. AVM và CSM V2 qua ca tuần tự. Ở ca tranh lối, dời riêng đích robot để kéo dài lượt từ 5,05 lên 10,25 s vẫn không giúp người qua tâm cửa. AVM cuối lượt cách đích 3,433 m và CSM V2 cách 3,206 m. Giữ lỗi giao cắt đối xứng của CSM V2, đồng thời tái xác nhận ca solver contact âm và 45 điểm đo có Δ dự báo bằng zero khi chỉ đổi vận tốc actor tại cùng XY/yaw. [CSM V2](../../backends/CSM_JUPEDSIM_V2.md)

#### Bằng chứng MuJoCo và tái lập

Dùng runtime cô lập 04U đã dựng ở TASK 08B. MuJoCo 4.0.0, CPython 3.12.3, JuPedSim `1.4.2+task04u`, RVO2 0.0.0 và Shapely 2.1.2. Ba tệp phương trình native khớp archive pin từng byte. Hai lần parity native trùng byte. Trace, manifest, fingerprint và log lưu dưới `runs/task_08c/`. Git HEAD `b50d0844`, working tree dirty từ 08B. Tệp nguồn nghiên cứu giữ SHA-256 `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`.

| Phép kiểm | Kết quả |
| --- | --- |
| 34 fixture của evaluator 06/07 | 31 PASS, 3 FAIL hành vi, 0 lỗi hạ tầng. FAIL gồm ORCA Reverse và hai lượt tranh cửa. Analyzer kiểm 12 cặp đối chứng cùng scene, seed, lệnh và prefix. Head-on Advance replay trùng trace, contact, metric và provenance ở 4/4 backend với episode ID mới. |
| 28 lượt chạy đến trạng thái kết thúc, cộng 4 replay trên binding Easy của 08B | Cả 32 lượt hợp lệ hạ tầng. Dùng seed chẩn đoán 11/12, đủ sáu Card và hai subtype OT mỗi backend. Replay FA trùng sáu kênh sau khi bỏ riêng episode ID. Executed của người khớp sai phân GT ở mọi tick. Bốn lỗi an toàn được giữ trong số liệu bên dưới. |
| Dry-run nghiên cứu | 48/48 slot Vòng 0 có binding, bốn backend sẵn sàng và không lỗi chuẩn bị. Các phép thử trên dùng cho kiểm định kỹ thuật. Vòng 0 chính thức vẫn theo runbook checkout sạch. |

| Backend | Robot task pass trong 7 lượt chẩn đoán | Lỗi an toàn giữ lại |
| --- | ---: | --- |
| ORCA | 6 | ND |
| SFM | 6 | ND |
| AVM | 5 | ND, MG |
| CSM V2 | 7 | — |

CSM V2 ở ND có robot đạt đích và người `NO_PROGRESS`. Báo cáo tách trạng thái nhiệm vụ robot, label và hành vi người.

```bash
P=runs/task_08b/jupedsim_04u/venv/bin/python
run_headless() { env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B "$@"; }
run_headless -m experiments.social_navigation.evaluate_avm_csm --output runs/task_08c/avm --backend avm_jupedsim
run_headless -m experiments.social_navigation.evaluate_avm_csm --output runs/task_08c/csm --backend csm_jupedsim
run_headless -m experiments.social_navigation.evaluate_avm_csm --output runs/task_08c/baselines --backend orca --backend sfm_jupedsim
run_headless -m experiments.social_navigation.evaluate_avm_csm --output runs/task_08c/contested --scene door_060_contested
run_headless -m pytest -q -rs -s --basetemp runs/task_08c/final_pairs tests/integration/test_jupedsim_universal.py
run_headless -m experiments.social_navigation.analyze_avm_csm --root runs/task_08c --historical runs/task_08c/final_pairs
run_headless -m scripts.benchmarks.plan plan --phase 0 --bindings configs/benchmarks/bindings.yaml --output runs/task_08c/round_0.json
run_headless -m scripts.benchmarks.plan dry-run --manifest runs/task_08c/round_0.json
run_headless -m pytest -q -rs --basetemp runs/task_08c/full_fixtures
```

Pytest bridge đạt 20 test với hai skip cho stock/04S. Toàn bộ suite đạt 251 test và bốn skip cho stock, 04R và 04S. Ruff trên bốn tệp Python của 08C và `git diff --check` đều đạt.

Khi tái chạy evaluator, dùng thư mục output mới. Probe kỹ thuật trên binding dùng `env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$PWD" "$P" -B runs/task_08c/validate_cards.py <backend>`. Log `root_cause_crossing.log`, `anticipation_grid.log` và `door_duration_final.log` lưu các phép phân tích nguyên nhân. [Runbook Vòng 0](../../benchmarks/ROUND_0_RUNBOOK.md)
