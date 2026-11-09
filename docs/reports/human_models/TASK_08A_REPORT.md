# TASK 08A · Hạ tầng benchmark Human Navigation

**PARTIAL.** Đã kiểm compiler, runner, metric và báo cáo trên fixture nhỏ. Ba kế hoạch nghiên cứu vẫn `BLOCKED_SPEC`. Chưa có điều kiện được duyệt để chạy hoặc kết quả nghiên cứu để tổng hợp.

## Nghiệm thu

| Tiêu chí | Quyết định và bằng chứng |
| --- | --- |
| Đối chiếu TASK 00–07, kiến trúc, protocol, schema và tiến độ | Đạt kiểm tra tài liệu. 04/05 là blocker lịch sử trên stock. 04U mở cổng tích hợp cho ba model JuPedSim trên wheel riêng. 05R giữ `INTERACTION_FAIL`. AVM giữ `PARTIAL`. CSM V2 giữ `backend PASS` và các ca FAIL. [Tiến độ](PROGRESS.md) [04U](TASK_04U_REPORT.md) [05R](TASK_05R_REPORT.md) [AVM closeout](TASK_06_CLOSEOUT_REPORT.md) [CSM V2](TASK_07_REPORT.md) |
| Sáu Card, geometry, mật độ, policy, action và nhãn | Có catalog và phép gán nhãn từ GT. Điều kiện executable vẫn thiếu binding khoa học. Neutral hoặc independent là các profile native hiện có thể xét duyệt. [Ma trận](../../benchmarks/SCENARIO_MATRIX.md) |
| Manifest lặp lại được, không trùng slot | Đạt trên 48/540/900 slot danh nghĩa. Shared seed từ 1000. Mỗi phase có JSON và CSV. Hash plan chung giữ danh tính lũy kế. |
| Reuse Phase 0 | Có 48 key trùng về cấu trúc trong A của Phase 1. Số episode có thể reuse hiện là 0. ND 2,1 m của quy tắc Easy chung khác ND 1,2 m của Card. Runtime chỉ reuse attempt cùng binding, config/MJCF, seed, backend và provenance, có kết quả hạ tầng hợp lệ. |
| CLI, isolation, retry, provenance và capability | Một CLI với các subcommand chung cho ba vòng. Dry-run 04U nhận diện đủ bốn native backend. Stock wheel bị từ chối khi chọn AVM. Thử thực thi slot chưa có binding trả exit 2 và chưa tạo output. Smoke giữ một attempt sau resume. |
| GT, metrics và reporting | Các test kiểm công thức, null, số lần contact, timeout, FP, CI và QA attempt. Smoke thật kiểm dấu thời gian, ba stage và displacement. Hạ tầng, hành vi người, task robot và chất lượng có trường riêng. |
| Hồi quy AVM/CSM V2 | Sáu test native/MuJoCo 04U chạy lại. Fixture lưu bằng chứng tranh cửa 0,60 m, giao cắt đối xứng và độ nhạy vận tốc. Không nâng trạng thái AVM hoặc đổi phương trình native. |
| Bảo vệ dữ liệu và giới hạn chạy | Hai smoke ORCA kỹ thuật 0,5 s, không episode nghiên cứu và không chiến dịch 48/540/900. Nguồn nghiên cứu nguyên hash. Raw log nằm dưới `runs/`. |

## Kế hoạch và blocker

| Vòng | Danh nghĩa lũy kế | Slot mới danh nghĩa | Binding duyệt | Episode nghiên cứu đã chạy | Manifest |
| --- | ---: | ---: | ---: | ---: | --- |
| 0 | 48 | 48 | 0 | 0 | [JSON](../../../runs/task_08a/round_0.json) · [CSV](../../../runs/task_08a/round_0.csv) |
| 1 | 540 | 492 | 0 | 0 | [JSON](../../../runs/task_08a/round_1.json) · [CSV](../../../runs/task_08a/round_1.csv) |
| 2 | 900 | 360 | 0 | 0 | [JSON](../../../runs/task_08a/round_2.json) · [CSV](../../../runs/task_08a/round_2.csv) |

Vòng 1 có 360 A và 180 B. Vòng 2 dành 15 seed bổ sung cho từng cặp Card/backend Critical của A theo công thức phase. Các điều kiện còn chặn gồm ND Easy, Reverse của ND/IN, ba mức MG, chiều rộng và phân bổ hai subtype OT, đối chứng Advance của B, và scene, tuyến, lịch robot, policy được duyệt cho từng Card. Các map và fixture hiện hữu cung cấp bằng chứng kỹ thuật rời rạc. Chúng chưa đủ để khóa sáu Card. [Deviation ledger](../../research/HUMAN_MODEL_DEVIATIONS.md) [Runbook](../../benchmarks/BENCHMARK_REPRODUCIBILITY.md)

## Bằng chứng thực thi

Các lệnh chạy từ root repo. `P=runs/task_04u/reproduction/venv/bin/python`. Các lần chạy native/MuJoCo xóa `PYTHONPATH`, `DISPLAY` và `WAYLAND_DISPLAY`.

| Lệnh | Kết quả |
| --- | --- |
| `$P -B -m pytest -q -rs tests/unit/test_benchmark_08a.py` | 10 passed. [Log](../../../runs/task_08a/pytest-benchmark.log) |
| `$P -B -m pytest -q -rs tests/integration/test_jupedsim_universal.py -k '(native_external or advance_stop_and_replay or (human_pair and crossing)) and (avm_jupedsim or csm_jupedsim)'` | 6 passed, 15 ngoài selection. Bao gồm replay, độ nhạy, crossing và requested/limited/executed/GT. [Log](../../../runs/task_08a/pytest-regressions.log) |
| `$P -B -m scripts.benchmarks.plan plan --phase N --output runs/task_08a/round_N.json` | Vòng 0, 1, 2 đều exit 0, phát JSON/CSV. |
| `$P -B -m scripts.benchmarks.plan dry-run --manifest runs/task_08a/round_N.json` | Exit 2 cho cả ba vòng. Tất cả slot bị chặn spec. [0](../../../runs/task_08a/dry_0.log) · [1](../../../runs/task_08a/dry_1.log) · [2](../../../runs/task_08a/dry_2.log) |
| `.venv/bin/python -B -m scripts.benchmarks.plan dry-run --manifest runs/task_08a/round_0.json --backend avm_jupedsim` | Exit 2. Stock thiếu coupling APIs 04U. [Log](../../../runs/task_08a/dry-stock.log) |
| `$P -B -m scripts.benchmarks.plan run --manifest runs/task_08a/round_0.json --backend orca --output-dir runs/task_08a/blocked --execute --max-episodes 1` | Exit 2 trước khi tạo output. [Log](../../../runs/task_08a/run-blocked.log) |
| `$P -B -m scripts.benchmarks.plan run --manifest runs/task_08a/smoke-plan.json --output-dir runs/task_08a/smoke_final --episode benchmark_smoke --execute --max-episodes 1` | Exit 0, kết quả episode `TIMEOUT`. Lặp đúng lệnh với `--resume` giữ một attempt. [Log](../../../runs/task_08a/smoke-final.log) · [QA dấu thời gian và motion](../../../runs/task_08a/smoke-invariants.log) |
| `$P -B -m scripts.benchmarks.plan report --run-dir runs/task_08a/smoke_final --output-dir runs/task_08a/report` | Exit 0. [Báo cáo JSON](../../../runs/task_08a/report/report.json) · [CSV từng episode](../../../runs/task_08a/report/per_episode.csv) |
| `.venv/bin/ruff check scripts/benchmarks src/narrow_nav/evaluation tests/unit/test_benchmark_08a.py` và `git diff --check` | Đạt trong phạm vi task. |

Smoke ORCA đo 11 mẫu trace kể cả reset, sáu GT 10 Hz, 10 batch mỗi stage và 0 contact solver. Sai số tối đa của vận tốc executed người đối chiếu dịch chuyển MuJoCo là 0 m/s. Quãng đường robot 0,13878 m, khoảng hở robot–người nhỏ nhất 5,22040 m và TTC nhỏ nhất 5,03118 s. Hard outcome `S=false, C=0, TO=true, FP=0`. Smoke không có denominator nghiên cứu, cảm biến được tắt và không sinh frame tổng hợp. Khoảng hở human–human không áp dụng cho fixture một người.

Preflight kỹ thuật FA dùng `_prepare` với binding tạm. Cả bốn backend có cùng khóa đối chứng, khẩu độ compiled 2,1 m, dư footprint lớn nhất 1,6 m và dư song song 1,24 m. Preflight xác minh tuyến và kiểm soát chung mà không step physics hoặc duyệt Card nghiên cứu. [Log](../../../runs/task_08a/preflight-fixture.log)

Hồi quy giữ AVM crossing 5,25 s không tiến triển, 2.743 substep guard, dự báo cuối cách 0,499803 m và GT 0,500002 m. Ca tranh cửa 0,60 m của CSM V2 có dư đi riêng +0,10 m, dư đi song song −0,26 m và `human_goal_not_reached`. Số liệu tranh cửa là fixture bằng chứng từ TASK 07, không phải lượt đo mới trong 08A. Hai kết quả không bị chuyển thành success. [Fixture dữ liệu](../../../tests/fixtures/benchmark/behavior_regressions.json) [Đối chiếu AVM](TASK_06_CLOSEOUT_REPORT.md) [Đối chiếu CSM](TASK_07_REPORT.md)

## Provenance và phạm vi

- Nhánh `mujoco-hummodel`, Git `7fb4fa232bbf52a98d8ab79279a32a4ae971afbe`, local và upstream tracking 0/0 lúc kiểm tra. Working tree đã dirty từ các task trước và có hai công cụ ORCA ngoài phạm vi. Mỗi attempt ghi commit, dirty flag và hash mã. Không stage hoặc commit.
- CPython 3.12.3, MuJoCo 4.0.0, NumPy 2.5.4, Shapely 2.2.0. RVO2 `1+pyrvo2.0.0.0`, SHA-256 binary `187edf0ab860020fa36834658b5ac1be5c4e9a45b9a6593412aa02b84501db48`.
- JuPedSim `1.4.2+task04u`, upstream commit `80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe`, wheel SHA-256 `f3cf132b3987c368448c11913e1d11bb73e335032553e6222a043c0720437e3c`, native binary SHA-256 `789bc8576238c4243e8ce393fcad882499ca03621434eb1ff0ef5313620e0e03`. [Recipe build](TASK_04U_REPORT.md#l68) [Provenance thực tế](../../../runs/task_04u/reproduction/provenance.log)
- SHA-256 của `HUMAN_MODEL_FINAL.md` trước và sau `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`. Mã task nằm trong `configs/benchmarks/`, `scripts/benchmarks/plan.py`, `src/narrow_nav/evaluation/`, một tệp test đơn vị và fixture evidence. Tài liệu trong `docs/benchmarks/`, báo cáo này, `PROGRESS.md` và deviation ledger.

Các chỉ số khảo sát, độ dễ đọc hành vi, learned realism, PI, độ kết dính nhóm và khoảng cách crossing riêng còn thiếu định nghĩa hoặc dữ liệu. Báo cáo giữ null với lý do. Ngưỡng stall 0,02 m/s, epsilon 1 µm, quy tắc chờ chủ ý và vùng encounter là quy ước đo kỹ thuật được ghi riêng. Geometry có thể cho một actor đi qua dù hai actor không thể đi cạnh nhau. Kết quả âm về đi song song được giữ cho phân tích tranh đường. [Metrics](../../benchmarks/METRICS_AND_THRESHOLDS.md) [QA](../../benchmarks/DATA_QA_PROTOCOL.md)
