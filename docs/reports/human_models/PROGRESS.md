# Tiến độ Human Models

Cập nhật ngày 09/11/2026 trên nhánh `mujoco-hummodel`, checkout `7fb4fa23`.

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

Runtime chuẩn là `/home/thaonhi/nav_ws/.venv/bin/python` cho scripted và ORCA. Dùng wheel cô lập 04U cho cả ba backend JuPedSim. Bản 04S vẫn tái lập SFM qua legacy API. [Hợp đồng external actor](../../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md)
