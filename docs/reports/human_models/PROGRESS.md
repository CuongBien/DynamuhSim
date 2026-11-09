# Tiến độ Human Models

Cập nhật ngày 09/11/2026 trên nhánh `mujoco-hummodel`, checkout `16ad6cae`.

| Task | Trạng thái | Bằng chứng |
| --- | --- | --- |
| TASK 00 · Runtime và dependency | PASS | [Báo cáo TASK 00](TASK_00_REPORT.md) |
| TASK 01 · Preflight | PASS cho biên hiện có và test hợp đồng. 147 test đạt. | [Báo cáo TASK 01](TASK_01_REPORT.md) |
| TASK 02 · Scripted tích hợp | PASS trên fixture MuJoCo headless | [Báo cáo TASK 02](TASK_02_REPORT.md) |
| TASK 03 · ORCA | PASS trên fixture native RVO2 và MuJoCo, có giữ ca contact và bế tắc | [Báo cáo TASK 03](TASK_03_REPORT.md) |
| TASK 04 · Bridge JuPedSim | Phép chiếu, clock và đối chiếu GT đạt trên fixture CFM v1. Robot động `BLOCKED_CAPABILITY`. | [Báo cáo TASK 04](TASK_04_REPORT.md) |
| TASK 05 · JuPedSim SFM | `BLOCKED_CAPABILITY` theo cổng robot động TASK 04. API SFM 1.4.2 đã xác minh. | [Báo cáo TASK 05](TASK_05_REPORT.md) |
| Benchmark 48/540/900 | BLOCKED_SPEC ở các điều kiện còn thiếu | [Ánh xạ nguồn](../../research/HUMAN_MODEL_SOURCE_MAPPING.md) · [Sổ sai khác](../../research/HUMAN_MODEL_DEVIATIONS.md) |

Runtime chuẩn là `/home/thaonhi/nav_ws/.venv/bin/python`. Entry point smoke hỗ trợ scripted và ORCA qua cùng limiter và mocap. Cổng tiếp theo cần xác minh cách ghép robot theo pose và vận tốc đã đo trước khi mở SFM, AVM và CSM.
