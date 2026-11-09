# Tiến độ Human Models

Cập nhật ngày 09/11/2026 trên nhánh `mujoco-hummodel`, checkout `09875d9f`.

| Task | Trạng thái | Bằng chứng |
| --- | --- | --- |
| TASK 00 · Runtime và dependency | PASS | [Báo cáo TASK 00](TASK_00_REPORT.md) |
| TASK 01 · Preflight | PASS cho biên hiện có và test hợp đồng. 147 test đạt. | [Báo cáo TASK 01](TASK_01_REPORT.md) |
| TASK 02 · Scripted tích hợp | PASS trên fixture MuJoCo headless | [Báo cáo TASK 02](TASK_02_REPORT.md) |
| TASK 03 · ORCA | PASS trên fixture native RVO2 và MuJoCo, có giữ ca contact và bế tắc | [Báo cáo TASK 03](TASK_03_REPORT.md) |
| TASK 04 · Bridge JuPedSim | Kế tiếp | [Biên điều khiển người](../../architecture/HUMAN_2D_3D_BRIDGE_NOTES.md) |
| Benchmark 48/540/900 | BLOCKED_SPEC ở các điều kiện còn thiếu | [Ánh xạ nguồn](../../research/HUMAN_MODEL_SOURCE_MAPPING.md) · [Sổ sai khác](../../research/HUMAN_MODEL_DEVIATIONS.md) |

Runtime chuẩn là `/home/thaonhi/nav_ws/.venv/bin/python`. Entry point smoke hỗ trợ scripted và ORCA qua cùng limiter và mocap. Cổng tiếp theo kiểm bridge JuPedSim với pose thực thi từ MuJoCo.
