# TASK 04 · Bridge JuPedSim và MuJoCo

**Trạng thái tổng thể · BLOCKED_CAPABILITY cho tương tác robot động.** Phép chiếu từ MJCF, nhịp dự báo và đối chiếu GT đã qua kiểm định kỹ thuật với JuPedSim CFM v1 và MuJoCo thật. JuPedSim 1.4.2 không cung cấp cổng đồng bộ pose/vận tốc robot ngoài mô phỏng. Probe lỗ di động có hiệu ứng hình học nhưng gặp trường hợp API từ chối. Cổng tích hợp SFM, AVM và CSM V2 vẫn đóng. [Kiến trúc](../../architecture/JUPEDSIM_MUJOCO_2D_3D_BRIDGE.md) · [API 1.4.2](../../backends/JUPEDSIM_API_COMPATIBILITY.md)

## Phạm vi và provenance

- Nhánh `mujoco-hummodel`, HEAD trước và sau `16ad6caef80d75fdb3881a73e721eb0dcb3382e3`. So với upstream local, 0 commit ahead và 0 behind. Tree và index sạch lúc bắt đầu. Sau chạy, tree dirty với các file TASK 04, index vẫn sạch. ADR-0006 có trên đĩa, rule `/docs/decisions` hiện bỏ qua file mới này. Không stage hoặc commit.
- `/tmp/codex_human_model/00_MASTER_RULES.md` vắng mặt. Dùng đầy đủ MASTER từ attachment TASK 04. Nguồn [HUMAN_MODEL_FINAL](../../research/HUMAN_MODEL_FINAL.md) có SHA-256 `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030` ở HEAD và working tree. Đây cũng là hash đã đối chiếu ở TASK 00. TASK 01 và 02 có báo cáo PASS cho các cổng làm điều kiện.
- Đã đối chiếu AGENTS, README, ROADMAP, toàn bộ `docs/architecture`, `docs/protocols`, `docs/schemas`, ADR-0001, pyproject, pin liên quan trong `uv.lock`, mã runtime, các test hình học, clock, dependency và các báo cáo TASK 00–03. Đối chiếu tài liệu JuPedSim stable với mã 1.4.2 và thử trực tiếp binding cài trong .venv.
- Python `/home/thaonhi/nav_ws/.venv/bin/python` CPython 3.12.3. JuPedSim 1.4.2, Shapely 2.2.0, MuJoCo 4.0.0. Dùng các dependency đã khóa. `git submodule status` trả 128 do checkout thiếu mapping Python-RVO2 trong `.gitmodules`, còn hai gitlink vẫn khớp index ban đầu.

## File thay đổi

| Nhóm | File |
| --- | --- |
| Hợp đồng hình học | `src/narrow_nav/core/planar_world.py` và `docs/schemas/STATE_AND_ACTION_SCHEMA.md` |
| Xuất hình học | `src/narrow_nav/simulation/geometry_inspector.py`, `world_geometry.py`, `mujoco/geometry.py` |
| Bridge chung | `src/narrow_nav/humans/jupedsim_bridge/{geometry,clock,model_factory,robot_proxy,state_sync}.py` |
| Tests | `tests/unit/test_jupedsim_bridge.py`, `tests/simulation/test_jupedsim_projection.py`, `tests/integration/test_jupedsim_bridge_runtime.py` |
| Kiến trúc và API | `docs/architecture/{HUMAN_2D_3D_BRIDGE_NOTES,JUPEDSIM_MUJOCO_2D_3D_BRIDGE}.md`, `docs/backends/JUPEDSIM_API_COMPATIBILITY.md`, `docs/decisions/ADR-0006-jupedsim-shadow-bridge.md` |
| Nghiệm thu | `docs/research/HUMAN_MODEL_DEVIATIONS.md`, `docs/reports/human_models/{PROGRESS,TASK_04_REPORT}.md` |

## Kiểm định

Lệnh dưới chạy từ root repo. Log lưu trong `runs/task_04_jupedsim/`.

| Lệnh | Exit | Kết quả |
| --- | --- | --- |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -B -m pytest -q -s tests/unit/test_jupedsim_bridge.py tests/simulation/test_jupedsim_projection.py tests/integration/test_jupedsim_bridge_runtime.py` | 0 | 17 passed. Log `bridge-final.log`. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -B -m pytest -q` | 0 | 191 passed, gồm regression rotate mặc định và các cổng trước. Log `pytest-final.log`. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -B -m narrow_nav.simulation.smoke --ticks 12 --vx 0.35 humans=orca` | 0 | 300 bước vật lý, người dịch −0,305 m theo X. Log `orca-smoke.log`. |
| `env -u PYTHONPATH PIP_NO_CACHE_DIR=1 .venv/bin/python -I -m pip check` | 0 | No broken requirements found. Log `pip-check-clean.log`. |
| `.venv/bin/ruff check . --output-format concise` và `.venv/bin/ruff format --check` trên 12 file Python TASK 04 | 0 | Lint và format đạt. |
| `git diff --check`, `git status --short --branch` | 0 | Diff sạch khoảng trắng, đúng nhánh và đúng phạm vi. |

Các vòng đầu giữ log lỗi trước khi sửa fixture và logic chọn component. Vòng full đầu `pytest.log` có 190 passed, 1 failed vì bật chấp nhận tường xoay mặc định. Đã chuyển sang phép chiếu opt-in và giữ kiểm tra từ chối xoay của runtime. `pip check` với shell ban đầu thấy package ROS qua `PYTHONPATH` và trả 1. Chạy lại với interpreter cô lập trả 0.

## Bằng chứng đo

| Scene MuJoCo đã resolve | Diện tích toàn miền đi được (m²) | Component | Lỗ trong component native | Cận xấp xỉ tĩnh (m) |
| --- | ---: | ---: | ---: | ---: |
| Corridor | 20,000000 | 1 | 0 | 0 |
| Blind corner | 23,720000 | 1 | 0 | 0 |
| Arena obstacle | 320,193882 | 2 | 4 | 0,000482399 |
| Hospital medium | 888,451030 | 1 | 13 | 0,000289439 |
| School floor | 6352,627808 | 2 | 28 | 0,000192959 |

Mọi tuyến robot và người trong năm cấu hình trên đạt kiểm spawn/goal và liên thông theo bán kính. Native JuPedSim nhận các polygon component và giữ đúng số lỗ. Fixture cửa xoay khớp **3.524/3.524** mẫu tia MuJoCo, giữ bề rộng 0,80 m và từ chối tuyến khi bán kính đạt 0,40 m. Fixture giao cắt giữ cả hai nhánh sàn và kiểm được hai tuyến vuông góc. Bản đồ riêng cho Card ND/IN và warehouse còn thiếu. Hình học fixture không đi vào Card nghiên cứu.

| Cổng | Bằng chứng |
| --- | --- |
| Tường và robot tĩnh | Hai probe CFM v1 đi vòng và vào dung sai đích. Khoảng hở footprint nhỏ nhất lần lượt 0,047475 m và 0,075477 m. |
| Đồng hồ và GT | Hai lượt 10 s cùng seed 11 cho pose và motion trace giống hệt. Mỗi lượt có 200 tick robot/quan sát, 100 callback người, 1.000 bước JPS và 5.000 bước MuJoCo. Hai lần đọc ở mốc 5 s giữ count = 500. Biên 10 s chỉ đo, không tiến thêm. |
| XY và validity | Cuối fixture MuJoCo tại `(1.419816, -0.462883)` m, dự báo JPS tại `(1.494744, -0.459605)` m. Sai lệch tối đa 0,075000 m, dưới cổng kỹ thuật 0,25 m. Đã thấy limiter đổi lệnh ở tick đầu, giữ lệnh qua tick xen giữa, không overlap, teleport, contact hay guard block trong fixture. Inject lệch 1 m bị từ chối trước bước native tiếp theo. Fixture dừng ở ngân sách 10 s. |
| Robot động | Footprint lấy mẫu từ robot MuJoCo dịch 0,247358 m tính đến lần lấy mẫu cuối. Tốc độ Y world khi đó 0,341204 m/s. Sai số footprint tối đa đo ở lưới 50 ms là 0,034422 m, so với cận 0,001 m cho probe đứng yên. Quỹ đạo người native đổi 0,509246 m so với bản không có lỗ. Robot replay cùng pose khi tắt probe. |
| Giới hạn API | Thử ghi pose/orientation agent nhận AttributeError. `switch_geometry` từ chối trường hợp phủ agent hoặc waypoint, không tăng iteration count. Dynamics robot ngoài và rebase pose người vẫn `BLOCKED_CAPABILITY`. Log `api-exploration.log` và `bridge-final.log`. |

`bridge-final.log` lưu `geometry_hash`, `projection_error`, `robot_proxy_error`, `state_sync_error`, `internal_dt` và `unsupported_capabilities`, kèm các cặp XY theo giây và endpoint 10 s. Số đo áp dụng cho fixture kỹ thuật. Tần số GT nghiên cứu 10 Hz vẫn là lớp xuất riêng.

## Sai khác và cổng kế tiếp

- `hm_jps_001` ghi rõ blocker API và phạm vi probe. Đặc tả nghiên cứu, ngưỡng khoa học, ngân sách 48/540/900 và tài nguyên Quaternius giữ theo nguồn. Benchmark và ba model nghiên cứu chưa chạy trong TASK 04. [Sổ sai khác](../../research/HUMAN_MODEL_DEVIATIONS.md)
- Trước khi mở backend nghiên cứu, cần chứng minh một cơ chế ghép robot external theo pose/vận tốc MuJoCo và xử lý chồng lấn bằng API được hỗ trợ, hoặc duyệt riêng một profile thí nghiệm chấp nhận proxy cùng sai số đo được. Trước benchmark còn các blocker `BLOCKED_SPEC` đã ghi.
