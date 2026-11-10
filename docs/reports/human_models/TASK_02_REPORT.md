# Nghiệm thu TASK 02 · Scripted, limiter và MuJoCo mocap

**PASS cho tích hợp scripted trên fixture headless.** Một người đi tuyến XY dài 5 m, dừng trong dung sai goal và tái lập khi reset cùng seed. Contact robot–tường và robot–người được đọc từ MuJoCo. Guard và clearance của người có bằng chứng riêng. Benchmark nghiên cứu còn `BLOCKED_SPEC` theo ledger. [Biên 2D–3D](../../architecture/HUMAN_2D_3D_BRIDGE_NOTES.md) · [Tiến độ](PROGRESS.md)

## Checkout và phạm vi

- Repo `/home/thaonhi/nav_ws`, nhánh `mujoco-hummodel`, HEAD trước và sau `09875d9fad40468697a2fe7d99a0fdc282f34780`. Upstream tracking `0 0`. Truy vấn GitHub lần đầu thiếu DNS, exit 128. Lần đọc lại trả cùng SHA, exit 0. Index trống, working tree dirty trước và sau vì TASK 00–01 và TASK 02. Không stage hoặc commit.
- Bản gốc bundle và `docs/research/HUMAN_MODEL_FINAL.md` có SHA-256 `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`. Đã đọc MASTER, prompt đính kèm, TASK 02, AGENTS, README, ROADMAP, architecture, protocols, schemas, các ADR và hồ sơ TASK 00–01. Đối chiếu `pyproject.toml`, `uv.lock`, YAML scene, core ports, lifecycle, adapter, geometry, collision, skin và test hiện có.
- Kiểm định với `/home/thaonhi/nav_ws/.venv/bin/python`, CPython 3.12.3 trên Ubuntu 24.04.5 x86_64, MuJoCo 4.0.0, backend `scripted` version `1`. Các phép thử tích hợp sử dụng MuJoCo thật. Fixture tiêm lỗi callback dùng để kiểm nhánh lỗi trước khi tiến physics.

## File thay đổi

| File hoặc nhóm | Thay đổi TASK 02 |
| --- | --- |
| `src/narrow_nav/humans/{registry,scripted}.py`, `configs/humans/scripted.yaml` | Registry tạo một backend. Scripted theo goal, giảm tốc gần đích, reset trạng thái và giữ roster. YAML công bố dung sai goal người 0,10 m. |
| `src/narrow_nav/core/protocols.py` | Cấu hình typed cho backend, provenance interface, clearance và telemetry guard tùy chọn cho schema v1. |
| `src/narrow_nav/simulation/{backend,configuration,lifecycle,mujoco_engine,smoke}.py` | Inject và kiểm backend, seed stream, callback theo lịch, giữ lệnh, giới hạn người, validation và telemetry headless. |
| `src/narrow_nav/simulation/{collision_monitor,world_geometry}.py` · `src/narrow_nav/simulation/mujoco/{adapter,geometry,human_visuals}.py` | Dịch chuyển root theo substep, kiểm footprint, signed clearance, contact solver và animation theo executed. |
| `tests/unit/{test_scripted_human,test_core_actions}.py` · `tests/integration/test_human_runtime.py` · `tests/simulation/test_human_contacts.py` | Test backend thuần, tương thích mặc định v1, tích hợp native, fault injection, solver contact thoáng qua, guard và Quaternius. |
| `README.md` · `docs/architecture/{01_MODULE_BOUNDARIES,02_DATA_FLOW,04_MUJOCO_MODELING,05_MUJOCO_RUNTIME,HUMAN_2D_3D_BRIDGE_NOTES}.md` · `docs/protocols/HUMAN_MODELS_RUNTIME_CONTRACT.md` | Luồng lắp ghép, runtime active/stationary và giới hạn biên 2D–3D. |
| `docs/schemas/{EPISODE_SCHEMA,STATE_AND_ACTION_SCHEMA}.md` · `docs/decisions/ADR-0004-scripted-mocap-runtime.md` · `docs/research/HUMAN_MODEL_DEVIATIONS.md` · `docs/reports/human_models/{TASK_02_REPORT,PROGRESS}.md` | Schema v1 mở rộng có mặc định, ADR và hồ sơ nghiệm thu. |

## Lệnh đã chạy

Chạy từ gốc repo. `P=.venv/bin/python`. Các log nằm trong `runs/task_02_scripted/` và được Git bỏ qua.

| Lệnh | Exit | Kết quả |
| --- | ---: | --- |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m pytest -q -rs` | 0 | 161 test đạt, không báo skip. Lượt cuối gồm test native, schema và test hồi quy. Xem `pytest-final.log`. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m pytest -q -rs tests/unit/test_scripted_human.py tests/unit/test_core_actions.py tests/integration/test_human_runtime.py tests/simulation/test_human_contacts.py` | 0 | 14 test ở lượt đầu trước khi bổ sung test walk skin và contact thoáng qua. Xem `focused.log`. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m narrow_nav.simulation.smoke --ticks 160 worlds.width_m=3 'scenarios.robot.initial_pose=[-4,-1.1,0]' 'scenarios.robot.goal=[4,-1.1]' 'scenarios.humans.0.initial_pose=[-2.4,-0.7,0]' 'scenarios.humans.0.goal=[2.4,0.7]'` | 0 | 4.000 bước vật lý. Người dịch `(4,776160; 1,393047)` m. Sai số tới goal khoảng 0,02483 m, dưới 0,10 m. Không có substep bị chặn. `diagonal-smoke.json`. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m narrow_nav.simulation.smoke --ticks 40 --vx 0.35 worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot` | 0 | 1.000 bước vật lý, robot tiến 0,65465 m và người dịch −1,425 m theo X. `mecanum-smoke.json`. |
| `env -u DISPLAY -u WAYLAND_DISPLAY PYTHONPATH="$PWD/humans" "$P" -B -m human_visual.demo --check-only --duration 2 --agents 2 --seed 42 --assets-dir "$PWD/sim/mujoco/assets/humans"` | 0 | Demo Quaternius gốc hoàn tất 61 frame mô phỏng qua đủ năm pha, 14 skin. Dùng bundle đã đóng gói trong repo vì `humans/assets/generated` vắng mặt. `quaternius-demo.log`. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -` chạy hai fixture guard từ test, trích xuất speed và clearance khi lần đầu bị chặn | 0 | `geometry-evidence.json` ghi số đo ba giai đoạn và số contact solver. |
| `"$P" -I -B -` kiểm link Markdown và log JSON, `.venv/bin/ruff format --check` trên file Python thuộc TASK 02 | 0 | 113 link cục bộ hợp lệ, bằng chứng native khớp và 17 file đúng định dạng. |
| `.venv/bin/ruff check .`, `git diff --check`, `git status --short --untracked-files=all`, `cmp` bản nguồn và bản bundle | 0 | Kiểm lint, whitespace, working tree và nội dung nguồn. |

## Bằng chứng chấp nhận

- **Lịch và lệnh.** Test 50 ms tạo limited đầu 0,075 m/s và cập nhật 0,15 m/s ở tick tiếp theo. Test 100 ms giữ 0,15 m/s qua hai tick robot đầu. Roster hai người giữ thứ tự và timestamp. Lệnh zero giữ pose/yaw. Test 5 m xác nhận executed bằng sai phân native, đứng yên trong 10 tick cuối và hai trace cùng seed khớp.
- **Vật lý và contact.** Hai ca solver robot–tường và robot–người có timestamp trên lưới 2 ms. Ca lướt qua robot ở X = 0,4995 m giữ các event trong tick dù cuối tick không còn cặp contact. Trong ca góc khuất, tick 30 có tốc độ requested 0,80, limited 0,45 và executed 0 m/s. Guard chặn 25 substep, khoảng hở người–tường còn 0,669 mm và có 0 contact solver. Hai người đối đầu bị chặn ở tick 24, khoảng hở 0,400 mm và có 0 contact solver.
- **Lỗi và hình ảnh.** Callback tạo NaN ở đầu tick lỗi làm episode `failed`, giữ hai tick đã công bố và traceback. Tiêm dịch chuyển root trái lệnh cũng làm tick thất bại và giữ prefix. Trace vật lý với Quaternius và proxy khớp trong fixture active. Quaternius chọn walk từ vận tốc executed, còn hai kiểu visual dùng cùng hull.

## Quyết định và cổng tiếp theo

- `MuJoCoEngine()` giữ profile stationary của caller hiện có. Entry point smoke và caller inject chạy profile kinematic. Goal 0,10 m thuộc scripted kỹ thuật.
- Root mocap không nhận đáp ứng lực từ solver. Guard bảo thủ giữ đồng thời các root ở substep bị chặn và có thể làm actor khác dừng theo. Khi phân tích tương tác, dùng telemetry guard cùng clearance. Người–tường và người–người ghi `not_observable_solver`. [ADR-0004](../../decisions/ADR-0004-scripted-mocap-runtime.md) · [Sai khác hm_runtime_001](../../research/HUMAN_MODEL_DEVIATIONS.md)
- TASK 03 kiểm ORCA qua cùng hợp đồng. Các blocker Card và plan từ TASK 01 vẫn áp dụng. Writer/reader GT 10 Hz, cảm biến và đối chứng nghiên cứu cần cổng riêng. [Ánh xạ nguồn](../../research/HUMAN_MODEL_SOURCE_MAPPING.md)
