# Nghiệm thu TASK 01 · Preflight hợp đồng, đồng hồ và hình học

**PASS cho preflight** · Các cổng đã triển khai đạt kiểm định trên MuJoCo 4.0.0 thật và test hợp đồng Python. **BLOCKED_SPEC cho kế hoạch benchmark** · Easy của ND/MG, lựa chọn mức OT, ba mức MG và nhánh Reverse còn cần làm rõ. Task này dùng profile người đứng yên và fixture kỹ thuật. [Hợp đồng runtime](../../protocols/HUMAN_MODELS_RUNTIME_CONTRACT.md) · [Timing và hình học](../../protocols/HUMAN_MODELS_TIMING_GEOMETRY.md)

## Checkout và phạm vi audit

- Repo `/home/thaonhi/nav_ws`, nhánh `mujoco-hummodel`. HEAD trước và sau `09875d9fad40468697a2fe7d99a0fdc282f34780`. Index ban đầu và cuối trống. Working tree dirty từ đầu với đầu ra TASK 00 và bản đặc tả untracked. So upstream trả `0 0`, lần tra SHA GitHub thành công trả cùng SHA. Lần đầu trong sandbox thiếu DNS, exit 128.
- Bản gốc trong bundle và bản checkout `docs/research/HUMAN_MODEL_FINAL.md` có cùng SHA-256 `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`. Đã đọc MASTER, prompt đính kèm, TASK 01, AGENTS, README, ROADMAP, ADR hiện có, toàn bộ tài liệu architecture/protocols/schemas ban đầu và đối chiếu lock từ TASK 00.
- Runtime kiểm định là `/home/thaonhi/nav_ws/.venv/bin/python`, CPython 3.12.3, Ubuntu 24.04.5 x86_64, MuJoCo 4.0.0. Chẩn đoán TASK 00 xác nhận JuPedSim 1.4.2 và RVO2 0.0.0 cùng interpreter; các binding này chỉ được gọi ở smoke dependency.

| Mã đã đọc | Hiện trạng xác minh |
| --- | --- |
| `core/{actions,states,protocols,types,metadata,seed}.py`, `humans/registry.py` | Cổng reset/propose, kiểu frame, tuple ID sắp xếp, schema v1 và seed stream đã có. Registry rỗng. Bổ sung phép chiếu GT dành cho người và validation roster để bên tích hợp gọi đúng biên. |
| `simulation/{lifecycle,timing,collision_monitor,state_extractor}.py` | Lệnh người zero, adapter advance theo nhịp robot, executed từ hai snapshot, observation sensor missing. Đồng hồ người/observation là mốc lịch. Runtime giữ trạng thái cuối hợp lệ khi lỗi. |
| `simulation/{configuration,world_builder,world_geometry,world_assets}.py` | Compose validate pose đầu/cuối và overlap ban đầu; builder dựng native hull và giữ asset Quaternius. Các check endpoint chưa kiểm kết nối đường đi. |
| `simulation/mujoco/{adapter,geometry}.py`, `sim/mujoco/models/humans/pedestrian.xml` và default geom | Adapter chỉ đặt tốc độ bánh. Người là mocap với mask chỉ ghép robot. Contact native và khoảng hở footprint riêng. Hình học chiếu chỉ hỗ trợ hộp song song trục và trụ đứng. |

## File thay đổi

| File | Phạm vi |
| --- | --- |
| `src/narrow_nav/core/{actions,geometry,protocols,states}.py` | Kiểm batch người, phép đổi vận tốc robot sang world và cổng tạo `HumanContext`. |
| `src/narrow_nav/simulation/{configuration,human_motion,timing,world_geometry}.py` | Validation giới hạn người, limiter/yaw thuần, lịch và modulo bước nội bộ, sàng bề rộng. |
| `src/narrow_nav/simulation/{lifecycle,mujoco/adapter}.py` | Đối chiếu stamp/roster, từ chối lệnh người không thực thi được, phát hiện NaN/Inf và biến đổi bất thường của root mocap đứng yên. |
| `tests/unit/{test_human_preflight,test_module_dependencies}.py` · `tests/integration/test_human_preflight_contract.py` · `tests/simulation/test_human_preflight_geometry.py` | Test hợp đồng và fault injection, native clock/sensor/contact mask, ca cổ chai. |
| `docs/protocols/HUMAN_MODELS_{RUNTIME_CONTRACT,TIMING_GEOMETRY}.md` · `docs/schemas/{STATE_AND_ACTION_SCHEMA,EPISODE_SCHEMA}.md` · `docs/decisions/ADR-0003-human-model-preflight.md` | Hợp đồng và ADR. Schema tiếp tục v1, không có trường mới cần migration. |
| `docs/research/HUMAN_MODEL_{SOURCE_MAPPING,DEVIATIONS}.md` · `docs/reports/human_models/{TASK_01_REPORT,PROGRESS}.md` | Ánh xạ Card, cổng, blocker và tiến độ hiện tại. |

## Kiểm định thực chạy

Các lệnh dưới đây chạy từ gốc repo với `P=.venv/bin/python` và `A=runs/task_01_preflight`. Log lưu trong `A`, được Git bỏ qua.

| Lệnh | Exit | Kết quả |
| --- | ---: | --- |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m pytest -q tests/simulation/test_engine_lifecycle.py tests/simulation/test_human_model.py tests/simulation/test_simulation_timing.py` | 0 | 9 test hồi quy sớm đạt, gồm Quaternius và reset. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m pytest -q -rs tests/unit/test_human_preflight.py tests/unit/test_module_dependencies.py tests/integration/test_human_preflight_contract.py tests/simulation/test_human_preflight_geometry.py tests/simulation/test_collision_monitor.py tests/simulation/test_simulation_timing.py` | 0 | 29 test preflight và hồi quy đạt. `preflight-pytest.log`. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m pytest -q -rs` | 0 | 147 test đạt, không báo skip. `pytest.log`. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m narrow_nav.simulation.smoke --ticks 2 --vx 0.35 worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot` | 0 | 2 tick, 50 bước MuJoCo, 0,1 s. `runtime-smoke.json`. |
| `env -u DISPLAY -u WAYLAND_DISPLAY "$P" -I -B scripts/diagnostics/check_human_env.py --strict --json` | 0 | 5/5 smoke dependency. `dependencies.json`. |
| `"$P" -I -B -m pip --disable-pip-version-check check` | 0 | Không có dependency lỗi. |
| `"$P" -I -B -` kiểm link Markdown, ID Card và log JSON | 0 | 87 link/mốc cục bộ, sáu ID nguồn và bằng chứng timing/dependency khớp. `docs-evidence-check.log`. |
| `.venv/bin/ruff check .`, `git diff --check` và `git status --short --untracked-files=all` | 0 | Lint và diff đạt. Đã đọc tracked diff và tệp mới thuộc task. |

Test dùng MuJoCo thật cho sensor missing, contact robot–tường/người, kiểm mask người–tường/người–người và bảo vệ Quaternius. Ca lỗi tiêm snapshot sai tại biên trả kết quả của adapter, hoặc tiêm trạng thái native NaN/Inf, dịch ngang, nâng Z và xuyên tường vào root mocap đứng yên. Lifecycle từ chối tick lỗi, giữ prefix và phục hồi bằng reset. Fixture backend ở unit chỉ kiểm cổng/seed/roster, bao gồm no-op cho actor đến goal. Nó không đại diện cho backend đang chạy trong MuJoCo.

### Đồng hồ và bề rộng

Đo thêm bằng `"$P" -B -` trong môi trường headless. Lệnh Python compose `experiments.duration_ns=5000000000`, gọi `MuJoCoEngine.reset(seed=17)` hai lần, mỗi lần 100 lệnh `stop()`, rồi đối chiếu native time và stamp. Hai episode ID khác nhau cho cùng số đo. Log `timing-feasibility.json` lưu cả sai số và sàng bề rộng theo profile.

| Tick | `time_ns` | Bước vật lý | Sai số tuyệt đối native (s) |
| ---: | ---: | ---: | ---: |
| 1 | 50.000.000 | 25 | 2,78×10⁻¹⁷ |
| 10 | 500.000.000 | 250 | 3,33×10⁻¹⁶ |
| 100 | 5.000.000.000 | 2.500 | 3,29×10⁻¹³ |

Stamp nguyên khớp ở mọi tick. Bước vật lý vẫn 2 ms. Test chọn mốc GT mỗi hai tick robot, chưa ghi episode 10 Hz. Sàng bề rộng và fixture chứng minh ca cửa 0,60 m chặn hai actor đi song song với cả hai profile, và chặn một đĩa fixture bán kính 0,32 m dù hai đầu hợp lệ. Khe 0,90 m chặn vi sai và người đi cạnh nhau theo bán kính đã chọn, còn dư 0,04 m cho mecanum và người. Số dư là điều kiện cần, chưa phải bằng chứng reachability của Card. [Bảng sàng đầy đủ](../../protocols/HUMAN_MODELS_TIMING_GEOMETRY.md#sàng-lọc-physical_feasibility)

## Sai khác và cổng tiếp theo

- [Sổ sai khác](../../research/HUMAN_MODEL_DEVIATIONS.md) giữ nguồn nguyên văn, nêu xung đột Easy, Reverse, lũy kế và độ phủ nhóm, cùng provenance cho các ngưỡng thăm dò. Các mục `BLOCKED_SPEC` chặn sinh kế hoạch đầy đủ. Khoảng 48/540/900 chỉ được kiểm về công thức phân bổ trong task này.
- TASK 02 nối một backend scripted, gọi reset/propose thật, giữ lệnh khác zero qua tick trung gian, kiểm lỗi callback giữ prefix, tiến root mocap và đo geometric clearance người–tường/người–người ở substep. Kiểm tra moving-human penetration, dừng ở goal và xuất log 10 Hz còn cần bằng chứng ở cổng tương ứng. Sensor dựa trên perception vẫn chưa khả dụng.
