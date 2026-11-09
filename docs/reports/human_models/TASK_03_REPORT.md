# Nghiệm thu TASK 03 · ORCA thuần

**PASS cho baseline ORCA qua RVO2 và MuJoCo trên fixture kỹ thuật.** Backend phát lệnh người từ snapshot thật, qua limiter/mocap và tới GT native. Cùng seed tái lập. Ca tương tác lỗi vẫn nằm trong log. Benchmark 48/540/900 còn `BLOCKED_SPEC`. [Hợp đồng ORCA](../../backends/ORCA_PURE.md) · [Tiến độ](PROGRESS.md)

## Checkout, dependency và nguồn

- Root `/home/thaonhi/nav_ws`, nhánh `mujoco-hummodel`, HEAD trước và sau `09875d9fad40468697a2fe7d99a0fdc282f34780`. Index trống, working tree dirty trước và sau. So tracking `0 0`. Truy vấn GitHub lần đầu thiếu DNS (128), lần đọc lại trả cùng SHA (0).
- Bundle `/tmp/codex_human_model/` vắng mặt trong phiên này. Đã dùng MASTER đầy đủ trong tệp đính kèm và chỉ dẫn TASK 03 trong yêu cầu. Bản `docs/research/HUMAN_MODEL_FINAL.md` có SHA-256 `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`, khớp hash bản gốc đã xác nhận trong TASK 02. Đã đọc AGENTS, README, ROADMAP, architecture/protocols/schemas, ADR, hồ sơ TASK 00–02 và đối chiếu pyproject/lockfile.
- Runtime `/home/thaonhi/nav_ws/.venv/bin/python`, CPython 3.12.3, MuJoCo 4.0.0. Import `rvo2` nạp extension CPython 3.12 x86_64 thật. Metadata `pyrvo2==0.0.0` ghi nguồn Git `c2c46ba8d59556aa10faf03479293236efea154d`, khớp `uv.lock`, gitlink và HEAD checkout vendor. Đọc binding Cython, obstacle API và hiệu chỉnh 0,5 trong solver C++. Tài liệu chính thức của nhóm tác giả xác nhận nguyên tắc tương hỗ. Link PDF `ORCA-1.pdf` trên trang tác giả trả 404, nên đối chiếu bằng mô tả của tác giả và mã nguồn pin thực cài. [ORCA](https://gamma.cs.unc.edu/ORCA/) · [RVO2](https://gamma-web.iacs.umd.edu/RVO2/documentation/2.0/)

## File thay đổi

| File | Vai trò |
| --- | --- |
| `src/narrow_nav/humans/backends/orca.py`, `src/narrow_nav/humans/registry.py`, `configs/humans/orca.yaml` | Backend riêng, thông số có profile rõ, lazy import binding và registry. |
| `src/narrow_nav/humans/{goal,scripted}.py` | Dùng chung phép tính preferred speed chừa quãng phanh. Phép tính cũ của scripted giữ nguyên. |
| `src/narrow_nav/core/protocols.py`, `src/narrow_nav/simulation/lifecycle.py` | Cấp trần tốc độ của profile qua trường cấu hình tùy chọn. API snapshot v1 giữ nguyên. |
| `experiments/social_navigation/evaluate_orca.py`, `tests/unit/test_orca_human.py`, `tests/integration/test_orca_runtime.py`, `tests/integration/test_human_runtime.py` | Evaluator một fixture, native unit và integration. Đổi ID kiểm registry âm sau khi đăng ký ORCA. |
| `README.md`, `docs/backends/ORCA_PURE.md`, `docs/decisions/ADR-0005-orca-external-robot.md`, `docs/schemas/STATE_AND_ACTION_SCHEMA.md`, `docs/research/HUMAN_MODEL_DEVIATIONS.md`, `docs/reports/human_models/{TASK_03_REPORT,PROGRESS}.md` | Hợp đồng, lý do chọn profile, sai khác, bằng chứng và tiến độ. |

## Lệnh và kết quả

Chạy từ gốc repo. `P=.venv/bin/python`. Các lệnh physics bỏ `PYTHONPATH`, `DISPLAY`, `WAYLAND_DISPLAY`. JSON và log trong `runs/task_03_orca/` được Git bỏ qua.

| Lệnh | Exit | Kết quả |
| --- | ---: | --- |
| `env -u DISPLAY -u WAYLAND_DISPLAY "$P" -I -B scripts/diagnostics/check_human_env.py --strict` | 0 | Binding RVO2, phiên bản và native smoke đạt cùng interpreter. |
| `"$P" -I -B -c '…'` với `import rvo2`, `inspect.signature` và `importlib.metadata` | 0 | `addObstacle`, `processObstacles`, `setAgentPosition/Velocity/PrefVelocity/Radius/MaxSpeed`, `getAgentVelocity` và `doStep` hiện diện. Tạo scripted qua registry trong subprocess riêng không nạp binding native. |
| `"$P" -B -m pytest -q tests/unit/test_orca_human.py` | 0 | Sáu test native solver, gồm zero neighbors, head-on lệch, giao cắt 90°, cửa hẹp, tường chặn, deadlock đối xứng và resync. |
| `"$P" -B -m narrow_nav.simulation.smoke --ticks 12 --vx 0.35 humans=orca` | 0 | 300 bước vật lý, người dịch −0,3050 m theo X trong 0,6 s. `smoke.json`. |
| `"$P" -B experiments/social_navigation/evaluate_orca.py --ticks 160 --policy <advance/stop/yield/reverse>` | 0 mỗi lệnh | Bốn episode kỹ thuật trên cùng cấu hình gốc. Dấu vận tốc robot thực thi trong cửa sổ policy được kiểm bằng integration test. |
| `"$P" -B experiments/social_navigation/evaluate_orca.py --ticks 260 worlds.length_m=20 worlds.width_m=6 'scenarios.robot.initial_pose=[-8,-2,0]' 'scenarios.robot.goal=[8,-2]' 'scenarios.humans=[{id:human_a,initial_pose:[-2,0,0],goal:[2,0]},{id:human_b,initial_pose:[2,0,0],goal:[-2,0]}]'` | 0 | Hai người bế tắc đối xứng. `symmetric-deadlock.json`. |
| `"$P" -B experiments/social_navigation/evaluate_orca.py --ticks 180 robots=mobile_robot worlds.width_m=0.75 'scenarios.robot.initial_pose=[2.9,0,0]' 'scenarios.robot.goal=[4.65,0]' 'scenarios.humans.0.initial_pose=[4.2,0,0]' 'scenarios.humans.0.goal=[-2,0]'` | 0 | Kết thúc tương tác `robot_contact` tại tick 71. `constrained-contact.json`. |
| `"$P" -B -m pytest -q -rs` | 0 | 174 test đạt, không báo skip. `pytest-final.log`. |
| `.venv/bin/ruff check .` | 1 ở lượt cuối | Ruff quét cây Blender được giải nén dưới `tmp/` xuất hiện trong lúc thực hiện task và báo lỗi mã upstream. |
| `.venv/bin/ruff check . --extend-exclude tmp --output-format concise` và `.venv/bin/ruff format --check` cho các file Python của TASK 03 | 0 | Lint mã repo và định dạng 10 file thuộc task đạt. |
| `git diff --check`, `git status --short --untracked-files=all` và kiểm link/log bằng `"$P" -I -B -` | 0 | 87 link cục bộ, SHA nguồn, sáu log đo native và kết quả pytest khớp. |

Lượt kiểm ban đầu gặp tên tham số pytest dành riêng `request` (exit 2). Sau khi sửa tên, test phát hiện người vượt goal 0,12 m khi preferred speed chỉ chặn theo `distance/dt`. Đã dùng cận phanh của profile trước solve. Fixture rộng dự kiến tạo contact chỉ cho thấy overlap footprint nhỏ, nên ca contact bắt buộc được kiểm lại với fixture hẹp và xác nhận bằng solver. Các log lượt đầu có tiền tố `focused`; căn cứ nghiệm thu là lượt cuối.

Trong lúc thực hiện task, working tree có thêm thay đổi ngoài phạm vi ở `.gitignore` và `tmp/`. Rule `/docs/decisions` hiện bỏ qua ADR-0005 khi xem file untracked. ADR vẫn có tại đường dẫn yêu cầu trong checkout. TASK 03 giữ nguyên thay đổi ngoài phạm vi và không stage.

## Đo thực tế

Bốn policy chạy từng episode seed 42 với cùng fixture mặc định. Số đo dưới đây là của người và không dùng để xếp hạng policy.

| Policy | Speed mean/max (m/s) | Accel mean/max (m/s²) | Jerk mean/max (m/s³) | Min robot–người (m) | Time-to-goal (s) | Encounters / mẫu contact native |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Advance | 0,747 / 0,800 | 0,203 / 3,00 | 7,79 / 60,00 | 0,000556 | 7,65 | 1 / 0 |
| Stop | 0,747 / 0,800 | 0,205 / 3,00 | 7,88 / 60,00 | 0,000642 | 7,65 | 1 / 0 |
| Yield | 0,747 / 0,800 | 0,197 / 3,00 | 7,57 / 60,00 | 0,284298 | 7,65 | 1 / 0 |
| Reverse | 0,747 / 0,800 | 0,206 / 3,00 | 7,90 / 60,00 | 0,000003 | 7,65 | 1 / 0 |

Đỉnh gia tốc lấy sai phân mỗi tick robot 50 ms trong khi limited người được cập nhật mỗi 100 ms. Số đo này phụ thuộc lịch cập nhật và khác với ngân sách đổi lệnh chia theo period người. Trace giữ cả ba giai đoạn để đối chiếu.

- **Dương tính** · Fixture tuyến chéo 5 m với robot đứng yên đạt dung sai, executed khớp sai phân native và 10 tick cuối đứng yên. Hai episode reset seed 71 có trace người trùng nhau và ID episode khác nhau. Unit solver cho hai người đi qua head-on lệch và giao cắt, giữ khoảng hở theo bán kính.
- **Bế tắc** · Cặp đối xứng giữ min clearance người–người 5,01 mm và stall tương tác 5,4 s mỗi người. Cả hai chưa tới goal khi kết thúc fixture, không có mẫu solver contact. Wall stall riêng trong test chỉ có một người.
- **Robot external** · Khe 0,75 m có 15 mẫu solver contact robot–người dù min footprint cuối tick dương 0,277 mm. Người chưa tới goal. Số dương tại snapshot không thay thế contact trong substep.

## Sai khác và cổng tiếp theo

Profile `reciprocal_external_observed` giữ nửa hiệu chỉnh native cho người và dùng robot external theo vận tốc đo. Chỉ ghi output người. Solver không điều khiển robot và ca contact được giữ nguyên. Fixture kỹ thuật giữ nguyên các ID, ngưỡng và ngân sách nghiên cứu. [Sai khác hm_orca_001](../../research/HUMAN_MODEL_DEVIATIONS.md) · [ADR-0005](../../decisions/ADR-0005-orca-external-robot.md)

TASK 04 kiểm bridge JuPedSim và khả năng đồng bộ theo từng model. Card còn thiếu thông số và các điều kiện benchmark vẫn bị chặn như TASK 01. [Ánh xạ nguồn](../../research/HUMAN_MODEL_SOURCE_MAPPING.md)
