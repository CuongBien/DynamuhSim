# TASK 04R và 05R · Đồng bộ robot động JuPedSim và MuJoCo

**04R · BLOCKED_UPSTREAM_API. 05R · Chờ cổng 04R.** Probe dùng đúng native `SocialForceModel`, `SocialForceModelAgentParameters` và `SocialForceModelState` của wheel 1.4.2 với runtime MuJoCo. Dựng lại simulation đưa đúng pose và vận tốc đo vào boundary nhưng proxy robot sau đó tự tích phân như một agent SFM. Các phép đo thuộc profile `sfm_reconstruction_probe`. [Probe có thể chạy lại](../../../experiments/social_navigation/probe_jupedsim_coupling.py) · [Đề xuất API upstream](../../decisions/ADR-0007-jupedsim-external-actor.md)

## Checkout, môi trường và phạm vi

- Nhánh `mujoco-hummodel`, HEAD trước và sau `7fb4fa232bbf52a98d8ab79279a32a4ae971afbe`, upstream local 0/0. Hai file `orca_progress_tools/orca_sweep.py` và `orca_progress_tools/view_orca_live_debug.py` đã dirty khi bắt đầu. Cả hai được giữ nguyên. Index sạch.
- `/home/thaonhi/nav_ws/.venv/bin/python` là CPython 3.12.3. JuPedSim 1.4.2 build `80d0b3b1`, GNU 14.2.1, ABI cp312 x86-64. MuJoCo 4.0.0, Shapely 2.2.0 và `pyrvo2` 0.0.0 import `rvo2`. Đã đọc build info và các đường dẫn extension thật. [Preflight](../../../runs/task_04r/preflight.log)
- Đã đối chiếu AGENTS, toàn bộ nguồn nghiên cứu, deviation ledger, các báo cáo TASK 02–05 và tiến độ, API docs và kiến trúc/schema được chỉ định, mã bridge, lifecycle, adapter, các test liên quan, `pyproject.toml` và `uv.lock`. Hash nguồn làm việc và HEAD cùng là `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`.
- Thêm script probe và `tests/integration/test_sfm_coupling_probe.py`. Cập nhật bridge docs, API compatibility, SFM docs, TASK 05, tiến độ và ledger. Thêm ADR-0007 và thu hẹp rule ignore để ADR mới hiện trong Git. Các ADR cũ đã ignore vẫn được giữ nguyên. Không stage hoặc commit.

## Các cơ chế API đã thử

| Cơ chế | Kết quả native 1.4.2 |
| --- | --- |
| Python wrapper và binding Agent | `position` và `orientation` đều chỉ đọc ở cả hai tầng. Gán `agent.velocity = (9, 9)` chỉ tạo thuộc tính Python và state native đọc lại `(0, 0)`. |
| `mark_agent_for_removal`, rồi `add_agent` | Sau khi mark vẫn lookup được ID cũ. Chèn lại tại cùng XY lỗi `too close`. Sau đúng một bước, lookup ID cũ lỗi `unknown Agent` và chèn lại cấp ID mới. Trong phép thử này `removed_agents()` trả rỗng sau bước, vì vậy bằng chứng xóa dựa trên lookup và khả năng chèn lại. |
| Direct Steering và state model | Setter vận tốc SFM đọc lại đúng `(0.2, -0.1)`. Proxy đứng với target ở chính pose và desired speed bằng 0 vẫn bị lực SFM dịch chuyển. Proxy có vận tốc đầu 0,35 m/s trở thành 0,498057 m/s sau 10 bước của ca thử và lệch 0,009567 m so với đường vận tốc hằng. |
| Chèn khi overlap | Với proxy bán kính 0,18 m và người bán kính 0,25 m, chèn ở khoảng cách tâm 0 và 0,10 m bị từ chối. Chèn ở 0,35 m được chấp nhận dù hai footprint còn chồng lấn. |
| Dựng lại native từ trạng thái đo | Một người đi thẳng với một waypoint khớp từng mốc của simulation liên tục trong 100 bước, sai số XY và velocity đều 0. Mỗi reconstruction vẫn có ID mới. Với robot, trạng thái chèn khớp sample đầu nhưng proxy tự tích phân trong cả cửa sổ. |
| `switch_geometry` | Lỗ tĩnh ngoài agent và waypoint được nhận. Lỗ phủ agent hoặc waypoint bị từ chối riêng biệt. Dời footprint tới vùng trống được chấp nhận. Tổng số bước vẫn 0 qua cả ba phép cập nhật hình học. |

Nguồn SFM 1.4.2 dùng vận tốc tương đối ở số hạng ma sát tiếp tuyến khi khoảng cách tâm nhỏ hơn tổng bán kính. Giữ nguyên XY ở khoảng cách 0,60 m, đổi vận tốc proxy Y từ 0 lên 0,40 m/s cho cùng vận tốc người sau một bước. Ở khoảng cách 0,35 m, cùng phép đổi làm vận tốc Y người tăng 0,96 m/s. Đổi vận tốc hướng tâm trong ca overlap cho kết quả người như khi robot đứng. Đây là đáp ứng ma sát của model trong overlap, tách khỏi clearance và contact MuJoCo. [Nguồn SFM v1.4.2, AgentForce và ForceBetweenPoints](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/SocialForceModel.cpp#L133-L173) · [Thứ tự iterate và removal](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/Simulation.cpp#L42-L57)

## Phép đo SFM và MuJoCo

Cùng seed 11, một cấu hình đối đầu và một đoạn tiến chung. Tách lệnh robot tại **0,70 s**. Các lượt 3 s dùng 60 tick robot, 30 callback người, 300 bước native và 1.500 bước MuJoCo. Mỗi lần dựng lại native có count 0 → 10. Đối chiếu requested với dịch chuyển dự báo và executed với sai phân GT từng cửa sổ. Limited giữ qua hai tick. Lệnh robot đi riêng qua adapter. JSON lưu cấu hình, hash geometry, pose/velocity robot, ba motion stage, contact theo substep và guard.

| Nhánh | Vận tốc robot world cuối `(vx, vy)` m/s | Clearance người–robot nhỏ nhất (m) | Lệch robot proxy so với GT lớn nhất (m) |
| --- | --- | ---: | ---: |
| Advance | `(0,355864, 0)` | 0,143963 | 0,022064 |
| Stop | `(−0,001284, 0)` | 0,182938 | 0,012645 |
| Yield | `(0,012288, −0,211231)` | 0,320599 | 0,011669 |
| Reverse | `(−0,263404, 0)` | 0,224116 | 0,011669 |

- Độ chênh lớn nhất của vector requested người giữa advance và yield là **0,583532 m/s**. Kết quả gồm hiệu ứng từ quỹ đạo robot khác nhau và sai lệch proxy. Thử cùng XY ở trên phân biệt giới hạn đáp ứng vận tốc trước overlap.
- Đi thẳng tách làn đạt 60 tick. Giao cắt với robot quay π/2 cho vận tốc cuối world `vy = 0,355707 m/s` dù lệnh là `vx = 0,35 m/s` trong robot_base. Sai lệch lớn nhất của robot proxy và người so với MuJoCo trong ca giao cắt là 0,045121 m và 0,021264 m. Ca waypoint ghi 11 callback robot che waypoint mà giữ nguyên goal. Cuối lượt waypoint đã thông lại.
- Trong ca hẹp, guard giữ người ở **175** substep, sai lệch XY dự báo–executed của người đạt **0,072417 m**. Proxy robot lệch tới **0,119054 m** và sai lệch vector vận tốc cuối cửa sổ tới **1,882976 m/s**. Callback ở 1,70 s nhận clearance footprint **−0,032335 m**, trả lỗi `observed_robot_human_footprint_overlap` trước bước SFM tiếp theo, giữ prefix 34 tick và 170 bước SFM. Prefix có 0 solver contact. Đối chứng robot tiến với người MuJoCo đứng yên ghi **654** mẫu substep native `robot_human` và clearance nhỏ nhất −0,061486 m. Đối chứng này tách riêng khỏi SFM. Mask người–người và người–tường vẫn `not_observable_solver`.
- Hai lượt seed 11 có trace bằng nhau, episode ID riêng. Seed 12 có seed stream metadata khác và trace vẫn bằng nhau do probe không rút số ngẫu nhiên. Một lượt kỹ thuật 3 s dùng khoảng 0,75–1,35 s wall time trong lần ghi log này, gồm compile, reset, dựng lại SFM và ghi trace trong bộ nhớ.
- Bản đồ rộng có hash `6f5ad2d7d18e…`, bản hẹp `7bf89c178b8e…`. Log giữ hash đầy đủ. Hai phép chiếu box có cận xấp xỉ footprint tĩnh 0 m.

## Quyết định

| Phương án | Model và trạng thái robot | Overlap và re-sync | Liên tục, replay, chi phí | Quyết định |
| --- | --- | --- | --- | --- |
| Lỗ footprint | Giữ dynamics người native. Lỗ đặt theo XY đo, không có vận tốc robot. | Từ chối lỗ phủ agent hoặc waypoint. Không rebase người. | Giữ simulation và ID. Cần đổi geometry/routing khi robot di chuyển. Ca tái nhập vùng trống chạy ở cùng iteration. Dùng wheel stock và Shapely đã pin. | Loại cho tương tác động. |
| Dựng lại với agent robot | Giữ phương trình SFM. XY và velocity đúng lúc chèn. Proxy tiếp tục tự điều chỉnh dưới lực và trôi khỏi robot thật. | Chặn overlap quan sát được và giữ prefix. Tạo lại người từ pose/velocity thực thi, mất ID/stage state native. | Một instance mỗi 100 ms. Replay tái lập trên fixture. Tương đương đã kiểm chỉ cho một người thẳng, một waypoint. Dùng dependency hiện có. | Giữ chẩn đoán độc lập. |
| External actor và batch rebase trong upstream | Giữ phương trình SFM. Thiết kế actor chỉ vào neighborhood, có XY và velocity đo tại boundary, cùng chính sách dự báo ghi riêng. | Validate batch trước lực native. Giữ ID và stage người. Có mã lỗi overlap, waypoint và re-entry cần nghiệm thu. | Một simulation, API C/Python mới, ABI và replay phải kiểm trên wheel đóng gói riêng. Chưa build, chưa đo. | Đề xuất trong ADR-0007. Chưa mở gate. |

## Bảng nghiệm thu và lệnh

Mọi lệnh chạy từ root repo. Các lệnh Python mô phỏng dùng prefix `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -B`. Log nằm dưới `runs/task_04r/`.

| Cổng | Lệnh sau prefix hoặc lệnh đầy đủ | Exit | Kết quả và log |
| --- | --- | ---: | --- |
| Preflight | `.venv/bin/python -B -` với probe inline, và `env -u PYTHONPATH PIP_NO_CACHE_DIR=1 .venv/bin/python -I -m pip check` | 0 | `preflight.log`. No broken requirements found trong interpreter cô lập. |
| Public API, robot động, overlap, re-sync | `-m experiments.social_navigation.probe_jupedsim_coupling --require-gate` | 2 | Blocker được tái hiện và trả exit 2 chủ ý. `native-probes.json`, `native-probes.stderr`, `native-probes.exit`. |
| Chạy một nhánh | `-m experiments.social_navigation.probe_jupedsim_coupling --fixture head_on --policy yield --seed 11` | 0 | Chẩn đoán hoàn thành và JSON giữ `BLOCKED_UPSTREAM_API`. `head-on-yield.json`. |
| 17 test TASK 04 | `-m pytest -q tests/unit/test_jupedsim_bridge.py tests/simulation/test_jupedsim_projection.py tests/integration/test_jupedsim_bridge_runtime.py` | 0 | 17 passed. `pytest-task04.log`. |
| 17 test TASK 04 và 8 regression SFM mới | Lệnh trên thêm `tests/integration/test_sfm_coupling_probe.py` | 0 | 25 passed. `pytest-targeted.log`. |
| Full suite | `-m pytest -q` | 0 | 199 passed. `pytest-full.log`. |
| Lint/format trong phạm vi | `.venv/bin/ruff check src tests experiments/social_navigation --output-format concise` và `.venv/bin/ruff format --check experiments/social_navigation/probe_jupedsim_coupling.py tests/integration/test_sfm_coupling_probe.py` | 0 | `ruff-scoped.log`, hai file Python đã format. |
| Ruff toàn repo | `.venv/bin/ruff check . --output-format concise` | 1 | 7 lỗi trên hai file `orca_progress_tools/` ngoài task. `ruff-root.log`. |
| Diff | `git diff --check` | 0 | Kiểm thêm tài liệu mới và liên kết cục bộ. |
| Native extension | ADR-0007 | — | Đặc tả upstream. Chưa có wheel hoặc ABI test của fork. |
| SFM production và GUI | Chờ gate 04R | — | Native motion đã đo trong probe, còn episode nghiệm thu `sfm_jupedsim`, YAML và viewer bị chặn. |

Các log ban đầu giữ lỗi yaw fixture không hợp lệ và test giả định limiter tác động ở tick đầu. Kiểm tra limiter được đặt đúng ở mốc 1,50 s, nơi sai lệch requested–limited đo được 0,015625 m/s. Mọi ca overlap và gate âm vẫn nằm trong suite và log.

Deviation `hm_jps_002` ghi rõ điều kiện chưa đủ. Cổng kế tiếp là hiện thực ADR-0007 trên build tách biệt và kiểm chứng pose/vận tốc external khi quay, rebase không đổi roster, overlap và re-entry người–robot trong một episode. Tích hợp `sfm_jupedsim` chỉ bắt đầu sau cổng đó. [Sổ sai khác](../../research/HUMAN_MODEL_DEVIATIONS.md)
