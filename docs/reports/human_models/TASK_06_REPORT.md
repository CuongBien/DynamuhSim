# TASK 06 · JuPedSim AnticipationVelocityModel

**Cập nhật nghiệm thu · `PARTIAL`.** 04U đã mở cổng tích hợp AVM native vào MuJoCo. Closeout xác nhận độ nhạy vận tốc robot và kiểm lại bốn hành động robot, góc khuất, giao cắt và cửa. `BEHAVIOR_PASS` còn vướng giao cắt người–người đối xứng và ca tranh cửa 0,60 m. Hồ sơ dưới đây giữ nguyên bối cảnh lúc stock và 04S còn chặn coupling. [Báo cáo closeout](TASK_06_CLOSEOUT_REPORT.md) · [Cổng tích hợp 04U](TASK_04U_REPORT.md)

## Hồ sơ trước TASK 04U

**Trạng thái · `BLOCKED_CAPABILITY`.** JuPedSim AVM native chạy được trong 2D. Điều kiện TASK 04 PASS cho robot động vẫn thiếu. Stock 1.4.2 không có API external/rebase. Hai API thử nghiệm của 04S từ chối model AVM. [TASK 04](TASK_04_REPORT.md) · [TASK 04S](TASK_04S_REPORT.md) · [API và tham số AVM](../../backends/AVM_JUPEDSIM.md)

## Checkout và tài liệu

- Nhánh `mujoco-hummodel`, SHA trước/sau `7fb4fa232bbf52a98d8ab79279a32a4ae971afbe`, lệch upstream local 0/0. Index sạch. Checkout đã dirty trước kiểm định, gồm các phần 04R, 04S, 05R và hai tệp `orca_progress_tools/`. Các thay đổi 06 không yêu cầu checkout nhánh khác hay cập nhật dependency.
- Đường dẫn `/tmp/codex_human_model/00_MASTER_RULES.md` vắng mặt. Áp dụng toàn văn MASTER trong attachment TASK 06. Nguồn nghiên cứu trong working tree và HEAD có cùng SHA-256 `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030` trước và sau.
- Đối chiếu `HUMAN_MODEL_FINAL.md`, `AGENTS.md`, `README.md`, `ROADMAP.md`, toàn bộ `docs/architecture/`, `docs/protocols/`, `docs/schemas/`, ADR-0001, ADR-0007, `pyproject.toml`, pin trong `uv.lock`, tài liệu API chính thức, báo cáo và test 04/04R/04S/05/05R, bridge và patch 04S. [Nguồn nghiên cứu](../../research/HUMAN_MODEL_FINAL.md) · [Nguồn AVM 1.4.2](https://github.com/PedestrianDynamics/jupedsim/blob/v1.4.2/libsimulator/src/AnticipationVelocityModel.cpp)
- Cả hai runtime chạy CPython 3.12.3 cp312 x86-64, MuJoCo 4.0.0, Shapely 2.2.0 và NumPy 2.5.4. `.venv/bin/python` dùng wheel stock 1.4.2, binary SHA-256 `906db6cf6714c7cf9021ec2315c16bd9a54ad84edad30bc0a3ce44b9949f75bd`. `runs/task_04s/reproduction/venv/bin/python` dùng build local `1.4.2+task04s`, binary SHA-256 `77257fc9679c3ac2691495c509ef444937301351c175525231628bc1ceae091f`, patch `0d79faa084acf5b863a3944258499837b46f7f9ccae56d8119b3a555bab710e4`, nền `80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe`. [Chữ ký, metadata wheel và ABI](../../../runs/task_06/preflight.log)

## Rà sơ bộ TASK 05 cũ

| Tiêu chí | Trạng thái hiện tại |
| --- | --- |
| Gate TASK 04 và runtime stock | Báo cáo lịch sử `BLOCKED_CAPABILITY` còn đúng cho stock. Test stock tái xác nhận. |
| Backend, config và provenance | 05R bổ sung `jupedsim_sfm.py`, `sfm_jupedsim.yaml` và test tích hợp. Backend dùng đúng hai lớp `SocialForceModel` / `SocialForceModelAgentParameters`, chung geometry và clock. Đường thực thi SFM độc lập với ORCA, SocNavGym và HuNavSim. Build 04S là điều kiện bắt buộc. |
| Journey, reset và GT | Regression trên build 04S kiểm journey/stage và ID, robot external giữ mẫu GT, rebase người, nhịp 50/100 ms và replay. Các phép đo thực thi và contact nằm trong 05R. |
| Profile nghiên cứu | `cooperative`, `neutral`, `non_cooperative` được phân tích trong tài liệu. 05R dùng lực native và chưa hiệu chuẩn hay gán các nhãn chính sách đó cho fixture. |
| Coverage và gate tương tác | 05R có free walk, đối đầu/lách qua ở Easy, robot tiến/dừng, replay, số đo requested/limited/executed, clearance, contact và đồ thị. Critical/Hard và replay hẹp có contact. Chưa có bằng chứng nghiệm thu SFM riêng cho cửa hẹp. Kết quả giữ nguyên `IMPLEMENTATION_PASS`, `SYNC_PASS`, `INTERACTION_FAIL` trên build thử nghiệm. |

[Báo cáo TASK 05 lịch sử](TASK_05_REPORT.md) · [Báo cáo và biểu đồ TASK 05R](TASK_05R_REPORT.md) · [Tài liệu SFM](../../backends/SFM_JUPEDSIM.md)

## Bằng chứng TASK 06

Hai fixture 2D người–người dùng cùng corridor rộng 6 m chiếu từ MJCF biên dịch, hai agent bán kính 0,2 m, seed native 11 và dt 10 ms. Tại t = 1 s, hai nhánh có cùng tọa độ và cùng lịch sử native trước nhánh. Chỉ đổi `anticipation_time` từ 1 s sang 0 s cho bước kế. Tốc độ cực đại đo ở bước phân nhánh là 1,2 m/s. Replay cùng seed trùng tọa độ trên từng build. Đây là fixture kỹ thuật, tách khỏi episode của Card.

| Fixture | Clearance footprint nhỏ nhất tại biên nhánh và bước kế | Chênh dịch chuyển lớn nhất sau một bước |
| --- | ---: | ---: |
| Crossing 90° | 0,952970 m | 0,490495 mm |
| Head-on | 1,553816 m | 0,218510 mm |

Native AVM tính anticipation từ vị trí, hướng và vận tốc hiện tại của láng giềng, bao gồm vận tốc nội bộ từ bước đã chạy. Phép thử trên cô lập thay đổi horizon trong 2D. Probe coupling riêng lấy `HumanContext` ở reset thật của MuJoCo, radius robot 0,25 m và clock native 10 ms. Stock thiếu cả hai phương thức. Build 04S trả `experimental SFM coupling requires SFM with dt=10 ms` cho đăng ký robot lẫn rebase người. Sau lỗi, roster vẫn có một agent, vị trí người giữ nguyên và iteration count bằng 0. `Agent.position` từ chối gán và AVM state không lộ property `velocity` ở wrapper lẫn binding. [Log stock](../../../runs/task_06/avm-stock.log) · [Log build 04S](../../../runs/task_06/avm-04s.log) · [Patch 04S](../../../experiments/social_navigation/patches/jupedsim_v1_4_2_task04s.patch)

| Hạng mục nghiệm thu | Kết quả |
| --- | --- |
| Exact AVM classes và tham số, native stepping | Đã xác minh trong 2D trên cả hai build. [Bảng tham số](../../backends/AVM_JUPEDSIM.md) |
| Robot động, human rebase, finite requested qua limiter và GT/contact AVM | `BLOCKED_CAPABILITY`. Chưa có backend/config sản phẩm hay manifest episode AVM. |
| AVM end-to-end crossing, head-on, blind corner, bottleneck và robot advance/stop | Chưa chạy vì thiếu coupling. Hai phép thử 2D không cung cấp GT trajectory/contact MuJoCo của AVM. |
| Đối chứng ORCA/SFM | Chưa có khóa cặp chung với một episode AVM hợp lệ. Không chấm baseline từ dữ liệu probe. |

## Lệnh và kết quả

Chạy tại root repo. Các lệnh Python dùng `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY`. `S` là `.venv/bin/python` và `E` là `runs/task_04s/reproduction/venv/bin/python`.

| Lệnh | Exit | Kết quả |
| --- | ---: | --- |
| `$S -B -m pytest -q -s tests/integration/test_avm_bridge_capability.py` | 0 | 3 passed. Hai fixture native và test xác nhận thiếu API. [Log](../../../runs/task_06/avm-stock.log) |
| `$E -B -m pytest -q -s tests/integration/test_avm_bridge_capability.py` | 0 | 3 passed. Hai fixture native và test xác nhận AVM bị từ chối bởi API SFM. [Log](../../../runs/task_06/avm-04s.log) |
| `$S -B -m pytest -q -s tests/unit/test_jupedsim_bridge.py tests/simulation/test_jupedsim_projection.py tests/integration/test_jupedsim_bridge_runtime.py tests/integration/test_sfm_coupling_probe.py tests/integration/test_sfm_research_backend.py` | 0 | 26 passed, 3 skip cần build 04S. [Log](../../../runs/task_06/stock-sfm-bridge.log) |
| `$E -B -m pytest -q -s tests/integration/test_jupedsim_external_actor.py tests/integration/test_sfm_research_backend.py` | 0 | 9 passed, 1 skip dành cho stock. [Log](../../../runs/task_06/04s-sfm-regression.log) |
| `$S -B -m narrow_nav.simulation.smoke --ticks 1 humans=avm_jupedsim` | 1 | Hydra báo `Could not find 'humans/avm_jupedsim'`, phù hợp cổng đăng ký chưa mở. [Log](../../../runs/task_06/avm-entrypoint.log) |
| `tar -xOf runs/task_04s/jupedsim-1.4.2.tar.gz jupedsim-1.4.2/libsimulator/src/AnticipationVelocityModel.cpp` so bằng `cmp` với tệp tương ứng trong `runs/task_04s/reproduction/source` | 0 | Mã C++ AVM trong build 04S trùng từng byte với archive 1.4.2. [Log](../../../runs/task_06/avm-source-check.log) |
| `.venv/bin/ruff check src tests experiments/social_navigation`, `.venv/bin/ruff format --check tests/integration/test_avm_bridge_capability.py` và `git diff --check` | 0 | Lint giới hạn, định dạng và diff đạt. [Log lint](../../../runs/task_06/ruff-scoped.log) |
| `.venv/bin/ruff check .` | 1 | Còn 7 lỗi sẵn có trong hai tệp `orca_progress_tools/`. [Log](../../../runs/task_06/ruff-root.log) |

## Tệp thay đổi và cổng tiếp theo

- Tạo `tests/integration/test_avm_bridge_capability.py`, `docs/backends/AVM_JUPEDSIM.md` và báo cáo này. Cập nhật `docs/reports/human_models/PROGRESS.md` và `docs/research/HUMAN_MODEL_DEVIATIONS.md` (`hm_jps_005`). Log dưới `runs/task_06/` được Git bỏ qua. Hồ sơ TASK 05 lịch sử vẫn giữ nguyên.
- Trước khi tạo `jupedsim_avm.py` và `avm_jupedsim.yaml`, cần kiểm định API external AVM với robot MuJoCo và human rebase giữ nguyên phương trình native. Sau đó mới mở ca MuJoCo advance/stop và các đối chứng có khóa scene/seed/robot profile chung. Phạm vi hiện tại không có episode benchmark.
