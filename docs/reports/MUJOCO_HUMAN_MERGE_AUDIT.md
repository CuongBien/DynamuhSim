# Kiểm toán merge MuJoCo và mô hình người

## Trạng thái ban đầu

- Nhánh `integration/mujoco-human-20261110-215812` đang merge. `HEAD` và `origin/mujoco` cùng trỏ tới `f7b25876265aa903d351c4e00ebcb520c9bac795`. `MERGE_HEAD` và `origin/mujoco-hummodel` cùng trỏ tới `95291a3f79b139e974e9ae753287ce79dad3a775`.
- Merge base là `b5f2fb3cab56c9b19f30c31e68343ec884cfa0f8`. Git báo 11 đường dẫn unmerged, mỗi đường dẫn có đủ ba stage. Tệp thứ 12 trong yêu cầu, schema state/action, đã có stage 0 nhưng chứa hai mô tả runtime trái nhau. Working tree ban đầu không có thay đổi ngoài các mục merge.
- Bản chụp ban đầu, các phiên bản stage 1/2/3, tệp có marker và log kiểm thử nằm trong `/tmp/narrow_nav_merge.cypDJM3Y/`. `initial.txt` ghi `git status`, nhánh, SHA, merge base và index unmerged.

## Ma trận xử lý 12 tệp

| Tệp | Base | `mujoco` | `mujoco-hummodel` | Cách xử lý và rủi ro hồi quy | Kiểm tra |
| --- | --- | --- | --- | --- | --- |
| `README.md` | TASK 03 với người tĩnh và sensor missing | D435i, world, recorder, teleop | Smoke active, stationary cho caller cũ, ORCA | Gộp mô tả theo entry point hiện tại và thêm lệnh chạy kết hợp. Rủi ro nhầm chế độ mặc định. | Lệnh smoke kết hợp, liên kết |
| `docs/architecture/02_DATA_FLOW.md` | Sơ đồ đích | Depth đo, depth sạch và recorder | Context cho backend người | Đổi đoạn trạng thái và thứ tự reset/step. Ghi kênh ghi log riêng. Rủi ro nhầm mốc lấy mẫu và quyền truy cập. | Sơ đồ, schema và test D435i |
| `docs/architecture/05_MUJOCO_RUNTIME.md` | Runtime stationary | Camera và lịch substep | Inject một backend và contact/clearance | Ghi đồng hồ active, ba giai đoạn lệnh và render sau cập nhật hình ảnh. Rủi ro đếm hai lần ở biên tick. | Timing, contact, test kết hợp |
| `docs/schemas/STATE_AND_ACTION_SCHEMA.md` | Hợp đồng v1 | `CleanDepthFrame` và mẫu sensor | `PlanarWorld`, lệnh người, clearance | Gỡ mô tả trạng thái lặp sau auto-merge và giữ cả hai phần mở rộng. Rủi ro nhầm ground truth với quan sát. | Schema, core và depth |
| `pyproject.toml` | Extra dev và sim | Thêm `human-models` | Mở rộng extra và pin RVO2 | Giữ hợp của các extra và cấu hình build đã có. Rủi ro đổi ABI hoặc nguồn pin. | TOML, uv lock |
| `core/protocols.py` | Cổng adapter, policy và backend | `AdapterSnapshot.clean_depth` | `PlanarWorld`, backend config, clearance, guard count | Hợp nhất import và giữ các trường tùy chọn cuối từng kiểu. Rủi ro mất dữ liệu hoặc đổi constructor. | Test core, bridge, depth |
| `simulation/backend.py` | Metadata và cổng runtime | `DepthStreamMetadata`, `get_clean_depth()` | Backend provenance, solver coverage, traceback | Hợp nhất metadata với mặc định tương thích cho trường mới. Giữ `depth_stream` ở vị trí của nhánh camera, lifecycle điền trường tùy chọn theo tên. | D435i, runtime người, recorder |
| `simulation/configuration.py` | Validate scene, robot, timing | Parse và kiểm profile D435i | `HumanMotionLimits` và backend config | Giữ cả hai phép kiểm ở biên cấu hình. Rủi ro chấp nhận lịch hoặc profile sai. | Cấu hình, timing, test kết hợp |
| `simulation/lifecycle.py` | Lệnh người zero và slot sensor missing | Seed depth, cặp mẫu sạch/đo, công bố observation | Reset/propose, validation stamp và roster, limiter, clearance | Reset adapter với seed camera rồi reset backend. Step kiểm đầu ra vật lý và lịch sensor trước khi công bố. Rủi ro stale frame, lệch tick hoặc công bố một phần khi lỗi. | Contract, engine lifecycle, D435i, backend runtime |
| `simulation/mujoco/adapter.py` | Substep robot, người mocap tĩnh | Renderer D435i và thu theo substep | Mocap active, guard, contact solver và animation thực thi | Mỗi substep áp mocap hợp lệ, gọi `mj_step` một lần, lấy contact và refresh state. Khi đến mốc capture hoặc cuối tick, cập nhật animation theo dịch chuyển trong khoảng kể từ lần cập nhật trước. Camera render sau cập nhật hình ảnh. Rủi ro nhân đôi vật lý, contact hoặc animation. | Contact, Quaternius, test kết hợp và replay |
| `simulation/mujoco/geometry.py` | Hình chiếu sàn/tường | Bỏ cap hốc nhường khỏi footprint ở mặt sàn | Kiểm Z, hình học trụ và hộp xoay trong chế độ inspect | Dùng tọa độ world, giữ ngoại lệ cap và điều kiện upright. Chế độ inspect opt-in hỗ trợ tường xoay, runtime giữ mặc định trục thẳng. Rủi ro bít cửa/hốc hoặc sai footprint. | Authored worlds, corridor width, projection |
| `uv.lock` | Đồ thị dev/sim | Thêm đồ thị JuPedSim | Cùng đồ thị, thêm RVO2 và metadata extra | Dùng stage 3 làm seed hợp lệ trong thư mục tạm và chạy uv. Kết quả giống seed từng byte. Rủi ro đổi pin hoặc lock khác khai báo. | `uv lock --offline`, `uv lock --check --offline` |

Các đường dẫn package viết gọn trong bảng đều thuộc `src/narrow_nav/`.

## Tính năng và bất biến đã giữ

- MuJoCo 4.0.0 tiếp tục sở hữu model MJCF, vật lý robot vi sai/mecanum và contact native. Các world trường học, bệnh viện, hành lang, cửa, hốc nhường và góc khuất giữ nguyên asset, pose spawn và hình học. Cap nổi vẫn là geom native, footprint sàn giữ khoảng mở. Đọc lại logic `world_geometry.py`, `geometry_inspector.py`, `collision_monitor.py` và fixture world/projection.
- D435i giữ calibration, frame quang học, capture reset/substep, model nhiễu, `DepthStreamMetadata`, `get_clean_depth()`, recorder NPZ/JSONL và công cụ teleop. Depth đo vào `Observation`. Depth sạch, ground truth, contact và clearance đi qua các cổng kiểm định riêng. LiDAR, IMU và odometry vẫn báo `missing`.
- Registry giữ scripted, ORCA thuần và ba model JuPedSim SFM, AVM, CSM V2. Một backend được inject cho toàn episode. Reset nhận roster ID, planar world, giới hạn và seed độc lập. Callback đọc `HumanContext` tại biên robot đến lịch người. Motion batch giữ `requested`, `limited`, `executed` cùng ID và stamp đầu tick. Bridge JuPedSim giữ API external actor và đồng bộ từ trạng thái MuJoCo. Đã đọc lại các thay đổi tự gộp ở `actions.py`, `states.py`, `human_visuals.py`, `mujoco_engine.py`, `smoke.py` và `timing.py`.
- Snapshot cuối tick có stamp và roster đã xác thực. Mẫu D435i được công bố theo index, frame và thời điểm capture trên lưới physics. Observation chỉ thấy mẫu đo sẵn sàng tại biên công bố. Guard ghi số substep giữ pose. Mask solver công bố robot–tường và robot–người. Clearance đo cả người–tường và người–người qua footprint phẳng riêng.
- Cổng công khai giữ `SimulationBackend`, `SimulatorAdapter` và `HumanControllerBackend`. Phần giao cắt của hai nhánh là `AdapterSnapshot` mang mẫu sạch/đo, `AdapterStepResult` mang bộ đếm guard, `SimulationStepResult` mang ba batch lệnh, mẫu đo và chẩn đoán hình học, cùng `SimulationMetadata` mang cả provenance người và camera. Tầng `core` giữ kiểu độc lập engine, đối tượng native nằm trong adapter.
- Benchmark giữ cấu hình, scenario card, metric, fixture, QA và quy tắc giữ attempt lỗi. Recorder D435i hiện dùng profile stationary mặc định. Benchmark người hiện dùng danh sách sensor rỗng. Kiểm chứng hai đường lưu ở chế độ của từng entry point, và kiểm chứng profile người + camera qua runtime chung. Giữ nguyên bằng chứng benchmark và hai gitlink.

## Dependency và môi trường

- `uv 0.13.7` resolve 42 package offline từ seed stage 3 với `pyproject.toml` đã gộp. Kiểm lại lock ở workspace. Giữ extra `dev`, `sim`, `human-models`, `human-orca`, `human-jupedsim`, `human-all`. Python project vẫn `>=3.11`, MuJoCo extra vẫn `>=4.0` và lock chọn 4.0.0. Python-RVO2 giữ git rev `c2c46ba8d59556aa10faf03479293236efea154d`, build pin Cython 3.1.4 và setuptools 80.9.0.
- Lock chứa JuPedSim stock 1.4.2. Các ca coupled dùng wheel cục bộ `1.4.2+task04u` CPython 3.12 x86-64 theo hợp đồng hiện có. Wheel dùng để kiểm có SHA-256 `76997cc89c2cbfd44fb7279925cfa2ac884a06802b14bd2c3f1bed8d8eebd94c`. Backend runtime báo hash native `5ebdd4593786734392fefa0c3748f4431d3de53fb857614f6a36a8919b23ec65`. Hai bản vá 04S/04U giữ nguyên. [Hợp đồng JuPedSim](../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md)
- Hai venv tạm dùng CPython 3.12.3 và dùng lại dependency MuJoCo, RVO2 cùng dependency test đã có. Cài wheel 04U và stock lần lượt vào hai prefix riêng. Wheel stock khớp SHA-256 trong lock. `.pth` trong các venv tạm chỉ đúng checkout để subprocess vẫn import khi bỏ `PYTHONPATH`. `.venv` và môi trường native có sẵn giữ nguyên.

## Kiểm chứng

Chạy từ gốc repository. `audit=/tmp/narrow_nav_merge.cypDJM3Y`, `py="$audit/venv_04u/bin/python"`, `stock="$audit/venv_stock/bin/python"`.

| Lệnh hoặc nhóm | Kết quả |
| --- | --- |
| `.venv/bin/uv lock --offline --project "$audit" --cache-dir "$audit/uv-cache"` và `.venv/bin/uv lock --check --offline --cache-dir "$audit/uv-cache"` | PASS · 42 package, không đổi phiên bản khóa giữa hai nhánh |
| `"$py" -m compileall -q src scripts tests` | PASS |
| `env -u PYTHONPATH "$py" -m pip check` | PASS · No broken requirements found |
| Ruff check và format check cho bảy tệp Python trực tiếp sửa | PASS |
| Liên kết đường dẫn trong README, hai tệp kiến trúc và schema state/action | PASS · 50 đích liên kết cục bộ |
| Nhóm sensor và human ưu tiên với pytest `-q -ra` headless | 97 PASS, 0 FAIL, 2 SKIP dành cho stock |
| Toàn bộ suite với wheel 04U và EGL | 276 PASS, 0 FAIL, 4 SKIP trong 124,69 s |
| Bốn test dành riêng cho stock, chạy trên wheel stock đã khóa | 4 PASS, 0 FAIL, 0 SKIP trong 0,50 s |
| Smoke runtime mecanum + scripted + D435i | PASS · 4 tick, 100 substep, 0,2 s mô phỏng, robot tiến 0,03518 m, người tiến 0,045 m về đích |
| Smoke teleop trên GLFW/Xvfb cô lập | PASS · Điều khiển mecanum và render depth đo cùng ba chế độ clean, mask và error bằng viewer thật |
| Benchmark ORCA `plan`, `dry-run`, `run`, `report` trên một slot smoke | PASS · `benchmark_smoke TIMEOUT` hợp lệ theo ngân sách 0,5 s, report và QA được ghi vào thư mục tạm |

```bash
env -u DISPLAY -u WAYLAND_DISPLAY MUJOCO_GL=egl PYTHONPATH="$PWD/src" "$py" -m pytest -q -ra

env -u DISPLAY -u WAYLAND_DISPLAY MUJOCO_GL=egl PYTHONPATH="$PWD/src" "$py" -m pytest -q -ra \
  tests/simulation/test_d435i_depth.py tests/simulation/test_teleop_depth.py \
  tests/simulation/test_robot_control.py tests/simulation/test_world_geometry.py \
  tests/integration/test_human_runtime.py tests/integration/test_orca_runtime.py \
  tests/integration/test_jupedsim_bridge_runtime.py tests/integration/test_jupedsim_universal.py \
  tests/simulation/test_human_contacts.py tests/simulation/test_jupedsim_projection.py

env -u DISPLAY -u WAYLAND_DISPLAY MUJOCO_GL=egl PYTHONPATH="$PWD/src" "$stock" -m pytest -q -ra \
  tests/integration/test_jupedsim_universal.py::test_older_build_rejects_04u_backends \
  tests/integration/test_sfm_coupling_probe.py::test_public_surface_and_replacement_preserve_negative_evidence \
  tests/integration/test_sfm_research_backend.py::test_stock_sfm_cannot_fall_back_to_an_autonomous_proxy

env -u DISPLAY -u WAYLAND_DISPLAY MUJOCO_GL=egl "$py" -m narrow_nav.simulation.smoke \
  --ticks 4 --vx 0.3 robots=mecanum_robot humans=scripted sensors=d435i

env -u PYTHONPATH -u WAYLAND_DISPLAY MUJOCO_GL=glfw \
  xvfb-run -a -s '-screen 0 1600x900x24' "$py" "$audit/teleop_glfw.py"

"$py" -B -m scripts.benchmarks.plan plan --phase smoke --output "$audit/benchmark_smoke.json"
"$py" -B -m scripts.benchmarks.plan dry-run --manifest "$audit/benchmark_smoke.json"
env -u DISPLAY -u WAYLAND_DISPLAY -u PYTHONPATH "$py" -B -m scripts.benchmarks.plan run \
  --manifest "$audit/benchmark_smoke.json" --episode benchmark_smoke \
  --output-dir "$audit/benchmark-smoke" --execute --max-episodes 1
"$py" -B -m scripts.benchmarks.plan report --run-dir "$audit/benchmark-smoke" \
  --output-dir "$audit/benchmark-report"
```

Test hồi quy kết hợp dùng model Quaternius, camera D435i noise bật và period capture 20 ms trên tick robot 50 ms. Cả robot và người đi trong một engine thật. Kiểm 11 cặp frame đo/sạch trên mỗi lượt, index/frame/shape/mask và mốc sẵn sàng, cùng chuyển động executed khớp dịch chuyển mỗi tick. Hai reset cùng seed cho pose và các frame giống nhau. Các test native chạy ORCA và lần lượt SFM, AVM, CSM V2. Suite có ca doorway, passing bay, blind corner, school, hospital và contact trong substep.

Bốn skip ở lượt 04U gồm hai ca AVM/CSM phải từ chối build cũ, một probe fingerprint stock 04R và một ca stock SFM phải từ chối proxy. Cả bốn đã chạy riêng trên bản stock. Các lần chẩn đoán đầu thấy 6 lỗi subprocess ORCA do thiếu import khi bỏ `PYTHONPATH`, rồi 20 lỗi 04U do wheel được tham chiếu qua `.pth` từ ngoài prefix và bị guard từ chối. Sửa cách chuẩn bị môi trường kiểm, giữ nguyên guard, test và ngưỡng. `pip check` trong shell ban đầu thấy hai dependency thiếu của ROS `generate-parameter-library-py` qua `PYTHONPATH`. Chạy lại trong môi trường dự án đã tách ROS cho kết quả PASS.

## Phạm vi diff và quyết định

- Tệp trực tiếp thay đổi gồm 12 tệp trong ma trận, `tests/simulation/test_d435i_depth.py` và báo cáo này. Đối chiếu hash index xác nhận 140 thay đổi tự gộp giữ nguyên blob. Schema state/action là mục tự gộp duy nhất được chỉnh trong lần này.
- Whitespace đã có trước trong file Markdown lịch sử và dòng ngữ cảnh của hai bản vá JuPedSim làm `git diff --cached --check` toàn index báo lỗi. Các hunk thực hiện trong lần này qua kiểm tra whitespace riêng. Giữ nguyên nội dung lịch sử và byte của bản vá.
- Các báo cáo lịch sử từ nhánh người có liên kết tới tài liệu giao thức và ADR chưa hiện diện trong cây merge, gồm `HUMAN_MODELS_RUNTIME_CONTRACT.md`, `HUMAN_MODELS_TIMING_GEOMETRY.md` và ADR-0004. Tài liệu trực tiếp chỉnh dùng đích hiện diện. Smoke teleop kiểm viewer thật trên Xvfb bằng callback phím. Benchmark chỉ chạy một smoke.
- **MERGE_READY** cho review thủ công. Index có 0 mục unmerged và source sạch marker. Các cổng runtime, test, dependency và phạm vi staged đều đạt. Merge đang mở để người dùng duyệt và commit.
