# Nghiệm thu TASK 00 · Môi trường và dependency

**PASS cho cổng dependency trên `/home/thaonhi/nav_ws/.venv/bin/python`** · Ubuntu 24.04.5 LTS x86_64, glibc 2.39, CPython 3.12.3. Một interpreter nạp MJCF và chạy MuJoCo, tạo và tiến ba model JuPedSim, rồi tạo và tiến RVO2. Các phép thử dùng fixture nhỏ. Benchmark nghiên cứu chưa chạy.

## Checkout và đầu vào

- Gốc Git `/home/thaonhi/nav_ws`, nhánh `mujoco-hummodel`. HEAD trước và sau là `09875d9fad40468697a2fe7d99a0fdc282f34780`. `git rev-list --left-right --count HEAD...@{upstream}` trả `0 0`. `git ls-remote --exit-code origin refs/heads/mujoco-hummodel` trả cùng SHA.
- Dirty trước và sau. Trước audit, index và diff tracked trống, đã có `docs/research/HUMAN_MODEL_FINAL.md` untracked. SHA-256 bản checkout và bản gốc trong bundle đều là `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`. Không stage hoặc commit.
- Đọc `00_MASTER_RULES.md`, `TASK_00_ENV_AUDIT.md`, prompt đính kèm, `HUMAN_MODEL_FINAL.md`, `AGENTS.md`, `README.md`, `docs/ROADMAP.md`, ADR-0001, toàn bộ Markdown trong `docs/architecture/`, `docs/protocols/` và `docs/schemas/`. Đối chiếu `pyproject.toml`, `uv.lock`, `humans/requirements*.txt`, entry point smoke, test hiện có và mã dựng/API của hai vendor. Bản ACM theo DOI trả 403. Trang arXiv của tác giả xác nhận bài cùng tên. Cổng này không dùng ngưỡng, kết luận hoặc metric từ bài báo. [Francis và cộng sự](https://arxiv.org/abs/2306.16740) · [API JuPedSim](https://www.jupedsim.org/stable/api/jupedsim/index.html) · [ORCA](https://gamma.cs.unc.edu/ORCA/) · [RVO2](https://gamma-web.iacs.umd.edu/RVO2/documentation/2.0/)

| Gitlink | SHA trong index và HEAD của checkout lồng | Trạng thái |
| --- | --- | --- |
| `experiments/social_navigation/vendor/Python-RVO2` | `c2c46ba8d59556aa10faf03479293236efea154d` | Detached HEAD, clean |
| `experiments/social_navigation/vendor/SocNavGym` | `95fbc9cebe4357201145a7d0a28c7736c0efe6cf` | Detached HEAD, clean |

`git submodule status` trả 128 vì repo thiếu mapping `.gitmodules`. Đã đối chiếu trực tiếp index và hai checkout lồng. uv lấy binding theo SHA cố định từ upstream, không cần `submodule update`. SocNavGym không thuộc bộ dependency của bốn thuật toán.

## Lựa chọn runtime và thay đổi

`.venv` của repo đã có MuJoCo, JuPedSim và các gói tương ứng với test của repo. `ai12` có RVO2 và thiếu MuJoCo/JuPedSim. Python hệ thống và Conda base thiếu cả ba binding. uv 0.13.7 resolve 42 package sau thay đổi, thêm một package `pyrvo2` và giữ các phiên bản cũ trong lockfile. Dry run giới hạn package chỉ lên kế hoạch thêm `pyrvo2`. Sau khi cài, diff của hai file `pip list --format=freeze` xác nhận đó là thay đổi duy nhất trong venv. [Ma trận đầy đủ](../../setup/ENV_MATRIX.md)

| File | Thay đổi |
| --- | --- |
| [pyproject.toml](../../../pyproject.toml) | Thêm ba extra, nguồn Git và pin build dependency của RVO2. Giữ extra `human-models` cho caller hiện có. |
| [uv.lock](../../../uv.lock) | Khóa nguồn RVO2 và resolve ba extra. Giữ MuJoCo 4.0.0, JuPedSim 1.4.2 và NumPy 2.5.4 cho Python 3.12. |
| [check_human_env.py](../../../scripts/diagnostics/check_human_env.py) | Chẩn đoán headless, output text/JSON và cổng kiểm metadata `--strict`. |
| [ENV_MATRIX.md](../../setup/ENV_MATRIX.md) · [HUMAN_MODELS_ENV.md](../../setup/HUMAN_MODELS_ENV.md) | Ma trận interpreter, ABI, dependency, hướng dẫn chạy, cài và hoàn tác. |
| [HUMAN_MODEL_DEVIATIONS.md](../../research/HUMAN_MODEL_DEVIATIONS.md) · báo cáo này | Ghi sai khác engine theo ADR và hồ sơ nghiệm thu. |

## Kiểm chứng

Các lệnh dưới đây chạy từ gốc repo. `P=/home/thaonhi/nav_ws/.venv/bin/python` và `A=runs/task_00_env_audit` là biến rút gọn. Các lệnh có pipe trong lần audit bật `set -o pipefail`.

| Lệnh | Exit | Kết quả |
| --- | --- | --- |
| `which -a python python3 pip uv conda ruff` và `printenv` cho các biến đường dẫn Python/Conda/uv/display | 1 với `which` do một số tên vắng mặt; 0 với `printenv` | PATH thấy `python3`, `pip`, `uv` và `conda`. `PYTHONPATH` có ROS. Không kích hoạt venv hoặc Conda. |
| `CONDA_NO_PLUGINS=true /home/thaonhi/miniconda3/bin/conda env list`, `UV_NO_CACHE=1 uv python list --only-installed` | 0, 0 | Conda có `ai12` và base. uv phát hiện CPython hệ thống 3.12.3. |
| `<python> -I -B -c '…'` dùng `sys`, `platform`, `sysconfig`, `importlib.metadata` và import thật. `<python> -I -B -m pip list --format=freeze` và `<python> -I -B -m pip check` | Inventory/freeze 0. Pip check 0 cho repo, `ai12`, Conda base và 1 cho system | Đã đối chiếu phiên bản, SOABI, tag wheel và module origin. System Python thiếu `cffi` theo metadata của PyNaCl. [Ma trận](../../setup/ENV_MATRIX.md) |
| `uv lock --cache-dir "$A/uv-cache"`, `uv --offline lock --check --cache-dir "$A/uv-cache"` | 0, 0 | Resolve 42 package. Git fetch theo SHA và metadata build thành công. |
| `env -u VIRTUAL_ENV -u UV_PROJECT_ENVIRONMENT -u PYTHONPATH uv --offline sync --locked --inexact --python "$P" --extra human-orca --only-install-package pyrvo2 --cache-dir "$A/uv-cache"` | 0 | Dry run trước với `--dry-run` báo thêm một package. Lần thật dựng wheel và cài đúng một package. `direct_url.json` ghi đúng SHA Git. |
| `env -u VIRTUAL_ENV -u UV_PROJECT_ENVIRONMENT -u PYTHONPATH uv --offline sync --locked --inexact --check --no-install-project --python "$P" --extra dev --extra sim --extra human-all --cache-dir "$A/uv-cache"` | 0 | Kiểm 38 package. Không có thay đổi dependency cần đồng bộ. |
| `"$P" -I -B -m pip --disable-pip-version-check install --no-index --no-deps --dry-run --ignore-installed "$A/wheelhouse/pyrvo2-0.0.0-cp312-cp312-linux_x86_64.whl"` | 0 | pip chấp nhận wheel tag và dự kiến cài `pyrvo2-0.0.0`. |
| `env -u DISPLAY -u WAYLAND_DISPLAY "$P" -I -B scripts/diagnostics/check_human_env.py --strict --json` | 0 sau cài | 5/5 PASS. Dữ liệu chi tiết ở bảng dưới. |
| Cùng script ở chế độ text mặc định | 0 sau cài | 5/5 PASS. |
| Cùng script `--strict --json` trước cài ở repo và ở `ai12` | 1, 1 | Repo báo đúng thiếu `rvo2` và chạy được bốn phép còn lại. `ai12` tiến RVO2 và báo thiếu MuJoCo/JuPedSim. Hai lần chạy là ca đối chứng lỗi thiếu gói. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m pytest -q -rs` | 0 trước và sau | 124 passed mỗi lần, gồm unit, integration, simulation và test Quaternius. Không báo skip. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$P" -B -m narrow_nav.simulation.smoke --ticks 2 --vx 0.35 worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot` | 0 | MuJoCo 4.0.0 tiến 2 tick điều khiển, 50 bước vật lý, 0,1 s. Robot tiến khoảng 0,00956 m. Các sensor vẫn được báo chưa khả dụng. |
| `.venv/bin/ruff check .` và `.venv/bin/ruff format --check scripts/diagnostics/check_human_env.py` | 0, 0 | Lint và định dạng đạt. |
| Kiểm tra đích và mốc dòng của link Markdown cục bộ bằng `pathlib`, parse lock bằng `tomllib` và log smoke bằng `json` | 0 | 22 link cục bộ hợp lệ. Extra, pin, SHA binding và trạng thái từng smoke khớp. |
| `git status --short`, `git diff --check`, đọc diff tracked và từng file mới | 0 | Thay đổi giới hạn ở các file nêu trên. File nguồn nghiên cứu tồn tại trước audit và có cùng SHA. |

| Phép thử native trên `P` | Điều kiện và kết quả |
| --- | --- |
| MuJoCo 4.0.0 | `MjModel.from_xml_path` nạp `sim/mujoco/worlds/corridor/main.xml` với 5 geom. `mj_step` đưa đồng hồ đến 0,002 s. |
| SFM, JuPedSim 1.4.2 | `Simulation` với `SocialForceModel` và `SocialForceModelAgentParameters`. Sau 5 bước đến 0,05 s, agent tiến theo +X khoảng 0,002337 m. |
| AVM, JuPedSim 1.4.2 | `AnticipationVelocityModel(rng_seed=0)` và `AnticipationVelocityModelAgentParameters`. Sau 5 bước đến 0,05 s, agent tiến 0,06 m theo +X. |
| CSM, JuPedSim 1.4.2 | `CollisionFreeSpeedModelV2` và `CollisionFreeSpeedModelV2AgentParameters`. Sau 5 bước đến 0,05 s, agent tiến 0,06 m theo +X. |
| ORCA, `pyrvo2` 0.0.0 | `rvo2.PyRVOSimulator`, một agent với vận tốc ưu tiên (1,0) m/s. `doStep` đưa đồng hồ đến 0,0099999998 s và agent đến (0,0099999998, 0) m. |

Fixture JuPedSim dùng một agent trong polygon chữ nhật, waypoint (2, 0) m, bước 0,01 s. Chữ ký runtime của model, lớp tham số và `Simulation.iterate(count=1)` nằm trong log JSON. RVO2 công bố chữ ký các method qua `inspect.signature`. Chữ ký constructor được đối chiếu với Cython vì lớp native không export chữ ký introspection. Các kết quả xác nhận API và khả năng chạy chung trong một interpreter. Phạm vi này chưa kiểm ghép robot MuJoCo với agent JuPedSim.

## Bằng chứng và các lỗi đã phân loại

Log cục bộ ở `runs/task_00_env_audit/`, được Git bỏ qua. `smoke-standard.json` lưu kết quả và chữ ký native. `smoke-.venv-before.json` và `smoke-ai12-before.json` lưu ca thiếu gói. `git-provenance.log`, `runtime-smoke.json`, `pytest-after.log`, `uv-*.log`, các file `*.freeze.txt` và `wheel-sha256.txt` lưu bằng chứng bổ sung. [Hướng dẫn và SHA wheel](../../setup/HUMAN_MODELS_ENV.md)

- **Cổng dependency chuẩn** · Không còn blocker. Build lần đầu cần mã GitHub, build dependency PyPI và toolchain C++. Chỉ mục PyPI `pyrvo2` và `rvo2` trả 404. Offline trước khi chuẩn bị cache không lấy được SHA Git. Sau khi resolve nguồn, build và cài offline thành công.
- **Các lỗi ngoài runtime chuẩn** · System Python có lỗi metadata PyNaCl. `git submodule status` thiếu mapping. Venv ngoài repo vắng mặt và thư mục SocNavGym cũ thiếu interpreter. Lần gọi smoke runtime đầu truyền `--output-dir` mà CLI không hỗ trợ (exit 2). Lần chạy lại dùng redirect stdout và đạt (exit 0).
- **Sai khác đặc tả** · Dùng headless MuJoCo từ Vòng 0 theo ADR-0001, mục `hm_env_001`. Các mâu thuẫn scenario, policy, ngân sách, ngưỡng và định nghĩa metric dành cho task thiết kế tiếp theo. [Sổ sai khác](../../research/HUMAN_MODEL_DEVIATIONS.md)
- **Cổng kế tiếp** · TASK 01 preflight xác nhận hợp đồng, policy, nhịp điều khiển và cấu hình scenario trước tích hợp. Giữ đầu vào robot theo kênh quan sát của repo. [Schema state/action](../../schemas/STATE_AND_ACTION_SCHEMA.md)
