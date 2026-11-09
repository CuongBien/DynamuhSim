# Môi trường cho bốn mô hình người

Dùng `/home/thaonhi/nav_ws/.venv/bin/python` trên Ubuntu 24.04 x86_64 với CPython 3.12.3. Runtime đã kiểm chứng dùng MuJoCo 4.0.0 cho vật lý và mô hình Quaternius, JuPedSim 1.4.2 cho SFM/AVM/CSM, và Python-RVO2 cho ORCA. Mỗi episode chọn một backend hành vi người theo giao thức đối chứng. [Ma trận môi trường](ENV_MATRIX.md) · [Giao thức hành vi](../protocols/HUMAN_BEHAVIOR_PROTOCOL.md)

| Extra | Phân phối và API |
| --- | --- |
| `human-orca` | `pyrvo2==0.0.0` cung cấp `rvo2.PyRVOSimulator`. uv khóa nguồn tại SHA `c2c46ba8d59556aa10faf03479293236efea154d`, trùng gitlink Python-RVO2. |
| `human-jupedsim` | `jupedsim==1.4.2` cung cấp `SocialForceModel`, `AnticipationVelocityModel` và `CollisionFreeSpeedModelV2`. |
| `human-all` | Kết hợp hai extra trên. |
| `human-models` | Tên extra tương thích ngược, vẫn chỉ cài JuPedSim 1.4.2. |
| `sim` | Yêu cầu `mujoco>=4.0`, khóa MuJoCo, Core và Studio tại 4.0.0. |

Chạy lệnh từ gốc repo. Để kiểm tra môi trường chuẩn hiện tại, chạy chẩn đoán sau.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY /home/thaonhi/nav_ws/.venv/bin/python -I -B \
  scripts/diagnostics/check_human_env.py --strict --json
```

Cả chế độ mặc định và `--strict` trả 0 khi cả năm phép thử đạt, 1 khi một phép thử thiếu dependency, lỗi API hoặc không tiến được. `--strict` kiểm thêm metadata phiên bản JuPedSim/RVO2 và đối chiếu phiên bản distribution MuJoCo với engine. `--json` ghi origin của module, tag wheel nếu có, chữ ký API đọc được và số đo smoke. Constructor Cython của `PyRVOSimulator` không cung cấp chữ ký cho `inspect.signature`. Constructor vẫn được gọi và kiểm định bằng một bước mô phỏng. [Chữ ký Cython](https://github.com/sybrenstuvel/Python-RVO2/blob/c2c46ba8d59556aa10faf03479293236efea154d/src/rvo2.pyx#L78)

## Cài theo lockfile

uv 0.13.7 dựng RVO2 từ nguồn bằng Cython 3.1.4, setuptools 80.9.0, CMake 3.28.3 và GCC/G++ 13.3.0 trên máy kiểm định. GitHub cung cấp nguồn theo SHA đã khóa và PyPI cung cấp build dependency. Hai chỉ mục PyPI mang tên `pyrvo2` và `rvo2` đều trả 404 trong lần audit. `tool.uv.sources` chỉ có hiệu lực với uv. [Nguồn Python-RVO2](https://github.com/sybrenstuvel/Python-RVO2/tree/c2c46ba8d59556aa10faf03479293236efea154d) · [Khai báo dependency](../../pyproject.toml)

Với venv repo đã có `dev`, `sim` và JuPedSim như lúc audit, dùng lệnh sau để thêm riêng binding RVO2. Có thể thêm `--dry-run` để xem kế hoạch trước. Lần kiểm định đã chạy offline sau khi cache được chuẩn bị trong bước khóa dependency.

```bash
env -u VIRTUAL_ENV -u UV_PROJECT_ENVIRONMENT -u PYTHONPATH uv sync \
  --locked --inexact --python /home/thaonhi/nav_ws/.venv/bin/python \
  --extra human-orca --only-install-package pyrvo2
```

Để kiểm tra cả bộ dependency đã cài với lockfile, dùng `uv sync --locked --inexact --check --no-install-project --extra dev --extra sim --extra human-all` với cùng interpreter. `humans/requirements*.txt` thuộc demo Quaternius và có bộ pin riêng. [Yêu cầu demo](../../humans/requirements.txt)

## Cài wheel cục bộ và hoàn tác

Nếu cần dùng pip, wheel đã dựng trong lần audit nằm tại đường dẫn sau. Chỉ dùng wheel đã kiểm SHA-256 và đúng tag `cp312-cp312-linux_x86_64`. Lệnh pip đã được kiểm bằng `--dry-run --ignore-installed` trên runtime chuẩn. Lần cài thật dùng uv với cùng wheel.

```bash
/home/thaonhi/nav_ws/.venv/bin/python -m pip install --no-index --no-deps \
  /home/thaonhi/nav_ws/runs/task_00_env_audit/wheelhouse/pyrvo2-0.0.0-cp312-cp312-linux_x86_64.whl
```

SHA-256 của wheel trong lần kiểm định là `46a2fe36d8eb51230969f1dc1828860738f0c0f45b27d73fc10346d060dcaf51`. Nếu cần hoàn tác lần cài đã kiểm định, gỡ đúng gói đã thêm bằng `/home/thaonhi/nav_ws/.venv/bin/python -m pip uninstall pyrvo2`, rồi chạy `-m pip check` với cùng interpreter. Lệnh hoàn tác dành cho trường hợp cần quay về trạng thái trước audit.

Các phép smoke kiểm tra dependency và ABI với hình học nhỏ. Điều khiển người trong runtime chung, cách biểu diễn robot MuJoCo khi tương tác với JuPedSim và chất lượng quỹ đạo thuộc các cổng tích hợp sau. [Báo cáo TASK 00](../reports/human_models/TASK_00_REPORT.md)
