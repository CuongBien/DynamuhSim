# Ma trận môi trường TASK 00

Máy kiểm định chạy Ubuntu 24.04.5 LTS, Linux x86_64, glibc 2.39. Các phép import dùng interpreter tuyệt đối với `-I -B` để cách ly `PYTHONPATH` của phiên shell. Phiên kiểm định có đường dẫn ROS trong `PYTHONPATH` và có `DISPLAY=:1`. Lệnh smoke bỏ cả `DISPLAY` và `WAYLAND_DISPLAY`.

## Interpreter

| ID | Executable | Python và ABI | `pip check` | Mục đích |
| --- | --- | --- | --- | --- |
| Repo | `/home/thaonhi/nav_ws/.venv/bin/python` | CPython 3.12.3, `cpython-312-x86_64-linux-gnu`, base `/usr` | PASS (0) trước và sau | Runtime chuẩn duy nhất cho ORCA, SFM, AVM và CSM |
| ai12 | `/home/thaonhi/miniconda3/envs/ai12/bin/python` | CPython 3.12.14, `cpython-312-x86_64-linux-gnu` | PASS (0) | Đối chiếu binding ORCA đã có |
| System | `/usr/bin/python3` | CPython 3.12.3, `cpython-312-x86_64-linux-gnu` | FAIL (1), `pynacl 1.5.0 requires cffi` | Python gốc của venv repo |
| Conda base | `/home/thaonhi/miniconda3/bin/python` | CPython 3.14.7, `cpython-314-x86_64-linux-gnu` | PASS (0) | Đối chiếu ABI khác |
| venv ngoài repo | `/home/thaonhi/mujoco_ws/.venv/bin/python` | Venv không tồn tại | Chưa chạy | Không có runtime để thử |
| SocNavGym cũ | `/home/thaonhi/nav_ws/experiments/social_navigation/.venv-socnavgym/bin/python` | Chỉ thấy thư mục `lib/`, thiếu `bin/` và `pyvenv.cfg` | Chưa chạy | Dấu vết môi trường thí nghiệm cũ |

`uv 0.13.7` tại `/home/thaonhi/.local/bin/uv` chỉ phát hiện Python hệ thống khi chạy `UV_NO_CACHE=1 uv python list --only-installed`. Không tìm thấy Python do uv quản lý. `conda env list` thấy `base` và `ai12`.

## Dependency và import

`ML27/28` là hai tag `cp312-cp312-manylinux_2_27_x86_64` và `cp312-cp312-manylinux_2_28_x86_64`. `ML24/28` dùng `manylinux_2_24_x86_64` và `manylinux_2_28_x86_64` với cùng ABI. `L312` là `cp312-cp312-linux_x86_64`. Dấu **FAIL** trong bảng chỉ kết quả import thiếu module trên interpreter tương ứng.

| Phân phối → import | Repo | ai12 | System | Conda base | Vai trò |
| --- | --- | --- | --- | --- | --- |
| `mujoco` → `mujoco` | 4.0.0, `py3-none-any`, PASS | FAIL | FAIL | FAIL | API Python và gói điều phối |
| `mujoco-core`, `mujoco-studio` | Cả hai 4.0.0, ML27/28. Binding và engine 4.0.0 chạy qua `mujoco` | Thiếu | Thiếu | Thiếu | Binary MuJoCo và Studio. Smoke chỉ dùng binding vật lý |
| `jupedsim` → `jupedsim` | 1.4.2, ML27/28, PASS | FAIL | FAIL | FAIL | SFM, AVM, CSM |
| `pyrvo2` → `rvo2` | 0.0.0, L312, FAIL trước cài, PASS sau cài | 0.0.0, L312, PASS | FAIL | FAIL | ORCA qua `PyRVOSimulator` |
| `numpy` → `numpy` | 2.5.4, ML27/28, PASS | 2.5.3, ML27/28, PASS | 1.26.4, L312, PASS | FAIL | Mảng số cho MuJoCo, JuPedSim và Quaternius |
| `shapely` → `shapely` | 2.2.0, ML24/28, PASS | 2.2.0, ML24/28, PASS | FAIL | FAIL | Dependency hình học của JuPedSim |
| `scipy` → `scipy` | FAIL | 1.18.1, ML27/28, PASS | 1.11.4, L312, PASS | FAIL | Kiểm kê tương thích. Bộ smoke và các extra mới không yêu cầu SciPy |

`import pyrvo2` trả `ModuleNotFoundError` trên cả bốn interpreter đã kiểm kê. Tên `rov2` trong yêu cầu được hiểu là **RVO2**. Bản binding đã xác minh có distribution `pyrvo2` và import `rvo2`. [Khai báo extension và distribution](https://github.com/sybrenstuvel/Python-RVO2/blob/c2c46ba8d59556aa10faf03479293236efea154d/setup.py)

Trên CPython 3.12, wheel MuJoCo và JuPedSim đã cài có tag phù hợp với glibc 2.39, đúng nhánh NumPy 2.5.4 trong lockfile. Kiểm tra import, bước native, `pip check` và test Quaternius đều chạy thành công với tổ hợp này. Extension RVO2 tự dựng liên kết `libstdc++`, `libm`, `libgcc_s` và `libc`. Mã dựng không lấy header hay link NumPy. Wheel có tag Linux x86_64 cục bộ nên chỉ xác nhận trên máy và ABI đã thử. [Lockfile](../../uv.lock) · [Hướng dẫn cài](HUMAN_MODELS_ENV.md) · [Báo cáo nghiệm thu](../reports/human_models/TASK_00_REPORT.md)
