# TASK 05 · JuPedSim SocialForceModel

**Trạng thái · BLOCKED_CAPABILITY.** TASK 04 vẫn bị chặn ở khả năng đồng bộ robot động. Wheel JuPedSim 1.4.2 có hai lớp SFM yêu cầu và cho ghi vận tốc nội bộ SFM. Pose agent chỉ đọc. Probe CFM v1 tái hiện `switch_geometry` từ chối footprint phủ agent hoặc waypoint. Các kiểm tra dưới đây xác nhận blocker, chưa nghiệm thu backend SFM. [Báo cáo TASK 04](TASK_04_REPORT.md) · [API SFM và giới hạn policy](../../backends/SFM_JUPEDSIM.md)

## Checkout và hồ sơ

- Nhánh `mujoco-hummodel`, HEAD trước/sau `16ad6caef80d75fdb3881a73e721eb0dcb3382e3`. Upstream local lệch 0/0. Working tree đã dirty với TASK 04 và `orca_progress_tools/` trước khi bắt đầu. Index sạch. TASK 05 chỉ thêm hai tài liệu này và cập nhật một mục trong `PROGRESS.md`.
- `/tmp/codex_human_model/00_MASTER_RULES.md` vắng mặt. Dùng MASTER đầy đủ từ attachment TASK 05. SHA-256 của `HUMAN_MODEL_FINAL.md` trong working tree và HEAD cùng là `a6848d5ded183ebe046d529a832d93000c44118344bdda487919784df8716030`. [Nguồn nghiên cứu](../../research/HUMAN_MODEL_FINAL.md)
- Đã đọc nguồn, AGENTS, README, ROADMAP, ADR-0001, các thư mục `docs/architecture/`, `docs/protocols/`, `docs/schemas/`, `pyproject.toml`, pin JuPedSim trong `uv.lock` và đối chiếu mã bridge, registry, báo cáo cùng test TASK 04.
- Runtime `/home/thaonhi/nav_ws/.venv/bin/python`, CPython 3.12.3, JuPedSim 1.4.2 với extension native cp312 x86-64, MuJoCo 4.0.0 và Shapely 2.2.0.

## Kiểm định đã chạy

| Lệnh | Exit | Kết quả |
| --- | --- | --- |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -B -` với probe API inline | 0 | Xác minh chữ ký model, agent và journey, quyền ghi các property, trạng thái gate và registry. Log `runs/task_05_sfm/native-api.log`. |
| `env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -B -m pytest -q -s tests/unit/test_jupedsim_bridge.py tests/simulation/test_jupedsim_projection.py tests/integration/test_jupedsim_bridge_runtime.py` | 0 | 17 test bridge đạt. Probe robot động xác nhận blocker trên MuJoCo và CFM v1 thật. Log `runs/task_05_sfm/bridge-gate.log`. |
| `.venv/bin/ruff check src/narrow_nav tests --output-format concise` | 0 | Lint mã sản phẩm và test đạt. |
| `env -u PYTHONPATH PIP_NO_CACHE_DIR=1 .venv/bin/python -I -m pip check` | 0 | Dependency trong interpreter cô lập đạt. |
| `git diff --check` và `git status --short --branch` | 0 | Không lỗi khoảng trắng. Xác nhận checkout và phạm vi. |

Probe lỗ di động từ pose robot thật có sai số footprint lấy mẫu tối đa **0,034422 m** ở lưới đo 50 ms. JuPedSim từ chối lỗ phủ agent và phủ waypoint, iteration count giữ nguyên khi bị từ chối. Sai số và quỹ đạo trong log thuộc CFM v1. [Giải thích năng lực API](../../backends/JUPEDSIM_API_COMPATIBILITY.md)

## Deliverable và cổng mở lại

- Đã lưu [kiểm chứng API và profile SFM](../../backends/SFM_JUPEDSIM.md), báo cáo này và [tiến độ](PROGRESS.md). Sai khác đã được theo dõi tại `hm_jps_001`. Ngân sách và ngưỡng nghiên cứu giữ theo nguồn. [Sổ sai khác](../../research/HUMAN_MODEL_DEVIATIONS.md)
- Backend `jupedsim_sfm.py`, YAML `sfm_jupedsim.yaml` và suite nghiệm thu SFM đang chờ TASK 04. Free walking, đối đầu, vượt bên, tường/cửa, robot stop/advance, replay, vận tốc/gia tốc, GT/contact SFM và đồ thị chuỗi thời gian chưa được thực hiện. Dữ liệu probe CFM v1 không được dùng làm bằng chứng SFM.
- Cổng kế tiếp là chứng minh cách ghép pose và vận tốc robot MuJoCo vào tương tác, xử lý footprint chồng lấn và giữ nhịp bridge. Sau đó mới đăng ký `sfm_jupedsim` và kiểm định một episode native.

## Kiểm tra cổng lại trong TASK 05R

**BLOCKED_UPSTREAM_API.** TASK 04R dùng đúng `SocialForceModel` và `SocialForceModelAgentParameters` của wheel 1.4.2 trong probe đo với MuJoCo thật. Robot trong probe bị tích phân như agent JuPedSim sau khi nhận pose/vận tốc đo ở mỗi boundary. Sai lệch này chặn dùng kết quả làm nghiệm thu backend. Cổng 05R vẫn chờ API external và rebase. [Bằng chứng 04R](TASK_04R_REPORT.md) · [Đề xuất upstream](../../decisions/ADR-0007-jupedsim-external-actor.md)
