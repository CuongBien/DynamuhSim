# NarrowNav Research

Dự án nghiên cứu dự đoán quỹ đạo di chuyển của người để cải thiện lập kế hoạch đường đi cho robot tự hành trong không gian hẹp. Nghiên cứu bắt đầu từ các tình huống như hành lang, cửa hẹp và giao cắt khuất tầm nhìn. MuJoCo là engine duy nhất để mô phỏng chuyển động và cảm biến.

## Trạng thái

**Hiện tại.** Runtime headless chạy với robot vi sai hoặc mecanum. Entry point smoke hỗ trợ `humans=scripted` và baseline `humans=orca` qua cùng limiter và mocap. Caller dùng `MuJoCoEngine()` có thể tái hiện profile người đứng yên. Các slot cảm biến vẫn trả `missing`. [Runtime hiện tại](docs/architecture/05_MUJOCO_RUNTIME.md) · [ORCA và lệnh chạy](docs/backends/ORCA_PURE.md)

**Việc tiếp theo.** Nối bridge JuPedSim, rồi kiểm từng model. Thu dataset hành vi sau khi cảm biến và đường ghi/đọc episode đã qua kiểm định. [Tiến độ mô hình người](docs/reports/human_models/PROGRESS.md) · [Giao thức hành vi](docs/protocols/HUMAN_BEHAVIOR_PROTOCOL.md)

## Cài đặt và kiểm tra

Chạy từ thư mục gốc của repository với Python 3.11 trở lên. Phần mô phỏng yêu cầu MuJoCo 4.0 trở lên qua extra `sim`.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e '.[dev,sim]'
env -u DISPLAY -u WAYLAND_DISPLAY pytest -q
ruff check .
```

Nếu nâng cấp một `.venv` đang dùng MuJoCo 3.x, kích hoạt môi trường đó và chạy `python -m pip uninstall -y mujoco` trước khi cài lại `'.[dev,sim]'`. MuJoCo 4.0 tách binding và Studio thành các gói riêng. [Ghi chú nâng cấp MuJoCo 4.0](https://mujoco.readthedocs.io/en/latest/changelog.html#version-4-0-0-november-6-2026)

Để compose, kiểm tra và in cấu hình đã resolve, truyền override nhóm hoặc trường theo cú pháp Hydra.

```bash
narrow-nav-config worlds=corridor_090 scenarios=corridor_090 humans=scripted \
  seed=123 simulation.control_dt_ns=40000000
```

Lệnh trên resolve YAML cho hành lang rộng 0,90 m, bước điều khiển 40 ms, bước vật lý 2 ms và ngân sách episode 20 s. Dùng `--config-dir` khi gọi từ ngoài repository. Chuyển stdout vào `runs/<run_id>/resolved.yaml` để giữ bản YAML đã resolve. Tính SHA-256 trên chính các byte đã lưu để gắn vào metadata.

Trong cấu hình mẫu, `initial_pose` là `[x, y, yaw]`, `goal` là `[x, y]` và cả hai dùng frame `world`. Chiều dài theo X và chiều rộng thông thủy theo Y được đo từ tâm hành lang, với tọa độ tính bằng mét và yaw bằng radian.

`blind_corner` có một hốc nhường đường trước góc khuất. `passing_bay` là bài toán hành lang thẳng có hốc tránh. `build_world()` dựng MJCF từ cấu hình đã resolve. `inspect_world()` kiểm tra hình học đã biên dịch. [Báo cáo mô hình MuJoCo](docs/architecture/04_MUJOCO_MODELING.md)

Để xem scene trong cửa sổ MuJoCo, chạy lệnh sau trong phiên desktop có màn hình.

```bash
python -m narrow_nav.simulation.mujoco.view_world \
  worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot
```

Thêm cặp override world/scenario ở trên để đổi môi trường. Viewer mở từ trên xuống tại trạng thái ban đầu, tự căn theo kích thước world và ẩn hai bảng điều khiển. Cuộn chuột để zoom, kéo để đổi góc nhìn. Dùng Tab hoặc Shift+Tab để hiện lại các bảng.

Dùng `--focus-robot` để xem cận LiDAR YDLIDAR X3 và trụ lắp camera RealSense D435i trên mecanum.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world --focus-robot robots=mecanum_robot
```

### World chuyển từ DynamuhSim

Các world `corridor_070`, `corridor_090`, `corridor_120`, các mức `hospital_*`, `school_*` và `arena_*` dùng chung `MuJoCoEngine` với các world tham số. Chọn cặp world/scenario cùng tên, hoặc dùng `worlds=school_floor scenarios=school_floor_baseline` để chạy bản trường học với robot duy nhất.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world \
  worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot
env -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -m narrow_nav.simulation.smoke \
  --ticks 40 --vx 0.35 worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot
```

Lệnh đầu mở trạng thái ban đầu để kiểm tra bố cục. Lệnh thứ hai điều khiển bánh robot qua runtime và chạy 1.000 bước vật lý. Với vòng điều khiển riêng, nạp cùng cấu hình vào `MuJoCoEngine.load()`, gọi `reset()` rồi truyền `RobotAction` có timestamp hiện tại vào `step()`. Dùng `experiments.duration_ns=90000000000` nếu cần ngân sách 90 s cho các hành trình dài. [Nguồn hình học, bảng world và giới hạn](docs/architecture/04_MUJOCO_MODELING.md#world-chuyển-từ-dynamuhsim)

### Ngữ cảnh bề rộng hành lang

Dùng `corridor_070`, `corridor_090` hoặc `corridor_120` để chọn một trong ba hành lang thẳng 30 m có bề rộng cố định. Ba mức hospital dùng cùng mặt bằng hành lang 3 m, 12 ghế chờ ngoài phòng và 10 hốc nhường đường xen kẽ hai bên.

Với `blind_corner`, `passing_bay` và các world `school_*`, thêm `corridor_width=w070`, `corridor_width=w090` hoặc `corridor_width=w120` để chọn thông thủy 0,70 m, 0,90 m hoặc 1,20 m. Bỏ override để dùng kích thước gốc.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world \
  worlds=school_floor scenarios=school_floor robots=mecanum_robot corridor_width=w070
```

[Lệnh mở và kiểm tra từng world](docs/MUJOCO_WORLD_COMMANDS.md) · [Ma trận và cách đo](docs/architecture/04_MUJOCO_MODELING.md#ngữ-cảnh-bề-rộng-hành-lang)

### Hốc nhường đường trong trường học

Các hốc màu xanh có ba vách collision và miệng riêng mở ra hành lang. Ở cấu hình gốc mỗi hốc rộng thông thủy 0,90 m và sâu 0,95 m. Các bản school lớn có 13 hốc phân bố trên sáu đoạn hành lang. Bản `school_arena_1` có 8 hốc cách nhau 4 m.

```bash
# Toàn cảnh các hốc, với một robot để dễ quan sát.
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world \
  worlds=school_floor scenarios=school_floor_baseline robots=mecanum_robot

# Xem cận robot đỗ trong hốc đầu tiên và hướng ra miệng hốc.
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world --focus-bay a_01 \
  worlds=school_floor scenarios=school_floor_baseline robots=mecanum_robot \
  'scenarios.robot.initial_pose=[-12.55,-9.10,1.5707963267948966]'
```

### Mẫu người Quaternius

Runtime và viewer dùng mặc định mesh nam/nữ Quaternius đã đóng gói trong checkout. Người có animation đứng chờ và giữ pose XY/yaw của body vật lý. Hình trụ collision nằm trong nhóm hiển thị ẩn. Dùng `simulation.human_visuals=proxy` để xem lại hình trụ minh họa hoặc chạy cấu hình gọn hơn khi chỉ đo vật lý.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world --focus-human human_01 \
  worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot
```

Dùng chuột để xoay và zoom quanh người. Hãy bỏ `--focus-human` để mở toàn cảnh. `arena_rvo2` có cả hai mẫu nam/nữ. Trong hospital, dùng `--focus-human hospital_waiting_n_01_1` để xem một người ngồi chờ. [Mô hình render, collision và nguồn tài nguyên](docs/architecture/04_MUJOCO_MODELING.md#hình-ảnh-người-quaternius)

Chạy smoke test headless 1.000 bước cho từng drivetrain.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY python -m narrow_nav.simulation.smoke --ticks 40 --vx 0.35
env -u DISPLAY -u WAYLAND_DISPLAY python -m narrow_nav.simulation.smoke --ticks 40 --vy 0.25 robots=mecanum_robot
```

`MuJoCoEngine` hỗ trợ `load`, `reset`, `step`, `stop`, đọc observation/ground truth/metadata và `close`. Depth, LiDAR, IMU và odometry được đánh dấu chưa khả dụng. [API runtime và model mecanum](docs/architecture/05_MUJOCO_RUNTIME.md)

## Tài liệu

| Nội dung | Tài liệu |
| --- | --- |
| Trạng thái triển khai và thứ tự công việc | [Lộ trình](docs/ROADMAP.md), [Cấu trúc checkout](Architect.md) |
| Phạm vi hệ thống, quyền sở hữu và luồng dữ liệu | [Tổng quan](docs/architecture/00_SYSTEM_OVERVIEW.md), [Ranh giới module](docs/architecture/01_MODULE_BOUNDARIES.md), [Luồng dữ liệu](docs/architecture/02_DATA_FLOW.md), [Quy tắc phụ thuộc](docs/architecture/03_DEPENDENCY_RULES.md) |
| Dữ liệu giữa các module | [State và action](docs/schemas/STATE_AND_ACTION_SCHEMA.md), [Episode](docs/schemas/EPISODE_SCHEMA.md) |
| Giả thuyết và đối chứng | [Mục tiêu](docs/research/RESEARCH_OBJECTIVES.md), [Câu hỏi](docs/research/RESEARCH_QUESTIONS.md), [Ma trận baseline](docs/research/BASELINE_MATRIX.md) |
| Thiết kế thí nghiệm và điều kiện nghiệm thu | [Hành vi người](docs/protocols/HUMAN_BEHAVIOR_PROTOCOL.md), [Kiểm định mô phỏng](docs/protocols/SIMULATION_VALIDATION_PROTOCOL.md) |
| Quyết định và thứ tự triển khai | [ADR MuJoCo](docs/decisions/ADR-0001-mujoco-only.md), [Lộ trình](docs/ROADMAP.md) |
| Làm việc trong repository | [Quy tắc cho Codex](AGENTS.md), [Hướng dẫn đóng góp](CONTRIBUTING.md) |
