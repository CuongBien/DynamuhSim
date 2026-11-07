# NarrowNav Research

Dự án nghiên cứu dự đoán quỹ đạo di chuyển của người để cải thiện lập kế hoạch đường đi cho robot tự hành trong không gian hẹp. Nghiên cứu bắt đầu từ các tình huống như hành lang, cửa hẹp và giao cắt khuất tầm nhìn. MuJoCo là engine duy nhất để mô phỏng chuyển động và cảm biến.

## Trạng thái

TASK 03 cung cấp runtime episode headless cho hành lang thẳng, góc khuất và hốc tránh, với robot vi sai hoặc mecanum và người mocap đứng yên. Package Python `narrow_nav` giữ các hợp đồng dữ liệu v1. Điểm vào của Hydra là `configs/config.yaml`. Hai gitlink có sẵn dưới `experiments/social_navigation/vendor/` nằm ngoài package.

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
narrow-nav-config worlds=corridor scenarios=head_on humans=scripted \
  seed=123 worlds.width_m=2.8 simulation.control_dt_ns=40000000
```

Lệnh trên chọn hành lang rộng 2,8 m, bước điều khiển 40 ms, bước vật lý 2 ms và thời lượng episode 20 s. Dùng `--config-dir` khi gọi từ ngoài repository. Chuyển stdout vào `runs/<run_id>/resolved.yaml` để giữ bản YAML đã resolve. Tính SHA-256 trên chính các byte đã lưu để gắn vào metadata.

Trong cấu hình mẫu, `initial_pose` là `[x, y, yaw]`, `goal` là `[x, y]` và cả hai dùng frame `world`. Chiều dài theo X và chiều rộng thông thủy theo Y được đo từ tâm hành lang, với tọa độ tính bằng mét và yaw bằng radian.

Để thử cấu hình góc khuất hoặc hốc tránh, dùng cặp override `worlds=blind_corner scenarios=blind_corner` hoặc `worlds=passing_bay scenarios=passing_bay`. `build_world()` dựng MJCF từ cấu hình đã resolve. `inspect_world()` kiểm tra hình học đã biên dịch. [Báo cáo mô hình MuJoCo](docs/architecture/04_MUJOCO_MODELING.md)

Để xem scene trong cửa sổ MuJoCo, chạy lệnh sau trong phiên desktop có màn hình.

```bash
python -m narrow_nav.simulation.mujoco.view_world robots=mecanum_robot
```

Thêm cặp override world/scenario ở trên để đổi môi trường. Viewer mở từ trên xuống tại trạng thái ban đầu, tự căn theo kích thước world và ẩn hai bảng điều khiển. Cuộn chuột để zoom, kéo để đổi góc nhìn. Dùng Tab hoặc Shift+Tab để hiện lại các bảng.

### World chuyển từ DynamuhSim

Các world `corridor_070`, `corridor_090`, `corridor_120`, các mức `hospital_*`, `school_*` và `arena_*` dùng chung `MuJoCoEngine` với các world tham số. Chọn cặp world/scenario cùng tên, hoặc dùng `worlds=school_floor scenarios=school_floor_baseline` để chạy bản trường học với robot duy nhất.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world \
  worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot
env -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -m narrow_nav.simulation.smoke \
  --ticks 40 --vx 0.35 worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot
```

Lệnh đầu mở trạng thái ban đầu để kiểm tra bố cục. Lệnh thứ hai điều khiển bánh robot qua runtime và chạy 1.000 bước vật lý. Với vòng điều khiển riêng, nạp cùng cấu hình vào `MuJoCoEngine.load()`, gọi `reset()` rồi truyền `RobotAction` có timestamp hiện tại vào `step()`. Dùng `experiments.duration_ns=90000000000` nếu cần ngân sách 90 s cho các hành trình dài. [Nguồn hình học, bảng world và giới hạn](docs/architecture/04_MUJOCO_MODELING.md#world-chuyển-từ-dynamuhsim)

### Mẫu người Quaternius

Runtime và viewer dùng mặc định mesh nam/nữ Quaternius đã đóng gói trong checkout. Người có animation đứng chờ và giữ pose XY/yaw của body vật lý. Hình trụ collision nằm trong nhóm hiển thị ẩn. Dùng `simulation.human_visuals=proxy` để xem lại hình trụ minh họa hoặc chạy cấu hình gọn hơn khi chỉ đo vật lý.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world --focus-human human_01 \
  worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot
```

Dùng chuột để xoay và zoom quanh người. Hãy bỏ `--focus-human` để mở toàn cảnh. `arena_rvo2` có cả hai mẫu nam/nữ. Trong `hospital_easy`, có thể lấy nét vào `hospital_room_05_seated_1`. [Mô hình render, collision và nguồn tài nguyên](docs/architecture/04_MUJOCO_MODELING.md#hình-ảnh-người-quaternius)

Chạy smoke test headless 1.000 bước cho từng drivetrain.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY python -m narrow_nav.simulation.smoke --ticks 40 --vx 0.35
env -u DISPLAY -u WAYLAND_DISPLAY python -m narrow_nav.simulation.smoke --ticks 40 --vy 0.25 robots=mecanum_robot
```

`MuJoCoEngine` hỗ trợ `load`, `reset`, `step`, `stop`, đọc observation/ground truth/metadata và `close`. Depth, LiDAR, IMU và odometry được đánh dấu chưa khả dụng. [API runtime và model mecanum](docs/architecture/05_MUJOCO_RUNTIME.md)

## Tài liệu

| Nội dung | Tài liệu |
| --- | --- |
| Phạm vi hệ thống, quyền sở hữu và luồng dữ liệu | [Tổng quan](docs/architecture/00_SYSTEM_OVERVIEW.md), [Ranh giới module](docs/architecture/01_MODULE_BOUNDARIES.md), [Luồng dữ liệu](docs/architecture/02_DATA_FLOW.md), [Quy tắc phụ thuộc](docs/architecture/03_DEPENDENCY_RULES.md) |
| Dữ liệu giữa các module | [State và action](docs/schemas/STATE_AND_ACTION_SCHEMA.md), [Episode](docs/schemas/EPISODE_SCHEMA.md) |
| Giả thuyết và đối chứng | [Mục tiêu](docs/research/RESEARCH_OBJECTIVES.md), [Câu hỏi](docs/research/RESEARCH_QUESTIONS.md), [Ma trận baseline](docs/research/BASELINE_MATRIX.md) |
| Thiết kế thí nghiệm và điều kiện nghiệm thu | [Hành vi người](docs/protocols/HUMAN_BEHAVIOR_PROTOCOL.md), [Kiểm định mô phỏng](docs/protocols/SIMULATION_VALIDATION_PROTOCOL.md) |
| Quyết định và thứ tự triển khai | [ADR MuJoCo](docs/decisions/ADR-0001-mujoco-only.md), [Lộ trình](docs/ROADMAP.md) |
| Làm việc trong repository | [Quy tắc cho Codex](AGENTS.md), [Hướng dẫn đóng góp](CONTRIBUTING.md) |
