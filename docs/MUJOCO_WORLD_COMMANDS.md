# Lệnh mở từng môi trường trong MuJoCo

Chạy từ thư mục gốc dự án trong phiên desktop có màn hình. Mỗi dòng lệnh mở một cửa sổ MuJoCo với robot mecanum. Viewer hiển thị vị trí xuất phát của robot và người. Cuộn chuột để zoom, kéo chuột để đổi góc nhìn. Đóng cửa sổ trước khi chạy lệnh tiếp theo.

Để lái robot và xem ảnh D435i đã xử lý, dùng danh sách teleop có đủ các world, kể cả nhóm arena. [Lệnh teleop D435i](MUJOCO_TELEOP_DEPTH.md#chọn-môi-trường)

```bash
cd ~/nav_ws
```

Danh sách gồm ba hành lang cố định, ba mức bệnh viện và 21 tổ hợp bề rộng cho `blind_corner`, `passing_bay` và các scenario trường học. Các world `arena_obstacle`, `arena_dataset` và `arena_rvo2` nằm ngoài danh sách. Bề rộng hành lang đo giữa hai mặt tường ở đoạn trống. Chỉ các mục có `corridor_width` mới dùng preset. [Chi tiết về hình học và các bố cục dùng chung](architecture/04_MUJOCO_MODELING.md#ngữ-cảnh-bề-rộng-hành-lang)

## Góc khuất và hốc tránh

### `blind_corner`

Hành lang chữ L với hai nhánh cùng bề rộng. Hốc nhường đường có miệng rộng 1,20 m và sâu 1,10 m, mở về phía +Y trước góc rẽ. Tâm miệng hốc ở X = 1,20 m.

```bash
# 0,70 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=blind_corner scenarios=blind_corner robots=mecanum_robot corridor_width=w070
# 0,90 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=blind_corner scenarios=blind_corner robots=mecanum_robot corridor_width=w090
# 1,20 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=blind_corner scenarios=blind_corner robots=mecanum_robot corridor_width=w120
```

### `passing_bay`

Hành lang có hốc tránh rộng thêm 1,00 m.

```bash
# 0,70 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=passing_bay scenarios=passing_bay robots=mecanum_robot corridor_width=w070
# 0,90 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=passing_bay scenarios=passing_bay robots=mecanum_robot corridor_width=w090
# 1,20 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=passing_bay scenarios=passing_bay robots=mecanum_robot corridor_width=w120
```

## Hành lang thẳng 30 m

Hai tường dọc và hai tường đầu tạo một hành lang có bề rộng cố định theo ID.

### `corridor_070` · 0,70 m

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=corridor_070 scenarios=corridor_070 robots=mecanum_robot
```

### `corridor_090` · 0,90 mnày là theo hướng computer vision hướng này ở vn khá là kén tìm mấy kcn chắc là có bên ngoài đa số là LLM thôi


```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=corridor_090 scenarios=corridor_090 robots=mecanum_robot
```

### `corridor_120` · 1,20 m

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=corridor_120 scenarios=corridor_120 robots=mecanum_robot
```

## Bệnh viện · hành lang 3,00 m

Ba mức mật độ dùng cùng mặt bằng với 20 cửa phòng, 12 ghế chờ đặt ngoài cửa và 10 hốc nhường đường riêng. Mỗi hốc rộng 1,30 m, sâu 1,20 m. Ghế và người ngồi làm giảm khoảng trống cục bộ. Chọn mật độ qua cặp world/scenario, với 10 người ngồi cố định ở cả ba mức.

### `hospital_easy` · 6 người trong scenario

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=hospital_easy scenarios=hospital_easy robots=mecanum_robot
```

### `hospital_medium` · 12 người trong scenario

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=hospital_medium scenarios=hospital_medium robots=mecanum_robot
```

### `hospital_hard` · 20 người trong scenario

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=hospital_hard scenarios=hospital_hard robots=mecanum_robot
```

Mở cận hốc đầu tiên phía bắc, gần X = −23,50 m.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world --focus-bay n_01 worlds=hospital_hard scenarios=hospital_hard robots=mecanum_robot
```

Mở cận một người ngồi chờ trên ghế phía hành lang.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world --focus-human hospital_waiting_n_01_1 worlds=hospital_hard scenarios=hospital_hard robots=mecanum_robot
```

## Trường học

### `school_arena`

Bố cục trường lớn, 8 người trong scenario.

```bash
# 0,70 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_arena scenarios=school_arena robots=mecanum_robot corridor_width=w070
# 0,90 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_arena scenarios=school_arena robots=mecanum_robot corridor_width=w090
# 1,20 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_arena scenarios=school_arena robots=mecanum_robot corridor_width=w120
```

### `school_arena_1`

Hành lang thẳng với 8 hốc nhường đường. Scenario chỉ có robot.

```bash
# 0,70 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_arena_1 scenarios=school_arena_1 robots=mecanum_robot corridor_width=w070
# 0,90 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_arena_1 scenarios=school_arena_1 robots=mecanum_robot corridor_width=w090
# 1,20 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_arena_1 scenarios=school_arena_1 robots=mecanum_robot corridor_width=w120
```

### `school_arena_hunav`

Dùng cùng hình học với `school_arena`, giữ scenario riêng.

```bash
# 0,70 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_arena_hunav scenarios=school_arena_hunav robots=mecanum_robot corridor_width=w070
# 0,90 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_arena_hunav scenarios=school_arena_hunav robots=mecanum_robot corridor_width=w090
# 1,20 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_arena_hunav scenarios=school_arena_hunav robots=mecanum_robot corridor_width=w120
```

### `school_floor`

Bố cục trường lớn có cầu thang và 10 người trong scenario.

```bash
# 0,70 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_floor scenarios=school_floor robots=mecanum_robot corridor_width=w070
# 0,90 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_floor scenarios=school_floor robots=mecanum_robot corridor_width=w090
# 1,20 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_floor scenarios=school_floor robots=mecanum_robot corridor_width=w120
```

### `school_floor_baseline`

Cùng bố cục `school_floor`, chỉ có robot.

```bash
# 0,70 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_floor scenarios=school_floor_baseline robots=mecanum_robot corridor_width=w070
# 0,90 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_floor scenarios=school_floor_baseline robots=mecanum_robot corridor_width=w090
# 1,20 m
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world worlds=school_floor scenarios=school_floor_baseline robots=mecanum_robot corridor_width=w120
```

## Kiểm tra thêm ở `school_floor`

Mở cận hốc nhường đường A01 ở bề rộng 0,70 m.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.view_world --focus-bay a_01 worlds=school_floor scenarios=school_floor_baseline robots=mecanum_robot corridor_width=w070
```

Chạy 1.000 bước vật lý với lệnh tiến cố định trong cùng cấu hình. Lệnh này chạy headless và in kết quả ra terminal.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -m narrow_nav.simulation.smoke --ticks 40 --vx 0.35 worlds=school_floor scenarios=school_floor_baseline robots=mecanum_robot corridor_width=w070
```

## Kiểm tra cấu hình trước khi triển khai hành vi

Ví dụ xuất cấu hình đã resolve và chạy smoke headless cho `hospital_hard` với cùng seed. Chạy từ thư mục gốc dự án. `smoke.json` chứa thống kê kiểm tra runtime. Pilot dataset có điều kiện nghiệm thu riêng ở TASK 04C. [Lộ trình](ROADMAP.md)

```bash
mkdir -p runs/hospital_hard_seed42
.venv/bin/narrow-nav-config worlds=hospital_hard scenarios=hospital_hard robots=mecanum_robot seed=42 > runs/hospital_hard_seed42/resolved.yaml
env -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -m narrow_nav.simulation.smoke --ticks 40 --vx 0.35 worlds=hospital_hard scenarios=hospital_hard robots=mecanum_robot seed=42 > runs/hospital_hard_seed42/smoke.json
```

- Trong mã chạy thí nghiệm, dùng `compose_config(overrides=...)`, `MuJoCoEngine.load(config)`, `reset()` và `step(RobotAction)`. Controller robot nhận `Observation`. Dùng `GroundTruthState` qua kênh riêng cho nhãn và đánh giá.
- Người trong `scenarios.humans` có ID, spawn và goal theo hệ tọa độ `world`. Runtime hiện tại giữ họ tại spawn. Chuyển động theo goal cần backend được kết nối qua hợp đồng của dự án. Người ngồi chờ hospital thuộc `worlds.human_props`, là hình học collision tĩnh và không nằm trong `GroundTruthState.humans`.
- Các ID hospital vẫn được dùng sau khi đổi bố cục và mật độ. Dùng fingerprint model/MJCF từ `engine.get_metadata()` để phân biệt các phiên bản bố cục. Khi triển khai bộ ghi, lưu fingerprint, seed, cấu hình đã resolve và phiên bản MuJoCo theo manifest. [Schema episode](schemas/EPISODE_SCHEMA.md) · [Giao thức đối chứng](protocols/HUMAN_BEHAVIOR_PROTOCOL.md)
