# Lái robot bằng bàn phím và xem camera depth

`narrow_nav.simulation.mujoco.teleop_depth` mở một cửa sổ để lái robot mecanum bằng bàn phím trong một world đã cấu hình và xem trực tiếp camera depth D435i gắn trên robot. Công cụ dùng để kiểm tra bố cục, tầm nhìn camera và khoảng hở khi đi qua hành lang hẹp. Với `sensors=d435i`, cửa sổ hiển thị trực tiếp mẫu đã xử lý từ `Observation` và ảnh đối chiếu từ kênh kiểm định. [Mã nguồn](../src/narrow_nav/simulation/mujoco/teleop_depth.py) · [Runtime MuJoCo](architecture/05_MUJOCO_RUNTIME.md)

## Yêu cầu

- Chạy từ thư mục gốc dự án trong phiên desktop có màn hình, với `.venv` đã cài `'.[dev,sim]'` theo [README](../README.md#cài-đặt-và-kiểm-tra).
- Chọn `robots=mecanum_robot`. Chỉ profile này có camera `robot/depth_camera` (424 × 240, cao 0,50 m, hướng +X). `mobile_robot` không có camera nên lệnh sẽ dừng với thông báo lỗi.
- Không cần cài thêm gói. Cửa sổ dùng `glfw`, một phụ thuộc bắt buộc của gói `mujoco`.

## Chạy nhanh

```bash
cd ~/nav_ws
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i
```

Có thể bỏ `sensors=d435i` để xem depth render và RGB như trước. Ngoài lựa chọn sensor, lệnh dùng các giá trị mặc định dưới đây. Override cùng khóa do người dùng truyền sẽ thay giá trị mặc định tương ứng. Các khóa khác vẫn giữ mặc định.

| Khóa | Mặc định | Ghi chú |
| --- | --- | --- |
| `worlds` | `corridor_090` | Hành lang thẳng 30 m, rộng 0,90 m. |
| `scenarios` | `corridor_090` | Phải khớp `worlds.id` của world đã chọn. |
| `robots` | `mecanum_robot` | Profile duy nhất có camera depth. |
| `experiments.duration_ns` | `600000000000` | Ngân sách 600 s thay cho 20 s của `smoke`. |

## Chọn môi trường

Chọn **một dòng** bên dưới để mở môi trường trong phiên desktop. Mỗi lệnh dùng robot mecanum và hiển thị depth D435i đã xử lý ở ô trên, depth sạch cùng mốc ở ô dưới. Nhấn **V** để đổi ô dưới sang mask hoặc bản đồ sai số, **P** để giữ frame đang xem. [Toàn bộ phím điều khiển](#điều-khiển)

### Hành lang, góc khuất và hốc tránh

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=corridor scenarios=head_on
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=corridor_070 scenarios=corridor_070
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=corridor_090 scenarios=corridor_090
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=corridor_120 scenarios=corridor_120
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=blind_corner scenarios=blind_corner
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=passing_bay scenarios=passing_bay
```

### Bệnh viện

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=hospital_easy scenarios=hospital_easy
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=hospital_medium scenarios=hospital_medium
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=hospital_hard scenarios=hospital_hard
```

### Trường học

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=school_arena scenarios=school_arena
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=school_arena_1 scenarios=school_arena_1
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=school_arena_hunav scenarios=school_arena_hunav
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=school_floor scenarios=school_floor
# Cùng mặt bằng school_floor với riêng robot.
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=school_floor scenarios=school_floor_baseline
```

### Arena

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=arena_obstacle scenarios=arena_obstacle
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=arena_dataset scenarios=arena_dataset
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=arena_rvo2 scenarios=arena_rvo2
```

`arena_dataset` dùng hình học của `arena_obstacle` và cấu hình riêng cho vị trí xuất phát, đích và danh sách người. Robot xuất phát tại `scenarios.robot.initial_pose`. Người trong scenario hiện vẫn đứng tại spawn. [Lộ trình backend người](ROADMAP.md)

### Biến thể bề rộng

Thêm một trong ba override `corridor_width=w070`, `corridor_width=w090` hoặc `corridor_width=w120` để chọn bề rộng thông thủy tương ứng 0,70 m, 0,90 m hoặc 1,20 m. Áp dụng cho `blind_corner`, `passing_bay` và cả năm lệnh trường học ở trên. Bỏ override để dùng hình học gốc. Với ba hành lang thẳng 30 m, chọn trực tiếp world `corridor_070`, `corridor_090` hoặc `corridor_120`. [Cách đo bề rộng](architecture/04_MUJOCO_MODELING.md#ngữ-cảnh-bề-rộng-hành-lang)

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i \
  worlds=blind_corner scenarios=blind_corner corridor_width=w070
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i \
  worlds=school_floor scenarios=school_floor_baseline corridor_width=w090
```

### Đối chiếu các chế độ nhiễu

Dùng cùng cặp world/scenario và `seed` để xem ba chế độ. `sensors.noise.enabled=false` cho phép đo lý tưởng trong dải đo camera. `sensors.noise.hole_probability=0` giữ mô hình stereo và bỏ riêng các lỗ vuông tổng hợp. Các override này dùng được với mọi lệnh môi trường ở trên. [Tham số D435i và cách lưu báo cáo](MUJOCO_D435I_DEPTH.md)

```bash
# Mô hình D435i mặc định
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i seed=42
# Đối chứng lý tưởng
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i seed=42 sensors.noise.enabled=false
# Giữ nhiễu stereo và bỏ lỗ vuông
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i seed=42 sensors.noise.hole_probability=0
```

## Tùy chọn dòng lệnh

| Tùy chọn | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `--speed` | `0.5` | Tỷ lệ tốc độ ban đầu so với `robots.max_linear_speed_mps` và `robots.max_angular_speed_radps`, trong khoảng 0,1–1. |
| `--top-down` | Camera chase | Khởi động với góc nhìn từ trên xuống, cố định tại vị trí xuất phát để thấy robot dịch chuyển so với sàn. |
| `--depth-min` | `0.105` với D435i, `0.2` với preview | Khoảng cách nhỏ nhất được tô màu, đơn vị mét. |
| `--depth-max` | `6.0` với D435i, `5.0` với preview | Khoảng cách lớn nhất được tô màu, đơn vị mét. |
| `--config-dir` | `configs/` của repo | Thư mục cấu hình Hydra. |

Khi bật `sensors=d435i`, dải màu mặc định lấy từ khoảng đo của profile. Các cờ `--depth-min` và `--depth-max` chỉ chỉnh hiển thị.

Ví dụ, bắt đầu ở 30 % tốc độ và tô màu depth trong khoảng 0,3–3 m.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth --speed 0.3 --depth-min 0.3 --depth-max 3 \
  sensors=d435i worlds=hospital_medium scenarios=hospital_medium
```

## Điều khiển

Lệnh vận tốc tính trong frame robot. Nhả hết phím thì lệnh về zero và limiter giảm tốc theo gia tốc cho phép.

| Phím | Tác dụng |
| --- | --- |
| W / S hoặc ↑ / ↓ | Tiến / lùi (`vx`). |
| A / D | Đi ngang trái / phải (`vy`). Với robot vi sai, A/D quay tại chỗ. |
| Q / E hoặc ← / → | Quay trái / phải (`wz`). |
| Giữ Space | Gửi lệnh zero, kể cả khi đang giữ phím khác. |
| `-` / `=` | Giảm / tăng tốc độ 10 %, trong khoảng 10–100 %. |
| R | Reset episode về trạng thái ban đầu với cùng seed. |
| C | Đổi camera: `chase` bám sau robot, `track` bám vị trí nhưng giữ hướng nhìn, `free` tự do. |
| V | Đổi ô dưới giữa depth sạch, mask hợp lệ và sai số đo (với `sensors=d435i`). |
| P | Tạm dừng / chạy tiếp mô phỏng và giữ frame đang xem. |
| H | Hiện / ẩn bảng phím tắt. |
| Esc | Thoát. |

Bắt đầu kéo hoặc cuộn chuột trong khung scene bên trái. Kéo chuột trái để xoay, kéo chuột phải để dời tâm nhìn, cuộn lên để phóng to và cuộn xuống để thu nhỏ. Giữ Shift khi kéo để đổi hướng xoay hoặc mặt phẳng dời. Chế độ `chase` giữ góc xoay tương đối với hướng robot. Cả `chase` và `track` giữ độ lệch tâm nhìn khi bám theo robot.

Với `mecanum_robot` ở 50 %, lệnh tối đa là 0,30 m/s và 0,90 rad/s. Limiter của runtime giới hạn gia tốc 1,5 m/s² và 3,0 rad/s², cùng tốc độ quay bánh. Lệnh hiển thị là lệnh yêu cầu, còn twist robot là vận tốc đo từ mô phỏng.

Để kiểm tra riêng chuyển động ngang, mở arena với góc nhìn cố định từ trên rồi giữ A hoặc D khoảng một giây. `Command.vy` lần lượt dương và âm, còn `wz` bằng zero. Quan sát vị trí robot so với sàn và giá trị yaw trong HUD. Trong `corridor_090`, giữ A/D đến sát tường sẽ kết thúc episode theo điều kiện `robot_contact`. Nhấn R để thử lại.

```bash
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth --top-down \
  sensors=d435i robots=mecanum_robot worlds=arena_dataset scenarios=arena_dataset
```

Để đo riêng dịch chuyển ngang ở hai hướng trong một giây mô phỏng, so `displacement_world_m` và `yaw_rad` từ hai lệnh smoke sau.

```bash
for vy in 0.30 -0.30; do
  env -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/python -m narrow_nav.simulation.smoke \
    --ticks 20 --vy="$vy" robots=mecanum_robot worlds=arena_dataset scenarios=arena_dataset
done
```

## Đọc giao diện

| Vùng | Nội dung |
| --- | --- |
| Trái | Scene MuJoCo theo camera đang chọn. Góc dưới hiển thị thông tin giám sát. |
| Phải, trên | Với profile D435i, ảnh đo có nhiễu kèm số frame, thời gian capture, tỷ lệ pixel hợp lệ và khoảng cách gần nhất ở tâm. Ảnh depth tô màu. Gần là đỏ và vàng, xa dần là xanh lá rồi xanh dương. Pixel ngoài `[--depth-min, --depth-max]` hiển thị màu đen, gồm vật quá gần, quá xa và nền trống. Dòng `centre` là khoảng cách nhỏ nhất trong vùng 20 % ở tâm ảnh. |
| Phải, dưới | Với profile D435i, depth sạch cùng timestamp. Nhấn V để xem mask (trắng hợp lệ, đen invalid) hoặc sai số đo (xanh âm, trắng zero, đỏ dương, đen invalid). Chế độ xem trực tiếp dùng RGB ở ô này. |

| Trường | Ý nghĩa |
| --- | --- |
| `Scenario` | ID scenario đang chạy. |
| `Status` | `running`, `paused`, lý do kết thúc kèm gợi ý nhấn R, hoặc `failed: <lỗi>`. |
| `Time` | Thời gian mô phỏng và tick điều khiển. |
| `Command` | Lệnh yêu cầu `vx`, `vy` (m/s) và `wz` (rad/s). |
| `Robot twist` | Vận tốc robot đo được trong frame robot. |
| `Pose` | `x`, `y` (m) và `yaw` (độ) trong frame `world`. |
| `Clearance` | Khoảng hở phẳng từ footprint robot đến tường gần nhất và người gần nhất. |
| `Speed` | Tỷ lệ tốc độ hiện tại và tổng số sự kiện contact từ lần reset trước. |
| `Camera` | Chế độ camera hiện tại. |
| `Sim / real` | Tỷ lệ thời gian mô phỏng trên thời gian thực, trung bình mỗi giây. |

## Episode và cách kết thúc

Mỗi tick điều khiển gọi `SimulationLifecycle.step(RobotAction)`, nên dùng limiter, timing, contact và điều kiện kết thúc của runtime. Khi episode kết thúc, mô phỏng dừng cho đến khi nhấn R.

| Lý do | Khi nào xảy ra | Cách điều chỉnh |
| --- | --- | --- |
| `robot_contact` | Robot chạm tường hoặc người khi `experiments.terminate_on_contact=true`. | Thêm `experiments.terminate_on_contact=false` để tiếp tục sau va chạm. Số contact vẫn được đếm. |
| `goal_reached` | Tâm robot cách goal của scenario không quá `experiments.goal_tolerance_m`, mặc định 0,1 m. | Giữ nguyên để kiểm tra nhiệm vụ, hoặc chọn scenario khác. |
| `time_budget` | Hết `experiments.duration_ns`. | Tăng ví dụ `experiments.duration_ns=1800000000000` cho 30 phút. |

Vòng lặp bám thời gian thực với tối đa 4 tick mỗi khung hình. Khi máy chậm hơn thời gian thực, phần trễ bị bỏ để điều khiển vẫn phản hồi ngay. Khi đó `Sim / real` nhỏ hơn 1. Thời gian mô phỏng luôn tăng theo đúng `simulation.control_dt_ns`.

## Ranh giới dữ liệu

- Profile D435i hiển thị phép đo từ `Observation` và lấy ảnh sạch cùng mốc từ `get_clean_depth()` cho người vận hành đối chiếu. Để ghi episode và tạo báo cáo SVG xem lại, dùng [recorder D435i](MUJOCO_D435I_DEPTH.md).
- Thông tin giám sát đọc `GroundTruthState` và khoảng hở hình học. Không dùng các giá trị này làm đầu vào cho robot policy.
- Công cụ không ghi `runs/<run_id>/`. Metadata episode vẫn được tạo trong bộ nhớ theo runtime, gồm commit và trạng thái dirty của Git. Để lưu cấu hình đã resolve của một phiên, chạy `narrow-nav-config` với cùng override. [Lệnh kiểm tra cấu hình](MUJOCO_WORLD_COMMANDS.md#kiểm-tra-cấu-hình-trước-khi-triển-khai-hành-vi)

## Xử lý sự cố

| Thông báo hoặc hiện tượng | Cách xử lý |
| --- | --- |
| `GLFW could not start; run in a desktop session` | Không có màn hình. Chạy trong phiên desktop, hoặc kiểm tra `DISPLAY`/`WAYLAND_DISPLAY` khi dùng SSH. |
| `robot profile 'mobile_robot' has no depth_camera` | Thêm `robots=mecanum_robot`. |
| `scenarios.world_id must match the selected world` | Chọn scenario cùng world, ví dụ `worlds=school_floor scenarios=school_floor_baseline`. |
| Robot không chạy | Bấm vào cửa sổ để lấy focus, kiểm tra `Status` có đang `paused` hoặc đã kết thúc không, rồi nhấn P hoặc R. |
| Robot dừng ngay sau khi xuất phát | Thường là `robot_contact` do sát tường. Nhấn R, hoặc tắt `experiments.terminate_on_contact`. |
| Ảnh depth gần như đen | Vật ngoài khoảng tô màu. Tăng `--depth-max` khi nhìn dọc hành lang dài. |
| `Sim / real` nhỏ hơn 1 | World có nhiều người và hình học, ví dụ `hospital_hard`. Thu nhỏ cửa sổ hoặc chọn world nhẹ hơn. |

## Kiểm tra sau khi sửa công cụ

Chạy từ thư mục gốc dự án. Test headless kiểm tra ghép override, ánh xạ phím, đổi depth sang mét, tô màu, định dạng bộ đệm pixel và thao tác kéo/cuộn qua binding MuJoCo. Mở cửa sổ để kiểm tra phím, chuột và bố cục, với mẫu depth ở trên và ảnh đối chiếu ở dưới bên phải.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY .venv/bin/pytest -q tests/simulation/test_teleop_depth.py
.venv/bin/ruff check src/narrow_nav/simulation/mujoco/teleop_depth.py tests/simulation/test_teleop_depth.py
.venv/bin/python -m narrow_nav.simulation.mujoco.teleop_depth sensors=d435i worlds=corridor_090 scenarios=corridor_090
```

Trên Linux có EGL, dùng smoke test để xác nhận world nạp được và camera thu depth qua tick đầu mà không cần cửa sổ. Thay cặp world/scenario và thêm preset bề rộng tương ứng khi cần.

```bash
env -u DISPLAY -u WAYLAND_DISPLAY MUJOCO_GL=egl .venv/bin/python \
  -m narrow_nav.simulation.smoke --ticks 1 sensors=d435i robots=mecanum_robot \
  worlds=school_floor scenarios=school_floor_baseline corridor_width=w090
```
