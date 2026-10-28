# Capture cho lượt chạy mới

**Recipe mặc định v3:** `configs/capture/moderate_v3.json` giữ camera, footprint,
speed, budget 90s và tail 3,2s của v1. Goal checker kiểm XY/yaw đồng thời với
tolerance nội bộ 0,05m/0,15rad; GoalCritic và PathFollowCritic chuyển ở 0,45m.
Cả hai costmap nhận tia LiDAR +Inf để xoá vật cản đã đi; ghi vật cản trong 3,0m,
xoá tới 3,5m để đầu tia không bị ghi thành vòng vật cản giả. Mọi tham số được
đối chiếu live. Ngưỡng nghiệm thu tọa độ thật vẫn 0,20m/0,25rad; GT không đi vào
controller. V1 và v2 giữ nguyên để tái lập; v2 là candidate chưa đạt seed45,
không phải recipe được nghiệm thu. Chọn rõ bằng `--config` khi cần tái lập.

Bản v3 đã kiểm chứng Gazebo cho standing/walking seed45 và46: 4/4 đạt
ngưỡng tọa độ thật; 8/8 audit dữ liệu/đích đạt. Chi tiết các lượt candidate
thất bại, số đo cuối và giới hạn độ phủ ảnh nằm trong
[`navigation_goal_fix_2026-10-28.json`](verification/navigation_goal_fix_2026-10-28.json).
Các seed47–49 chưa được chạy lại trong đợt này.

Entry point chuẩn: `tools/capture_episode.py`. Camera/world/config/recorder là API trong package `python/dynamuhsim/`, cùng runner Nav2 đã dùng cho ba pilot. Không sinh Python driver, không thực thi code trong reports và không sửa source bằng thay chuỗi khi chạy.

## Ranh giới

- Dùng Mecanum, nhà xưởng, native group worker, start (−28,0), goal (−20,0), native dt0,1s và physics dt0,0025s hiện có.
- Config `configs/capture/moderate.json`: seed45–49, budget90s, label tail3,2s, robot radius0,25m, padding0,01m, inflation0,8m và MPPI vx/vy giới hạn0,30/0,20m/s. Config version1 khóa treatment đã nghiệm thu; thay thuật toán/camera cần version/protocol khác.
- `dataset01`, các split/nhãn/quarantine, driver trong reports và manifest `wm_capture_freeze/1` giữ nguyên. API mới tạo `dynamuhsim_capture_freeze/2` ở thư mục mới; không sửa hash lịch sử để nhận implementation mới.
- Đây là capture episode mới. Các lệnh cohort/resume/export cũ còn phục vụ tái lập dataset01; API này không tự thu10 episode, không append dữ liệu mới vào dataset01 và không thay đổi loader/train/test.

## Code chuẩn

| Trách nhiệm | File |
|---|---|
| Freeze/config, capture hooks, live parameters, label tail/quiesce | `python/dynamuhsim/runtime/capture.py` |
| Pause/quiesce trước ordered shutdown, dùng chung với Nav2 | `python/dynamuhsim/runtime/shutdown.py` |
| MCAP recorder, events/scene LABEL_ONLY, ON/OFF float32 | `python/dynamuhsim/runtime/capture_recording.py` |
| Camera XML và moderate Nav2 treatment | `python/dynamuhsim/simulation/capture.py` |
| Launch Mecanum và bridge; URDF sở hữu camera_link TF | `python/dynamuhsim/simulation/capture_launch.py` |
| Action, worker, telemetry và teardown dùng chung | `python/dynamuhsim/runtime/nav2_trial.py` |
| Storage/QoS và treatment | `configs/capture/` |

Camera XML được dựng theo cấu trúc, giữ prefix `gz:expressed_in` cho bốn bánh. Byte SHA256 phải bằng `14c15412664eb37dd2f0fb2d61a79b09f98722f204f97d97212f0ec814ad3d9f`, giống camera native đã nghiệm thu. Camera_link fixed TF trùng URDF được loại bằng cấu trúc launch mới. Không vá file launch cũ.

## Chuẩn bị môi trường

Cần ROS2 Jazzy/Gazebo Harmonic, GPU NVIDIA, robot packages đã build từ source checkout tương ứng, SocNavGym vendor/venv, response adapter và depth filter. Mỗi worktree cần dependency riêng hoặc mapping ngoài Git rõ ràng; chỉ source install của build tương ứng.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 experiments/social_navigation/navigation/common_response/preflight.py
bash experiments/social_navigation/navigation/socnavgym_response_fix/run.sh build
bash experiments/social_navigation/perception/depth_proxy_ablation/run.sh build
mkdir -p experiments/social_navigation/navigation/early_avoidance_pilot/generated
g++ -std=c++17 -O2 \
  experiments/social_navigation/navigation/early_avoidance_pilot/check_friction.cc \
  -o experiments/social_navigation/navigation/early_avoidance_pilot/generated/check_friction \
  $(pkg-config --cflags --libs sdformat14)
```

Các gate cũ của group/empty control vẫn bắt buộc. `prepare` kiểm camera bằng SDFormat friction checker, kiểm config/gates và freeze source/binary/config/assets. Thiếu dependency thì dựng đúng dependency trước; không cần chạy lại các cohort cũ.

## Một episode mới

Từ gốc repository, chọn tên thư mục chưa tồn tại; đường dẫn phải nằm trong checkout:

```bash
RUN_DIR="$PWD/experiments/social_navigation/benchmark/generated/capture_new_001"
python3 tools/capture_episode.py prepare \
  --run-dir "$RUN_DIR" --scenario standing --seed 45
python3 tools/capture_episode.py verify --run-dir "$RUN_DIR"
python3 tools/capture_episode.py run --run-dir "$RUN_DIR"
```

Dùng `--seed 45` (có dấu cách); seed phải có trong config. `--scenario walking` dùng cùng API. `--gui` ở bước prepare là tùy chọn; native kiểm chứng mặc định headless. Có thể dùng `--config /absolute/path/inside/checkout/config.json` để chỉ rõ recipe; version1 giữ nguyên camera/treatment, budget có thể giảm cho diagnostic và vẫn lưu đúng trong freeze. Diagnostic ngắn không tự đủ điều kiện học WM.

`prepare` tạo `plan.json`, `preflight.json`, `assets/mecanum_camera.sdf`, `capture_freeze.json`. `run` kiểm lại freeze trước khi tạo `runtime/` và `episode/`; mọi lượt mới có source provenance riêng. Không ghi đè thư mục đã có, không tự retry.

Đầu ra `episode/`: MCAP trong `raw_bag/`, `events.jsonl`, telemetry JSONL, paired depth `.npy`/index, live parameters, result và preview RGB thật. Schema event/scene và topic recorder cũ được giữ. GT người chỉ nằm trong luồng nhãn; không cấp cho controller. Chỉ publisher Nav2 được phép trên cmd_vel. Camera observer dùng minh họa, không làm input WM.

Sau action, recorder giữ thêm3,2s sim cho label tail. TIMEOUT/ABORT/không đạt đích vẫn là outcome được giữ. Recorder chết/MCAP đóng lỗi là lỗi hạ tầng. Khi đóng: recorder/pairs, GUI nếu có, pause/quiesce, Gazebo, ROS launch parent rồi owned cleanup; không kill theo tên process chung.

## Kiểm tra

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
```

Tests đối chiếu camera byte hash và config với implementation cũ, giữ float32 endian/stride, kiểm freeze mutation/schema, và wiring capture qua chính runner dùng chung. Kiểm chứng native và giới hạn sẽ được ghi trong `docs/verification/`; unit tests không thay thế MCAP/renderer thực.

## Kiểm chứng ngày 24/10/2026

[Biên bản và hash bằng chứng](verification/capture_native_2026-10-24.json). Runtime kiểm chứng ở commit `29ea6b3`; **37 unit/in-memory integration tests PASS**.

Một episode mới, headless, standing seed45, config moderate mặc định, chạy ROS2/Gazebo/NVIDIA thật:

| Kiểm tra | Kết quả |
|---|---|
| Capture integrity audit | PASS, 25 kiểm tra |
| Điều hướng | TIMEOUT sau90,0275s sim; không đạt đích thật; giữ nguyên outcome |
| Sai số goal thật cuối lượt | XY1,602m; yaw1,543rad |
| Recorder MCAP | Đóng returncode0; đủ16 topic có message |
| Depth ON/OFF | 1.036 timestamp ghép cặp |
| Depth từ MCAP đối chiếu `.npy` | 1.036 frame bằng từng bit float32 |
| Future tail thực tế | 3,365s; có36 frame depth sau action_end |
| Native camera/friction | SHA256 khớp camera đã nghiệm thu; SDFormat nhận4 frame bánh |
| TF camera | camera_link chỉ có parent base_link; optical frame nối camera_link |
| Đóng hệ thống | Pause xác nhận, sim time ổn định, filter teardown cân bằng; Gazebo clean; ROS launch0; không có child-exit error/owned process còn lại |
| Ghi đè | Prepare và run đều từ chối thư mục đã có; freeze/plan/result giữ nguyên byte |
| Lịch sử | 3.452 file reports/snapshot/freeze và source được tham chiếu giữ nguyên byte |

TIMEOUT không được đổi thành một lượt điều hướng thành công khác. Đây là kiểm tra capture integrity, chưa phải chứng nhận episode đủ điều kiện học WM. Không export/append vào dataset01, không mở lại test để chọn mô hình và không đổi nhãn/split.

Không có Python driver được tạo trong thư mục run. Raw bằng chứng ở `experiments/social_navigation/benchmark/generated/capture_migration_20261024/`, gồm MCAP khoảng76MB và toàn bộ ON/OFF/telemetry/media khoảng0,95GB logical. Dữ liệu này được ignore, không commit vào Git; manifest SHA256 cho phép đối chiếu từng file. Hai symlink dependency vendor/venv tạm của worktree đã được gỡ sau test; chuẩn bị lại dependency trước khi verify/rerun source tại commit tương ứng.

Recorder có cảnh báo thiếu embedded message definition cho `GoalStatusArray`; đã deserialize được cả3 message CDR bằng ROS2 action_msgs đã cài. Chưa xác nhận khả năng đọc action-status schema trên môi trường không có ROS. Native smoke mới chỉ gồm standing seed45; walking, GUI, exporter và train WM chưa được chạy qua API này. Checkout/index chính giữ nguyên.

## Trạng thái artifact sau bước dọn tiếp theo

Raw ON/OFF/MCAP của capture diagnostic ở trên đã được chuyển sang artifact local sau roundtrip và audit bitwise. Metadata/freeze/result/log vẫn tại chỗ; xem `artifact_ref.json` và [hướng dẫn audit/artifact](ANALYSIS_AND_ARTIFACTS.md) để restore vào thư mục mới. Bộ dataset01 không thay đổi. Biên bản capture ở commit29ea6b3 giữ nguyên các kết quả ban đầu.

## Nghiệm thu lại ngày 27/10/2026 sau cleanup

[Biên bản và hash bằng chứng](verification/capture_native_2026-10-27.json). Một lượt headless standing45 tại `36184b8`, config moderate mặc định, native ROS2/Gazebo/NVIDIA, prefix build riêng đã nghiệm thu. Không đổi budget90s, tail3,2s, controller, threshold hoặc dữ liệu cũ; không retry.

| Hạng mục | Kết quả |
|---|---|
| Capture integrity | **PASS 28/28 checks** |
| Navigation goal audit | **FAIL** đúng ở `actual_goal_required`; action TIMEOUT sau90,005s sim |
| Goal thật tại action_end | XY0,199m; yaw1,000rad, chưa đạt điều kiện action/yaw |
| MCAP | Recorder0; 16 topic có message; 1.019 raw depth frames |
| Auxiliary ON/OFF | 1.012 timestamp ghép cặp; toàn bộ1.012 OFF khớp MCAP từng bit float32 |
| Frame chỉ có trong raw | 7 frame ở warmup2,8–3,4s; không có thiếu auxiliary ở active/tail |
| Label tail | 3,34s; 35 depth frames sau action_end, gồm33 trong tail và2 khi shutdown |
| Camera/TF/friction | Camera đúng hash; camera_link có một parent; native checker nhận4 friction frames |
| Teardown | Filter pre/post1.021/1.021; recorder0, Gazebo clean, ROS launch0, không còn phiên sở hữu |
| Media | Ba PNG và GIF90 frame ghi thật,640×480,22,5s phát4×; đã decode/kiểm duration |

Cảnh báo thiếu embedded schema của GoalStatusArray vẫn có; cả ba message CDR đọc được bằng ROS đang cài (EXECUTING→CANCELING→CANCELED). Chưa xác nhận đọc topic đó trên môi trường không có ROS schema tương ứng. MCAP là nguồn raw; không dùng auxiliary để thay toàn bộ raw khi có bảy frame warmup không được lưu cặp.

AMCL gần action_end cũng có yaw error0,933rad; pose Gazebo căn cùng timestamp khoảng1,011rad (lệch thời gian2,5ms). Đây là gợi ý phân tích đoạn cuối, chưa quy nguyên nhân cho controller/localization hoặc cleanup. Giữ TIMEOUT; chưa điều chỉnh goal checker hoặc chạy lại để lấy success.

Raw/telemetry/media và inventory2.090 file (~0,926GB logical) giữ local tại `benchmark/generated/capture_acceptance_20261027/`; không đưa raw vào Git hoặc append dataset01. Đây là nghiệm thu capture, chưa đánh giá đủ điều kiện học WM, walking, GUI hoặc xếp hạng navigation. Source/control cũ và checkout/index chính giữ nguyên.
