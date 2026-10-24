# Capture cho lượt chạy mới

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
python3 experiments/social_navigation/benchmark/common_response/preflight.py
bash experiments/social_navigation/benchmark/socnavgym_response_fix/run.sh build
bash experiments/social_navigation/benchmark/depth_proxy_ablation/run.sh build
mkdir -p experiments/social_navigation/benchmark/early_avoidance_pilot/generated
g++ -std=c++17 -O2 \
  experiments/social_navigation/benchmark/early_avoidance_pilot/check_friction.cc \
  -o experiments/social_navigation/benchmark/early_avoidance_pilot/generated/check_friction \
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
