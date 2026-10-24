# Runtime dùng chung cho pilot Nav2

Ba experiment `socnavgym_nav2_pilot`, `socnavgym_group_nav2` và `shared_nav2_pilot` dùng chung implementation trong `python/dynamuhsim/`. Các file `trial.py` và `warehouse.launch.py` cũ là entry point tương thích, không còn chứa bản sao của vòng chạy.

## Tìm code ở đâu

| Thành phần | File chuẩn |
|---|---|
| Khởi chạy worker, ROS/Gazebo, gates, teardown và provenance | `python/dynamuhsim/runtime/nav2_trial.py` |
| Khác biệt CLI, backend, nhóm và prerequisite gates | `python/dynamuhsim/runtime/profiles.py` |
| Chờ action, budget, cancel và metric đích thật | `python/dynamuhsim/runtime/navigation.py` |
| Nhận sensor/scene, xuất JSONL và camera preview | `python/dynamuhsim/runtime/recording.py` |
| Cấu hình adapter/world, bản sao YAML và launch description | `python/dynamuhsim/simulation/warehouse.py` |
| Một lệnh gọi các profile | `tools/nav2_trial.py` |
| Danh sách entry point và ranh giới migration | `docs/experiment_registry.json` |

Đây là ba profile của nhiệm vụ nhà xưởng hiện có: robot bắt đầu (−28,0), goal (−20,0), Mecanum, AMCL và Nav2 MPPI. Native dt CrowdSimPlus0,1s và ComplexityNav0,25s vẫn khác nhau theo protocol. Nhóm đứng/đi giữ worker và chính sách hiện có. Chưa thêm cấu hình route tùy ý hoặc thay đổi thuật toán người.

## Đọc CLI không cần ROS

Từ gốc checkout, dùng Python3.10 trở lên:

```bash
python3 tools/nav2_trial.py --help
python3 tools/nav2_trial.py --experiment socnavgym_group_nav2 --help
```

Không cần pip install để chạy các entry point từ source. `pyproject.toml` cung cấp package editable nếu một môi trường phát triển muốn cài; không cài lại dependency ROS/native framework thông qua package này.

## Chạy một lượt mới

Trước khi chạy thật cần ROS2 Jazzy/Gazebo và package robot đã build, NVIDIA renderer, adapter `.so`, vendor/venv SocNavGym hoặc Docker images được pin cho backend tương ứng. Các gate cũ vẫn bắt buộc; CLI không tự dựng hay chạy lại chúng. Source ROS và workspace install phải thuộc checkout/runtime phù hợp.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/nav2_trial.py \
  --experiment socnavgym_nav2_pilot \
  --condition people \
  --report-dir /absolute/path/to/a-new-report
```

Đối với nhóm:

```bash
python3 tools/nav2_trial.py \
  --experiment socnavgym_group_nav2 \
  --scenario standing --seed 42 \
  --report-dir /absolute/path/to/a-new-group-report
```

Đối với backend cô lập:

```bash
python3 tools/nav2_trial.py \
  --experiment shared_nav2_pilot \
  --model crowd_orca_plus \
  --report-dir /absolute/path/to/a-new-crowd-report
```

`bash experiments/social_navigation/benchmark/<profile>/run.sh live ...` và gọi trực tiếp `trial.py` vẫn có CLI cũ. Không ghi đè report/runtime đã có; mỗi lượt cần tên report mới. Chạy thêm lượt tạo bằng chứng mới, không thay thế FAIL/TIMEOUT đã ghi trước đó.

## Hợp đồng được giữ

- Nav2 action vẫn là nguồn điều khiển robot, không có publisher cmd_vel điều khiển bằng kịch bản.
- Người bị giữ đến khi goal được chấp nhận; start_gate không mở khi goal bị từ chối.
- Camera chỉ ghi trong phase active, preview4×; telemetry giữ tên stream/schema cũ.
- Goal action và tọa độ Gazebo thật được đánh giá riêng. XY≤0,20m và yaw≤0,25rad vẫn bắt buộc để actual_goal_reached.
- Budget simulation/wall clock, hủy action khi timeout và lỗi worker terminal được giữ.
- Nhóm mở GUI riêng sau readiness. Shared backend giữ Docker image/worker runtime đang pin và native dt.
- Teardown giữ thứ tự GUI (nếu có), server, container thuộc phiên, owned processes, ROS và RuntimeGuard cleanup. Không kill theo tên process chung.
- source_sha256 của lượt mới ghi cả entry point và các module dùng chung; kết quả cũ không được sửa hash để mang tên implementation mới.

## Phần lịch sử và pipeline chưa chuyển

Các `reports/**/source*`, driver đã đóng băng, result/protocol/frozen manifests và bản classifier portable giữ nguyên. Muốn tái lập chính xác một kết quả lịch sử, dùng source commit/manifest của kết quả đó; runner mới có provenance mới.

`dataset_pilot`, `depth_proxy_lifecycle`, `early_avoidance_pilot` và `wm_episode_batch` vẫn dùng camera/recording pipeline đã freeze. Đặc biệt dataset launch và các fixture trong capture_frozen/corpus frozen không được đổi trong đợt này. Không đổi teacher labels, causal masks, native friction prefix hoặc dữ liệu train/test. Chỉ archive stage khi không còn consumer code, không suy từ tên cũ rằng có thể xóa.

## Kiểm tra refactor

```bash
python3 -m unittest discover -s tests -p 'test_nav2*.py' -v
```

Fixtures `tests/fixtures/nav2_runtime/` được trích bằng thực thi riêng callback/config thuần từ commit9833565. Tests đối chiếu world bytes, telemetry JSONL, launch arguments, CLI, goal success-vs-true-pose, timeout, accepted-handle cleanup và lỗi worker. Integration tests dùng endpoint/process giả trong bộ nhớ cho năm cấu hình worker/GUI; không thay cho nghiệm thu Gazebo/ROS/Docker thực và không chứng minh physics hoặc renderer đã chạy lại.

Luồng dataset WM hiện tại vẫn được hướng dẫn tại [START_HERE_CURRENT](START_HERE_CURRENT.md).
