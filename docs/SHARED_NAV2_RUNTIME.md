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

Với checkout/worktree mới, dựng dependency ignored cho backend dùng chung trước lượt chạy:

```bash
python3 experiments/social_navigation/benchmark/common_response/preflight.py
```

Preflight hiện có đối chiếu input/hash đã khóa và dựng `common_response/generated/crowd_kernel_n6.py` cùng `complexitynav_geometry/generated/{native_contract.py,swept.py,layout.json}`. Không cần chạy lại cohort offline để có các file này. Có Docker image chưa đủ để chạy ComplexityNav nếu thiếu các file được mount từ host. Vendor/venv SocNavGym, ROS packages và adapter `.so` vẫn cần chuẩn bị riêng.

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
- Teardown giữ thứ tự GUI (nếu có), server, container thuộc phiên, owned processes, ROS và RuntimeGuard cleanup. ROS launch nhận SIGINT tại parent và có 15s để tự dừng node con; chỉ fallback sang process group khi hết grace period. Không kill theo tên process chung.
- source_sha256 của lượt mới ghi cả entry point và các module dùng chung; kết quả cũ không được sửa hash để mang tên implementation mới.

## Điều phối cohort cũ đã gỡ

Các script chỉ chạy/tiếp tục vào output cố định đã được gỡ khỏi cấp entrypoint:

- `common_gazebo/cohort.py`, `socnavgym_response_fix/cohort.py`
- `socnavgym_group_nav2/cohort.py`, `socnavgym_group_nav2/headless_cohort.py`
- `shared_nav2_pilot/cohort.py`, `shared_nav2_pilot/continue_cohort.py`

Đường dẫn trên tính từ `experiments/social_navigation/benchmark/`. Mỗi bản còn snapshot nguyên byte trong report tương ứng và Git trước commit gỡ; dùng source revision đã ghi khi tái lập lịch sử. Receipt, failure, protocol và snapshot giữ nguyên. Các lệnh live/analyze từng lượt vẫn giữ; common/response là robot chạy velocity script, các profile Nav2 chạy controller như phần trên.

## Phần lịch sử và pipeline chưa chuyển

Các `reports/**/source*`, driver đã đóng băng, result/protocol/frozen manifests và bản classifier portable giữ nguyên. Muốn tái lập chính xác một kết quả lịch sử, dùng source commit/manifest của kết quả đó; runner mới có provenance mới.

`dataset_pilot`, `depth_proxy_lifecycle`, `early_avoidance_pilot` và `wm_episode_batch` vẫn dùng camera/recording pipeline đã freeze. Các entrypoint này phục vụ lịch sử; lượt capture mới dùng [API capture có freeze riêng](CAPTURE_RUNTIME.md), chạy qua runtime Nav2 chung và không sinh/vá Python driver. Đặc biệt dataset launch và các fixture trong capture_frozen/corpus frozen không được đổi trong đợt này. Không đổi teacher labels, causal masks, native friction prefix hoặc dữ liệu train/test. Chỉ archive stage khi không còn consumer code, không suy từ tên cũ rằng có thể xóa.

## Kiểm tra refactor

```bash
python3 -m unittest discover -s tests -p 'test_nav2*.py' -v
```

Fixtures `tests/fixtures/nav2_runtime/` được trích bằng thực thi riêng callback/config thuần từ commit9833565. Tests đối chiếu world bytes, telemetry JSONL, launch arguments, CLI, goal success-vs-true-pose, timeout, accepted-handle cleanup và lỗi worker. Integration tests dùng endpoint/process giả trong bộ nhớ cho năm cấu hình worker/GUI; không thay cho nghiệm thu Gazebo/ROS/Docker thực và không chứng minh physics hoặc renderer đã chạy lại.

Luồng dataset WM hiện tại vẫn được hướng dẫn tại [START_HERE_CURRENT](START_HERE_CURRENT.md).

## Kiểm chứng native ngày 24/10/2026

[Biên bản JSON và hash bằng chứng](verification/shared_nav2_native_2026-10-24.json) giữ cả kết quả không đạt và lần lỗi chuẩn bị môi trường. ROS2 Jazzy/Gazebo Harmonic 8.15.0, NVIDIA RTX 4060 Laptop, backend Docker ghim content ID; build riêng từ source của nhánh refactor, không source install của checkout chính đang có thay đổi.

Bộ runtime tại `61667d8` đã chạy thực và audit 5 cấu hình:

| Ca đo | Nav2 action | XY thật (m) | Yaw thật (rad) | Đạt đích thật |
|---|---|---:|---:|---|
| Không người | SUCCEEDED | 0,128 | 0,031 | Có |
| SocNavGym 6 người riêng lẻ | SUCCEEDED | 0,144 | 0,019 | Có |
| Nhóm đứng, seed 42 | SUCCEEDED | 0,236 | 0,213 | Không |
| CrowdSimPlus ORCA+ | SUCCEEDED | 0,183 | 0,124 | Có |
| ComplexityNav geometry mixed, sau preflight | SUCCEEDED | 0,181 | 0,201 | Có |

Cả 5 ca đo vượt kiểm tra config, 42 AABB, physics timestep, pose/sensor/control streams, costmap payload, provenance và process cleanup; bốn ca có người khớp native-to-Gazebo interpolation và giữ start gate/native timestep 0,1 hoặc 0,25s. Các swept gap được lưu là số đo, không suy thành chứng nhận an toàn.

Có 6 lần khởi chạy cho 5 cấu hình trên: lần ComplexityNav đầu tiên ERROR trước khi mở Gazebo do thiếu `generated/native_contract.py`. Đã dừng cohort, chạy preflight sẵn có để dựng dependency đúng hash rồi chạy ca mới với tên khác; ERROR cũ còn nguyên. Ca nhóm không đạt XY 0,20m vẫn giữ nguyên, không điều chỉnh Nav2 để thay kết quả.

Hai phép fault injection dùng ROS/Gazebo/Nav2 thật: budget 1s tạo TIMEOUT, worker lỗi sau 5 native updates tạo terminal ERROR và yêu cầu pause. Cả hai có log Nav2 xác nhận cancel và dọn hết process/container. Budget chỉ đổi trong test harness; protocol production 45s giữ nguyên.

Audit log phát hiện cách dừng cũ gửi SIGINT cho cả launch và node con, sau đó launch chuyển tiếp SIGINT lần nữa; nhiều node ghi exit -2/255 dù Gazebo thoát sạch và không sót process. Commit `6b8ae5e` sửa bằng SIGINT cho parent trước, có group fallback. **24 unit/in-memory integration tests PASS.** Đã chạy lại 4 ca native trên revision sửa shutdown: không người, nhóm đứng, timeout và worker fault; cả 4 ROS launch returncode 0, không có `process has died`, Gazebo clean và không còn owned process/container. Đây là các ca kiểm chứng revision mới, không thay các outcome trước.

| Ca kiểm chứng sau sửa shutdown | Kết quả điều hướng | Đích thật | Shutdown |
|---|---|---|---|
| Không người | SUCCEEDED | Có, XY 0,136m | PASS |
| Nhóm đứng, seed 42 | SUCCEEDED | Có, XY 0,188m | PASS |
| Ép budget 1s | TIMEOUT | Không | PASS; Nav2 xác nhận cancel |
| Worker fault sau 5 updates | ERROR được giữ | Không đánh giá | PASS; Nav2 xác nhận cancel |

Kết quả nhóm khác giữa hai lượt không chứng minh sửa shutdown làm điều hướng tốt hơn: chưa đồng bộ random state AMCL/MPPI và chưa chạy A/B để quy kết nguyên nhân. Lần sai số 0,236m vẫn được lưu và báo cáo.

Raw JSONL, launch/worker logs, GIF quay trực tiếp và scripts test được giữ trong `experiments/social_navigation/benchmark/generated/native_smoke_20261024/` tại worktree test. Chỉ biên bản gọn được commit, không đưa raw media/dataset mới vào Git. Vendor/venv là dependency ngoài Git; hai symlink tạm trong worktree đã được gỡ sau test, mapping và hash nằm trong raw evidence. Trạng thái/index của checkout chính được đối chiếu không đổi.

Giới hạn: mỗi cấu hình có một ca đo chính; chưa chạy A/B native runner cũ/mới, chưa đo success rate hoặc xếp hạng backend, và chưa chạy GUI thật. Nhánh shutdown mới kiểm tra Docker qua worker-fault; hai ca Docker bình thường ở bảng là bằng chứng tại `61667d8`. Chưa thay fixture/capture/frozen manifests hoặc nhãn/dataset. Bước kế tiếp là chuyển pipeline capture/fixture sang API/config rõ ràng, dựng freeze mới cho lượt mới và giữ bằng chứng lịch sử.
