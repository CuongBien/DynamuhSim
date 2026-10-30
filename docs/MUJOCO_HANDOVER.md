# Runbook nhận bàn giao MuJoCo

## 1. Dự án đang làm gì?

DynamuhSim mô phỏng robot Mecanum điều hướng giữa người trong nhà xưởng, ghi LiDAR 2D + depth + IMU/odom/action, rồi xuất bộ mẫu nhân quả cho các bước WM/RL sau này. Hệ tham chiếu hiện là Ubuntu24.04/WSL2 + Python3.12.3 + ROS2Jazzy + GazeboHarmonic8, GPU RTX4060. Navigation dùng AMCL, NavFn/Dijkstra và MPPI Omni. Chưa huấn luyện WM/RL.

Nhánh `feat/mujoco-backend` dành cho người nhận. Mốc trước port là `dd7a2b03d9fe40841052c0db6c7dc52619e13e67` trên `test/gazebo-harmonic-benchmark`. Mốc protected data/runtime là `121a727`; manifest `configs/handoff/baseline.json` giữ116hash. **Chưa có MuJoCo implementation**, chưa pin/cài một phiên bản MuJoCo; người port chọn phiên bản, ghi lock/inventory và nghiệm thu trên máy mình.

Các phần đã có:

- SocNavGym sáu người: hai nhóm trò chuyện giữ nhóm (`persistent_conversation_v1`) và natural walking với `group_cohesion_v2`, clock `sim_v1`. Không mở rộng tất cả framework người cùng lúc khi port đầu tiên. Các experiment CrowdSimPlus/ComplexityNav và lịch sử khác vẫn giữ nguyên, có protocol riêng.
- Hai capture Gazebo cuối đạt full audit; pilot2episode/517window,910depth frame đối chiếu raw từng bit,2068BEV grid tái tính. Portable loader độc lập ROS và nhãn đã kiểm.
- Detector 3D đã merge, wrapper ROS/input đã kiểm nhưng chất lượng FAIL trên cả hai pilot. Giữ opt-in; không lấy làm GT/train hoặc dependency bắt buộc của port. [Báo cáo](verification/depth_detector_integration_2026-10-30/README.md), [khung detector trên depth](verification/depth_detector_overlays_2026-10-30/README.md).
- Chưa hợp lệ: H1–H5/persona, association sensor-track↔GT, rewardRL. Pilot là pilot_only, training_authorized=false và fit_allowed=false.

## 2. Clone, nhánh và giữ code cũ

```bash
git clone --branch feat/mujoco-backend https://github.com/CuongBien/DynamuhSim.git
cd DynamuhSim
git status --short --branch
```

Làm việc trên `feat/mujoco-backend` hoặc nhánh con từ nó. Giữ nguyên nhánh Gazebo tham chiếu; không force-push hoặc reset nhánh đó để thay bằng MuJoCo. Code cũ vẫn ở Git, nhưng raw/generated/venv/build không được Git backup. Giữ artifact/source-run hiện có và tạo output mới cho mỗi lượt.

Nếu cần checkout tham chiếu song song, từ checkout mới tạo worktree ở một đường dẫn chưa có:

```bash
git worktree add --detach ../DynamuhSim-gazebo-reference dd7a2b03d9fe40841052c0db6c7dc52619e13e67
```

Worktree mới không có venv/vendor/build/raw bị ignore; chuẩn bị dependency theo đúng checkout trước khi chạy. Không source nhầm install của workspace khác. Module mới dưới `python/dynamuhsim/**/*.py` sẽ nới source closure capture Gazebo; ưu tiên thêm backend trong thư mục riêng và tạo version/freeze mới khi cần thay shared code.

## Setup tối thiểu

Để đọc data và kiểm code contract, chỉ cần Python3.12 + NumPy1.26.4, không cần ROS/Gazebo/MuJoCo. Từ repository root:

```bash
python3 -m venv .venv-handoff-data
.venv-handoff-data/bin/python -m pip install -r requirements/handoff-data.txt
.venv-handoff-data/bin/python tools/mujoco_handoff.py check
```

Nếu Ubuntu báo thiếu ensurepip, cài gói `python3.12-venv` theo quản lý môi trường của máy. Trường hợp máy đã có system pip hỗ trợ --python, có thể dùng cách tách biệt sau (đã kiểm trên máy tham chiếu):

```bash
python3 -m venv --without-pip .venv-handoff-data
python3 -m pip --python .venv-handoff-data/bin/python install -r requirements/handoff-data.txt
.venv-handoff-data/bin/python tools/mujoco_handoff.py check
```

Không dùng pip để ghi đè Python/NumPy hệ thống đang phục vụ ROS. `.venv-handoff-data` và `.venv-mujoco` là hai môi trường riêng, đã ignore; chưa có requirements MuJoCo do backend chưa chọn phiên bản.

`tools/mujoco_handoff.py check` gọi lại các guard có sẵn: ancestry mốc dd7a2b0,116protected files, NumPy import và38contract smoke tests. Nó không clone/cài software, mở simulator hoặc tự tạo dữ liệu. PASS chỉ thuộc scope được ghi; nếu không truyền ZIP/dataset thì portable_data=NOT_CHECKED, backend vẫn NOT_IMPLEMENTED.

## 3. Dữ liệu cần nhận ngoài Git

| Artifact | Thông tin |
|---|---|
| Portable ZIP | handoff_data_pilot01.zip,31.706.091bytes, bung khoảng379MB |
| SHA256 | b64bdb394f4a2ddabc6d97f2e9ccd3704dd9c6b0855e5d52fb11c6eeaea23ee2 |
| Nội dung | 2episode/517window, arrays,calibration,masks,loader,audit và provenance |
| Không có trong ZIP | Raw MCAP và toàn bộ build/vendor/venv; raw chỉ cần khi audit/export lại từ nguồn |

Người gửi cung cấp ZIP qua artifact riêng. Bản local nguồn ở `experiments/social_navigation/datasets/handoff_pilot/generated/package01/handoff_data_pilot01.zip`; không coi đường dẫn này là sẵn có sau git clone. [Receipt package](demos/2026-10-29/handoff_pilot/package_receipt.json).

```bash
.venv-handoff-data/bin/python tools/mujoco_handoff.py check \
  --archive /absolute/path/handoff_data_pilot01.zip \
  --restore-dir /absolute/path/NEW_RESTORE_DIR \
  --output /absolute/path/NEW_CHECK.json
```

Thư mục restore và receipt phải mới, không ghi đè. Tool kiểm ZIP/hash/inventory trước khi chạy reader. Nếu đã restore:

```bash
.venv-handoff-data/bin/python tools/mujoco_handoff.py check --dataset /absolute/path/handoff_data_pilot01
```

Chạy inspector không cần simulator:

```bash
.venv-handoff-data/bin/python /absolute/path/handoff_data_pilot01/tools/inspect_pilot.py --dataset /absolute/path/handoff_data_pilot01
```

API dữ liệu: `Dataset(root).episode(name).observation(i)` lấy sensor history; `action_condition(i)` lấy future executed actions riêng; `supervision(i)` và `task_masks(i)` là nhãn/mask. Đọc ví dụ đầy đủ trong [HANDOFF_MUJOCO.md](HANDOFF_MUJOCO.md). Không cấp GT/future sensor/backend/seed vào observation.

## Chạy Gazebo tham chiếu

Dùng Python hệ thống, source ROS Jazzy và install của đúng workspace; không dùng venv chỉ có NumPy cho ROS/native runtime. [Chuẩn bị dependency đầy đủ](HANDOFF_SETUP.md#3-chạy-lại-gazebo-tham-chiếu), [các lệnh build](CAPTURE_RUNTIME.md#chuẩn-bị-môi-trường).

```bash
source /opt/ros/jazzy/setup.bash
# Nếu checkout chưa build ROS packages:
colcon build --base-paths src --executor sequential
source install/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 experiments/social_navigation/navigation/common_response/preflight.py
bash experiments/social_navigation/navigation/socnavgym_response_fix/run.sh build
bash experiments/social_navigation/perception/depth_proxy_ablation/run.sh build
mkdir -p python/dynamuhsim/runtime/native/generated
g++ -std=c++17 -O2 python/dynamuhsim/runtime/native/check_friction.cc \
  -o python/dynamuhsim/runtime/native/generated/check_friction \
  $(pkg-config --cflags --libs sdformat14)
python3 tools/handoff.py doctor --scope gazebo
```

SocNavGym vendor/venv được chuẩn bị riêng bởi `experiments/social_navigation/setup_socnavgym.sh`; script có vendor pins và từ chối checkout vendor đã sửa. Fresh machine vẫn cần ROS/Gazebo/Nav2/resource packages và build prerequisites; các lệnh ở trên không thay cho cài OS/ROS. Doctor chỉ kiểm prerequisites, không chứng nhận renderer. Bản tham chiếu WSL dùng D3D12; Ubuntu native và MuJoCo cần renderer setup/kiểm tra riêng. Ưu tiên GPU render và giữ thread caps1, không ép software rendering để lấy một lượt giả PASS.

In chính xác các lệnh prepare/verify/run/audit của recipe v7 hiện hành:

```bash
python3 tools/handoff.py reference-command --mode conversation --seed 46 \
  --run-dir "$PWD/experiments/social_navigation/benchmark/generated/handoff_reference/my_conversation01"
python3 tools/handoff.py reference-command --mode natural_group_walking --seed 46 \
  --run-dir "$PWD/experiments/social_navigation/benchmark/generated/handoff_reference/my_walking01"
```

Chúng chỉ in lệnh. Để chạy, thực hiện các lệnh đã in theo thứ tự; thêm --gui vào lệnh prepare trước khi tạo run nếu cần xem GUI. Không thêm --gui vào run, không dùng output cũ, không chuyển default v3 thành recipe đang nghiệm thu. Tham khảo [v7 profile](demos/2026-10-29/transport_order/collection_profile.json).

## 4. Interface người port phải giữ

Loại topic lấy từ metadata của capture được nghiệm thu, đã ghi máy đọc được trong [mujoco_port.json](../configs/handoff/mujoco_port.json):

| Topic | Loại/ý nghĩa |
|---|---|
| /cmd_vel | geometry_msgs/msg/TwistStamped; vx,vy,wz theo body axes base_link/base_footprint |
| /odom | nav_msgs/msg/Odometry; pose/twist theo frame khai báo, không coi commanded velocity là measured velocity |
| /scan | sensor_msgs/msg/LaserScan,360rays theo calibration thực tế, giữ invalid-ray mask |
| /imu | sensor_msgs/msg/Imu, giữ orientation-valid, đúng đơn vị/trục |
| /camera/depth/image_rect_raw | sensor_msgs/msg/Image, reference32FC1,240×424,metric optical-Z; phân biệt depth Z với radial range |
| /camera/camera_info | sensor_msgs/msg/CameraInfo, K/TF phải khớp camera thực tế |
| /tf,/tf_static,/clock | TFMessage/Clock, frame tree và integer simulation ns nhất quán |
| /dataset/events,/dataset/scene_poses,/common/ground_truth | std_msgs/msg/String JSON theo recorder/worker schema; scene/group GT chỉ LABEL |

Nguồn chi tiết: `python/dynamuhsim/runtime/capture_recording.py`, `configs/capture/qos.yaml`, [backend_contract.json](../configs/handoff/backend_contract.json).

- Robot command là body vx/vy/wz. Worker người nhận pose và **measured velocity trong world**, không đưa thẳng body command vào đó.
- Worker request frame=world,epoch0,time=t,dt0.1; reply state_time=t+0.1. Nội suy before→humans qua interval đó, yaw theo shortest wrap. LABEL dùng actual applied pose, metadata groups_before cho interval hiện tại.
- Pause không step tuổi người; start gate là goal được Nav2 chấp nhận; reset tạo episode/worker session mới. Không tự extrapolate qua clock discontinuity hoặc NaN.
- Giữ event ready→goal_accepted→action_end→observation_end; label tail≥3,2s.
- Robot collision AABB0,24×0,28m,radius≈0,16806m; Nav2radius0,17m,padding0,01m,inflation0,45m. Đây là reference cần đo lại trên backend mới.
- Camera reference x0,12m,z0,08m trong base_link; lấy đủ optical transform từ calibration. World factory source layout ở `experiments/social_navigation/warehouse/layout.json`; start[-28,0,0],goal[-20,0,0].
- Data history20queries ở10Hz; future action30intervals, human trajectory30steps, future observed BEV tại+0,5/+1/+3s. BEV64×64,cell0,1m; unknown=-1 không phải free. Availability≤query≤anchor và input/GT separation là bắt buộc. [Schema đầy đủ](HANDOFF_MUJOCO.md).

## 5. Bắt đầu code ở đâu và giữ gì?

[Workspace mới](../experiments/social_navigation/mujoco_backend/README.md) có các responsibility folders assets/runtime/sensors/human_adapter/ros_bridge/checks. Thêm implementation, requirements đã pin và entrypoint MuJoCo rõ ràng ở đây; các README không phải một adapter abstraction đã chạy.

| Giữ/tái dùng | Phần cần tạo hoặc nghiệm thu mới |
|---|---|
| Schema,loader,sampling/BEV math và masks | Collector/raw adapter MuJoCo với calibration/timing đúng |
| SocNavGym intent/group/lifecycle/safety logic | Đọc measured robot state, áp pose/collision/render người và actual-state labels |
| Nav2 AMCL/NavFn/MPPI Omni và interface command | ROS bridge/launch, map/TF, physical actuation và có thể tuning controller |
| Geometry/layout tham chiếu | Model engine mới, contact/actuator/sensors và kiểm dimensions |
| Baseline Gazebo,receipts,raw,pilot | New source freeze,output,audit và backend-specific shutdown proof |

Giữ code không bảo đảm output/physics/detector metrics giống hệt; cần đo lại. Nếu đổi raw format, viết adapter vào schema portable. Nếu vẫn ROS/MCAP đúng topic/type/frame, có thể tái dùng reader; metadata calibration/provenance vẫn phải mới. Không sửa code trong reports/archive rồi thực thi nó như runner hiện hành.

## 6. Sáu mốc hoàn thành port

M1–M6 trong `mujoco_port.json` đều đang NOT_STARTED:

1. Robot/world: vx/vy/wz đúng dấu/đơn vị, actual displacement, footprint và collision có số đo.
2. Sensors: kiểm depth optical-Z bằng vật ở khoảng cách biết trước; K/TF,LiDAR/IMU/odom/invalid masks đúng.
3. Time: clock monotonic, pause/reset/start gate/label tail đúng, không rò state/ID giữa episode.
4. Human adapter: một cảnh conversation và một natural walking, groups/poses là actual applied states; giữ logic đã chốt.
5. ROS/Nav2: goal action và actual goal đều kiểm, chỉ Nav2 sở hữu command; kiểm footprint/clearance và owned shutdown.
6. Data: xuất hai episode mới, audit raw→depth/action/BEV/causality/masks/standalone loader/label independence. Khác số window/physics không tự là lỗi; sai contract hoặc mất sensor là lỗi. Chưa train trên hai smoke episodes.

Sau M6 mới chốt variation/seeds/train-validation-test split để thu batch. Giữ mọi FAIL/ERROR mới cùng protocol; không sửa ngưỡng sau kết quả để chuyển FAIL thành PASS.

## 7. Người gửi/người nhận cần bàn giao gì?

- Người gửi: nhánh code, ZIP portable+SHA; raw/source-run nếu cần re-audit MCAP; tham chiếu receipts và source revisions. Không gửi một thư mục build/install/venv như dependency portable.
- Người nhận: chạy preflight, xác nhận scope dữ liệu đã nhận, pin MuJoCo/runtime dependencies, triển khai từng milestone và lưu receipts mới.
- Current limitations: một nhà xưởng/cùng avatar, chưa chứng minh tổng quát; detector3D FAIL, H1–H5/association/reward masked; conversation active sidestep và arrival/expiry query chưa được exercise. Bằng chứng Gazebo cũ không tự trở thành MuJoCo PASS.

[Trang vào gốc](../README.md) · [Ranh giới backend](BACKEND_BOUNDARY.md) · [Data contract đã đóng băng](HANDOFF_MUJOCO.md) · [Setup chi tiết](HANDOFF_SETUP.md).
