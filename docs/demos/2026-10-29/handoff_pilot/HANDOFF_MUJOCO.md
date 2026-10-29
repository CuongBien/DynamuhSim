# Bàn giao backend MuJoCo — mốc dừng phase Gazebo

## Mốc bàn giao

Phase hiện tại đã có cấu hình mô phỏng và **pilot dữ liệu end-to-end đã kiểm tra**, đủ để tạm dừng trước khi chuyển backend. Không còn cần thu batch lớn hay train WM/RL trong phase này.

- Recipe `factory_social_pilot_v1`: `docs/demos/2026-10-29/transport_order/collection_profile.json`. Robot mặc định không bị thay kiến trúc.
- Runtime mô phỏng đã kiểm chứng: `e0b5dd6`; recipe/báo cáo `21cbb37`. Exporter pilot `45fcdcf`, auditor `36c5070`.
- Hai nguồn raw đã PASS: `experiments/social_navigation/benchmark/generated/transport_order_20261029/conversation_s48_transport/` và `walking_s46_transport/`.
- Pilot portable: `experiments/social_navigation/datasets/handoff_pilot/generated/pilot01/dataset/`. Audit: `generated/audit01/audit.json` cùng stage.
- Kết quả: **517 window** (264 trò chuyện,253 đi bộ), **910 depth frame** khớp raw từng byte, **2.068 BEV grid** được tính lại. Loader độc lập ROS và observation độc lập nhãn PASS.
- `dataset01`, mọi FAIL/ERROR lịch sử, ROS/Nav2 mặc định giữ nguyên. Chưa train WM/RL. Pilot mới dùng `pilot_only`, không phải split train/validation/test.

## Đọc bộ mẫu không cần chạy simulator

Gói ZIP có thư mục `handoff_data_pilot01/`; giải nén vào một thư mục mới. Chỉ cần Python + NumPy:

```bash
cd handoff_data_pilot01
python tools/inspect_pilot.py --dataset .
```

Hoặc trong repository:

```bash
python experiments/social_navigation/datasets/handoff_pilot/inspect_pilot.py --dataset experiments/social_navigation/datasets/handoff_pilot/generated/pilot01/dataset
```

API NumPy hiện có được copy nguyên byte trong `tools/loader.py`:

```python
from pathlib import Path
import sys
root = Path('/absolute/path/handoff_data_pilot01')
sys.path.insert(0, str(root / 'tools'))
from loader import Dataset

e = Dataset(root).episode('pilot_conversation')
x = e.observation(50)
a = e.action_condition(50)
y = e.supervision(50)
m = e.task_masks(50)
```

Gói portable chứa arrays, calibration, nguồn/hash và audit; raw MCAP vẫn ở các `source_run` trên máy gốc, không nằm trong ZIP. Đọc mẫu không cần raw, ROS, Gazebo hoặc repository. Tái kiểm raw cần original MCAP và ROS Jazzy CDR reader.

## Một window và ý nghĩa mask

| Thành phần | Shape / ý nghĩa |
| --- | --- |
| Depth history |20×240×424,float32,m; giữ NaN/Inf; history query t−1,9s…t |
| LiDAR history |20×360; range + per-ray finite/range mask; frame-valid mask riêng |
| IMU history |20×10:qx,qy,qz,qw,wx,wy,wz,ax,ay,az; orientation-valid mask riêng |
| Odom history |20×9:local x,y,yaw tại estimated anchor,vx,vy,vz,wx,wy,wz |
| Action history / future condition |20×3 /30×3;vx,vy,wz; future executed ZOH tại t…t+2,9s |
| Current input BEV |64×64;depth hiện tại warp bằng odometry nhân quả |
| Future BEV |3×64×64 tại +0,5/+1/+3s;depth ghi thật, warp theo ego GT ở LABEL |
| Human trajectories |30×6×2 tương lai tại +0,1…+3s;GT LABEL ONLY |
| Group metadata |Theo frame thực tế; `group_id_future`,`group_mode_future`,`group_valid` trong LABEL |

Timestamp là integer simulation ns. Availability=`max(header_stamp,MCAP sim-clock reception)` là quy ước nhân quả bảo thủ, không phải đo độ trễ wall-clock. History có measurement và availability≤query≤anchor. Input goal dùng mục tiêu Nav2 đã khai báo và AMCL nhân quả; không lấy measured GT ego.

BEV: x tiến [0,6.4)m,y trái [−3.2,3.2)m,cell0.1m; −1=unknown,0=free,1=occupied. Chỉ tia depth hữu hạn trong[0.28,5.0]m được dùng. Đây là observed occupancy, không phải bản đồ omniscient hoặc social costmap. `cell_loss_valid` chỉ true cho ô quan sát được trong frame mục tiêu hợp lệ. `wm_observed_bev_train_valid` ở đây biểu thị target đủ dữ liệu theo schema cũ; **training_authorized=false và fit_allowed=false cho toàn pilot**.

H1–H5/persona, liên kết sensor-track với GT và reward RL đều chưa hợp lệ; mask=false,persona_index=−1,reward=NaN. Human IDs chỉ liên tục trong episode và chỉ nằm ở nhãn. Input API không chứa backend/seed/GT người/future sensor. Executed future actions có accessor riêng, không phải expert/counterfactual actions.

## Phần dùng lại và phần cần chuyển

Dùng lại hợp đồng dữ liệu, loader, hàm lấy mẫu nhân quả trong `python/dynamuhsim/datasets/normalization.py`, sensor/SE2/raster trong `datasets/perception_target_pilot`, WM targets trong `datasets/wm_episode_batch/targets.py`. Exporter mới ở `datasets/handoff_pilot/export.py`; adapter đọc raw hiện là MCAP/ROS, đầu ra arrays độc lập simulator.

Model người: SocNavGym kernel và safety/intent adapters vẫn ở `experiments/social_navigation/navigation/` và `python/dynamuhsim/simulation/`. Giao diện worker nhận world robot pose/velocity,time,dt=0,1s,epoch và trả human/group states. Chế độ trò chuyện `persistent_conversation_v1` đặt native `can_disperse=False` cho hai triplet; mode walking giữ lifecycle tự nhiên với `group_cohesion_v2` + `sim_v1`. Khi port, giữ riêng logic người với việc áp pose/render/collision trong MuJoCo.

Phần cần thay là dựng world/robot/sensor, simulator clock/pose adapter và bridge/output collector. Các SDF/Gazebo plugin/GZ transport, ROS launch teardown receipts là phần backend hiện tại; hãy xây adapter tương ứng và bằng chứng kết thúc episode cho MuJoCo.

Nguồn geometry đã nghiệm thu nằm trong `source_run/assets/mecanum_camera.sdf`, `mecanum_aligned.urdf`, `runtime/warehouse.sdf`, `runtime/nav2_runtime.yaml`; helper `python/dynamuhsim/simulation/robot_geometry.py`. World layout ở `experiments/social_navigation/warehouse/layout.json`. Robot collision AABB24×28cm,enclosing radius≈0,16806m;Nav2radius0,17m+padding0,01m,inflation0,45m. Action là planar body vx,vy,wz. Camera mount x0,12m,z0,08m trong base_link; lấy toàn bộ optical transform từ calibration của từng episode.

## Checklist nghiệm thu MuJoCo trước thu tiếp

1. Robot Mecanum nhận vx/vy/wz đúng trục/dấu/đơn vị; đo actual pose/odom và footprint trong world. Kiểm tra Nav2 goal và collision geometry bằng dữ liệu của backend mới.
2. Clock integer ns tăng liên tục trong episode; pause không làm tuổi mô phỏng tăng; reset tách epoch/episode. Giữ event ready→goal_accepted→action_end→observation_end và tail≥3,2s.
3. Depth là metric optical-Z trong camera frame với K/TF thực tế; kiểm tra tường/đồ vật ở khoảng cách biết trước. LiDAR geometry và invalid-ray mask,IMU thứ tự kênh/validity,odom/action frame cần thống nhất với calibration.
4. Chạy một cảnh trò chuyện giữ nhóm và một cảnh natural walking; ghi actual human/group state theo từng frame. Kiểm tra goal,safety,gap và tính toàn vẹn sensor; nhóm walking được phép đổi thành viên.
5. Xuất cùng file/API và chạy kiểm tra nhân quả,raw→depth,action resampling,BEV warp/masks,standalone loader và label independence. Điều kiện physical/shutdown đặc thù MuJoCo phải có protocol riêng; các audit FAIL Gazebo cũ giữ nguyên.
6. Sau khi backend mới đạt, khai báo variation/seed/split và thu batch. Chưa train trên hai episode smoke này; chúng dùng các seed đã xem khi kiểm chứng simulator.

## Giới hạn đã biết

Một nhà xưởng, cùng avatar, tuyến robot và ít seed; chưa chứng minh tổng quát. Ba lượt trò chuyện giữ nhóm không có actual sidestep, nên active-yield correction NOT_EXERCISED. Clock cảnh đứng xác minh snapshot age; arrival/expiry query NOT_EXERCISED. Walking dùng query verifier gốc. Thứ tự dừng bốn bridge đã qua hai lượt bản sửa mới, không có cam kết hết mọi race condition; mọi episode mới vẫn cần audit.

Báo cáo current: `docs/START_HERE_CURRENT.md`, `docs/demos/2026-10-29/handoff_pilot/`. Đây là mốc dừng hợp lý của phase Gazebo sau kiểm tra pilot dữ liệu nhỏ; người tiếp nhận có thể bắt đầu adapter MuJoCo.
