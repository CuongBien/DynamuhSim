# DYNAMUHSIM WORLD MODEL — DATASET PROFILE FOR AI

> Automatically generated from ACTUAL locally available dataset. Episode-level counts come from dataset.json; numerical distributions are SAMPLED and MUST NOT be quoted as full-dataset exhaustive measurements.

**Root:** `/home/thaonhi/nav_ws/wm_restore/wm_episode_batch/generated/dataset01`
**Dataset schema:** `wm_sensor_action_dataset/1`
**Dataset backend:** `SocNavGym`
**Episode counts:** 8 (split: {'train': 5, 'validation': 2, 'test': 1}; scenario: {'standing': 4, 'walking': 4})
**Test labels inspected:** False

## 1. What the dataset represents

- Real Gazebo-recorded episode traces. The robot is controlled by Nav2/MPPI; humans are simulated by a SocNavGym response worker.
- Observation history: 20 queries at 10Hz, ~2 seconds; LiDAR native 5Hz may be held with age mask; Depth 240×424.
- Candidate future action conditioning in this corpus: 30 *actually executed* robot actions at 10Hz (t..t+2.9s), **not** outcomes for alternative candidate actions.
- Future human GT: 30 steps × 6 IDs × XY in true robot frame at anchor; human detection/tracker association is NOT validated.
- Future observed BEV targets: [3,64,64] at +0.5/+1/+3.0s; 0.1m cells, x forward [0,6.4), y left [-3.2,3.2); -1 unknown, 0 observed free, 1 observed occupied.
- Future BEV comes from recorded future Depth frames (LABEL) and simulator ego-GT transforms, NOT neural predictions or complete 3D world state.
- H1–H5/persona and M1 sensor-track associations are NOT validated; reward is NaN and reward_valid=false; no completed PPO/World Model training.
- The independent experiment unit is EPISODE, NOT overlapping sliding windows. Do not random-split windows.

## 2. Episode table (metadata; exact)

| Episode | Split | Scenario | Seed | Windows | WM-valid | Nav2 status | Goal reached | Profiled windows |
|---|---|---|---:|---:|---:|---|---|---:|
| wm_standing_45 | train | standing | 45 | 851 | 851 | TIMEOUT | False | 120 |
| wm_walking_45 | train | walking | 45 | 597 | 597 | SUCCEEDED | True | 120 |
| wm_standing_46 | train | standing | 46 | 259 | 259 | SUCCEEDED | False | 120 |
| wm_walking_46 | train | walking | 46 | 268 | 268 | SUCCEEDED | True | 120 |
| wm_walking_47 | train | walking | 47 | 800 | 788 | TIMEOUT | False | 120 |
| wm_standing_48 | validation | standing | 48 | 802 | 802 | TIMEOUT | False | 120 |
| wm_walking_48 | validation | walking | 48 | 850 | 850 | TIMEOUT | False | 120 |
| wm_standing_49 | test | standing | 49 | 679 | 679 | SUCCEEDED | True | 0 |

## 3. Distribution statistics (sampled non-test; not full census)

- **depth_m**: {"count": 71530, "nonfinite": 0, "min": 0.29733, "p05": 0.30292, "p25": 0.41965, "median": 0.8249, "p75": 1.87194, "p95": 3.43103, "max": 4.99506, "mean": 1.32413}
- **human_displacement_3s**: {"count": 5040, "nonfinite": 0, "min": 0.0, "p05": 0.0, "p25": 0.0, "median": 0.24887, "p75": 1.37537, "p95": 2.10525, "max": 2.38848, "mean": 0.67682}
- **human_displacement_3s_standing**: {"count": 2160, "nonfinite": 0, "min": 0.0, "p05": 0.0, "p25": 0.0, "median": 0.0, "p75": 0.89057, "p95": 2.10904, "max": 2.38848, "mean": 0.47161}
- **human_displacement_3s_walking**: {"count": 2880, "nonfinite": 0, "min": 0.0, "p05": 0.0, "p25": 0.0, "median": 0.68064, "p75": 1.51559, "p95": 2.09998, "max": 2.38786, "mean": 0.83072}
- **lidar_min**: {"count": 840, "nonfinite": 0, "min": 0.27313, "p05": 0.76912, "p25": 0.85115, "median": 0.95185, "p75": 1.42473, "p95": 2.37942, "max": 2.98098, "mean": 1.19688}
- **vx**: {"count": 840, "nonfinite": 0, "min": -0.15054, "p05": -0.04737, "p25": 0.00131, "median": 0.14345, "p75": 0.27001, "p95": 0.29848, "max": 0.3, "mean": 0.13084}
- **vy**: {"count": 840, "nonfinite": 0, "min": -0.20014, "p05": -0.17071, "p25": -0.00292, "median": 0.06701, "p75": 0.17163, "p95": 0.19966, "max": 0.20049, "mean": 0.06937}
- **wz**: {"count": 840, "nonfinite": 0, "min": -0.8621, "p05": -0.30002, "p25": -0.09984, "median": -0.02378, "p75": 0.03209, "p95": 0.14205, "max": 0.58592, "mean": -0.04465}

### Mask validity (sampled for sensors; full per-episode masks where available)

- **LiDAR ray valid**: 95272/302400 = 0.31505
- **action history valid**: 16800/16800 = 1.0
- **depth history valid**: 16800/16800 = 1.0
- **depth valid pixels (sampled)**: 71530/133560 = 0.53556
- **executed_action_condition_valid**: 4427/4427 = 1.0
- **goal_valid**: 701/840 = 0.83452
- **imu history valid**: 16800/16800 = 1.0
- **m1_track_aligned_valid**: 0/796860 = 0.0
- **odom history valid**: 16800/16800 = 1.0
- **persona_valid**: 0/26562 = 0.0
- **rl_reward_valid**: 0/4427 = 0.0
- **scan history valid**: 16800/16800 = 1.0
- **wm_observed_bev_train_valid**: 4415/4427 = 0.99729

### Sampled future BEV cell balance

- +0.5s: unknown=0.759, free=0.23011, occupied=0.01089
- +1.0s: unknown=0.76053, free=0.22873, occupied=0.01074
- +3.0s: unknown=0.77246, free=0.21761, occupied=0.00992

## 4. Data dictionary (unique file/field definitions)

| Role | File | Field | dtype | Example shape | Meaning |
|---|---|---|---|---|---|
| OBSERVATION_INPUT | `bev_anchor_estimated_SENSOR_ONLY.npz` | `anchor_ns` | `int64` | `[851]` | Thoi diem neo cua cua so, nanosecond simulation. |
| OBSERVATION_INPUT | `bev_anchor_estimated_SENSOR_ONLY.npz` | `anchor_odom_available_ns` | `int64` | `[851]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `bev_anchor_estimated_SENSOR_ONLY.npz` | `anchor_odom_stamp_ns` | `int64` | `[851]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `bev_anchor_estimated_SENSOR_ONLY.npz` | `depth_index` | `int32` | `[851]` | [N,20] indices tro vao depth_SENSOR_ONLY.npy; khong phai mang pixel. |
| OBSERVATION_INPUT | `bev_anchor_estimated_SENSOR_ONLY.npz` | `depth_stamp_ns` | `int64` | `[851]` | Timestamp Depth frame da su dung. |
| OBSERVATION_INPUT | `bev_anchor_estimated_SENSOR_ONLY.npz` | `frame_valid` | `bool` | `[851]` | Mask bo du lieu BEV o muc frame/horizon. |
| OBSERVATION_INPUT | `bev_anchor_estimated_SENSOR_ONLY.npz` | `occupancy` | `int8` | `[851, 64, 64]` | BEV -1 unknown, 0 observed free, 1 observed occupied; vai tro input/label phu thuoc file. |
| OBSERVATION_INPUT | `bev_anchor_estimated_SENSOR_ONLY.npz` | `source_odom_available_ns` | `int64` | `[851]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `bev_anchor_estimated_SENSOR_ONLY.npz` | `source_odom_stamp_ns` | `int64` | `[851]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `depth_SENSOR_ONLY.npy` | `<npy-array>` | `float32` | `[1038, 240, 424]` | Raw float32 depth frames [F,240,424]; NaN/Inf retained; index via depth_index. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `action__available_ns` | `int64` | `[4015]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `action__received_ns` | `int64` | `[4015]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `action__stamp_ns` | `int64` | `[4015]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `action__values` | `float32` | `[4015, 3]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `amcl__available_ns` | `int64` | `[185]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `amcl__received_ns` | `int64` | `[185]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `amcl__stamp_ns` | `int64` | `[185]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `amcl__values` | `float32` | `[185, 3]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `depth__available_ns` | `int64` | `[1038]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `depth__received_ns` | `int64` | `[1038]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `depth__stamp_ns` | `int64` | `[1038]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `depth__values` | `float32` | `[1038, 1]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `imu__available_ns` | `int64` | `[10370]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `imu__received_ns` | `int64` | `[10370]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `imu__stamp_ns` | `int64` | `[10370]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `imu__values` | `float32` | `[10370, 10]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `odom__available_ns` | `int64` | `[2962]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `odom__received_ns` | `int64` | `[2962]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `odom__stamp_ns` | `int64` | `[2962]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `odom__values` | `float32` | `[2962, 9]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `scan__available_ns` | `int64` | `[519]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `scan__received_ns` | `int64` | `[519]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `scan__stamp_ns` | `int64` | `[519]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `scan__valid` | `bool` | `[519, 360]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `observations_SENSOR_ONLY.npz` | `scan__values` | `float32` | `[519, 360]` | Undocumented by this script; inspect source or protocol. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `action` | `float32` | `[851, 20, 3]` | [N,20,3]: lenh da thuc hien trong history, vx/vy/wz (base axes). |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `action_available_ns` | `int64` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `action_stamp_ns` | `int64` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `action_valid` | `bool` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `anchor_ns` | `int64` | `[851]` | Thoi diem neo cua cua so, nanosecond simulation. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `depth_available_ns` | `int64` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `depth_index` | `int32` | `[851, 20]` | [N,20] indices tro vao depth_SENSOR_ONLY.npy; khong phai mang pixel. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `depth_stamp_ns` | `int64` | `[851, 20]` | Timestamp Depth frame da su dung. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `depth_valid` | `bool` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `goal_available_ns` | `int64` | `[851]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `goal_stamp_ns` | `int64` | `[851]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `goal_valid` | `bool` | `[851]` | Mask goal con hop le; stale >1 giay -> zero + false. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `goal_xy_from_amcl` | `float64` | `[851, 2]` | [N,2] goal trong he robot tinh theo AMCL nhan qua, co goal_valid. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `history_query_ns` | `int64` | `[851, 20]` | 20 query tu t-1.9s toi t, 10Hz; timestamp cua observation history. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `imu` | `float32` | `[851, 20, 10]` | [N,20,10]: quaternion xyzw + angular velocity xyz + linear acceleration xyz. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `imu_available_ns` | `int64` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `imu_stamp_ns` | `int64` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `imu_valid` | `bool` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `odom_available_ns` | `int64` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `odom_local_pose_and_twist` | `float32` | `[851, 20, 9]` | [N,20,9]: local XY/yaw + linear xyz + angular xyz; estimated, khong phai simulator GT. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `odom_stamp_ns` | `int64` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `odom_valid` | `bool` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `scan_available_ns` | `int64` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `scan_ranges_m` | `float32` | `[851, 20, 360]` | [N,20,360] LiDAR range (m) co mask; invalid ray da duoc ghi 0. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `scan_ray_valid` | `bool` | `[851, 20, 360]` | Mask cho moi ray LiDAR; khong lay value=0 lam khoang cach that. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `scan_stamp_ns` | `int64` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| OBSERVATION_INPUT | `samples_SENSOR_ONLY.npz` | `scan_valid` | `bool` | `[851, 20]` | Timestamp ns / availability ns / validity mask for the named modality. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `anchor_ns` | `int64` | `[851]` | Thoi diem neo cua cua so, nanosecond simulation. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `anchor_pose_world_gt` | `float64` | `[851, 3]` | Simulator ground-truth robot pose tai anchor, CHI nhan/evaluation. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `cell_loss_valid` | `bool` | `[851, 3, 64, 64]` | Mask cell observed va frame supported; chi tinh loss tai mask true. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `depth_index` | `int64` | `[851, 3]` | [N,20] indices tro vao depth_SENSOR_ONLY.npy; khong phai mang pixel. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `depth_stamp_ns` | `int64` | `[851, 3]` | Timestamp Depth frame da su dung. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `depth_time_error_ns` | `int64` | `[851, 3]` | Sai so thoi gian cua target Depth frame. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `frame_valid` | `bool` | `[851, 3]` | Mask bo du lieu BEV o muc frame/horizon. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `future_pose_world_gt` | `float64` | `[851, 3, 3]` | Simulator ground-truth robot poses cho future depth target, CHI nhan/evaluation. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `occupancy` | `int8` | `[851, 3, 64, 64]` | BEV -1 unknown, 0 observed free, 1 observed occupied; vai tro input/label phu thuoc file. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `offset_ns` | `int64` | `[3]` | [3] horizon +0.5/+1/+3 giay cho future observed BEV. |
| SUPERVISION_GT_ONLY | `future_observed_bev_LABEL_ONLY.npz` | `query_ns` | `int64` | `[851, 3]` | Moc thoi gian can lay nhan Depth tuong lai. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `executed_action_available_ns` | `int64` | `[851, 30]` | Timestamp availability cua lenh da ghi. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `executed_action_future` | `float32` | `[851, 30, 3]` | [N,30,3] lenh robot da thuc hien t..t+2.9s; action condition tren mot trajectory, khong phai counterfactual. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `executed_action_query_ns` | `int64` | `[851, 30]` | [N,30] moc hoi cua 30 lenh tuong lai. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `executed_action_stamp_ns` | `int64` | `[851, 30]` | Timestamp cua lenh tuong lai da ghi. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `future_stamp_ns` | `int64` | `[851, 30]` | [N,30] moc thoi gian nhan quy dao nguoi tu t+0.1..t+3.0s. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `group_id_future` | `int32` | `[851, 30, 6]` | Nhom nguoi tuong lai (LABEL simulator); khong phai observation. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `group_mode_future` | `int8` | `[851, 30, 6]` | Nhom 0:none, 1:walking, 2:standing (LABEL). |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `human_ids` | `int32` | `[6]` | Danh sach ID con nguoi chi on dinh trong episode. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `human_valid` | `bool` | `[851, 30, 6]` | Support noi suy GT, KHONG phai nhan da detect duoc nguoi. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `human_xy_at_anchor_m` | `float64` | `[851, 6, 2]` | [N,6,2] vi tri nguoi tai anchor, LABEL GT. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `human_xy_future_m` | `float64` | `[851, 30, 6, 2]` | [N,30,6,2] quy dao 6 nguoi trong he robot that tai anchor, LABEL GT. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `persona_index` | `int8` | `[851, 6]` | Danh muc persona; hien tai -1/unavailable, KHONG phai H1-H5 ground truth. |
| SUPERVISION_GT_ONLY | `future_trajectories_actions_LABEL_ONLY.npz` | `persona_valid` | `bool` | `[851, 6]` | Mask persona; hien tai tat ca false, khong train head H1-H5. |
| SUPERVISION_GT_ONLY | `task_masks_LABEL_ONLY.npz` | `executed_action_condition_valid` | `bool` | `[851]` | Mask action condition hop le. |
| SUPERVISION_GT_ONLY | `task_masks_LABEL_ONLY.npz` | `m1_track_aligned_valid` | `bool` | `[851, 30, 6]` | Mask lien ket tracker-sensor voi ground truth ID; hien tai false. |
| SUPERVISION_GT_ONLY | `task_masks_LABEL_ONLY.npz` | `persona_valid` | `bool` | `[851, 6]` | Mask persona; hien tai tat ca false, khong train head H1-H5. |
| SUPERVISION_GT_ONLY | `task_masks_LABEL_ONLY.npz` | `rl_reward` | `float32` | `[851]` | Reward RL; hien tai NaN/unavailable; KHONG doi NaN thanh zero. |
| SUPERVISION_GT_ONLY | `task_masks_LABEL_ONLY.npz` | `rl_reward_valid` | `bool` | `[851]` | Mask reward RL; hien tai false. |
| SUPERVISION_GT_ONLY | `task_masks_LABEL_ONLY.npz` | `wm_observed_bev_train_valid` | `bool` | `[851]` | Mask ho tro task WM observed future BEV, KHONG phai quyen train test. |

## 5. File roles

- `samples_SENSOR_ONLY.npz` — Sensor observation windows (LiDAR, IMU, odom, cmd_vel, goal, frame indices and masks).
- `observations_SENSOR_ONLY.npz` — Full timestamped sensor arrays retained for audits; not the recommended causal reader.
- `depth_SENSOR_ONLY.npy` — Raw float32 depth frames [F,240,424]; NaN/Inf retained; index via depth_index.
- `bev_anchor_estimated_SENSOR_ONLY.npz` — Causal input BEV from Depth and odometry, estimated anchor.
- `calibration_SENSOR_ONLY.json` — Camera K/P/D, optical/base frames, extrinsics.
- `lidar_geometry_SENSOR_ONLY.json` — LiDAR angular/range geometry and units.
- `future_trajectories_actions_LABEL_ONLY.npz` — Future simulator human GT trajectories + executed future action conditioning.
- `future_observed_bev_LABEL_ONLY.npz` — Future observed-depth occupancy labels + per-cell/per-frame masks.
- `human_states_LABEL_ONLY.jsonl` — Per-time human/robot simulator GT; NEVER online robot input.
- `task_masks_LABEL_ONLY.npz` — Validity flags for WM, association, persona and RL reward.
- `episode_end_LABEL_ONLY.json` — Navigation terminated/truncated metadata; not validated RL reward.
- `episode.json` — Canonical episode metadata: ID, split, scenario, seed, counts and outcome.
- `inventory.json` — File checksums and size evidence at episode level.
- `dataset.json` — Canonical dataset manifest and episode splits.
- `capture_protocol.json` — Collection protocol and scene/control assumptions.
- `export_protocol.json` — Schema/target export definitions.
- `quarantine_review.json` — Quarantine decisions for failed capture attempts.
- `capture_result.json` — Capture runtime and navigation result provenance.
- `capture_audit.json` — Checks of capture validity.
- `raw_export_audit.json` — Checks of raw MCAP to normalized arrays.

## 6. Important quality / research caveats

- `*_SENSOR_ONLY` is the inference-time branch. `*_LABEL_ONLY` contains privileged future/human GT and must NOT be concatenated into online observation features.
- `human_valid` means simulator interpolation supported; not human seen in camera or associated to track ID.
- `wm_observed_bev_train_valid` is task validity and does NOT override the train/validation/test split.
- BEV `-1` is unknown, never free; calculate supervised loss only on `cell_loss_valid`.
- One action sequence per recorded state is observational, not a controlled intervention/counterfactual set. Dataset does not prove robot-action causation of human response.
- A high proportion of standing windows does not establish H4 persona; grouped standing members may sidestep and group lifecycle may change.
- 8 accepted episodes in the scanned dataset, a small batch from one route/simulator. Overlapping windows do not increase effective number of independent trials.
- Test is held out by default. This script does not measure World Model prediction accuracy because no model predictions were provided.
- Check provenance when extracting restored data; raw MCAP, compiled plugin and original experimental environment may be absent in a Git clone.

## 7. Integrity signals (sampled)

- Causal timestamps valid: 84000/84000 eligible sampled modality-history timestamps.
- Non-test task-eligible WM windows (sum of explicit masks): 4415
- Array schema field rows: 644; field header errors: 0

## 8. Outputs & provenance

- `DATASET_CONTEXT.json`: machine-readable dataset metadata, episodes, sample stats, full unique field dictionary.
- `FIELD_SCHEMA.csv`: every array key + exact [shape,dtype] by file (excluding sealed test unless opted in).
- `EPISODE_TABLE.csv`: exact canonical metadata episode table.
- `ALL_PATHS.csv` and `DIRECTORY_TREE.txt`: produced by 01_inventory.py, not repeated inside this prompt.
- PNG charts: 01_windows_by_episode.png, 02_vx.png, 03_vy.png, 04_wz.png, 05_lidar_min.png, 06_depth_m.png, 07_human_displacement_3s.png, 08_future_bev_balance.png

## 9. Warnings / skipped items

- SEALED test episode wm_standing_49: metadata only; no input/label arrays sampled.

