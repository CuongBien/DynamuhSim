# DynamuhSim — bàn giao backend MuJoCo

**Nhánh làm việc: `feat/mujoco-backend`.** Dự án robot Mecanum trên Ubuntu 24.04 + ROS 2 Jazzy, có LiDAR 2D, depth camera, IMU và Nav2. Mục tiêu nhánh này là thêm backend MuJoCo, tái sử dụng hành vi người và pipeline dữ liệu, giữ Gazebo làm tham chiếu.

**MuJoCo hiện chưa được triển khai.** Nhánh đã có workspace riêng, task manifest, hướng dẫn và preflight. WM/RL chưa train; detector 3D mới vẫn experimental và không đạt gate thay bản cũ.

## Người nhận bắt đầu ở đây

| Đọc theo thứ tự | Nội dung |
|---|---|
| **[MUJOCO_HANDOVER.md](docs/MUJOCO_HANDOVER.md)** | Tổng quan dự án, clone/setup, cách chạy, interface ROS, mốc cần port và nghiệm thu |
| [HANDOFF_SETUP.md](docs/HANDOFF_SETUP.md) | Dependency tách ba mức: đọc data / export MCAP / Gazebo reference |
| [BACKEND_BOUNDARY.md](docs/BACKEND_BOUNDARY.md) | Ownership source, frame vận tốc, human worker, clock/reset và source freeze |
| [HANDOFF_MUJOCO.md](docs/HANDOFF_MUJOCO.md) | Hợp đồng dữ liệu đã chốt, shape/mask, geometry và checklist backend mới |
| [Workspace MuJoCo](experiments/social_navigation/mujoco_backend/README.md) | Nơi thêm code/assets/checks; hiện là scaffold |

## Clone đúng nhánh và kiểm tra nhanh

```bash
git clone --branch feat/mujoco-backend https://github.com/CuongBien/DynamuhSim.git
cd DynamuhSim
python3 tools/mujoco_handoff.py check
```

Cần Python 3.12 + NumPy; nếu thiếu, làm theo [setup tối thiểu](docs/MUJOCO_HANDOVER.md#setup-tối-thiểu). Lệnh trên không mở simulator hoặc tự cài software. PASS là code/contracts đã kiểm; trường `mujoco_backend=NOT_IMPLEMENTED` vẫn được giữ. Khi chưa nhận ZIP, portable_data được ghi NOT_CHECKED.

Nhận thêm `handoff_data_pilot01.zip` qua artifact riêng rồi kiểm:

```bash
python3 tools/mujoco_handoff.py check \
  --archive /absolute/path/handoff_data_pilot01.zip \
  --restore-dir /absolute/path/NEW_RESTORE_DIR \
  --output /absolute/path/NEW_RECEIPT.json
```

**Full ZIP/raw/generated không nằm trong Git.** Pilot có 2 episode, 517 window, 910 depth frame; reader không cần ROS/Gazebo. SHA256 ZIP: `b64bdb394f4a2ddabc6d97f2e9ccd3704dd9c6b0855e5d52fb11c6eeaea23ee2`. [Artifact và dữ liệu mẫu](docs/demos/2026-10-29/handoff_pilot/README.md).

## Mốc tham chiếu và quy tắc làm việc

- Giữ nhánh **`test/gazebo-harmonic-benchmark` tại `dd7a2b0`** làm mốc code trước port. Source/data baseline được guard là `121a727`; hai mốc này có vai trò khác nhau.
- Thêm MuJoCo dưới `experiments/social_navigation/mujoco_backend/`, dùng launch/entrypoint riêng. Các thư mục con hiện chỉ có README, không phải implementation đã chạy.
- Giữ code Gazebo, raw, reports, nguồn freeze và các outcome FAIL/ERROR. Không sửa checksum cũ để chấp nhận code mới.
- Có thể giữ Nav2/logic người/loader; physics, sensor outputs và kết quả điều hướng vẫn cần nghiệm thu lại trên engine mới.
- H1–H5/persona, sensor↔GT association, reward RL và quyền train hiện chưa hợp lệ. GT chỉ ở LABEL/evaluator, không làm input điều khiển/detector.

## Luồng hiện tại của dự án

| Thành phần | Nguồn chính / trạng thái |
|---|---|
| Nhà xưởng + robot/cảm biến tham chiếu | [warehouse](experiments/social_navigation/warehouse/), [robot geometry](python/dynamuhsim/simulation/robot_geometry.py), [capture v7](configs/capture/geometry_v7_mppi_resample.json) |
| Điều hướng | AMCL + NavFn/Dijkstra + MPPI Omni; [runtime Nav2](docs/SHARED_NAV2_RUNTIME.md) |
| Người | SocNavGym: persistent conversation và natural walking/group_cohesion_v2; [backend contract](configs/handoff/backend_contract.json) |
| Dữ liệu | [handoff pilot](experiments/social_navigation/datasets/handoff_pilot/README.md), [schema/loader](docs/HANDOFF_MUJOCO.md) |
| Perception | [detector thử nghiệm](docs/DEPTH_PERSON_EXPERIMENT.md), [kết quả + overlay](docs/verification/depth_detector_overlays_2026-10-30/README.md); chưa đạt tổng quát hóa |
| Công việc port | [mujoco_port.json](configs/handoff/mujoco_port.json), trạng thái M1–M6 đều NOT_STARTED |

## Chạy Gazebo tham chiếu và đọc lịch sử

Dùng [runbook](docs/MUJOCO_HANDOVER.md#chạy-gazebo-tham-chiếu) và [setup Gazebo](docs/HANDOFF_SETUP.md#3-chạy-lại-gazebo-tham-chiếu). `tools/handoff.py reference-command` in rõ recipe v7; có thể thêm `--gui` vào lệnh prepare trước khi tạo run mới. Không dùng default v3 lịch sử khi muốn tái lập recipe đã chốt.

Tài liệu sâu: [capture](docs/CAPTURE_RUNTIME.md), [world assets](docs/WORLD_ASSETS.md), [artifact/hydration](docs/HISTORICAL_ARTIFACTS.md), [dữ liệu hiện tại](docs/START_HERE_CURRENT.md). [README Gazebo đầy đủ trước bàn giao](https://github.com/CuongBien/DynamuhSim/blob/dd7a2b03d9fe40841052c0db6c7dc52619e13e67/README.md) giữ các lệnh corridor/hospital/school và experiment cũ; chúng có config/controller riêng, không đại diện recipe nhà xưởng v7.
