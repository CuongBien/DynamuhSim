# Setup bàn giao theo nhu cầu

Nhánh port: feat/mujoco-backend. [Runbook người nhận](MUJOCO_HANDOVER.md), [setup tối thiểu và fallback khi thiếu ensurepip](MUJOCO_HANDOVER.md#setup-tối-thiểu), preflight một lệnh: `python3 tools/mujoco_handoff.py check`.

Reference đã chạy:Ubuntu24.04/WSL2,Python3.12.3,ROS2Jazzy,GazeboHarmonic,NVIDIA RTX4060. Phiên bản thực tế và vendor commits nằm ở `configs/handoff/reference_environment.json`;đây là inventory đã dùng,không phải tuyên bố mọi hệ điều hành/version đều đã kiểm chứng.

## 1. Chỉ đọc dữ liệu / chuẩn bị MuJoCo contract

Cần Python + NumPy. Có thể tạo venv riêng:

```bash
python3 -m venv .venv-handoff-data
source .venv-handoff-data/bin/activate
python -m pip install -r requirements/handoff-data.txt
python tools/handoff.py doctor --scope data
```

Trên Windows dùng Python/venv tương ứng;portable data/loader chỉ dùng NumPy. Không cần ROS/Gazebo/torch/SocNavGym khi đọc ZIP. `pyproject.toml` mô tả shared library,không tự provision toàn bộ ROS/simulator stack.

Nhận ZIP `handoff_data_pilot01.zip` qua artifact bàn giao. SHA256 đã chốt:

```text
b64bdb394f4a2ddabc6d97f2e9ccd3704dd9c6b0855e5d52fb11c6eeaea23ee2
```

Chạy `smoke-data --archive ... --work-dir ...` như HANDOFF_START. Full data không tải về chỉ bằng git clone. Inspector,arrays,calibration,masks,audit có trong ZIP;raw MCAP vẫn ở máy nguồn. Không ghi đè một thư mục đã restore.

## 2. Đọc MCAP / export / audit raw

Dùng Python hệ thống tương thích ROS Jazzy. Reference dùng NumPy1.26.4,SciPy1.11.4,Matplotlib3.6.3,Pillow10.2.0,PyYAML6.0.1 từ môi trường Ubuntu cùng ROS bindings. Inventory là số đo hệ thống;không dùng một pip lock của môi trường SocNavGym để ghi đè Python hệ thống.

```bash
source /opt/ros/jazzy/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 tools/handoff.py doctor --scope export
```

Doctor import-check các module thực sự cần,không chạy node,install package hoặc gọi simulator. Nếu thiếu module Ubuntu/ROS,chuẩn bị dependency đó trong môi trường phù hợp rồi chạy lại doctor. `requirements/handoff-data.txt` chỉ dành cho reader,không thay thế việc cài ROS Jazzy/bindings. Muốn export lại phải có raw source đúng protocol và chọn output mới;hướng dẫn ở `experiments/social_navigation/datasets/handoff_pilot/README.md`.

## 3. Chạy lại Gazebo tham chiếu

Dùng checkout/build cùng máy. Cần ROS packages build trong `src`,turtlebot/resources,venv/vendor SocNavGym,response adapter,depth filter và SDFormat checker. Các bước dựng hiện có ở [CAPTURE_RUNTIME](CAPTURE_RUNTIME.md#chuẩn-bị-môi-trường). Dùng `colcon build --base-paths src --executor sequential` khi dựng ROS workspace để không quét nested checkout khác.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 tools/handoff.py doctor --scope gazebo
```

Mode này kiểm thêm native module/tools/binary paths,venv và query tên GPU/driver bằng nvidia-smi. Đây là kiểm tra prerequisite,**actual rendering NOT_RUN**;NVIDIA query không thay hardware-renderer acceptance của một capture. Runtime Gazebo tham chiếu hiện có wrapper WSL D3D12 nên doctor báo rõ dependency `/usr/lib/wsl/lib/libd3d12core.so`;máy Ubuntu native/MuJoCo cần backend setup/acceptance tương ứng.

`setup_socnavgym.sh` đã có vendor commit pins và venv riêng;script từ chối vendor checkout có thay đổi. Xem script/requirements trước khi chủ động chạy;doctor không gọi script,clone hoặc reset vendor. Không chuyển build/install/venv từ máy cũ thành dependency ngầm của người nhận.

## Kiểm tra nhanh và giới hạn

`python3 tools/handoff.py baseline` kiểm116protected byte hashes,không cần raw/venv. `smoke-contract` gọi các test sẵn có cho causal timing,frame/depth,goal clock,group lifecycle,ready/bridge ownership vàportable reader;không khởi động Gazebo. Native robot/world/perception acceptance đã có ở receipt mốc bàn giao;vòng cleanup không thay thông số/thuật toán nên không thu thêm native cohort.

Không có bước cài MuJoCo hoặc trainer trong setup này. Việc port thuộc phase sau;không cần cài toàn bộ Gazebo stack để bắt đầu đọc schema và pilot.
