# Vòng 2 — CrowdSimPlus độc lập

Vòng 1 đã commit tại **91f3620**. Vòng này dùng CrowdSimPlus/SICNav bản T-RO
2024, commit `d526a07c013c621fa1ad50894b6ba639f61063de`, với Python **3.8.13**
trong Docker. Không đổi Python 3.12, ROS 2 Jazzy, Gazebo hoặc code robot.

## Chạy lại

Từ thư mục gốc DynamuhSim:

```bash
bash experiments/social_navigation/round2/run.sh build
bash experiments/social_navigation/round2/run.sh demo
```

`demo` chạy **sicnav/simple_test.py nguyên bản**, DWA và config nguyên bản,
test_case 0. Demo ghi MP4 và pickle summary vào reports/official_run; không mở
cửa sổ GUI vì container dùng Matplotlib Agg. Có thể mở video sau khi chạy.

Container hiện đã build sẵn với tag `dynamuhsim-crowdsimplus:round2`. Khi chạy,
container không có mạng và chỉ mount thư mục round2 để ghi kết quả. Image dùng
Debian 11 + Python 3.8.13; Ubuntu 24.04 là host. Không phải chứng nhận chạy native
Python 3.12 hay tương thích ROS Jazzy trực tiếp. Image khoảng 3.5GB; không cần GPU.

Một lượt quan sát tùy chỉnh:

```bash
bash experiments/social_navigation/round2/run.sh scene \
  --scenario circle --humans 8 --human-policy orca_plus --seed 0 --video

bash experiments/social_navigation/round2/run.sh scene \
  --scenario bottleneck --humans 3 --human-policy sfm --seed 2 --video
```

- `--scenario`: circle, hallway hoặc bottleneck.
- `--human-policy`: orca_plus hoặc sfm.
- `--robot-policy`: mặc định dwa (unicycle); có tùy chọn orca_plus (holonomic),
  nhưng ma trận vòng này chỉ kiểm tra DWA, chưa kiểm chứng tùy chọn holonomic.
- `--seed`: upstream test_case, framework tự cộng offset theo phase.
- `--invisible-robot`: người không được đưa trạng thái robot vào observation.
- `--seconds`: giới hạn mô phỏng, mặc định 30s; timestep 0.25s.
- `--video`: renderer gốc xuất MP4. Renderer upstream phát 8fps với timestep
  0.25s, nên video chạy **2× tốc độ mô phỏng**, không phải thời gian thực.

Chạy toàn bộ 24 cấu hình hoặc chỉ các lượt chưa chạy/bị lỗi:

```bash
bash experiments/social_navigation/round2/run.sh matrix
bash experiments/social_navigation/round2/run.sh matrix --resume
```

Các lượt chạy tuần tự; ba test_case 0,1,2 cho mỗi điều kiện. Runner ngoài source
lưu config và trạng thái người/robot từng bước. Tất cả kết quả cùng tên sẽ bị
ghi đè khi chạy lại. `run_matrix.py` có timeout 180s thời gian thực cho mỗi lượt.

## Kết quả và giới hạn

Xem [báo cáo](reports/ROUND2.md), [bảng CSV](reports/results.csv) và
[tổng hợp JSON](reports/analysis.json). Demo gốc chạy thành công. Ma trận cuối
hoàn tất 24 lượt, gồm cả timeout mô phỏng là kết quả hợp lệ. Ban đầu nhánh robot
vô hình bị lỗi thiếu `env.mode`; runner thêm alias `env.mode='test'` cho nhánh
logging này, không chỉnh thuật toán hoặc file source upstream.

CrowdSimPlus có ORCA/SFM và static line obstacles, nhưng bản thử chưa có model
nhóm đi, nhóm trò chuyện, hình thành/tan nhóm hoặc group ID như SocNavGym.
Những người tình cờ đi gần nhau không được gọi là một nhóm xã hội.

Chưa nối simulator vào Gazebo hoặc chạy trong nhà xưởng vòng 1. Không so kết
quả DWA unicycle 2D với MPPI Mecanum trong ROS. Chưa chạy ComplexityNav.

## Phân tích lại dữ liệu

Có thể dùng venv phân tích sẵn có từ vòng 1; việc này chỉ đọc JSON/MP4, không
chạy CrowdSimPlus trong venv đó:

```bash
experiments/social_navigation/.venv-socnavgym/bin/python \
  experiments/social_navigation/round2/analyze.py
```

Dockerfile ghim Python, source và các dependency quan trọng. `requirements.txt`
chứa thư viện phân tích/runtime; Gym 0.21 và Torch CPU 1.13.1 được ghim trực tiếp
trong Dockerfile. HSL/Acados, diffusion prediction và SB3 training không cài vì
không dùng cho demo DWA/simulator. Manifest và freeze thực tế ở reports.

## Cập nhật sau vòng độc lập

CrowdSimPlus ORCA+ đã được đưa vào [Gazebo qua adapter Docker riêng](../crowdsimplus_adapter/README.md), giữ robot Mecanum/Nav2 hiện tại. Xem [report tích hợp](../benchmark/crowdsim_comparison/reports/run01/CROWDSIM_GAZEBO_REPORT.md); kết quả vòng 2 phía trên vẫn là standalone 2D, không được trộn với benchmark Gazebo.
