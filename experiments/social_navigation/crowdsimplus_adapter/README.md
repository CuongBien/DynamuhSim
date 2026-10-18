# CrowdSimPlus → Gazebo Harmonic / ROS 2 Jazzy

Adapter thử nghiệm dùng CrowdSimPlus trong image Python 3.8 đã ghim ở vòng 2; robot Mecanum/Nav2 vẫn chạy trong ROS 2 Jazzy trên host Ubuntu 24.04. Không cài CrowdSimPlus vào Python hệ thống 3.12, không sửa source vendor, robot hoặc workspace build/install.

## Build và chạy

Từ repository root:

```bash
# Nếu chưa có image, dùng Dockerfile đã ghim source của vòng 2
bash experiments/social_navigation/round2/run.sh build

# Chỉ build transport plugin trong experiments, không build lại robot
cmake -S experiments/social_navigation/crowdsimplus_adapter \
  -B experiments/social_navigation/crowdsimplus_adapter/build
cmake --build experiments/social_navigation/crowdsimplus_adapter/build -j 4

# Demo nhà xưởng/Gazebo + robot Nav2, dùng directory mới
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend crowdsimplus --condition counterflow --seed 43 --gui --diagnostics \
  --report-dir experiments/social_navigation/benchmark/crowdsim_comparison/reports/gui01
```

Seed fixture hiện hỗ trợ 42/43; `standing` là stationary control, không phải mô hình hội thoại native. Bỏ `--gui` vẫn chạy Gazebo physics/rendering và thu camera PNG/GIF. Không mở hai trial cùng ROS domain. Trial từ chối ghi đè report đã có.

```bash
bash experiments/social_navigation/benchmark/run.sh crowdsim-matrix \
  --report-dir experiments/social_navigation/benchmark/crowdsim_comparison/reports/run02
python3 experiments/social_navigation/benchmark/analyze_crowdsim.py \
  experiments/social_navigation/benchmark/crowdsim_comparison/reports/run02
```

Ma trận 7 lượt: CrowdSimPlus counterflow/standing seed 42/43, SocNavGym robot-aware group adapter counterflow seed 42/43, CrowdSimPlus invisible-robot counterflow seed 43. Chạy tuần tự, diagnostics bật ở cả hai backend. Comparator SocNavGym là bản adapter đã chỉnh ở vòng trước, **không phải hành vi nguyên bản upstream**.

## Cơ chế

- Gazebo gửi pose và vận tốc robot thực qua Unix socket mỗi 0,1 s simulation. Docker tắt network, root filesystem read-only, chỉ mount code read-only và runtime/socket riêng. Worker không dùng ROS; Nav2 là controller robot duy nhất.
- Native `Human.act`/`ORCAPlus.predict`, `CrowdSimPlus.constrain_agent_action_exact`, `Human.step` được gọi theo snapshot đồng thời như upstream `env.step`. Native robot act/step, reward, reset và episode termination không chạy trong bridge.
- Ba người có radius 0,36 m, preferred speed 0,8 m/s; robot radius danh định 0,25 m. ORCAPlus giữ native safety_space 0,01 m, cộng thêm 0,01 m trong solver: agent effective radius có margin 0,02 m so với proxy.
- Native demo dùng timestep 0,25 s; benchmark đổi config về 0,1 s để khớp nhịp adapter SocNavGym. Các override này được ghi trong metadata/config; không gọi đây là benchmark default nguyên bản.
- 42 khối nhà xưởng được chuyển thành 168 đoạn biên. Native static constraint chặn bước xuyên qua line; không thêm guard tránh người hay sửa pose sau bước.
- Initial pose lấy từ fixture pilot đã commit, hash kiểm tra đúng scene/map/Nav2. Các trường group_id trong fixture nguồn bị bỏ qua ở CrowdSimPlus; ground truth xuất groups rỗng. Counterflow dùng waypoint center chung và mỗi người đổi hướng khi native `reached_destination` đạt; đây là patrol ngoài episode, không phải group cohesion. Standing được giữ yên bởi fixture ngoài policy.
- Proxy cylinder và actor có prefix `crowdsimplus_`; topic `/crowdsimplus/ground_truth`, camera `/crowdsimplus/observer`. Plugin C++ riêng `libcrowdsimplus_adapter.so`; adapter SocNavGym cũ giữ nguyên.

CrowdSimPlus bản này có người tránh nhau và robot, nhưng không có group ID, group walking/cohesion, conversation hoặc formation. Không gọi những người đi gần nhau là nhóm xã hội. Head/yaw visual giữ hướng vận tốc gần nhất khi dừng; native holonomic state không đổi.

[Report tích hợp](../benchmark/crowdsim_comparison/reports/run01/CROWDSIM_GAZEBO_REPORT.md). Raw captures, log, container validation và official demo rerun nằm trong các thư mục generated/ (ignored); report giữ hash/path để truy xuất.

## SFM native trong Gazebo

Có thể chọn policy người bằng `--human-policy sfm`; mặc định vẫn là `orca_plus`.
Chỉ hỗ trợ backend `crowdsimplus`. Policy SFM và native static constraint không bị
chỉnh sửa; robot vẫn do Nav2 điều khiển. SFM không có membership/cohesion hoặc hội thoại.

```bash
# Dùng thư mục report mới cho mỗi lần chạy.
bash experiments/social_navigation/benchmark/run.sh trial \
  --backend crowdsimplus --human-policy sfm --condition counterflow --seed 42 \
  --gui --diagnostics \
  --report-dir experiments/social_navigation/benchmark/generated/sfm_demo_01
```

SFM dùng A/B/KI và các tham số static từ config upstream, timestep benchmark
0,1 s, human radius 0,36 m, preferred speed 0,8 m/s. Trong nhà xưởng
`is_bottleneck=False`: nhánh native hallway dựa vào thứ tự segment không được
áp vào danh sách 168 segment của nhà xưởng. Demo 2D upstream dùng dt 0,25 s
và DWA được lưu riêng; không gọi cấu hình Gazebo là demo nguyên bản.

[So sánh SFM/ORCA+](../benchmark/sfm_comparison/reports/run01/SFM_GAZEBO_REPORT.md).
Ma trận có hai seed, chạy từng lượt; chưa phải xếp hạng thống kê.

