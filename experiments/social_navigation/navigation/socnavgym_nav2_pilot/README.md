# Pilot Nav2 — SocNavGym response v2

[Báo cáo, số đo và GIF thật](../../../archive/social_navigation/reports/socnavgym_nav2_pilot/NAV2_PILOT_REPORT.md).3 lượt đã định đều Nav2 SUCCEEDED và tới đích thật trong20cm/yaw0.25rad:1 empty control,2 phiên6 người giao cắt. Đây là pilot nhỏ one-seed, chưa phải benchmark Nav2 chung ba framework hoặc group lifecycle.

Robot bắt đầu(−28,0,0),goal(−20,0,0),budget45sim s. Nav2 MPPI/AMCL,map/robot/cảm biến giữ nguyên; bản YAML runtime chỉ đổi amcl initial_pose.x thành−28. Người dùng kernel/projector đã kiểm ởcommit8aaa357,6 cá nhân radius0.36m, planning0.1s; physics0.0025s. Native humans đứng yên trong warmup, bắt đầu khi goal được nhận. Robot nhận cmd_vel từ Nav2 controller/behavior, không có publisher velocity script trong recorder.

Từ root repo trong Ubuntu/WSL, chọn report directory MỚI:

```bash
# Empty control trước một cohort mới
bash experiments/social_navigation/navigation/socnavgym_nav2_pilot/run.sh live \
  --condition empty \
  --report-dir experiments/social_navigation/navigation/socnavgym_nav2_pilot/reports/my_empty
bash experiments/social_navigation/navigation/socnavgym_nav2_pilot/run.sh analyze \
  --live experiments/social_navigation/navigation/socnavgym_nav2_pilot/reports/my_empty

# Demo robot tự điều hướng qua6 người
bash experiments/social_navigation/navigation/socnavgym_nav2_pilot/run.sh live \
  --condition people --gui \
  --report-dir experiments/social_navigation/navigation/socnavgym_nav2_pilot/reports/my_people
bash experiments/social_navigation/navigation/socnavgym_nav2_pilot/run.sh analyze \
  --live experiments/social_navigation/navigation/socnavgym_nav2_pilot/reports/my_people
```

GUI tự đóng khi action kết thúc/budget hết. Cohort đã đo dùng GUI ởpeople01, headless ởempty01/people02; không có xác nhận kéo/zoom riêng từ người dùng. Trong các phiên people đã đo,27.6–29.2sim s tương ứng69.6–121.2wall s. GPU NVIDIA đã được preflight/Ogre kiểm; rendering/camera/GPU LiDAR dùng GPU, native Python/ORCA/Nav2/physics vẫn dùng CPU. Không fallback software hoặc đổi ROS Python. Native worker giới hạn2 CPU/one BLAS thread.

Dùng ROS domain91 và partition Gazebo riêng, lock chung common_gazebo/response_fix. Không chạy đồng thời các experiment khác dùng domain91. Giữ phiên chạy tới lúc tự lưu/cleanup; không reset world trong session. Worker fatal/socket loss dùng terminal Pause interlock đã kiểm ởcommit8aaa357; pilot Nav2 này không inject lỗi và chưa xác nhận action cancel state khi interlock kích hoạt.

Dependencies hiện có: ROS2 Jazzy/workspace install, SocNavGym Python3.12 venv, numpy/RVO2, analyzer Shapely/PIL/Matplotlib. Tái dùng build/libresponse_adapter.so của experiment socnavgym_response_fix; nếu thiếu, chỉ build plugin thử nghiệm bằng run.sh build ởfolder đó, không build workspace robot. Source/protocol/runtime config hashes được kiểm. Checkpoint bảo toàn Git/source là local ignored; máy khác cần checkpoint tương đương hoặc cohort mới để audit, không ghi đè report cũ.

Gate/socket test trong reports/gate01 đã kiểm10 transport ticks frozen(native_step0),20 ticks sau gate khớp offline v2 và clock lỗi đóng terminal. Producer `test_gate.py` ghi cohort cố định đã nghỉ dùng; receipt PASS vẫn được runtime kiểm tra, với [source đã ghim](https://github.com/CuongBien/DynamuhSim/tree/68b2dded70b1ae41361f2e1aa8dad593ae2111f6/experiments/social_navigation/benchmark) để tái lập. Một lỗi import Log ởharness được sửa trước khi tạo simulator; attempt/source cũ được giữ, không cộng thành failed goal. Installed launch bytes khớp source đã bảo toàn.

Analyzer giữ action status riêng actual world XY/yaw. Nó kiểm42 boxes/proxy radius, pose interpolation, actual swept-disc/AABB và sampled rotated footprint, LiDAR, costmap/commands/plans, AMCL/TF và odom. Slow-near/occupied-ring là số đo mô tả, không chứng minh social yielding hoặc attribution của từng costmap cell. Hai people sessions không đồng bộ AMCL/MPPI/physics seeds, RTF khác nhau; không xếp hạng framework hoặc coi ba lượt là success-rate estimate. Tại goal chỉ kiểm XY/yaw/action, không nghiệm thu đứng yên lâu tại đích.

Raw poses,scan,odom,AMCL,commands,plans,costmaps,TF/feedback,worker và SDF/config/log có gzip byte-identical. Camera640×480,1frame/sim-second,4× simulation time, không phải AI hoặc quay màn hình realtime. Góc nhìn cố định không nhất thiết thấy mọi người mọi lúc. Bước tiếp: cùng pilot Nav2 cho CrowdSimPlus ORCA+ và ComplexityNav geometry để đối chiếu; group+Nav2 của SocNavGym vẫn là pilot riêng cần làm trước khi gọi toàn bộ social navigation hoàn tất. Không World Model/Isaac Sim hoặc đổi kiến trúc robot.
