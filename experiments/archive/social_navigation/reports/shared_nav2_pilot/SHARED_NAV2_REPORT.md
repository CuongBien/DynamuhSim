# Pilot Nav2 cùng nhiệm vụ cho ba framework

Ngày22/10/2026, xuất phát từ commit `bb45f1b`. Đã chạy tuần tự4 lượt mới:2 CrowdSimPlus ORCA+,2 ComplexityNav geometry. Hai lượt SocNavGym v2 và empty control dùng dữ liệu đã commit, không chạy lại để chọn kết quả. Đây là pilot tích hợp với robot tự điều hướng trong Gazebo, không phải thống kê hoặc xếp hạng simulator.

| Framework / lượt | Action | Đích thật | XY (cm) | Sim / wall (s) | Min robot–người (cm) |
|---|---|---|---:|---:|---:|
|SocNavGym v2 / people01|SUCCEEDED|Đạt|17.86|29.24 / 121.2|6.18|
|SocNavGym v2 / people02|SUCCEEDED|Đạt|14.54|27.61 / 69.6|19.71|
|crowd_orca_plus / crowd01|SUCCEEDED|Đạt|17.53|20.81 / 75.1|3.93|
|crowd_orca_plus / crowd02|SUCCEEDED|Không đạt|20.59|27.26 / 78.8|2.88|
|complex_geometry_mixed / complex01|SUCCEEDED|Đạt|12.45|21.80 / 29.8|3.21|
|complex_geometry_mixed / complex02|SUCCEEDED|Đạt|18.38|26.01 / 31.8|5.80|


Actual goal yêu cầu SUCCEEDED và world XY≤0.20m/yaw≤0.25rad tại pose ghi sau settle0.4wall s. **Crowd02 báo SUCCEEDED nhưng XY20.59cm: giữ không đạt**, không nâng ngưỡng hoặc chạy thay. Empty control cũ đạt trong15.72sim s. Goal outcome độc lập với clearance; không nghiệm thu đứng yên lâu ởđích.

## Cùng nhiệm vụ và khác biệt native

Nhà xưởng60×40m/42 boxes,6 người r0.36m, robot nominal disc r0.25m, start(−28,0,0),goal(−20,0,0),budget45sim s, fixture crossing_near/seed42. Map, Mecanum/LiDAR/IMU, MPPI/AMCL giữ nguyên. Runtime YAML chỉ đổi AMCL initial x từ−27 sang−28 như Soc cohort. Audit xác nhận YAML byte-identical giữa6 people runs; world XML giống nhau sau khi chỉ chuẩn hóa socket path/native planning dt. Installed launch khớp source. Physics0.0025s. /cmd_vel chỉ có controller_server/behavior_server theo graph kiểm, không robot velocity script.

Người đứng yên trong warmup, bắt đầu native sau accepted goal. Gate mỗi backend:10 transport ticks native_step0,20 bước sau gate khớp exact offline đã commit, clock lỗi terminal. Crowd dùng native ORCA+ act/constrain/step,dt0.1s,v_pref0.8m/s và bookkeeping6 người đã kiểm. CN dùng3ORCA+3SFM native intention **cộng roadmap/ORCA geometry projection**,dt0.25s,native intended speed chủ yếu0.5m/s,projection ceiling0.8m/s; không phải native geometry hoặc trained RGL. Không áp sửa motion Soc v2 chohai backend này; chỉ tái dùng transport/interpolation/terminal Pause plugin.

Native robot observation/intention assumptions vẫn khác: CSP observable position/velocity/radius; CN/Soc native inferred intention riêng, final geometry dùng velocity đo. Soc v2 có projection riêng. Vì vậy same physical task không phải equal internal observation/speed hoặc ablation thuật toán. Native waypoint/route lifecycle sau các lần tớiđích vẫn giữ nguyên.

![Pose Gazebo và tốc độ robot](actual_paths_and_speed.png)

Vòng tròn xám là người ban đầu; plot đường robot không vẽ toàn bộ human traces. Raw giữ đủ pose người.

## Kết quả đo mới

- **crowd01**: min gap người–người 4.00 cm, người–tĩnh 2.00 cm; sampled rotated footprint–người 13.86 cm. Path 7.992 m, y range -0.277..0.220 m; slow-near 0.000 sim s. LiDAR 7541 eligible beams, 99.30% khớp trong15cm, IDs [1, 2, 3, 4, 5, 6]; occupied-ring 160/160. AMCL XY max 0.144m; RTF 0.277.
- **crowd02**: min gap người–người 4.00 cm, người–tĩnh 2.00 cm; sampled rotated footprint–người 13.03 cm. Path 9.292 m, y range -0.106..1.020 m; slow-near 0.170 sim s. LiDAR 7798 eligible beams, 99.38% khớp trong15cm, IDs [1, 2, 3, 4, 5, 6]; occupied-ring 180/183. AMCL XY max 0.195m; RTF 0.346.
- **complex01**: min gap người–người 8.23 cm, người–tĩnh 2.00 cm; sampled rotated footprint–người 14.06 cm. Path 8.406 m, y range -0.588..0.026 m; slow-near 0.000 sim s. LiDAR 6944 eligible beams, 99.47% khớp trong15cm, IDs [1, 2, 3, 4, 5, 6]; occupied-ring 155/156. AMCL XY max 0.168m; RTF 0.732.
- **complex02**: min gap người–người 6.55 cm, người–tĩnh 2.00 cm; sampled rotated footprint–người 15.56 cm. Path 9.397 m, y range -1.013..0.909 m; slow-near 0.370 sim s. LiDAR 6745 eligible beams, 99.13% khớp trong15cm, IDs [1, 2, 3, 4, 6]; occupied-ring 160/160. AMCL XY max 0.169m; RTF 0.817.


4/4 lượt mới không ghi overlap>5mm theo swept discs/AABBs giữa các pose samples. Nominal robot disc0.25m; rotated rectangle0.24×0.20m chỉ đo tại samples. Khoảng hở vài cm không chứng nhận social comfort. LiDAR chỉ đối chiếu tia đủ điều kiện trước static boxes, không phải recall6 người. Occupied-ring mô tả costmap cells≥65 gần bề mặt người, không chứng minh nguồn từng cell. Slow-near/path cong không tự chứng minh nhường do xã hội.

## Runtime và provenance

Ubuntu24.04/Jazzy/Python3.12.3 chạy robot/recorder; native CSP/CN Python3.8.13 trong Docker content-ID ghim,network none,2CPU/4GB/one BLAS thread. Không đổi Python hệ thống. NVIDIA D3D12 được kiểm preflight/Ogre ở4 lượt; shutdown sạch, không còn process/container thuộc phiên. Native/ORCA/Nav2/physics vẫn cần CPU. GUI/RTF/scheduling khác nhau; AMCL/MPPI/physics chưa joint-seeded. Không suy success rate hoặc xếp hạng runtime từ2 lượt/model.

Analyzer đã dừng tại crowd02 khi so pose cuối trong cleanup với pose kết luận trước cleanup. Pose tail lệch rất nhỏ nhưng vẫn ngoài20cm. Đã sửa **analyzer** để tìm đúng final_world_pose trong actual active stream. Không chạy lại simulator, sửa worker/Nav2/tolerance hoặc goal outcome. Assertion/source ban đầu và audit_returncode1 giữ trong audit_attempts/source_snapshot/initial_execution.json; final_audit riêng. Raw streams và gzip byte-identical giữ cả shutdown tail. Source runtime/worker/protocol hashes ởresult; source snapshot ban đầu và final manifest công bố riêng. Khi đóng gói, cohort.py thêm guard chống ghi đè receipt trước mọi lần chạy lại; source lúc chạy vẫn lưu.

Camera RGB640×480,1frame mỗi sim-second; GIF phát4× simulation time, **thu từ Gazebo thật, không AI**, không screen recording realtime. GUI yêu cầu ởcrowd01/complex01; chưa có xác nhận kéo/zoom riêng cho cohort này. Camera cố định không đảm bảo thấy mọi người suốt lượt.

![CrowdSimPlus — camera thật](crowd01/camera_preview_4x.gif)

![ComplexityNav geometry — camera thật](complex01/camera_preview_4x.gif)

## Nghiệm thu và bước tiếp theo

Cả3 nhánh đã có demo độc lập, adapter Gazebo và pilot Nav2; cohort này đối chiếu cùng physical task. Chưa gọi toàn bộ social navigation hoàn tất:6 cá nhân giao cắt, chưa nhóm/formation+Nav2 hoặc dense crowd nhiều seed. Không sửa baseline robot/Nav2/vendor, không World Model/Isaac.

Theo mục tiêu nhóm đi/đứng tương tác/formation, **SocNavGym lifecycle variant** vẫn là ứng viên chính dựa vào pilot lifecycle Gazebo trước; cần nhóm+Nav2 riêng. **CrowdSimPlus ORCA+** giữ baseline tránh người độc lập, chưa membership/conversation. **ComplexityNav geometry** là hybrid nghiên cứu choobstacle/không gian chật; không RGL checkpoint hoặc group model. Không chọn framework chỉ từgoal/min gap pilot này.

Bước tiếp có giới hạn: pilot SocNavGym nhóm+Nav2 với nhóm đi cùng và nhóm đứng trò chuyện; đo nhóm giữ/tan, goal thật và clearance. Sau đó chốt bộ demo/báo cáo lựa chọn theo năng lực đã đo. Không mở thêm framework hoặc ma trận AMCL.

[Protocol](../protocol.json) · [Số đo](summary.json) · [Physical task audit](physical_task_audit.json) · [Gate](gate01/validation.json) · [Commands/returncodes](initial_execution.json). Từng folder có result/validation,raw gzip,PNG/GIF.
