# Pilot Nav2 với SocNavGym response v2

Ngày22/10/2026. Bắt đầu từ commit **`8aaa357`**, dùng bản v2 đã sửa giao cắt và terminal Pause. **Nav2 đã điều khiển robot tự tới goal trong cả3 lượt đã định**:1 empty control và2 session có6 người. Không phát lại robot velocity script; publisher ROS cmd_vel được kiểm là controller_server/behavior_server. Đây là nghiệm thu pilot nhỏ, không phải thống kê success rate hoặc so sánh ba simulator.

| Lượt | Nav2 action | Tới đích XY/yaw thật | Sai số XY | Thời gian sim/wall | Min robot–người (disc) |
|---|---|---|---:|---:|---:|
|empty01|SUCCEEDED|Đạt|0.1784m|15.72/15.8s|—|
|people01|SUCCEEDED|Đạt|0.1786m|29.24/121.2s|0.0618m|
|people02|SUCCEEDED|Đạt|0.1454m|27.61/69.6s|0.1971m|

Đạt actual goal yêu cầu action SUCCEEDED **và** final world XY<=0.20m, yaw<=0.25rad sau cửa sổ settle ngắn; không lấy AMCL estimate làm ground truth. Empty control đạt trước khi mở hai lượt có người. Không retry/chọn lại failed goal; một lỗi import Log trong harness xảy ra trước khi tạo simulator được giữ ở harness_import_attempt01 và sửa đúng message Jazzy.

## Bài thử và những phần giữ nguyên

Robot xuất phát(−28,0,0), goal(−20,0,0); budget45sim s.6 người,radius0.36m,seed42,initial/targets từ crossing_near đã khóa; nominal robot circle0.25m. Nhà xưởng60×40m,42 boxes,map và robot Mecanum/LiDAR/IMU giữ nguyên. Native planning0.1s, physics0.0025s, projection và Pause interlock byte-identical commit8aaa357; không build/sửa workspace robot hoặc tune MPPI/AMCL. Chỉ YAML runtime copy đổi initial_pose.x từ−27 sang−28; launch episode gate và AMCL transform_tolerance override0.5s gốc vẫn giữ. Installed launch files được đối chiếu với source đã bảo toàn.

Worker giữ người đứng yên trong warmup; chỉ gọi native step sau khi action goal được nhận và start_gate tạo. Test socket thật chứng minh10 warmup transport ticks có native_step0/human state đứng yên, rồi20 native ticks khớp exact với offline v2; clock sai gây đóng terminal. Warmup/static hold là của harness, không phải hành vi xã hội.6 người là cá nhân, formation/conversation tắt. Không được gán kết quả này cho mọi nhóm đứng hay nhóm đi. Người nhận robot ground truth như adapter trước; Nav2 dùng sensing/map/AMCL hiện có.

![Quỹ đạo và tốc độ robot đo từ Gazebo](nav2_actual_paths_and_speed.png)

## Quan sát trong hai lượt có người

- **people01**: robot y từ-0.501 tới0.322m, path8.239m; có1.695s tốc độ<0.05m/s khi disc gap<1m. LiDAR có14152 tia eligible,99.12% khớp trong15cm, IDs[1, 2, 3, 4, 5]. Costmap có occupied-ring trong430/441 quan sát đủ điều kiện, IDs[1, 2, 3, 4, 5].
- **people02**: robot y từ-0.715 tới0.299m, path8.502m; có0.000s tốc độ<0.05m/s khi disc gap<1m. LiDAR có13130 tia eligible,99.08% khớp trong15cm, IDs[1, 2, 3, 4, 5, 6]. Costmap có occupied-ring trong410/415 quan sát đủ điều kiện, IDs[1, 2, 3, 4, 6].

Quỹ đạo cong, các đoạn chậm và ô costmap gần người là bằng chứng mô tả phản ứng closed loop; không đếm chúng thành số lần nhường do xã hội hoặc chứng minh một nguyên nhân duy nhất. Người và robot đều có thể né. Occupied-ring tìm cells>=65 trong vòng radius0.26..0.46m quanh tâm người đo, đổi world→odom bằng odom/world robot pose gần thời điểm costmap; không chứng minh nguồn từng cell hoặc recall. Không phải cả6 người đều được LiDAR khớp trong từng session.

Không ghi overlap>5mm trong swept-disc/AABB trên hai lượt có người. Minima người–người, tĩnh–người và rotated-footprint robot–người giữ trong summary.json; footprint lấy đúng hình chữ nhật0.24×0.20m của YAML nhưng chỉ đo tại samples, không gọi là swept rotating-body proof. Clearance dương vài cm không tự chứng nhận social comfort. Góc nhìn đồ thị giới hạn aisle ROI; phần ngoài khung vẫn được lưu/đo.

## Thời gian chạy và giới hạn

Empty control chạy khoảng1× realtime; hai phiên có người khoảng0.24×(GUI) và0.40×(headless). GPU NVIDIA D3D12 được kiểm bằng preflight/Ogre ởcả3; camera/GPU LiDAR/rendering dùng GPU, native Python/ORCA, Nav2 và physics vẫn dùng CPU. Native worker2 CPU/one BLAS thread; không cài dependency hoặc đổi ROS Python. Chi phí native worker, GUI và scheduling khác nhau nên không coi chênh lệch thời gian là tác động thuần của hành vi người. Nav2 effective command timing cũng có thể khác theo RTF; không dùng bộ này xếp hạng runtime/framework.

Hai people sessions giữ fixture seed42 nhưng AMCL/MPPI/physics không được joint seed; trajectories khác nhau. Đây không phải ablation equal-robot-input: robot do Nav2 lập lại đường và giới hạnvx0.6m/s, khác forced crossing0.75m/s đã gây lỗi baseline. AMCL error được đối chiếu với actual pose, không coi goal success là localization hoàn hảo. Cửa sổ settle0.4wall s không đòi hỏi robot đứng yên lâu tại đích; tiêu chí đã khóa chỉ kiểm XY/yaw và action status. Chưa test crowd dày, lối hẹp với Nav2 v2, group lifecycle/conversation, nhiều seed, thời gian dài hoặc so Nav2 cùng chuẩn với CrowdSimPlus/ComplexityNav. Terminal Pause đã kiểm ởcommit trước; cohort này không chèn fault giữa goal Nav2, nên chưa nhận claim cancel/action-state khi interlock kích hoạt.

![Gazebo RGB thật — people01,4× simulation time](people01/camera_preview_4x.gif)

![Gazebo RGB thật — people02,4× simulation time](people02/camera_preview_4x.gif)

Ảnh/GIF là RGB camera Gazebo đã chạy,640×480,1frame/simulation-second, quantize khi đóng GIF, phát4× theo simulation time; không phải AI hoặc screen recording realtime. GUI mở ởpeople01; chưa có xác nhận kéo/zoom riêng từ người dùng cho cohort này. Observer cố định có thể không thấy mọi người suốt lượt; dữ liệu pose vẫn lưu đầy đủ.

## Quyết định và bước tiếp theo

**Đã nghiệm thu pilot Nav2 nhỏ cho6 người cá nhân với SocNavGym v2** theo goal thật và dữ liệu đã kiểm. Không sửa robot/Nav2 baseline hoặc tự chọn AMCL profile mới. Mốc này được lưu với commit message `feat: validate SocNavGym v2 with bounded Nav2 warehouse pilot`. Bước tiếp theo là áp dụng cùng bài Nav2/start-goal/geometry cho CrowdSimPlus ORCA+ và ComplexityNav geometry, chạy tuần tự rồi đối chiếu; sau đó mới chốt so sánh chung. Khả năng nhóm đi/đứng tương tác của SocNavGym tiếp tục là tiêu chí chọn mô hình riêng, cần pilot nhóm+Nav2 trước khi gọi toàn bộ social navigation hoàn tất. Không World Model hoặc Isaac Sim trong bước này.

[Protocol](../protocol.json) · [Số đo từng lượt](summary.json) · [Gate/IPC](gate01/validation.json) · [Installed launch audit](installed_launch_audit.json). Từng report con chứa action/actual outcome, commands,plans,costmaps,AMCL/TF,scene poses,Lidar/odom và worker trace gzip byte-identical, source/config/world hashes, PNG/GIF. Source/status-doc edits được ghi rõ; staging và thay đổi bên ngoài experiment được bảo toàn.
