# Tay chân replay, costmap và khe hở — 29/10/2026

## Đang dùng thuật toán nào?

Cấu hình thực tế được đọc từ capture/runtime/nav2_runtime.yaml của lượt demo:

- Global: nav2_navfn_planner::NavfnPlanner, use_astar=false → Dijkstra.
- Local: nav2_mppi_controller::MPPIController, motion_model=Omni; 20Hz, batch280, 30step×0,05s = horizon1,5s.
- AMCL OmniMotionModel định vị; costmap obstacle lấy /scan LiDAR, chưa lấy classifier/depth như một detector live. Không có WM/RL điều khiển robot.

Đối chiếu thuật toán: [NavFn Jazzy](https://api.nav2.org/nav2-jazzy/html/classnav2__navfn__planner_1_1NavFn.html), [MPPI Jazzy](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/).

## Vì sao tay chân khác video?

Video web là camera thật từ lượt Nav2 vật lý. Cửa sổ sau đó là replay nhẹ XY/yaw; bản cũ chủ động SetAnimationTime(0), nên trượt người mà tay/chân cố định. Đã đổi sang tích lũy quãng đường ghi được ×4,1534, cùng hệ số gait của live adapter. Vị trí và yaw không đổi; góc khớp gốc không được ghi nên đây là animation dựng lại. Người đứng yên không được ép bước chân. Người dùng đã xác nhận nhìn thấy tay/chân chuyển động trong phiên mới. Bản build mới dùng generated/build_gait để tách khỏi cache/binary cũ.

## Đoạn giây5

Video mã hóa4fps từ camera1Hz: video5s/frame20 = sim33s; kể cả chọn playbackRate0,25 thì timecode vẫn ánh xạ như vậy. Không nhầm với 5s kể từ lúc action bắt đầu ở sim12,4625s.

[Video camera + costmap đồng bộ](camera_costmap_4x.webm) · [Tổng quan](navigation_costmap.png) · [Phóng to giây5](costmap_video5s.png).

- Khe A H5–H6: khoảng trống giữa hai collision cylinder0,527m. Chassis thấy được0,24×0,20m; toàn robot còn bánh/cảm biến. Nav2 dùng robot_radius0,25m + padding0,01m, hình vòng danh định xấp xỉ0,52m; padding thực được rời rạc theo footprint/cell. Dư hình học chỉ7,08mm, trong khi cell5cm. Midpoint được ghi cost99 (inscribed), không phải ô trống dù mesh nhìn nhỏ.
- Khe B H1–H5: rộng1,299m, midpointcost31. Không chứng minh khe này bất khả thi; cũng chưa thử đường tắt bằng mô phỏng phản chứng. Robot đang bám global path vòng phía trên, tại video5s lệnh gần dừng và xoay: vx0,00165m/s, vy−0,00086m/s, wz−0,274rad/s. Không có critic score/rollout từng candidate trong log, nên không khẳng định nguyên nhân tối ưu duy nhất.
- Inflation0,8m/cost_scaling_factor3,0 là vùng chi phí mềm, không phải toàn vùng cấm. Bề rộng cylinder người0,72m và footprint robot lớn hơn phần nhìn thấy là hai yếu tố khác với inflation.

## Đọc hình và giới hạn

Heatmap là **local OccupancyGrid thật** đã ghi, chuẩn hóa0–100; không phải costmap nội bộ0–255, BEV depth hoặc bản đồ vẽ giả. /plan ghi trong map được đổi sang odom bằng TF ước lượng đã ghi. Đường xanh lá là odometry đã thực thi; nét chấm là 8s tiếp theo đã xảy ra, chỉ để chẩn đoán, không phải MPPI dự đoán. Hình người/circle dùng GT căn theo ego để giải thích hình học; GT không đi vào điều khiển.

Không có global costmap array hoặc trajectory critic breakdown trong bag. Do đó kết luận về việc chọn đường rộng hơn là suy luận phù hợp cấu hình/log, không chứng minh đường vòng là phương án tối ưu hoặc mọi khe đều cấm. Không giảm footprint/inflation hoặc sửa recipe để tạo đoạn robot lách qua trong lần này.

Code draw_costmap.py và make_overlay_video.py đọc raw demo tại đường dẫn máy gốc ghi trong script; output đồ thị nằm cạnh script, cần full raw khi tái lập. 69 frame video overlay được decode/đếm, từng frame camera giữ từ nguồn gốc cùng timestamp. Script vẽ lại chỉ ghi output phân tích, không đổi raw capture/model/nhãn.

## Kiểm tra bổ sung

Đã đo collision toàn robot (24×28cm), native footprint52cm và lệch URDF/SDF/TF. [Xem phân tích footprint/inflation](../footprint_review/README.md). Vùng52cm là cấu hình bảo thủ đang dùng, không phải kích thước thật của chassis.
