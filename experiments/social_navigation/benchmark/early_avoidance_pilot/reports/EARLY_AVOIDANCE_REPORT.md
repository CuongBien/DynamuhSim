# Pilot tránh nhóm sớm bằng Nav2 — 22/10/2026

**Một lượt hợp lệ đã hoàn thành cả hành trình:** avoidance_moderate01 dùng robot/odometry/AMCL gốc, SocNavGym standingv3 seed42, camera depth OFF cùng camera ON đối chứng. Nav2 SUCCEEDED và tọa độ Gazebo thật đạt **14.42cm /0.0685rad**, trong ngưỡng20cm/0.25rad. Đây là1pilot sau lựa chọn thích nghi, **không phải success rate, chứng nhận social comfort hoặc cấu hình mặc định mới**.

Một lỗi sao chép XML đã được phát hiện và sửa trước lượt hợp lệ: [đính chính bắt buộc đọc](ERRATA.md). Ba lượt đầu trong giai đoạn này dùng sai cách diễn giải hệ quy chiếu ma sát bánh, không được coi là benchmark nguyên bản robot. Mã ROS src/, worker/native/vendor, dữ liệu cũ và staging của người dùng được giữ nguyên.

## Thay đổi và phạm vi

Runtime Nav2 dùng circle radius0.25m (cùng bán kính danh nghĩa trong guard người), padding0.01m thay footprint nhỏ0.24×0.20m của YAML cũ; inflation0.8m/scaling3 thay0.2m/scaling8. MPPI moderate vxmax0.30/vxmin−0.15/vymax0.20m/s, thay0.60/−0.20/0.40. Giữ radius-mode CostCritic consider_footprint=false; không dùng footprint-mode với circle vì Jazzy sẽ từ chối cấu hình đó. Các tham số được đọc lại từ node đang chạy trước khi gửi goal, lưu trong live_parameters.json.

Đây là thay đổi kết hợp shape/cost/speed, không tách tác dụng riêng từng tham số. Inflation chỉ dựa LiDAR; không đưa tọa độ người, nhóm hoặc tương lai từ simulator vào planner ở lượt đạt. Hull nhóm chỉ dùng để phân tích offline. Physical robot không đổi kích thước/bánh/friction số học. Namespace repair khôi phục đúng diễn giải friction gốc, không tuning vật lý.

Giữ worker standingv3, cap0.20m/s cho thành viên nhóm đứng gần robot, guard2mm, start(−28,0), goal(−20,0),6người, low camera(.12,0,.08) so với base_link, filter binary cũ, native LiDAR/IMU. Native lifecycle vẫn có thể cho người rời nhóm; họ trở thành người đi tự do với cap gốc0.8m/s. Không gán H1–H5 hoặc tuyên bố giữ cả6người trò chuyện suốt87s.

Ngân sách90sim s được khai báo trước lượt đầu: riêng8m/0.15mps đã53.3s, chưa detour. Khác ngân sách45s cũ nên không so thời gian hoàn tất như cùng budget. Không nới ngưỡng goal/clearance hoặc nâng tốc độ người để lấy PASS.

## Lịch sử mọi lượt

| Lượt | Thiết lập | Kết quả đo | Diễn giải |
|---|---|---|---|
|avoidance01|slow, AMCL, prefix ma sát sai|TIMEOUT90s; goal thật10.12m|Setup vật lý không hợp lệ; giữ dữ liệu.|
|avoidance_truth01|slow, robot-truth odometry/TF, prefix sai|TIMEOUT90s;3.90m|Đối chứng oracle không sửa lỗi XML; bị loại.|
|avoidance_truth_wide01|như trên, chỉ wz_std0.3→1.2|TIMEOUT90s;5.41m|Không dùng để kết luận native robot/controller.|
|avoidance_fixed01|khôi phục prefix gốc, AMCL, slow0.15/0.10|TIMEOUT90s;3.76m|Thiết lập hợp lệ nhưng chưa tới đích; giữ FAIL.|
|avoidance_moderate01|prefix gốc, AMCL, moderate0.30/0.20|SUCCEEDED87.31s;0.144m|Đạt tiêu chí pilot đã đặt.|

[Cohort ledger](cohort_ledger.json) giữ trạng thái, các mất frame phụ, raw count và scope oracle. Đây là các giả thuyết thích nghi lần lượt, không phải ma trận preregister ngẫu nhiên hoặc5lượt dùng tính tỷ lệ thành công. Diffs/manifests/driver thực tế được lưu trong [drivers](drivers/) và implementation/. Các kết quả TIMEOUT/setup sai không bị xóa, chạy đè hoặc thay bằng lượt thành công.

## Lỗi XML và kiểm chứng sửa

Bộ tạo camera runtime ở mốc trước dùng ElementTree, đổi literal gz:expressed_in thành ns0:expressed_in. XML expanded-name vẫn bằng nhau nên audit canonical XML đã bỏ sót, nhưng gz-physics7 DART gọi HasAttribute với đúng chuỗi gz:expressed_in, làm mất frame base_link ở4bánh. Số mu/mu2/fdir1, collision và src/ vẫn bằng nhau nhưng **diễn giải vật lý không bằng nhau**.

Native checker với SDK đang cài: robot gốc4/4, bản sai0/4, bản sửa4/4. [Kết quả](friction_native_check.json). Pilot dataset cũ string-only vẫn4/4; [đối chiếu](original_pilot_friction_check.json). Commit fix riêng33427f4 đưa namespace_prepare.py và regression native; không sửa/raw lại các experiment cũ. Lượt sửa slow giảm median sai lệch TF–Gazebo còn0.120m/0.009rad, nhưng vẫn giữ kết quả timeout. Không còn diễn giải các lượt prefix sai như lỗi AMCL thuần hoặc chỉ lỗi người.

Nguồn gốc: [gz-physics7 DART friction-frame lookup](https://raw.githubusercontent.com/gazebosim/gz-physics/gz-physics7/dartsim/src/SDFFeatures.cc). Tham số Nav2 đối chiếu source [Jazzy CostCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_mppi_controller/src/critics/cost_critic.cpp), [InflationLayer](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_costmap_2d/plugins/inflation_layer.cpp). Thử wz_std dựa vào [MPPI1.3.13 optimizer](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.13/nav2_mppi_controller/src/optimizer.cpp) và [noise generator](https://raw.githubusercontent.com/ros-navigation/navigation2/1.3.13/nav2_mppi_controller/src/noise_generator.cpp); không coi giả thuyết ấy là nguyên nhân đã được chứng minh.

## Nghiệm thu avoidance_moderate01

| Tiêu chí / dữ liệu | Kết quả |
|---|---:|
|Action / vị trí thật|SUCCEEDED; XY0.14415m; yaw0.06853rad|
|Thời gian active / budget|87.3075s /90s|
|Khoảng hở đĩa robot–người nhỏ nhất|0.59246m; sampled và swept-linear|
|Robot center / disc vào hull nhóm đang đứng|0 /0 lần theo pose quan sát|
|Số pose–group được đánh giá|3,282|
|Human adapter fault|0|
|Tốc độ thành viên đứng nhích tránh được kích hoạt|max0.0m/s; không cần nhích trong các mẫu này|
|Người tự do/rời nhóm|max≈0.8m/s theo native cap gốc|
|Cặp depth ON/OFF|1,008; active875; active có effect696|
|Callback OFF pre/post|1,015/1,015|
|Raw MCAP|16topic;1,013depth OFF;507scan;10,120IMU|
|Nhãn sau action_end|3.3475s|
|GPU/shutdown|NVIDIA RTX4060Laptop; Gazebo clean exit; cleanup rỗng|

Khoảng hở là metric đĩa danh nghĩa r_robot0.25/r_human0.36; swept-linear giữa pose samples, không phải contact force/mesh-contact continuous guarantee. Hull từ nhóm standing hiện hữu ở timestamp quá khứ gần nhất, không từ fixture cố định bất kể lifecycle. Không ghi nhận intrusion trong số mẫu trên, chưa chứng nhận mọi tương tác xã hội hoặc dense crowd.

![Quỹ đạo Gazebo thật](avoidance_moderate01/actual_path.png)

Các vòng cam trong đồ thị là **vị trí người ban đầu**; có người đã di chuyển/rời nhóm trước khi robot qua đó. Không dùng khoảng cách từ đường robot tới vòng ban đầu để đánh giá va chạm. Metric dùng pose người thật theo thời gian. Đường đi quanh nhóm này còn chậm/gấp khúc, chưa tối ưu thời gian hoặc tính tự nhiên.

![Observer ghi thật từ Gazebo, phát4x](avoidance_moderate01/camera_preview_4x.gif)

Ảnh/GIF là callback Gazebo thật; path/BEV là phép vẽ từ dữ liệu đã ghi, không phải AI sinh hoặc World Model dự đoán. [Raw result](avoidance_moderate01/result.json), [metrics](avoidance_moderate01/summary.json), [readback](avoidance_moderate01/live_parameters.json).

## Audit và giới hạn dữ liệu

[Audit](avoidance_moderate01/audit.json) xác nhận1,008ảnh OFF ở cả NPY và MCAP bằng bitwise;5frame MCAP thiếu ở subscriber phụ, đều warmup trước goal. Không nói toàn bộ1,013frame đã được NPY ghi đủ. MCAP là bản raw chính; [raw manifest](avoidance_moderate01/raw_manifest.json) và SHA các cặp ảnh được giữ. Các lượt khác cũng ghi rõ mismatch delivery; audit ban đầu dừng khi gặp frame thiếu được lưu audit_initial_issue, không che bằng so sánh phần giao mà bỏ phần thiếu.

Robot model đối chiếu bằng cả canonical XML **và native literal-prefix checker**, world bằng nhau sau chuẩn hóa socket, native worker/vendor/filter/source ROS bằng SHA cũ. Những oracle controls chỉ thay odometry output/TF ở runtime, không phải bản đạt. Noisy LiDAR giữ nguyên; không tuyên bố live dynamic LiDAR bằng bitwise với một run khác. Depth filter restore cân bằng và sensor record đầy đủ trong phạm vi đã đo. Shutdown pause vẫn là mitigation, không phải sửa dứt điểm driver.

AMCL ở lượt đạt vẫn có median XYerror0.126m, max0.255m, median yawerror0.014rad; không gọi định vị hoàn hảo. Một seed sau tuning chưa đủ xác nhận độ ổn định, tốc độ realtime hoặc generalization.

## Bước tiếp theo

Có thể dùng episode đã đạt để kiểm tiếp **định dạng dữ liệu portable**, provenance profile/friction/filter, timestamp, nhãn3s và masks. Chưa train: H1–H5 vẫn unknown/masked, association và định nghĩa future BEV còn phải chốt. Cần một xác nhận độc lập nhỏ trước batch lớn; HuNavSim tiếp tục held-out. Không tự áp profile vào ROS mặc định, không World Model/Isaac hoặc benchmark thêm framework trong mốc này.
