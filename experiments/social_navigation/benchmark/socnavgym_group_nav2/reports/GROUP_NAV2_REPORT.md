# SocNavGym nhóm đi / đứng trò chuyện với Nav2

Ngày22/10/2026, bắt đầu từ commit `ebbd455`. Giữ hai lượt nhóm đi v2 và hai lượt nhóm đứng v3; GUI lỗi gốc và standing v2 bị terminal stop được lưu riêng. Các phiên bản có thay đổi mô hình nên không gộp thành một cohort đồng nhất. Đây là pilot nhóm+Nav2, không phải thống kê success rate hoặc chứng nhận tính tự nhiên.

| Lượt / seed | Nav2 / đích thật | XY (cm) | Sim / wall (s) | Min robot–người (cm) | Nhóm đi ≥2s/0.5m | Nhóm đứng ≥2s |
|---|---|---:|---:|---:|---|---|
|walking01_v2 / 42|SUCCEEDED / Không đạt|21.12|15.54 / 49.6|69.31|Có|Có|
|walking02_v2 / 43|SUCCEEDED / Không đạt|29.77|32.78 / 77.1|3.88|Có|Có|
|standing01_v3_headless / 42|SUCCEEDED / Đạt|18.87|25.51 / 62.3|3.73|Chưa|Có|
|standing02_v3_headless / 43|SUCCEEDED / Đạt|15.85|19.30 / 59.8|3.77|Chưa|Có|


Đích thật yêu cầu action SUCCEEDED và world XY≤0.20m/yaw≤0.25rad tại pose sau settle0.4wall s. Không thay ngưỡng, chạy thay failed goal hoặc lấy AMCL estimate làm ground truth. Empty control và6 cá nhân từ cohort trước chỉ làm tham chiếu, không phải matched-group ablation.

## Bài thử và các thay đổi được khai báo

Cùng robot Mecanum/LiDAR/IMU, Nav2 MPPI/AMCL, factory60×40m/42 boxes, start(−28,0,0),goal(−20,0,0),budget45sim s.6 người radius0.36m, nominal robot circle0.25m, physics0.0025s/native0.1s. Bắt đầu bằng2 nhóm3 người tại(−25.5,0)/(−22.5,0), radius bố trí0.72m; seed42/43 cho từng walking/standing. **Nhóm ban đầu là fixture tường minh, không phải formation tự phát**. Walking initial goal(−20.5,0); standing quay vào tâm. Sau đó native formation0.03/dispersal0.01 và native goals vẫn chạy.

Tái dùng ContinuousGroups giữ ID và liên tục khi commit nhóm, cùng response_v2 global ORCA/per-human convex projection/swept filter. Walking giữ bản v2. Standing v3 cho phép thành viên nhóm đứng cách tâm robot<1.4m nhích tránh với speed cap0.2m/s khi convex projection cần; preferred velocity vẫn0. Người đứng độc lập không được mở khóa. Nhóm đứng có thể dịch nhẹ, không còn là các human bị khóa bất động. Hình học giữ swept floor2mm, không sửa robot hoặc nới ngưỡng. Đây là variant sửa chuyển động, không native nguyên bản; Nav2 chỉ nhận LiDAR/costmap, không social group-aware controller. Standing conversation là bố trí đứng và hướng nhìn, không phát lời thoại; clip đi bộ giữ ở tốc độ0, chưa clip nói chuyện riêng.

Kernel cũ lấy mẫu trong8×4m local aisle. Khi đúng native group goal_radius1.44m, đổi đích nhóm có thể không tìm được chỗ trống và chờ30wall s. Probe offline03 được dừng có ghi nguyên nhân. Cấu hình mới riêng dùng vùng lấy mẫu thực60×40m/origin(0,0),giữ đủ42 walls và thuật toán native; không thêm tường giả, sửa vendor hoặc thay robot/Nav2. Goal sampling rộng không có nghĩa đã nghiệm thu người đi khắp nhà xưởng. Probe offline01/02 trước đó dùng bán kính goal cá nhân nhỏ hơn, chỉ giữ làm lịch sử fixture; không gộp thành bằng chứng native-group final.

Offline04 khóa walking v2; offline05 kiểm bốn cấu hình×100 bước với nhánh đứng v3 và exact20 repeat, swept geometry độc lập. reaction_gate phát lại một interval robot đo tại lỗi standing v2: fixed reference không khả thi, v3 khả thi với cap0.2m/s; far control khớp. Đây là one-interval counterfactual, không phải replay toàn bộ cùng trajectory. Gate03:10 warmup transport ticks giữ nguyên toàn bộ native/human/group state, sau gate20 bước khớp exact offline04, clock lỗi terminal. Trong Gazebo chỉ mở gate khi accepted NavigateToPose; không publisher robot velocity script. YAML runtime chỉ đổi AMCL initial x−27→−28, robot/config/vendor/source cũ và staging của người dùng giữ nguyên.

![Quỹ đạo thật và tâm nhóm đo từ pose Gazebo](group_actual_paths.png)

Đồ thị giới hạn ROI quanh robot; human paths ra ngoài khung vẫn giữ trong dữ liệu và kiểm hình học. Tâm nhóm tính lại từ pose thật của các thành viên theo native membership, không lấy nhãn center làm bằng chứng chuyển động.

## Quan sát nhóm và cảm biến

- **walking01_v2**: đoạn nhóm đi dài nhất 7.061m; đoạn nhóm đứng dài nhất 2.500s; đường kính cặp thành viên lớn nhất 2.092m. Có 1 mốc đổi membership/loại nhóm; group objects mới []. LiDAR 7571 tia đủ điều kiện, 97.69% khớp trong15cm, IDs[1, 2, 3, 5]; AMCL XY max 0.160m, RTF 0.313.
- **walking02_v2**: đoạn nhóm đi dài nhất 10.253m; đoạn nhóm đứng dài nhất 16.700s; đường kính cặp thành viên lớn nhất 2.637m. Có 4 mốc đổi membership/loại nhóm; group objects mới [3]. LiDAR 19080 tia đủ điều kiện, 98.85% khớp trong15cm, IDs[1, 2, 3, 4, 5, 6]; AMCL XY max 0.146m, RTF 0.425.
- **standing01_v3_headless**: đoạn nhóm đi dài nhất 0.000m; đoạn nhóm đứng dài nhất 25.500s; đường kính cặp thành viên lớn nhất 1.386m. Có 1 mốc đổi membership/loại nhóm; group objects mới []. LiDAR 19714 tia đủ điều kiện, 98.90% khớp trong15cm, IDs[1, 2, 3, 4, 5, 6]; AMCL XY max 0.232m, RTF 0.409.
- **standing02_v3_headless**: đoạn nhóm đi dài nhất 0.000m; đoạn nhóm đứng dài nhất 18.700s; đường kính cặp thành viên lớn nhất 1.377m. Có 2 mốc đổi membership/loại nhóm; group objects mới []. LiDAR 11822 tia đủ điều kiện, 98.66% khớp trong15cm, IDs[1, 2, 3, 4, 5, 6]; AMCL XY max 0.156m, RTF 0.323.


- standing01_v3_headless: 105 member-updates có nhích tránh; max kế hoạch 0.1497m/s, được đối chiếu actual member speed trong từng segment.
- standing02_v3_headless: 53 member-updates có nhích tránh; max kế hoạch 0.1262m/s, được đối chiếu actual member speed trong từng segment.

Moving coverage nghĩa có một đoạn membership/loại liên tục≥2s và actual centroid path≥0.5m; standing coverage chỉ nghĩa membership mang nhãn standing≥2s, không khẳng định mọi thành viên đứng bất động. Đường kính, member peak speed và fraction quay vào tâm trong30° lưu từng segment ởvalidation/summary. Không đồng nhất coverage với cohesion tốt, độ tự nhiên hoặc robot đã nhường theo xã hội. Tan/đổi nhóm là hành vi được giữ, không lỗi bị xóa. Nav2 có thể đi xuyên khoảng trống giữa các thành viên nhóm trò chuyện; tâm robot có lúc chỉ cách tâm nhóm khoảng0.12m trong standing seed42. Đây là tránh hình học từng người, chưa tránh không gian xã hội của nhóm. Bản nhích tránh giúp human motion khả thi trong các lượt đo, không biến Nav2 thành social group-aware planner. Phần standing_reaction trong validation ghi số update nhích thật và tốc độ kế hoạch tối đa, được kiểm≤0.2m/s. Source group ID theo object trong một session; human IDs1..6 giữ xuyên thay đổi.

Clearance dùng swept discs/AABBs giữa actual pose samples; rotated robot footprint0.24×0.20m chỉ tại samples. Các số này không tự chứng nhận social comfort. LiDAR kiểm tia trước static occlusion; costmap occupied-ring là bằng chứng mô tả, không attribution/recall. Goal, geometry và group coverage được đánh giá riêng.

## Lượt lỗi gốc và runtime

**walking01 đầu tiên lỗi NVIDIA/WSL khi GUI khởi tạo Ogre/EGL**, chỉ một native active update, chưa đánh giá Nav2. Log có stack libnvwgf2umx.so/segmentation fault; giữ result/validation/raw gzip/source cũ. Cohort gốc đã dừng. Sau đó chốt protocol mới với nhãn_v2, khởi động server trước và GUI riêng sau khi sensor/AMCL/Nav2 sẵn sàng; đồng thời sửa fixture goal_radius và vùng sampling như trên. Đây là bộ đo sau thay đổi được khai báo, không retry âm thầm hoặc sửa outcome cũ. Trong v2, standing01 dừng bảo vệ sau15.06sim s vì robot lọt vào khe giữa thành viên bị khóa; prefix gap robot–người còn4.93mm, bước dự đoán kế không giữ được floor2mm. Standing seed43 v2 chưa chạy theo quy tắc dừng. Chỉ sau reaction/offline/gate mới mới đo hai standing v3; không gọi v3 là native hoặc sửa kết quả v2. Chưa chứng minh đã loại bỏ mọi lỗi driver WSL.

GPU NVIDIA được kiểm preflight/Ogre cho các lượt nghiệm thu; worker giới hạn2 CPU/one BLAS thread, Nav2/physics/ORCA vẫn cần CPU. GUI vàRTF khác nhau, không joint-seed AMCL/MPPI/physics; không xếp hạng runtime hoặc suy tỷ lệ thành công tổng quát. Server/GUI kết thúc theo phiên, không kill tiến trình không liên quan. Walking01_v2 đã chạy GUI; đứng v3 đầu tiên vẫn bị Qt Failed to create OpenGL context và abort sau27 native updates dù startup tuần tự. Giữ lỗi, không coi GUI đã sửa dứt điểm. Sau đó chốt hai standing v3 headless, giữ nguyên worker/motion/Nav2 và ghi camera GPU thật. Chưa có xác nhận kéo/zoom riêng của người dùng cho cohort này.

Ảnh/GIF từ RGB camera Gazebo thật640×480/1frame mỗi sim-second, phát4× simulation time; không AI và không quay màn hình realtime. Góc camera không thấy mọi người suốt lượt.

![Nhóm đi — camera Gazebo thật](walking01_v2/camera_preview_4x.gif)

![Nhóm đứng — camera Gazebo thật](standing01_v3_headless/camera_preview_4x.gif)

## Quyết định demo và phần còn lại

Bảng trên là nghiệm thu thực tế của pilot; chưa gọi dense crowd/không gian chật hoặc nhiều seed đã đạt. SocNavGym variant phù hợp nhất để tiếp tục demo nhóm đi/đứng/lifecycle trong các nhánh đã khảo sát; CSP ORCA+ giữ baseline người độc lập, CN geometry giữ nhánh hybrid hình học chưa trained RGL. Không thay backend mặc định hoặc kiến trúc robot, không World Model/Isaac.

Bộ hướng dẫn DEMO_GUIDE.md chốt cách trình bày và bảng lựa chọn dựa trên bằng chứng đã lưu; các failure/partial coverage vẫn hiển thị. Hai walking action SUCCEEDED nhưng actual XY21.12/29.77cm vượt20cm, vì vậy navigation acceptance của walking chưa đạt. Chưa chẩn đoán nhân quả đầy đủ lỗi goal và không tune AMCL/Nav2 trong pilot này. Những mở rộng như dense crowd/narrow-aisle nhóm, in-place reset hoặc chất lượng animation là nhiệm vụ bổ sung, không tự gộp thành nghiệm thu pilot hiện tại.

[Protocol](../protocol.json) · [Số đo](summary.json) · [Offline cuối](offline05/validation.json) · [Gate cuối](gate03/validation.json) · [Lượt GUI lỗi](walking01/validation.json) · [Lệnh walking và lỗi standing cũ](v2_execution.json) · [Lệnh standing v3 headless](standing_headless_execution.json) · [Lượt GUI v3 lỗi](standing01_v3/validation.json) · [Điều chỉnh headless](headless_amendment.json) · [Standing v2 bị dừng](standing01_v2/validation.json). Raw pose/scan/odom/commands/plan/costmap/AMCL/TF/worker/world/config/log có gzip byte-identical trong từng folder.
