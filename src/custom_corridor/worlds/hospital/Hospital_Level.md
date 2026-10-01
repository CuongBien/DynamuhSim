# Hospital Easy, Medium và Hard

Tài liệu cho DynamuhSim, Ubuntu 24.04, ROS 2 Jazzy và Gazebo Sim 8. Dựa trên mã nguồn DynamuhSim(4).zip và bản sửa launch phân biệt ba mức. Hard hiện dùng H8–H14; **chưa áp dụng giường bệnh hoặc chậu hoa**.

## 1. Bố trí chung

| Thành phần | Thông số |
|---|---|
| Kích thước bệnh viện | 60 × 17 m |
| Hành lang | Rộng 5 m |
| Phòng | 20 phòng, 10 Bắc và 10 Nam |
| Kích thước phòng | Rộng 5,6 m, sâu 6 m |
| Cửa | Rộng 1,2 m |
| Tường | Cao 2,5 m |
| Ghế | Hai bên cửa phòng |
| Không gian | Không trần, không tường bịt hai đầu hành lang |
| Robot | Burger, spawn tại x=-18, y=0, hướng +x |

Trục x chạy dọc hành lang; y dương là Bắc, y âm là Nam. Thời gian actor là giây mô phỏng.

## 2. So sánh ba level

| Nội dung | Easy | Medium | Hard |
|---|---|---|---|
| Người di chuyển | 3 | 7 | 14 |
| Người tĩnh | 2 ngồi + 1 đứng | 3 đứng | 3 đứng kế thừa Medium |
| Xe đẩy | 0 | 1 | 2 |
| Tổng thẻ actor trong SDF | 3 | 8 | 16 |
| Ghế | Chân ghế riêng | Đế kín thấp | Kế thừa Medium |
| Hành vi | Tuyến cố định, vào/ra phòng | Thêm đối đầu, cùng chiều, băng ngang, xe và nhường robot | Thêm ra/vào cửa, băng ngang liên tiếp, nhóm và xe thứ hai |
| SDF | hospital_easy.sdf | hospital_medium.sdf | hospital_hard.sdf |

Xe đẩy cũng dùng actor, nên số thẻ actor khác số người. Người tĩnh là model, không tính vào thẻ actor. Mức độ khó nêu ở đây là thiết kế kịch bản, chưa phải kết quả benchmark navigation.

## 3. Hospital Easy

Dùng cho kiểm tra nền: world, robot, cảm biến, bản đồ và các tuyến người cơ bản.

| Actor | Kịch bản |
|---|---|
| hospital_walking_human | Từ hành lang trước phòng 3 đến phòng 5 Nam, vào phòng rồi quay về; chu kỳ 41,2 s |
| hospital_walking_human_2 | Phòng 6 Nam → hành lang trước phòng 2, chờ 10 s → phòng 3 Bắc |
| hospital_walking_human_3 | Chờ cạnh ghế phòng 2 Nam, sau đó đi theo actor 2 vào phòng 3 Bắc |

Có 2 người ngồi và 1 người đứng ở vùng ghế phòng 5 Bắc. Easy giữ physics bước 0,001 s và shadows bật theo file Easy nguồn; Medium/Hard dùng bước 0,004 s, shadows tắt. Do đó khác biệt hiệu năng còn có thể do cấu hình mô phỏng.

Lưu ý: Easy nguồn có thanh đánh dấu cửa dạng vật rắn; actor 2/3 bật loop nhưng không có tuyến quay về khép kín trong danh sách waypoint. Đây là hành vi kế thừa, chưa được sửa trong bản gộp.

## 4. Hospital Medium

Kế thừa phòng/hành lang, dựng tập người riêng so với Easy.

| Người | Tên actor | Kịch bản |
|---|---|---|
| H1 | medium_walker_01 | Đi dọc đầu hành lang, đổi làn theo tuyến cố định để tránh vùng xe |
| H2 | medium_walker_02_head_on | Đi ngược chiều, quay lại trên đoạn x=2..8 |
| H3 | medium_walker_03_room_crossing | Phòng 5 Nam ↔ phòng 5 Bắc |
| H4 | medium_walker_04_same_direction | Đi dọc đoạn x=11..17 rồi quay lại |
| H5 | medium_walker_05_wait_then_cross | Chờ trong phòng 9 Nam rồi băng ngang sang Bắc |
| H6 | medium_walker_06_trolley_pusher | Đẩy xe trên đoạn x=-24..-18, y=0,6 |
| H7 | medium_walker_07_yielding | Đi ngang tại x=0; plugin nhường robot |
| Xe 1 | medium_trolley_06 | Chuyển động đồng bộ với H6 |

Ba người đứng ở vùng phòng 4 Nam, 7 Bắc và 8 Nam. H7 dùng HospitalYieldingSystem với model robot burger: robot gần đoạn băng ngang dưới 3,5 m thì người đi về bên gần nhất tại y=±1,35; robot xa trên 4,5 m liên tục 1 s, chờ thêm 0,5 s rồi tiếp tục. Cần kiểm chứng hành vi plugin trên máy chạy thực tế.

## 5. Hospital Hard

Giữ toàn bộ Medium, thêm H8–H14. Tên medium_* vẫn tồn tại trong Hard vì được kế thừa.

| Người | Tên actor | Kịch bản | Chu kỳ |
|---|---|---|---|
| H8 | hard_walker_08_door_exit | Ra phòng 5 Bắc, dừng trước cửa 4 s, đi về phía Tây rồi về phòng | 40 s |
| H9 | hard_walker_09_door_entry | Chờ 12 s ở hành lang, vào phòng 5 Nam, chờ 8 s rồi trở ra | 48 s |
| H10 | hard_walker_10_cross_south_to_north | Phòng 6 Nam → Bắc → quay về; bắt đầu sau 5 s | 28 s sau bắt đầu |
| H11 | hard_walker_11_cross_north_to_south | Phòng 7 Bắc → Nam → quay về; bắt đầu sau 8 s | 28 s sau bắt đầu |
| H12 | hard_walker_12_group | Đi cùng H13 từ x=11,5 đến 18,5 và quay về, dừng 5 s tại x=15,5 | 51 s |
| H13 | hard_walker_13_group | Đi song song H12, khoảng cách tâm 0,8 m; tốc độ 0,5 m/s | 51 s |
| H14 | hard_walker_14_trolley_pusher | Đẩy xe từ x=22 tới 12,8 tại y=-1,2; dừng giao đồ 8 s gần phòng 8; tốc độ 0,4 m/s | 71 s |
| Xe 2 | hard_trolley_14 | Đồng bộ pose và thời gian với H14 | 71 s |

H10/H11 dùng hai cửa kế nhau, lệch 3 s, tốc độ băng ngang 0,8 m/s. Nhóm H12/H13 thu hẹp lối ở giữa hành lang nhưng không chặn kín hành lang. H8–H14 chạy theo lịch cố định, không kích hoạt theo vị trí robot.

## 6. Chuẩn bị và build

Chạy tại **thư mục gốc DynamuhSim**; các lệnh dùng zsh. Cần generator gộp và launch hỗ trợ easy/medium/hard. Giữ đầy đủ mesh walk.dae, stand.dae, push_trolley.dae, trolley.dae và plugin nhường đường.

Sinh cả ba world và build một lần:

```bash
source /opt/ros/jazzy/setup.zsh
python3 src/custom_corridor/worlds/hospital/generate_hospital.py --all
colcon build --packages-select custom_corridor_plugins custom_corridor --symlink-install
```

Chỉ tiếp tục sau build thành công:

```bash
source install/setup.zsh
```

Nếu plugin đã build và không đổi, có thể chỉ build custom_corridor. Không chép build/install/log của máy khác; install trong ZIP nguồn có symlink tuyệt đối tới workspace của người tạo.

## 7. Lệnh chạy từng level

Chỉ mở **một phiên hospital tại một thời điểm**. Ctrl+C để đóng launch/Gazebo và Nav2 cũ trước khi đổi level. Terminal mới luôn source ROS và workspace.

### Easy — Terminal 1

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch custom_corridor hospital_arena.launch.py difficulty:=easy gui:=true rviz:=true
```

### Medium — Terminal 1

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch custom_corridor hospital_arena.launch.py difficulty:=medium gui:=true rviz:=true
```

### Hard — Terminal 1

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch custom_corridor hospital_arena.launch.py difficulty:=hard gui:=true rviz:=true
```

gui:=true mở Gazebo GUI; rviz:=true mở RViz. Đổi rviz:=false nếu chưa cần RViz. Các lệnh arena tạo world, robot, bridge và robot_state_publisher; **chưa chạy Nav2 và chưa tự đưa robot đến mục tiêu**.

Nếu vừa sửa generator, sinh lại đúng level và build trước launch. Ví dụ Medium:

```bash
python3 src/custom_corridor/worlds/hospital/generate_hospital.py --difficulty medium
colcon build --packages-select custom_corridor --symlink-install
source install/setup.zsh
```

Thay medium bằng easy hoặc hard khi cần. Mỗi lựa chọn chỉ ghi SDF tương ứng; --all ghi cả ba.

## 8. Chạy navigation — Terminal 2

Sau khi Terminal 1 mở world mong muốn:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch custom_corridor nav2_hospital.launch.py controller:=mppi map:=hospital/hospital_easy auto_route:=false
```

Dùng chung bản đồ nền vì ba mức dùng cùng bố trí phòng/hành lang; xác nhận bản đồ khớp world và footprint ghế thực tế. Người/xe di chuyển không nằm trong bản đồ tĩnh. Lệnh trên dùng auto_route:=false để chọn mục tiêu thủ công; đặt initial pose nếu cần rồi gửi goal trong RViz.

Nhiệm vụ thử nghiệm dự kiến: điểm xuất phát → phòng 5 Bắc → ra hành lang → phòng 5 Nam → ra hành lang → về điểm xuất phát. Để bật nhiệm vụ tự động có sẵn trong bản nguồn, thay auto_route:=false bằng auto_route:=true. Đóng phiên Nav2 cũ trước khi chạy lại. Bản nguồn mặc định auto_route=true; tài liệu ghi rõ tham số để tránh robot tự nhận nhiệm vụ khi chỉ muốn kiểm tra. Cần xác nhận kết quả tuyến tự động trên phiên mô phỏng thực tế.

## 9. Kiểm tra chạy đúng level

Bản sửa launch báo difficulty, world, đường dẫn SDF thực tế, số actor và GZ_PARTITION. Kết quả đúng:

| Level | World | Số actor, gồm xe |
|---|---|---|
| Easy | hospital_easy | 3 |
| Medium | hospital_medium | 8 |
| Hard | hospital_hard | 16 |

Kiểm tra package và file đã cài:

```bash
ros2 pkg prefix custom_corridor
rg -n '<world name=|hard_walker_|hard_trolley_' \
  "$(ros2 pkg prefix custom_corridor)/share/custom_corridor/worlds/hospital/hospital_medium.sdf"
```

Medium phải có hospital_medium và không có hard_walker_/hard_trolley_. Bản launch sửa dùng partition riêng để tránh Gazebo cũ; partition không tách các ROS topic. Vì vậy vẫn đóng phiên cũ trước khi chạy phiên mới.

Nếu dùng gz CLI ở terminal ngoài, đặt đúng giá trị GZ_PARTITION in trong log trước:

```bash
export GZ_PARTITION="gia_tri_in_trong_log_launch"
gz topic -l
```

Kiểm tra ROS không cần đặt partition:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic list
```

## 10. Giới hạn và đánh giá

- Người/xe động hiện dùng visual actor; chưa có collision chuyển động đồng bộ. Cần kiểm tra /scan và local costmap nhận được chúng trước khi kết luận robot tránh vật cản thành công.
- Người tĩnh, ghế và tường có collision. Đế ghế Medium/Hard hỗ trợ tạo hình học ở độ cao quét thấp; kiểm chứng scan thực tế.
- Actor scripted chưa tự né robot/actor khác. Có thể gặp nhau hoặc xuyên nhau; thêm người chưa đồng nghĩa đã có tương tác xã hội.
- H7 dùng vị trí robot có sẵn trong Gazebo để nhường, chưa nhận biết bằng camera/LiDAR.
- Bản gộp đã đối chiếu ba world với file nguồn. Launch đã kiểm tra logic bằng ROS module giả lập; chưa kiểm chứng runtime Gazebo trong môi trường tạo bản vá.
- Khi so sánh các level, dùng cùng tuyến mục tiêu và tốc độ robot; ghi tỷ lệ hoàn thành, thời gian, khoảng cách gần vật cản nhất, va chạm và thời gian đứng chờ. Ghi cả real-time factor để phân biệt tải mô phỏng với thay đổi điều khiển.
