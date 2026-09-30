# Hospital Medium

Hospital Medium là môi trường bệnh viện dùng để thử nghiệm robot di chuyển trong hành lang có người và xe đẩy. Môi trường sử dụng bố trí hành lang, phòng và ghế của Hospital Easy; các người trong Medium được xây dựng riêng.

## 1. Bố trí môi trường

| Thành phần | Thông số |
|---|---|
| Kích thước bệnh viện | 60 × 17 m |
| Hành lang | Rộng 5 m |
| Phòng | 20 phòng: 10 phía Bắc, 10 phía Nam |
| Kích thước phòng | Rộng 5,6 m; sâu 6 m |
| Tường | Cao 2,5 m |
| Cửa | Rộng 1,2 m |
| Ghế | 40 ghế, bố trí hai bên mỗi cửa |
| Phần dưới ghế | Đế kín từ sàn đến cao 0,36 m; có visual và collision |
| Trần và hai đầu hành lang | Không có trần, không có tường bịt hai đầu |
| Thanh đánh dấu cửa | Đã bỏ trong Medium để thông thoáng cửa |

Đế kín ghế giúp mặt phẳng quét LiDAR thấp có thể gặp hình học ghế. Cần kiểm tra `/scan` trong RViz trên máy chạy Gazebo để xác nhận dữ liệu thực tế.

## 2. Danh sách người và xe đẩy

Hiện có **7 người di chuyển, 3 người đứng yên và 1 xe đẩy**. Xe được biểu diễn bằng một actor riêng để chuyển động đồng bộ với người đẩy; tổng số thẻ `<actor>` trong SDF là 8, không phải 8 người.

| STT | Tên trong SDF | Kịch bản |
|---|---|---|
| 1 | `medium_walker_01` | Đi dọc đầu hành lang; chuyển sang tuyến đối diện để tránh xe theo lộ trình cố định |
| 2 | `medium_walker_02_head_on` | Đi ngược chiều và quay lại |
| 3 | `medium_walker_03_room_crossing` | Băng ngang qua phòng 5 Nam và phòng 5 Bắc |
| 4 | `medium_walker_04_same_direction` | Đi dọc hành lang rồi quay lại |
| 5 | `medium_walker_05_wait_then_cross` | Chờ trong phòng 9 Nam rồi băng qua phòng 9 Bắc |
| 6 | `medium_walker_06_trolley_pusher` | Đẩy xe ba tầng, đi và quay đầu trên đoạn đầu hành lang |
| 7 | `medium_walker_07_yielding` | Đi ngang tại giữa hành lang; nhường đường khi robot tới gần |
| 8 | `medium_standing_room_04_south` | Đứng trước phòng 4 Nam |
| 9 | `medium_standing_room_07_north` | Đứng trước phòng 7 Bắc |
| 10 | `medium_standing_room_08_south` | Đứng trước phòng 8 Nam |
| Xe | `medium_trolley_06` | Xe ba tầng, khung, tay đẩy và bốn bánh |

### Lộ trình các actor

Hệ tọa độ: trục `x` chạy dọc hành lang; phía Bắc có `y > 0`, phía Nam có `y < 0`. Đơn vị khoảng cách là mét; thời gian là giây mô phỏng.

| Actor | Lộ trình / chu kỳ |
|---|---|
| 1 | `x=-10 ↔ -26`; chuyển từ `y=0` sang `y=-1,3` trước khi vào vùng xe; chu kỳ 48 s |
| 2 | `x=8 ↔ 2`, `y=0`; chu kỳ 20 s |
| 3 | `x=-2,8`, `y=-4 ↔ 4`; chu kỳ 26 s |
| 4 | `x=11 ↔ 17`, `y=0`; chu kỳ 20 s |
| 5 | `x=19,6`, `y=-4 ↔ 4`; chờ 8 s trước lượt băng ngang đầu; chu kỳ 33,4 s |
| 6 và xe | `x=-24 ↔ -18`, `y=0,6`; chu kỳ 32 s, tốc độ trên đoạn thẳng 0,5 m/s |
| 7 | `x=0`, `y=-1,35 ↔ 1,35`; tốc độ đi bình thường 0,55 m/s, chờ 2 s ở mỗi đầu |

Actor 1 và xe có khoảng cách giữa hai tuyến là 1,9 m. Đây là bố trí tránh nhau theo lộ trình, chưa phải phản ứng theo cảm nhận người hoặc xe.

### Người đứng yên

| Vị trí | Tọa độ `(x, y)` | Hướng nhìn |
|---|---|---|
| Phòng 4 Nam | `(-9,4; -1)` | Dọc hành lang về phía `+x` |
| Phòng 7 Bắc | `(9,4; 1)` | Dọc hành lang về phía `-x` |
| Phòng 8 Nam | `(13; -1)` | Dọc hành lang về phía `+x` |

“Bên phải cửa” được xác định khi nhìn từ hành lang vào phòng. Mỗi người đứng có collision hình trụ bán kính 0,28 m, cao 1,70 m. Mesh `stand.dae` được tạo để hạ tay; cần quan sát Gazebo để xác nhận tư thế hiển thị.

## 3. Cơ chế nhường đường cho robot

Actor 7 sử dụng plugin riêng `HospitalYieldingSystem`, nạp bởi world Medium. Plugin đọc vị trí model robot có tên `burger` trong Gazebo.

1. Khi robot ở xa hoặc chưa có robot, người đi ngang giữa hai phía hành lang.
2. Khi robot cách **đoạn băng ngang** dưới 3,5 m, người đi về phía gần nhất tại `y=±1,35` với tốc độ 0,75 m/s rồi đứng chờ.
3. Khi robot cách đoạn băng ngang trên 4,5 m liên tục trong 1 s, người chờ thêm 0,5 s rồi tiếp tục tuyến đang đi.
4. Hoạt ảnh chân chỉ tăng thời gian khi người thực sự thay đổi vị trí; khi chờ thì thời gian hoạt ảnh được giữ lại.

Khoảng cách được tính đến toàn bộ đoạn băng ngang tại `x=0`, không chỉ đến vị trí tức thời của người. Hai ngưỡng 3,5 m và 4,5 m hạn chế việc đổi qua lại giữa đi và chờ.

Nếu robot dừng lâu trong vùng này, actor tiếp tục đứng chờ. Cơ chế dùng vị trí có sẵn trong mô phỏng, chưa dùng LiDAR hoặc camera để nhận biết robot, chưa dự đoán hướng robot và chưa tích hợp HuNav. Nó không bảo đảm tránh va chạm với mọi đường đi bất kỳ của robot.

## 4. Các file liên quan

Các đường dẫn dưới đây tính từ thư mục gốc dự án.

| File | Vai trò |
|---|---|
| `src/custom_corridor/worlds/hospital/generate_hospital.py` | Sinh world Easy hoặc Medium theo tham số `--difficulty` |
| `src/custom_corridor/worlds/hospital/hospital_medium.sdf` | World Medium được Gazebo nạp |
| `src/custom_corridor/launch/hospital_arena.launch.py` | Chọn world, mở Gazebo, tạo robot và ROS–Gazebo bridge |
| `src/custom_corridor/models/human/meshes/walk.dae` | Mesh và hoạt ảnh đi bộ |
| `src/custom_corridor/models/human/meshes/stand.dae` | Mesh tư thế cho người đứng |
| `src/custom_corridor/models/human/meshes/push_trolley.dae` | Hoạt ảnh người đẩy xe |
| `src/custom_corridor/models/human/meshes/trolley.dae` | Hình học và vật liệu xe |
| `src/custom_corridor_plugins/src/hospital_yielding_system.cc` | Điều khiển actor 7 nhường robot |
| `src/custom_corridor_plugins/CMakeLists.txt` | Build và cài thư viện plugin |

Thư viện sau khi build: `install/custom_corridor_plugins/lib/libhospital_yielding_system.so`.

Actor 1–6 sử dụng script của Gazebo. Actor 7 sử dụng plugin C++; plugin chỉ được nạp trong Medium. Code `multi_human_scenario_system.cpp` của School không được sửa để thêm hành vi này.

## 5. Build và chạy

Môi trường đích: Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim 8. Chạy tại thư mục gốc dự án; các lệnh dưới đây dùng zsh.

### Terminal 1: tạo world và mở mô phỏng

```bash
source /opt/ros/jazzy/setup.zsh
python3 src/custom_corridor/worlds/hospital/generate_hospital.py --difficulty medium
colcon build --packages-select custom_corridor_plugins custom_corridor --symlink-install
```

Chỉ chạy các lệnh tiếp theo sau khi build thành công:

```bash
source install/setup.zsh
ls -lh install/custom_corridor_plugins/lib/libhospital_yielding_system.so
ros2 launch custom_corridor hospital_arena.launch.py difficulty:=medium gui:=true rviz:=false
```

Khi thay đổi khai báo target trong CMake, thêm `--cmake-force-configure` vào lệnh build. Khi chỉ sửa C++ plugin, chỉ cần build `custom_corridor_plugins`. Khi chỉ đổi màu mesh, build `custom_corridor` rồi đóng và mở lại Gazebo; không cần sinh lại SDF.

### Terminal 2: mở navigation nếu cần

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
ros2 launch custom_corridor nav2_hospital.launch.py controller:=mppi map:=hospital/hospital_easy
```

Lệnh navigation sử dụng bản đồ tĩnh Hospital Easy vì bố trí phòng, cửa và footprint ghế tương ứng. Đây không phải bản đồ chứa người và xe di chuyển; cần kiểm tra sự khớp giữa bản đồ và world trên phiên bản dự án đang dùng.

## 6. Kiểm tra chức năng

| Kiểm tra | Kết quả cần quan sát |
|---|---|
| Actor 1–5 | Đi theo tuyến, quay đầu và chờ đúng kịch bản |
| Người đẩy xe | Người và xe di chuyển, quay đầu đồng bộ |
| Actor 7 khi robot ở xa | Chờ đầu tuyến rồi đi ngang hành lang |
| Actor 7 khi robot tới gần | Đi về phía bên cạnh và đứng nhường đường |
| Robot đã đi xa | Actor 7 tiếp tục đi |
| Người đứng | Đứng đúng vị trí, nhìn dọc hành lang |
| Ghế | Phần dưới kín; kiểm tra thấy trong `/scan` |
| Xe | Kiểm tra màu và tư thế tay trong Gazebo |

Có thể kiểm tra dữ liệu robot từ terminal khác:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
```

Các kiểm tra XML và generator đã được thực hiện. Bản sửa tìm actor đã được người dùng build thành công trên ROS/Gazebo đích. Bản sửa thông báo cập nhật chuyển động bằng `SetChanged()` mới được gửi và chưa có xác nhận hoạt động thực tế. Màu xe và tư thế đẩy xe cũng cần xác nhận lại sau bản sửa mesh.

## 7. Lỗi đã gặp

| Hiện tượng | Nguyên nhân / xử lý |
|---|---|
| Không tìm thấy `trolley.dae` | Chép đủ các file mesh vào đúng thư mục và build lại `custom_corridor` |
| Không tìm thấy `libhospital_yielding_system.so` | Kiểm tra target CMake, build thành công và đường dẫn plugin |
| Lỗi `operator!=` với `sdf::Actor` | Tìm actor bằng `Each<Name, Actor>` thay cho `EntityByComponents` so sánh Actor |
| Actor 7 hiện nhưng đứng yên | Bản sửa bổ sung `SetChanged()` cho `TrajectoryPose` và `AnimationTime`; cần chạy xác nhận |
| GUI trống hoặc treo | Chưa xác định nguyên nhân duy nhất; đóng các phiên cũ và thu log `gz sim -v 4` |
| Xe hiện trắng | Đã bổ sung vật liệu và normals; kết quả hiển thị sau sửa chưa được xác nhận |

Nếu file `.so` đã tồn tại nhưng Gazebo chưa tìm được, có thể đặt đường dẫn trong terminal chạy launch:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/install/custom_corridor_plugins/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

## 8. Giới hạn hiện tại và công việc tiếp theo

- Các actor di chuyển, kể cả xe, hiện là đối tượng visual; chưa có collision di chuyển đồng bộ. Không thể coi chúng đã là vật cản vật lý hoàn chỉnh cho kiểm thử robot.
- Ba người đứng yên và các vật thể tĩnh đã có collision.
- Actor 1–6 chưa tự né người hoặc robot; tuyến tránh xe của actor 1 được lập sẵn.
- Actor 7 có phản ứng theo vị trí robot trong mô phỏng; chưa dùng mô hình tương tác xã hội HuNav/Social Force.
- Các kịch bản còn cần xây dựng: `door_entry`, `door_exit`, `multi_crossing`, `group_blocking` và `dense_social`.

Ưu tiên tiếp theo: xác nhận actor 7 di chuyển và nhường robot đúng; xác nhận mesh xe; bổ sung collision/biểu diễn cảm biến cho người và xe di chuyển; sau đó thêm từng kịch bản mới để dễ phát hiện lỗi.
