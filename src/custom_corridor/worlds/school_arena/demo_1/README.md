# School Arena — Demo 1

Demo tự chứa dựa trên `../school_arena_hunav.sdf`. World gốc đã có 15 phòng
học cùng hành lang chính/phụ, 30 cửa, cầu thang, ghế, locker, góc khuất và đoạn
hẹp; cộng các vùng chức năng là gần 20 không gian trường học. World kế thừa
hình học gốc; generator chỉ ép sáu đạo cụ rời sát tường để giữ clearance cho
footprint và luôn sinh lại world, map, zone, graph từ cùng hệ tọa độ.

## Đầu ra bắt buộc

```text
school_floor.world
map.pgm
map.yaml
zones.yaml
human_navigation_graph.yaml
```

Các file hỗ trợ:

- `generate_demo.py`: sinh lại toàn bộ năm đầu ra từ world gốc.
- `validate_demo.py`: kiểm tra XML, occupancy, zone, graph và cạnh cắt tường.
- `school_demo.launch.py`: Gazebo + robot + bridge + Nav2 + RViz đồng bộ.
- `demo_robot.sdf`: robot màu cam, mũi chỉ hướng xanh, không phụ thuộc mesh để nhìn thấy.
- `demo_1.rviz`: hiển thị plan, footprint, Gazebo ground-truth pose và actual trail.
- `pose_sync_monitor.py`: đo sai lệch pose Gazebo với `map→base_footprint`.
- `collision_safe_pose_applier.py`: chặn mọi bước HuNav có footprint cắt occupancy.
- `nav2_plan_clearance_test.py`: kiểm tra live 6 global plan bằng footprint robot.
- `nav2_baseline_test.py`: test live TF, odom, scan, map, path clearance và nhiều goal Nav2.
- `hunav_runtime_probe.py`: đo chuyển động và khoảng cách giữa các HuNav agent.
- `LIVE_TEST_REPORT.md`: kết quả acceptance Gazebo/Nav2/HuNav đã chạy.
- `hunav_one_human.yaml`: một người Room01 → corridor → Room10.
- `hunav_social_test.yaml`: thêm người đi ngang để quan sát human-human force.
- `setup_demo_hunav_container.sh`: cài scenario/BT/bridge vào Docker.

## 1. Sinh lại world, map, zones và graph

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena/demo_1
./generate_demo.py
./validate_demo.py --write-report
```

Generator đọc collision tĩnh trong `school_floor.world` và raster trực tiếp
sang PGM 0.05 m/cell; không dùng ảnh vẽ tay. Map có kích thước 730 × 580,
origin `[-16.0, -14.0, 0.0]`. Muốn chỉnh layout gốc, sửa
`../generate_school_world.py`/world nguồn rồi chạy lại generator demo.

Kết quả validator hiện tại:

```text
PASS: Robot A → B
PASS: Human Room01 → Room12
PASS: Human Stair → Room05
PASS: Human Corridor → Room
```

PASS này xác nhận topology có đường đi và toàn footprint HuNav bán kính 0,22 m
trên mọi cạnh đều cách occupancy; không còn phép kiểm tra point-only. Báo cáo nằm tại `STEP1_TEST_REPORT.md`.

## 2. Semantic zones

`zones.yaml` dùng frame `map`, đơn vị mét và có đủ:

- `corridor`
- `classroom`
- `doorway`
- `intersection`
- `blind_corner`
- `bottleneck`
- `waiting_area`
- `stair`

Mỗi phòng và cửa có polygon riêng. Stair có `robot_access: false`.
Bottleneck ghi cả `effective_width` để planner cấp cao có thể giảm tốc/yield.

## 3. Human navigation graph

`human_navigation_graph.yaml` không cho random XY. Tuyến bắt buộc là:

```text
classroom → doorway → corridor → junction → doorway → classroom/stair
```

Mỗi phòng có center, predoor, door và approach. `predoor` buộc người đi
vuông góc qua cửa trước khi nhập hành lang, tránh cắt góc tường. Validator lấy
mẫu mỗi 0,025 m và fail nếu bất kỳ footprint bán kính 0,22 m chạm occupied cell.

## 4. Chạy Gazebo + Nav2 baseline

Build workspace trước:

```bash
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Terminal 1:

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena/demo_1
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
ros2 launch ./school_demo.launch.py gui:=true rviz:=true
```

Launch sẽ:

1. Mở `school_floor_baseline.world` (cùng geometry, bỏ proxy người để test robot).
2. Spawn `demo_robot.sdf` màu cam tại `(-13.5, -8.15)` và camera Gazebo nhìn sẵn vào vị trí này.
   Sau khi spawn, launch tự gọi `/gui/follow` và đặt camera bám model vật lý `robot`.
3. Bridge `/clock`, `/tf`, `/odom`, `/scan`, `/cmd_vel` và camera.
4. Chạy robot state publisher.
5. Chạy map server, AMCL, SmacPlanner2D, controller và BT Navigator.
6. Mở RViz bằng `demo_1.rviz` và chạy pose-sync monitor.

Goal checker chấp nhận vị trí trong bán kính 0,20 m và không bắt robot phải
xoay đúng yaw trong phòng hẹp. Thay đổi này chỉ tránh recovery loop tại đích;
planner vẫn dùng footprint đầy đủ, không đi unknown và kiểm tra clearance tới
tường như trước.

Kiểm tra nhanh:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action list | grep navigate_to_pose
ros2 topic echo /demo/gazebo_robot_pose --once
ros2 topic echo /demo/pose_sync_error --once
```

Trong RViz: global plan màu xanh lá, local plan màu xanh dương, actual trail
màu cam, mũi tên cam là pose thật Gazebo, hai polygon là footprint. Các điểm
đỏ mang tên `Laser hits (red points, not a path)` là laser scan, không phải
đường robot. Trong Gazebo robot có vỏ cam và thanh chỉ hướng xanh. Camera Gazebo tự bám
robot; có thể kéo chuột để đổi góc nhìn nhưng target follow vẫn là model vật
lý. Sai lệch
`/demo/pose_sync_error` nên dưới 0,20 m.

Không chạy đồng thời script cũ `../hunav_gz8_pose_applier.py`; hai pose
applier cùng ghi model là một nguyên nhân gây nhảy hoặc xuyên tường. Hai launch
trong demo này chỉ dùng `collision_safe_pose_applier.py`.

Không mở hai `school_demo.launch.py`/`school_hunav_demo.launch.py` cùng lúc vì
các phiên dùng chung `GZ_PARTITION=school_hunav`; chạy trùng sẽ trộn `/clock`,
`/tf` và làm Gazebo UI có vẻ lệch RViz.

Terminal 2 — kiểm tra sáu plan live trước (chạy nhanh):

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./nav2_plan_clearance_test.py
```

Kết quả đúng là 6 dòng `footprint-clear` và `PASS: all live Nav2 plans are
footprint-clear`. Sau đó chạy baseline nhiều goal:

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena/demo_1
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./nav2_baseline_test.py --timeout 180
```

Test chỉ PASS khi nhận được TF `map→odom→base_footprint`, `/odom`,
`/scan`, `/map`, Nav2 action server và robot hoàn thành lần lượt:

```text
A → Room05
Room05 → CorridorC
CorridorC → JunctionCD → JunctionDE → JunctionEF → Room12
```

## 5. Chạy HuNavSim cơ bản

Khởi động container theo README thư mục cha, sau đó:

```bash
docker start hunavsim_gz_fortress
cd ~/nav_ws/src/custom_corridor/worlds/school_arena/demo_1
./setup_demo_hunav_container.sh hunavsim_gz_fortress
```

Dừng launch baseline nếu đang chạy. Khởi động world có proxy HuNav bằng `ros2 launch ./school_hunav_demo.launch.py gui:=true rviz:=true`, rồi giữ terminal đó mở. Mở ba terminal Docker sau.

### Docker terminal A — loader một người

```bash
docker exec -it hunavsim_gz_fortress bash
source /opt/ros/humble/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
ros2 run hunav_agent_manager hunav_loader --ros-args --params-file \
  /home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/demo_1_hunav.yaml
```

### Docker terminal B — agent manager

```bash
docker exec -it hunavsim_gz_fortress bash
source /opt/ros/humble/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
ros2 run hunav_agent_manager hunav_agent_manager --ros-args \
  -p use_sim_time:=false \
  -p publish_tf:=false \
  -p publish_sfm_forces:=false
```

### Docker terminal C — adapter

```bash
docker exec -it hunavsim_gz_fortress bash
source /opt/ros/humble/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
python3 /tmp/school_arena/hunav_gz8_school_bridge.py \
  --scenario /home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/demo_1_hunav.yaml \
  --sdf /tmp/school_arena/demo_1_school_floor.world \
  --ros-args \
  -p world_name:=school_arena \
  -p robot_name:=robot \
  -p update_hz:=10.0 \
  -p human_z:=0.0 \
  -p route_half_width:=0.45
```

`demo_primary` bắt đầu trong Room01 và đi qua đúng chuỗi node graph đến
Room10. Behavior Tree `SchoolInteractiveTree` gọi native `AvoidRobot` khi
robot vào phạm vi 0.95 m; các hệ số obstacle/social/other force đều bật.

Một scenario chỉ có một người không thể kiểm chứng tương tác người-người. Để
quan sát social force human-human, dừng ba node trên và thay
`demo_1_hunav.yaml` bằng `demo_1_social.yaml` trong lệnh loader/adapter.
Scenario này thêm `demo_crossing` đi ngang hành lang nhưng giữ nguyên tuyến
của người chính.

## 6. Tiêu chí chấp nhận

Kiểm tra tĩnh:

```bash
./validate_demo.py --write-report
```

Kiểm tra planner live và robot live:

```bash
./nav2_plan_clearance_test.py
./nav2_baseline_test.py --timeout 180
```

Kiểm tra HuNav live:

```bash
ros2 topic hz /school_hunav/target_poses
```

Quan sát trong Gazebo rằng người đi đúng cửa/hành lang, không xuyên tường,
yield khi robot vào gần và đổi vận tốc khi gặp người đi ngang trong social
scenario. Không ghi PASS live nếu chưa chạy simulator; báo cáo tĩnh và kết quả
live được tách riêng để tránh kết luận sai.

