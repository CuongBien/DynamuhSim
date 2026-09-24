# school_arena

Gazebo Sim 8 / ROS 2 Jazzy school scenario.

- 1 floor
- 15 classrooms
- 2 door openings / classroom
- corridor width: 1.2 m
- exactly 5 right-angle turns around the stair core
- 8 moving pedestrian proxy models
- static benches, lockers, bins, backpacks and notice board

Generate the legacy TrajectoryFollower-only preview (not used by the HuNav
pipeline):
```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
python3 generate_school_world.py
```

Preview:
```bash
./run_school_arena.sh
```

The launcher uses the `school_hunav` Gazebo Transport partition so that an
already-running Gazebo world (for example `hunav_test`) cannot capture this
GUI's world controls or `/clock` topic. The simulation starts playing
automatically. Override the defaults when needed:

```bash
GZ_PARTITION=my_school_test GZ_VERBOSITY=4 ./run_school_arena.sh
```

To integrate with the existing TurtleBot / Nav2 launch, copy
`corridor_tb3.launch.py` to `school_tb3.launch.py` and point its world argument to:

```python
school_world = os.path.join(
    get_package_share_directory('custom_corridor'),
    'worlds', 'school_arena', 'school_arena_hunav.sdf'
)
```

Then:
```bash
cd ~/nav_ws
colcon build --symlink-install
source install/setup.bash
```

If the world is not copied into `install/`, ensure `CMakeLists.txt` installs `worlds`:
```cmake
install(
  DIRECTORY launch config maps models rviz scripts worlds
  DESTINATION share/${PROJECT_NAME}
)
```

`school_arena.sdf` is retained only as the legacy TrajectoryFollower preview.
The launcher and the pipeline below use `school_arena_hunav.sdf`: HuNav owns
all pedestrian motion and Gazebo displays ground-anchored visual proxies.

Dưới đây là pipeline đầy đủ, cập nhật cho:

- Gazebo School Arena.
- HuNavSim trong Docker.
- TurtleBot3 có camera.
- Người di chuyển theo HuNav.
- Cửa sổ camera điều khiển robot bằng `W/A/S/D`.

Trước khi chạy, nhấn `Ctrl+C` để dừng toàn bộ terminal cũ. Không chạy `run_school_teleop.sh` cùng lúc với cửa sổ camera.

## Chuẩn bị — chạy một lần

```bash
docker start hunavsim_gz_fortress

cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./stop_school_hunav_container.sh hunavsim_gz_fortress
./setup_school_hunav_container.sh hunavsim_gz_fortress
```

Kết quả:

```text
Installed school HuNav scenario, BT files and runtime bridge into hunavsim_gz_fortress
```

---

## Terminal 1 — Gazebo School Arena

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena

source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
export GZ_PARTITION=school_hunav

export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

./run_school_arena.sh
```

Chờ cửa sổ Gazebo tải xong rồi mới mở các terminal tiếp theo.

Giữ Terminal 1 chạy.

---

## Terminal 2 — HuNav Loader trong Docker

```bash
docker exec -it hunavsim_gz_fortress bash
```

Trong Docker:

```bash
source /opt/ros/humble/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

ros2 run hunav_agent_manager hunav_loader \
  --ros-args \
  --params-file \
  /home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/school_agents.yaml
```

Kết quả cần có:

```text
Number of agents: 8
GetParameters service created
```

Giữ Terminal 2 chạy.

---

## Terminal 3 — HuNav Agent Manager

```bash
docker exec -it hunavsim_gz_fortress bash
```

Trong Docker:

```bash
source /opt/ros/humble/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

ros2 run hunav_agent_manager hunav_agent_manager \
  --ros-args \
  -p use_sim_time:=false \
  -p publish_tf:=false \
  -p publish_sfm_forces:=false
```

Kết quả cần có:

```text
Successfully retrieved parameters from hunav_loader
BT nodes registered
```

Giữ Terminal 3 chạy.

---

## Terminal 4 — Gazebo bridge và pose-applier

Chạy trên host:

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena

source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
export GZ_PARTITION=school_hunav

./run_school_gz_bridge.sh
```

Terminal này chạy đồng thời:

- Gazebo/ROS bridge.
- HuNav pose-applier trên Jazzy.
- Bridge `/cmd_vel`.
- Bridge ảnh camera.

Các dòng mong đợi:

```text
HuNav target topic: /school_hunav/target_poses
Gazebo set_pose service: /world/school_arena/set_pose
Creating GZ->ROS Bridge: [/world/school_arena/dynamic_pose/info
Creating ROS->GZ service bridge: [/world/school_arena/set_pose
Creating ROS->GZ Bridge: [/cmd_vel
Creating GZ->ROS Bridge: [/robot/camera/image_raw
```

Giữ Terminal 4 chạy.

---

## Terminal 5 — Spawn TurtleBot3 có camera

Chạy trên host:

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./spawn_school_robot_camera.sh
```

Kết quả:

```text
Entity creation successful.
```

Sau khi thành công, Terminal 5 có thể đóng.

Mỗi lần restart Gazebo, bạn phải chạy lại bước spawn robot này.

---

## Terminal 6 — HuNav School Bridge trong Docker

Chỉ chạy sau khi robot đã được spawn thành công.

```bash
docker exec -it hunavsim_gz_fortress bash
```

Trong Docker:

```bash
source /opt/ros/humble/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

python3 /tmp/school_arena/hunav_gz8_school_bridge.py \
  --scenario \
  /home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/school_agents.yaml \
  --sdf \
  /tmp/school_arena/school_arena_hunav.sdf \
  --ros-args \
  -p world_name:=school_arena \
  -p robot_name:=robot \
  -p update_hz:=10.0 \
  -p human_z:=0.0 \
  -p route_half_width:=0.45
```

Kết quả cần có:

```text
Gazebo pose topic: /world/school_arena/dynamic_pose/info
Target pose topic: /school_hunav/target_poses
HuNav service: /compute_agents
Configured humans: student_head_on_A, ...
Loaded 122 static collision AABBs from SDF
```

Có thể xuất hiện một lần:

```text
Gazebo TF messages have empty child_frame_id; using validated pose slots
```

Cảnh báo này bình thường.

Không được xuất hiện liên tục:

```text
Waiting live Gazebo poses
```

Giữ Terminal 6 chạy.

---

## Terminal 7 — Camera robot và điều khiển trực tiếp

Chạy trên host:

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./run_robot_camera_teleop.sh
```

Một cửa sổ camera sẽ xuất hiện. Click vào cửa sổ rồi sử dụng:

```text
W       đi thẳng
S       đi lùi
A       quay trái
D       quay phải
X/K     dừng
Q/Esc   dừng và đóng cửa sổ
```

Không chạy thêm `run_school_teleop.sh`, vì hai chương trình sẽ cùng điều khiển `/cmd_vel`.

## Thứ tự tổng quát

```text
Terminal 1: Gazebo
Terminal 2: HuNav Loader
Terminal 3: HuNav Manager
Terminal 4: ROS/Gazebo bridge + pose-applier
Terminal 5: Spawn robot camera
Terminal 6: HuNav School Bridge
Terminal 7: Camera view + keyboard control
```

Khi tắt hệ thống, nên đóng theo thứ tự ngược lại: Terminal 7 → 6 → 4 → 3 → 2 → 1.
