# School Arena — Gazebo Sim 8 + HuNavSim + TurtleBot3

Hướng dẫn đầy đủ từ cài Docker, dựng HuNavSim, chuẩn bị ROS 2 đến chạy School
Arena với 8 người đi bộ, TurtleBot3 Burger, camera và điều khiển bàn phím.

> Các lệnh giả sử repository nằm tại `~/nav_ws`. Một số script dùng trực tiếp
> đường dẫn này, vì vậy nên giữ nguyên vị trí workspace.

## 1. Kiến trúc

```text
Docker: ROS 2 Humble + HuNavSim
  hunav_loader -> hunav_agent_manager -> hunav_gz8_school_bridge.py
                              │ Fast DDS, ROS_DOMAIN_ID=0
                              ▼
Host: ROS 2 Jazzy + ros_gz_bridge + Gazebo Sim 8
  school_arena_hunav.sdf + pose-applier + TurtleBot3 + camera
```

HuNavSim tính hành vi người đi bộ. Gazebo trên host hiển thị thế giới và áp
pose do HuNav trả về. Container không render Gazebo.

## 2. Yêu cầu

- Ubuntu 24.04 LTS.
- ROS 2 Jazzy và Gazebo Sim 8 (Harmonic) trên host.
- Docker Engine, Git và Internet trong lần cài đầu.
- Khoảng 20 GB dung lượng trống.
- Desktop X11 hoặc Wayland có XWayland.

## 3. Cài Docker Engine

Nếu `docker run --rm hello-world` đã chạy được, bỏ qua mục này.

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

. /etc/os-release
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu ${UBUNTU_CODENAME:-$VERSION_CODENAME} stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin

sudo usermod -aG docker "$USER"
newgrp docker
docker run --rm hello-world
```

Nếu vẫn gặp `permission denied`, đăng xuất Ubuntu rồi đăng nhập lại. Tham khảo
[Docker Engine for Ubuntu](https://docs.docker.com/engine/install/ubuntu/).

## 4. Cài ROS 2 Jazzy và dependency host

Cài ROS 2 Jazzy theo
[hướng dẫn chính thức](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html),
sau đó:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-rmw-fastrtps-cpp \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-description \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-cv-bridge \
  python3-colcon-common-extensions \
  python3-opencv \
  python3-yaml

source /opt/ros/jazzy/setup.bash
gz sim --versions
```

Gazebo phải trả về phiên bản `8.x`.

## 5. Tải và build DynamuhSim

```bash
git clone --recurse-submodules \
  https://github.com/CuongBien/DynamuhSim.git ~/nav_ws
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

cd ~/nav_ws/src/custom_corridor/worlds/school_arena
chmod +x run_*.sh spawn_*.sh setup_*.sh stop_*.sh
```

Nếu repository đã có sẵn, chỉ cần build lại workspace.

## 6. Cài HuNavSim 2.0 trong Docker

HuNavSim upstream dùng ROS 2 Humble, nên dự án chạy HuNav trong container và
giữ host ở Jazzy.

### 6.1. Build image và workspace

```bash
cd ~
git clone --branch v2.0 \
  https://github.com/robotics-upo/hunavsim_containers.git
cd ~/hunavsim_containers
chmod +x install.sh
./install.sh
```

Chọn:

```text
2. HuNavSim 2.0 + Gazebo Fortress + ROS 2 Humble
```

Script tạo image `gz_fortress_hunavsim`, workspace
`gazebo_fortress/hunav_gz_fortress_ws`, rồi clone HuNavSim và wrapper.
Kiểm tra:

```bash
docker image inspect gz_fortress_hunavsim >/dev/null && echo "HuNav image: OK"
test -d ~/hunavsim_containers/gazebo_fortress/hunav_gz_fortress_ws/src \
  && echo "HuNav workspace: OK"
```

Nguồn:
[hunavsim_containers v2.0](https://github.com/robotics-upo/hunavsim_containers/tree/v2.0).

### 6.2. Tạo container cố định

School Arena render Gazebo trên host nên không cần NVIDIA Container Toolkit:

```bash
cd ~/hunavsim_containers/gazebo_fortress

docker run -it \
  --name hunavsim_gz_fortress \
  --env="DISPLAY=$DISPLAY" \
  --env="LIBGL_ALWAYS_SOFTWARE=1" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network=host \
  --privileged \
  --mount type=bind,source="$PWD/hunav_gz_fortress_ws",target=/home/hunav_gz_fortress_ws \
  gz_fortress_hunavsim bash
```

Entrypoint build workspace lần đầu. Khi menu HuNavSim xuất hiện, giữ terminal
mở và không chọn scenario có sẵn. Nếu container đã tồn tại, dùng:

```bash
docker start hunavsim_gz_fortress
docker ps --filter name=hunavsim_gz_fortress
```

### 6.3. Cài School Arena vào container

Container phải ở trạng thái `Up`:

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./stop_school_hunav_container.sh hunavsim_gz_fortress
./setup_school_hunav_container.sh hunavsim_gz_fortress
```

Kết quả:

```text
Installed school HuNav scenario, BT files and runtime bridge into hunavsim_gz_fortress
```

Chạy lại bước này sau khi sửa YAML, Behavior Tree, bridge Python, SDF hoặc tạo
lại container.

## 7. Chạy toàn bộ mô phỏng

Dừng các phiên Gazebo/HuNav cũ. Không chạy `school_arena.sdf`: đây là preview
legacy có `TrajectoryFollower`. Pipeline này dùng `school_arena_hunav.sdf`.

### Terminal 1 — Gazebo trên host

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
export GZ_PARTITION=school_hunav
./run_school_arena.sh
```

Chờ Gazebo tải xong và đang chạy.

### Terminal 2 — HuNav Loader trong Docker

```bash
docker exec -it hunavsim_gz_fortress bash
```

Trong container:

```bash
source /opt/ros/humble/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

ros2 run hunav_agent_manager hunav_loader \
  --ros-args --params-file \
  /home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/school_agents.yaml
```

Cần thấy `Number of agents: 8` và `GetParameters service created`.

### Terminal 3 — HuNav Agent Manager trong Docker

```bash
docker exec -it hunavsim_gz_fortress bash
```

Trong container:

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

Cần thấy `Successfully retrieved parameters from hunav_loader` và
`BT nodes registered`.

### Terminal 4 — Gazebo/ROS bridge và pose-applier trên host

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

Terminal này bridge pose động, service `set_pose`, `/cmd_vel`, ảnh camera
và chạy pose-applier. Các dòng mong đợi:

```text
HuNav target topic: /school_hunav/target_poses
Gazebo set_pose service: /world/school_arena/set_pose
Creating GZ->ROS Bridge: [/world/school_arena/dynamic_pose/info
Creating ROS->GZ Bridge: [/cmd_vel
Creating GZ->ROS Bridge: [/robot/camera/image_raw
```

### Terminal 5 — Spawn TurtleBot3 có camera

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./spawn_school_robot_camera.sh
```

Cần thấy `Entity creation successful.`. Mỗi lần restart Gazebo phải spawn lại.

### Terminal 6 — School HuNav adapter trong Docker

Chỉ chạy sau khi robot đã spawn:

```bash
docker exec -it hunavsim_gz_fortress bash
```

Trong container:

```bash
source /opt/ros/humble/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

python3 /tmp/school_arena/hunav_gz8_school_bridge.py \
  --scenario /home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/school_agents.yaml \
  --sdf /tmp/school_arena/school_arena_hunav.sdf \
  --ros-args \
  -p world_name:=school_arena \
  -p robot_name:=robot \
  -p update_hz:=10.0 \
  -p human_z:=0.0 \
  -p route_half_width:=0.45
```

Kết quả mong đợi:

```text
Gazebo pose topic: /world/school_arena/dynamic_pose/info
Target pose topic: /school_hunav/target_poses
HuNav service: /compute_agents
Configured humans: student_head_on_A, ...
Loaded 122 static collision AABBs from SDF
```

Cảnh báo `Gazebo TF messages have empty child_frame_id; using validated pose
slots` xuất hiện một lần là bình thường. `Waiting live Gazebo poses` không
được lặp liên tục.

### Terminal 7 — Camera và điều khiển robot

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./run_robot_camera_teleop.sh
```

Click vào cửa sổ camera:

| Phím | Chức năng |
|---|---|
| `W` / `S` | Tiến / lùi |
| `A` / `D` | Quay trái / phải |
| `X` hoặc `K` | Dừng |
| `Q` hoặc `Esc` | Dừng và đóng |

Không dùng `Space` vì đây là phím pause Gazebo. Không chạy đồng thời
`run_school_teleop.sh`, vì hai chương trình cùng publish `/cmd_vel`.

Teleop không có camera (dùng thay Terminal 7):

```bash
./run_school_teleop.sh
```

Phím mặc định: `i` tiến, `,` lùi, `j/l` quay, `k` dừng.

## 8. Kiểm tra hệ thống

```bash
# Process HuNav
docker exec hunavsim_gz_fortress bash -lc \
  "pgrep -af 'hunav_loader|hunav_agent_manager|hunav_gz8_school_bridge'"

# Topic trên host
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
ros2 topic list | grep -E \
  'dynamic_pose|school_hunav/target_poses|robot/camera/image_raw|cmd_vel'
```

Kiểm tra tần số từng topic, dùng `Ctrl+C` giữa các lệnh:

```bash
ros2 topic hz /world/school_arena/dynamic_pose/info
ros2 topic hz /school_hunav/target_poses
ros2 topic hz /robot/camera/image_raw
```

Trong container, `ros2 service list | grep compute_agents` phải thấy
`/compute_agents`.

## 9. Dừng và chạy lại

Dừng bằng `Ctrl+C` theo thứ tự: Terminal 7 → 6 → 4 → 3 → 2 → 1.

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./stop_school_hunav_container.sh hunavsim_gz_fortress
docker stop hunavsim_gz_fortress
```

Lần chạy sau:

```bash
docker start hunavsim_gz_fortress
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./stop_school_hunav_container.sh hunavsim_gz_fortress
./setup_school_hunav_container.sh hunavsim_gz_fortress
```

Sau đó chạy lại Terminal 1–7.

## 10. Xử lý lỗi thường gặp

### Container name đã tồn tại

```bash
docker ps -a --filter name=hunavsim_gz_fortress
docker start hunavsim_gz_fortress
```

### Container thoát hoặc workspace chưa build

```bash
docker logs hunavsim_gz_fortress
docker start -ai hunavsim_gz_fortress
```

Build lại nếu cần:

```bash
docker exec -it hunavsim_gz_fortress bash
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
cd /home/hunav_gz_fortress_ws
colcon build --symlink-install
```

### Không tìm thấy scenario hoặc Behavior Tree

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./setup_school_hunav_container.sh hunavsim_gz_fortress
```

### Adapter lặp `Waiting live Gazebo poses`

Kiểm tra:

1. Terminal 1 và 4 cùng dùng `GZ_PARTITION=school_hunav`.
2. Host và container cùng dùng `ROS_DOMAIN_ID=0`, Fast DDS, UDPv4.
3. Terminal 1 dùng `school_arena_hunav.sdf`.
4. Terminal 4 vẫn chạy và không có phiên School Arena cũ.

```bash
GZ_PARTITION=school_hunav gz topic -l | grep school_arena
```

### Không có `/compute_agents`

Loader phải chạy trước manager. Dọn process cũ rồi chạy lại Terminal 2 và 3:

```bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena
./stop_school_hunav_container.sh hunavsim_gz_fortress
```

### Robot không spawn

```bash
GZ_PARTITION=school_hunav gz service -l | grep /world/school_arena/create
```

Chờ Gazebo tải xong. Không spawn model `robot` lần hai trong cùng phiên.

### Camera đen hoặc người không di chuyển

- Camera: kiểm tra Terminal 4, robot đã spawn và
  `ros2 topic hz /robot/camera/image_raw`.
- Người: loader có 8 agents; manager có `/compute_agents`; adapter không còn
  chờ pose; `/school_hunav/target_poses` có dữ liệu.
- Chỉ chạy một world `school_arena_hunav.sdf`, một adapter và một teleop.
- Sau khi sửa SDF phải restart Gazebo.

## 11. File chính

| File | Vai trò |
|---|---|
| `school_arena_hunav.sdf` | World chính do HuNav điều khiển |
| `school_arena.sdf` | Preview legacy, không dùng với HuNav |
| `school_agents.yaml` | 8 agents, start pose, goal và hành vi |
| `school_hunav_bt/` | Behavior Tree của người đi bộ |
| `school_gz8_bridge.yaml` | Cấu hình topic/service bridge |
| `hunav_gz8_school_bridge.py` | Adapter Gazebo 8 ↔ HuNavSim |
| `hunav_gz8_pose_applier.py` | Áp target pose vào Gazebo |
| `turtlebot3_burger_camera.sdf` | Robot và camera |
| `robot_camera_teleop.py` | Camera viewer và teleop WASD |

## 12. Thứ tự chạy rút gọn

```text
0. Docker: start container + setup School scenario
1. Host:   run_school_arena.sh
2. Docker: hunav_loader
3. Docker: hunav_agent_manager
4. Host:   run_school_gz_bridge.sh
5. Host:   spawn_school_robot_camera.sh
6. Docker: hunav_gz8_school_bridge.py
7. Host:   run_robot_camera_teleop.sh
```

Tham khảo:

- [HuNavSim v2.0](https://github.com/robotics-upo/hunav_sim/tree/v2.0)
- [HuNavSim containers v2.0](https://github.com/robotics-upo/hunavsim_containers/tree/v2.0)
- [HuNav Gazebo Fortress wrapper](https://github.com/robotics-upo/hunav_gazebo_fortress_wrapper/tree/v2.0)
