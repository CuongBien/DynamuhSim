# Hướng dẫn chạy Arena Dataset + Nav2 MPPI + Long Traverse

## 1. Mục tiêu

Tài liệu này mô tả quy trình chuẩn để chạy:

- Gazebo với môi trường `arena_dataset`
- Robot TurtleBot3
- Nav2 với controller MPPI
- Kịch bản `scenario_014`
- 24 human agents
- Thu dữ liệu Long Traverse bằng `run_long_traverse.sh`

> **World hiện tại**
>
> ```text
> /home/thaonhi/nav_ws/src/custom_corridor/worlds/arena_dataset/arena_dataset.sdf
> ```

Đường dẫn cũ:

```text
/home/thaonhi/nav_ws/src/custom_corridor/worlds/arena_dataset.sdf
```

không còn được sử dụng.

---

# 2. Hard Reset khi cần

Chỉ dùng khi Gazebo hoặc launch process cũ chưa thoát sạch, gây xung đột khi chạy lại.

```bash
pkill -f "corridor_tb3.launch.py"
pkill -f "gz sim"

sleep 2

ps aux | grep -E '[g]z sim|corridor_tb3'
```

Nếu lệnh cuối không còn trả về process Gazebo / `corridor_tb3`, hệ thống đã được dọn sạch.

---

# 3. Build lại môi trường và plugin

Dùng khi:

- sửa `arena_dataset`
- sửa launch/config trong `custom_corridor`
- sửa plugin human
- sửa `multi_human_scenario_system.cpp`
- build cũ bị lỗi hoặc không chắc workspace đang dùng binary mới

```bash
cd /home/thaonhi/nav_ws

source /opt/ros/jazzy/setup.bash

rm -rf \
  build/custom_corridor \
  build/custom_corridor_plugins \
  install/custom_corridor \
  install/custom_corridor_plugins

colcon build \
  --symlink-install \
  --packages-select custom_corridor custom_corridor_plugins

source install/setup.bash
```

Sau khi build lại plugin, cần **restart Gazebo** để Gazebo load lại file `.so` mới.

---

# 4. Biến môi trường chung

Mỗi terminal ROS 2 nên chạy các lệnh sau trước:

```bash
cd /home/thaonhi/nav_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Tất cả terminal phải sử dụng cùng:

```text
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

# 5. Terminal 1 — Chạy Gazebo + Arena Dataset

```bash
cd /home/thaonhi/nav_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch custom_corridor corridor_tb3.launch.py \
  width:=dataset \
  obstacle:=human \
  gui:=true
```

Giữ terminal này chạy.

World được sử dụng:

```text
/home/thaonhi/nav_ws/src/custom_corridor/worlds/arena_dataset/arena_dataset.sdf
```

## Kiểm tra nhanh human plugin

Mở terminal khác và chạy:

```bash
gz topic -l | grep multi_human
```

Hệ thống human đúng phải expose các topic liên quan đến:

```text
/multi_human/scenario
/multi_human/command
```

Nếu không thấy các topic này, cần kiểm tra lại plugin hoặc world trước khi chạy dataset.

---

# 6. Terminal 2 — Chạy Nav2 + MPPI

Mở terminal mới:

```bash
cd /home/thaonhi/nav_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 launch custom_corridor nav2_corridor.launch.py \
  controller:=mppi \
  map:=arena_obstacle
```

Chờ đến khi Nav2 hoàn tất lifecycle transition.

Dấu hiệu đúng:

```text
[lifecycle_manager_navigation]: Managed nodes are active
```

Localization cũng phải ở trạng thái active.

Giữ terminal này chạy.

---

# 7. Terminal 3 — Kiểm tra Nav2 và đặt Initial Pose

```bash
cd /home/thaonhi/nav_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 7.1. Kiểm tra node

```bash
ros2 node list | grep -E "map_server|amcl|planner_server|controller_server|bt_navigator"
```

Kỳ vọng:

```text
/amcl
/bt_navigator
/controller_server
/map_server
/planner_server
```

---

## 7.2. Kiểm tra NavigateToPose action

```bash
ros2 action list -t | grep navigate
```

Kỳ vọng:

```text
/navigate_to_pose [nav2_msgs/action/NavigateToPose]
```

---

## 7.3. Kiểm tra lifecycle

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
```

Tất cả node trên nên trả về:

```text
active [3]
```

---

## 7.4. Đặt Initial Pose

Robot spawn tại:

```text
x   = -13.0
y   = 0.0
yaw = 0.0
```

Publish `/initialpose`:

```bash
ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped \
  "{
    header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: -13.0, y: 0.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    }
  }"
```

---

## 7.5. Kiểm tra TF

```bash
ros2 run tf2_ros tf2_echo map base_footprint
```

Nếu transform được cập nhật liên tục thì TF đang hoạt động đúng.

Nhấn:

```text
Ctrl+C
```

để thoát `tf2_echo`.

---

# 8. Terminal 4 — Chạy Long Traverse và thu dataset

## 8.1. Scenarios (easy - medium)

``` bash
cd /home/thaonhi/nav_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd ~/nav_ws/experiments/trajectory_risk_v0
./run_v2_all.sh
```

## 8.2. Long Traverse (hard)

```bash
cd /home/thaonhi/nav_ws

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cd /home/thaonhi/nav_ws/experiments/trajectory_risk_v0

./run_long_traverse.sh
```

---

# 9. Pipeline Long Traverse

Luồng thực thi kỳ vọng:

```text
scenario_014
    ↓
24 human agents được khởi tạo
    ↓
human bắt đầu di chuyển
    ↓
rosbag bắt đầu ghi
    ↓
CSV logger bắt đầu ghi
    ↓
robot spawn tại (-13, 0)
    ↓
Nav2 + MPPI bắt đầu điều hướng
    ↓
robot di chuyển liên tục
    ↓
goal ≈ (4.3, 0)
    ↓
SUCCEEDED / FAILED / TIMEOUT
    ↓
dừng human agents
    ↓
lưu episode dataset
```

---

# 10. Kết quả thành công

Nav2 thành công khi log có:

```text
Result:
    error_code: 0

Goal finished with status: SUCCEEDED
```

Episode được lưu tại:

```text
/home/thaonhi/nav_ws/experiments/trajectory_risk_v0/data_long_traverse/episode_...
```

---

# 11. Kiểm tra khi robot chạy nhưng human không chạy

Nếu robot/Nav2 hoạt động nhưng human đứng yên hoặc không xuất hiện, kiểm tra theo thứ tự sau.

## 11.1. Kiểm tra plugin human đã được Gazebo load

```bash
gz topic -l | grep multi_human
```

Phải thấy:

```text
/multi_human/scenario
/multi_human/command
```

Nếu không thấy, lỗi nằm ở:

- plugin chưa được build/load
- `arena_dataset.sdf` chưa khai báo đúng plugin
- Gazebo đang dùng binary/plugin cũ

Sau khi build lại plugin phải restart Gazebo.

---

## 11.2. Kiểm tra scenario

```bash
cd /home/thaonhi/nav_ws/experiments/trajectory_risk_v0

python3 -m json.tool scenarios_v1/scenario_014.json > /dev/null \
  && echo "JSON OK"
```

---

## 11.3. Test publish scenario thủ công

```bash
cd /home/thaonhi/nav_ws/experiments/trajectory_risk_v0

python3 scripts/publish_gz_scenario.py \
  scenarios_v1/scenario_014.json
```

Sau đó gửi lệnh chạy:

```bash
gz topic \
  -t /multi_human/command \
  -m gz.msgs.StringMsg \
  -p 'data: "reset_start"'
```

Nếu human bắt đầu di chuyển thì:

- Gazebo đúng
- plugin đúng
- scenario đúng

Khi đó cần kiểm tra logic trong `run_long_traverse.sh`.

---

# 12. Lưu ý quan trọng

## Không chạy `collect_episode_v2.py`

Không sử dụng:

```bash
python3 collect_episode_v2.py
```

cho Long Traverse.

File này thuộc pipeline **Gate C**, dùng candidate trajectory ngắn và có logic reset robot sau từng candidate.

Long Traverse phải sử dụng:

```bash
./run_long_traverse.sh
```

---

# 13. Thứ tự khởi động chuẩn

Sau khi restart máy hoặc hard reset:

```text
1. Gazebo
   ↓
2. Nav2 + MPPI
   ↓
3. Kiểm tra Nav2 + publish /initialpose
   ↓
4. run_long_traverse.sh
```

Không nên đổi thứ tự này vì có thể gây:

- Nav2 chưa nhận simulation clock
- AMCL chưa sẵn sàng
- TF `map -> base_footprint` chưa tồn tại
- human plugin chưa sẵn sàng
- episode runner bắt đầu khi simulator chưa ổn định

---

# 14. Checklist trước khi thu dataset

Trước khi chạy `run_long_traverse.sh`, xác nhận:

- [ ] Gazebo đang chạy `arena_dataset`
- [ ] World path đúng: `worlds/arena_dataset/arena_dataset.sdf`
- [ ] `/multi_human/scenario` tồn tại
- [ ] `/multi_human/command` tồn tại
- [ ] `/map_server` active
- [ ] `/amcl` active
- [ ] `/planner_server` active
- [ ] `/controller_server` active
- [ ] `/bt_navigator` active
- [ ] `/navigate_to_pose` tồn tại
- [ ] `/initialpose` đã được publish
- [ ] TF `map -> base_footprint` hoạt động
- [ ] Không có Gazebo process cũ còn chạy
- [ ] Chạy Long Traverse bằng `run_long_traverse.sh`

---

# 15. Tóm tắt lệnh cần nhớ

## Environment

```bash
cd /home/thaonhi/nav_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Gazebo

```bash
ros2 launch custom_corridor corridor_tb3.launch.py \
  width:=dataset \
  obstacle:=human \
  gui:=true
```

## Nav2 MPPI

```bash
ros2 launch custom_corridor nav2_corridor.launch.py \
  controller:=mppi \
  map:=arena_obstacle
```

## Long Traverse

```bash
cd /home/thaonhi/nav_ws/experiments/trajectory_risk_v0
./run_long_traverse.sh
```

## Hard Reset

```bash
pkill -f "corridor_tb3.launch.py"
pkill -f "gz sim"
sleep 2
ps aux | grep -E '[g]z sim|corridor_tb3'
```
