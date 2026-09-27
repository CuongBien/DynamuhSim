# Scenario Generator V1 — lệnh chạy và kiểm thử

Scenario Generator dùng zones.yaml, human_navigation_graph.yaml, map.pgm và
world của Demo 1. Family hỗ trợ: empty, head_on, same_direction, crossing,
exit_room. Generator đặt episode vào generated/ep_xxxxxx; ID tăng theo các
thư mục đã tồn tại. Hãy dùng ID in ra sau lệnh generate, không giả định luôn
là ep_000001.

## 1. Chuẩn bị trên host

Build workspace nếu chưa build:

~~~bash
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena/demo_1/scenario_generator
~~~

Ở mọi terminal host chạy lệnh ROS 2, source môi trường:

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
~~~

Để dùng HuNav, khởi động container Demo 1 đã cấu hình:

~~~bash
docker start hunavsim_gz_fortress
~~~

## 2. Sinh episode

Chạy tại thư mục scenario_generator:

~~~bash
python3 scenario_generator.py --scenario empty --seed 100 --density low
python3 scenario_generator.py --scenario head_on --seed 42 --density low
python3 scenario_generator.py --scenario same_direction --seed 43 --density low
python3 scenario_generator.py --scenario crossing --seed 44 --density medium
python3 scenario_generator.py --scenario exit_room --seed 45 --density high
~~~

Sinh batch hoặc xem toàn bộ tùy chọn:

~~~bash
python3 scenario_generator.py --scenario head_on --num-episodes 20 --seed-start 100
python3 scenario_generator.py --scenario all --num-episodes 20 --seed-start 200
python3 scenario_generator.py --help
~~~

all luân phiên năm family. Nếu bỏ --seed và --seed-start, generator tự
chọn seed và in seed đã dùng. Có thể dùng --config configs/generator.yaml
hoặc --output <thư_mục>.

Mỗi episode chứa metadata.yaml (ID, seed, robot start pose), scenario.yaml
(route, goal, behavior, delay), humans.yaml (ROS 2 parameters cho HuNav),
school_floor.world, nav2_school.yaml và Behavior Tree XML cho từng human.
start_delay được school bridge xử lý. S00 empty không cần chạy các node HuNav.

## 3. Kiểm thử tĩnh

~~~bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena/demo_1/scenario_generator
python3 -m unittest discover -s tests -v
python3 ../validate_demo.py
~~~

Kiểm tra world và dữ liệu YAML của một episode. Thay ID bằng ID vừa sinh:

~~~bash
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
EP_ID=ep_000001
gz sdf -p "generated/$EP_ID/school_floor.world" >/dev/null
python3 - "generated/$EP_ID" <<'PY'
from pathlib import Path
import sys
import yaml

episode = Path(sys.argv[1])
metadata = yaml.safe_load((episode / "metadata.yaml").read_text())
scenario = yaml.safe_load((episode / "scenario.yaml").read_text())
params = yaml.safe_load((episode / "humans.yaml").read_text())["hunav_loader"]["ros__parameters"]
assert metadata["seed"] == scenario["seed"]
assert metadata["scenario_family"] == scenario["scenario_family"]
assert len(params.get("agents", [])) == metadata["humans"]["total"]
print("PASS", metadata["episode_id"], metadata["scenario_family"],
      metadata["humans"]["total"], "humans")
PY
~~~

Test deterministic với cùng seed/family/density/config (cmp im lặng và
exit code 0 nghĩa là PASS):

~~~bash
VERIFY_A="$(mktemp -d)"
VERIFY_B="$(mktemp -d)"
python3 scenario_generator.py --scenario head_on --seed 42 --density low --output "$VERIFY_A"
python3 scenario_generator.py --scenario head_on --seed 42 --density low --output "$VERIFY_B"
cmp "$VERIFY_A/ep_000001/scenario.yaml" "$VERIFY_B/ep_000001/scenario.yaml"
cmp "$VERIFY_A/ep_000001/humans.yaml" "$VERIFY_B/ep_000001/humans.yaml"
rm -r "$VERIFY_A" "$VERIFY_B"
~~~

ID trong metadata phụ thuộc thư mục output; nội dung scenario và HuNav
được tái lập với cùng input.

## 4. Cài episode vào Docker

Trên host, tại thư mục scenario_generator. Ví dụ dùng ep_000002 (head_on);
thay bằng ID human episode vừa sinh:

~~~bash
EP_ID=ep_000002
./install_episode.sh "generated/$EP_ID" hunavsim_gz_fortress
docker exec hunavsim_gz_fortress bash -lc   "head -80 /home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/$EP_ID.yaml"
~~~

Script copy humans.yaml thành scenarios/ep_xxxxxx.yaml, đồng thời copy world,
Behavior Tree và source bridge vào Docker. Chạy lại sau khi sinh/sửa episode
hoặc bridge. scenario.yaml không được dùng làm HuNav params file.

## 5. Chạy Gazebo, Nav2 và HuNav

Terminal host 1 — dừng các launch Demo 1 khác trước khi chạy:

~~~bash
cd ~/nav_ws/src/custom_corridor/worlds/school_arena/demo_1/scenario_generator
source /opt/ros/jazzy/setup.bash
source ~/nav_ws/install/setup.bash
EP_ID=ep_000002
./launch_episode.sh "generated/$EP_ID" gui:=true rviz:=true
~~~

Launch lấy robot start pose từ metadata.yaml, world và AMCL config từ episode.
Chạy không GUI bằng gui:=false rviz:=false. Đợi Nav2 active trước khi gửi
goal. Không chạy school_demo.launch.py hoặc school_hunav_demo.launch.py khác
cùng lúc.

Với bốn family có human, mở ba terminal Docker riêng. Trong mỗi terminal:

~~~bash
docker exec -it hunavsim_gz_fortress bash
source /opt/ros/humble/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
EP_ID=ep_000002
~~~

Đặt EP_ID giống nhau ở cả ba terminal và khớp với episode trên host.

Docker terminal A — loader:

~~~bash
ros2 run hunav_agent_manager hunav_loader --ros-args --params-file   "/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/$EP_ID.yaml"
~~~

Docker terminal B — manager:

~~~bash
ros2 run hunav_agent_manager hunav_agent_manager --ros-args   -p use_sim_time:=false -p publish_tf:=false -p publish_sfm_forces:=false
~~~

Docker terminal C — school bridge:

~~~bash
python3 /tmp/school_arena/hunav_gz8_school_bridge.py   --scenario "/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/$EP_ID.yaml"   --sdf "/tmp/school_arena/$EP_ID.world"   --ros-args   -p world_name:=school_arena -p robot_name:=robot   -p update_hz:=10.0 -p human_z:=0.0 -p route_half_width:=0.45
~~~

Với S00 empty, chỉ cần terminal host 1 để chạy Gazebo/Nav2. YAML S00
vẫn có thể dùng để test parser riêng.

## 6. Kiểm tra runtime trên host

Mở terminal mới, source ROS 2 như mục 1 rồi chạy từng lệnh:

~~~bash
ros2 topic echo /demo/pose_sync_error --once
ros2 run tf2_ros tf2_echo map base_footprint
ros2 topic hz /odom
ros2 topic hz /scan
ros2 action list | grep navigate_to_pose
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 topic hz /school_hunav/target_poses
~~~

Lệnh hz và tf2_echo chạy liên tục: dừng bằng Ctrl+C. Bỏ qua topic HuNav
với S00. Mục tiêu pose sync error dưới 0,10 m sau khi localization ổn định.
Đối chiếu AMCL với robot.start_pose trong generated/$EP_ID/metadata.yaml:

~~~bash
ros2 param get /amcl initial_pose.x
ros2 param get /amcl initial_pose.y
ros2 param get /amcl initial_pose.yaw
~~~

Robot goal nằm ở robot.goal_node trong scenario.yaml. V1 chưa tự gửi Nav2
goal hoặc ghi dataset.

## 7. Chỉ kiểm tra parser trong Docker

Sau mục 4, vào Docker và source môi trường như mục 5. Chạy loader với
--params-file trỏ vào scenarios/$EP_ID.yaml; PASS khi không có
"Couldn't parse params file". Dừng bằng Ctrl+C sau khi kiểm tra.

Để kiểm tra bridge parser cho các episode đã cài vào Docker (ví dụ ID 1–5
tương ứng S00, head_on, same_direction, crossing, exit_room):

~~~bash
python3 - <<'PY'
import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "school_bridge", "/tmp/school_arena/hunav_gz8_school_bridge.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
base = Path("/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios")
for episode_id in ("ep_000001", "ep_000002", "ep_000003", "ep_000004", "ep_000005"):
    cfg = module.SchoolHuNavBridge._load_scenario(base / f"{episode_id}.yaml")
    assert isinstance(cfg["agents"], list)
    assert isinstance(cfg["global_goals"], dict)
    assert all(name in cfg for name in cfg["agents"])
    print("PASS", episode_id, len(cfg["agents"]), "agents")
PY
~~~

Các ID ví dụ phải khớp với family trên máy bạn. Parser PASS chỉ xác nhận
schema YAML; chuyển động và va chạm cần kiểm tra ở runtime mục 5–6.
