MULTI-HUMAN SCENARIO SYSTEM — BUILD / MANUAL CHECK
=================================================

Run these commands from the DynamuhSim repository root in Ubuntu/WSL.
The active source is src/multi_human_scenario_system.cpp, already registered
in CMakeLists.txt. No source copy or CMake snippet is needed. The unused
_bu.cpp source was retired; Git revision 1f14851269402c0352edf4cc8eb34ee25d29c930
retains it.

1) Build the existing targets
-----------------------------

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select custom_corridor_plugins custom_corridor
source install/setup.bash

CMake builds the RVO2 static library and four system plugins: oscillating
obstacle, multi-human scenario, hospital yielding, and RVO2 human. Required
Gazebo development packages include gz-sim8, gz-plugin2, gz-transport13 and
nlohmann-json. See package.xml and CMakeLists.txt for the complete dependencies.

2) Select the implementation being checked
------------------------------------------

The default width:=dataset world now uses librvo2_human_system.so. To check
MultiHumanScenarioSystem specifically, select its retained legacy world:

export ROS_DOMAIN_ID=42
ros2 launch custom_corridor corridor_tb3.launch.py \
  width:=dataset obstacle:=none \
  custom_world:="$PWD/src/custom_corridor/worlds/arena_dataset/arena_dataset_bu.sdf"

That world configures four proxies, use_actor_visuals=false, and
auto_start_on_scenario=false. It is retained as a distinct runtime input;
the _bu suffix alone does not mean that this world is unused.

3) Load a scenario without starting motion
------------------------------------------

In another terminal, source the same ROS/workspace environment and set
ROS_DOMAIN_ID=42. Check transport topics:

gz topic -l | grep multi_human

Expected topics: /multi_human/scenario and /multi_human/command.

python3 experiments/trajectory_risk_v0/scripts/publish_gz_scenario.py \
  experiments/trajectory_risk_v0/scenarios/scenario_001.json --command reset

4) Check start, deterministic reset and hide
-------------------------------------------

gz topic -t /multi_human/command -m gz.msgs.StringMsg -p 'data: "start"'
gz topic -t /multi_human/command -m gz.msgs.StringMsg -p 'data: "reset"'
gz topic -t /multi_human/command -m gz.msgs.StringMsg -p 'data: "reset_start"'
gz topic -t /multi_human/command -m gz.msgs.StringMsg -p 'data: "hide_all"'

Verify scenario motion modes and that reset restores the same initial
positions before each C0..C4 rollout. Do not infer runtime acceptance merely
from a successful build or the presence of transport topics.

Gate-B remains required before regenerating the full risk dataset:
[ ] Four configured humans/proxies appear at the expected positions.
[ ] H1 scripted, H2 scripted, H3 autonomous and H4 stationary follow the scenario.
[ ] Actor visuals align with proxies in an actor-enabled test world.
[ ] LiDAR detects a proxy when a human crosses close to the robot.

The proxy-only legacy world does not establish actor alignment. The commands
above are manual checks, not a claim that a new physics/actor test has passed.
