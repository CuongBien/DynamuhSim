MULTI-HUMAN SCENARIO SYSTEM — INSTALL / TEST
=============================================

1) Copy source
--------------

mkdir -p ~/nav_ws/src/custom_corridor_plugins/src

cp multi_human_scenario_system.cpp \
  ~/nav_ws/src/custom_corridor_plugins/src/

2) Dependencies
---------------

sudo apt update
sudo apt install -y \
  libgz-sim8-dev \
  libgz-transport13-dev \
  nlohmann-json3-dev

Gazebo Harmonic uses gz-sim8 + gz-transport13.

3) Edit custom_corridor_plugins/CMakeLists.txt
---------------------------------------------

Append the contents of CMakeLists_multi_human_snippet.txt.

If your CMakeLists already has:

  find_package(gz-sim8 REQUIRED)

do NOT duplicate that line; keep only one.

If an install(TARGETS ...) section already exists, you can instead add
multi_human_scenario_system to that existing install list.

4) Add world plugin
-------------------

Insert arena_dataset_plugin_block.xml inside:

  <world name="arena_dataset">
      ...
      [PLUGIN HERE]
      ...
  </world>

Keep the four actors with:

  <auto_start>false</auto_start>

5) Build
--------

cd ~/nav_ws
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install \
  --packages-select custom_corridor_plugins custom_corridor

source install/setup.bash

Check library:

find ~/nav_ws/install -name \
  'libmulti_human_scenario_system.so' -print

6) Start world
--------------

export ROS_DOMAIN_ID=42

ros2 launch custom_corridor corridor_tb3.launch.py \
  width:=dataset \
  obstacle:=none

Gazebo terminal should print something similar to:

  [MultiHumanScenarioSystem] Configured.
  ...
  Bound H1: human_01_visual <-> human_01_proxy
  ...
  Bound H4: human_04_visual <-> human_04_proxy

7) Verify plugin transport topics
---------------------------------

gz topic -l | grep multi_human

Expected after publishers/subscribers are discovered:

  /multi_human/scenario
  /multi_human/command

8) Send the existing JSON scenario once
---------------------------------------

SCENARIO="$HOME/nav_ws/experiments/trajectory_risk_v0/scenarios/scenario_001.json"

PAYLOAD="$(python3 -c 'import json,sys; print(json.dumps(open(sys.argv[1]).read()))' "$SCENARIO")"

gz topic -t /multi_human/scenario \
  -m gz.msgs.StringMsg \
  -p "data: ${PAYLOAD}"

Important:
- Plugin loads + resets the scenario.
- It does NOT start moving yet because auto_start_on_scenario=false.

Gazebo terminal should print:

  Loaded episode 1 with 4 active humans. READY

9) Start
--------

gz topic -t /multi_human/command \
  -m gz.msgs.StringMsg \
  -p 'data: "start"'

H1 should move head-on.
H2 should cross.
H3 should follow waypoints.
H4 should remain stationary.

10) Deterministic reset test
----------------------------

gz topic -t /multi_human/command \
  -m gz.msgs.StringMsg \
  -p 'data: "reset"'

All four humans must return to the exact initial episode poses and stop.

Then:

gz topic -t /multi_human/command \
  -m gz.msgs.StringMsg \
  -p 'data: "reset_start"'

This is the command the C0..C4 collector should eventually send before every
candidate rollout.

11) Hide all
------------

gz topic -t /multi_human/command \
  -m gz.msgs.StringMsg \
  -p 'data: "hide_all"'

12) Gate-B acceptance
---------------------

PASS only if all four hold:

[ ] 4 visible humans appear at different positions.
[ ] H1 scripted, H2 scripted, H3 autonomous, H4 stationary behave correctly.
[ ] Each actor visual remains aligned with its physical proxy.
[ ] LiDAR detects the proxy when a human crosses close to the robot.

Do not regenerate the full risk dataset before this Gate passes.
