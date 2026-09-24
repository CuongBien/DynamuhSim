School Arena + Gazebo 8 + HuNavSim integration

Files:
- school_arena_hunav.sdf: HuNav world with visual-only, ground-anchored
  student_* proxies. Their model origin is z=0 and Gazebo physics cannot lift
  or tilt them.
- school_agents.yaml: HuNavSim scenario using the original start poses and route goals.
- school_hunav_bt/: interactive school BehaviorTree. RegularNav handles route
  following and social forces; the native HuNav AvoidRobot node yields briefly
  when the robot enters 0.95 m of personal space.
- school_gz8_bridge.yaml: ros_gz_bridge config for live Gazebo dynamic poses + set_pose service.
- hunav_gz8_school_bridge.py: live adapter. It DOES NOT compute navigation itself. It mirrors the official wrapper responsibilities:
  * reads current human/robot poses from Gazebo 8
  * derives current velocity
  * extracts static collision AABBs from school_arena_hunav.sdf
  * fills closest_obs with the single nearest point, matching the official
    HuNav Fortress wrapper
  * sends current_agents + robot to HuNavSim /compute_agents
  * constrains each route to the walkable corridor and prevents wall crossing
  * publishes HuNavSim targets to the Jazzy-side pose applier

Important:
- Do not run the legacy school_arena.sdf at the same time. Its
  gz-sim-trajectory-follower-system controllers will fight HuNavSim.
- HuNavSim behavior is computed by hunav_agent_manager / lightsfm / BT source; the bridge only moves state between Gazebo and HuNavSim.
- Robot model name must match the actual Gazebo model. Default is "robot"; override with ROS parameter robot_name.
- Use Fast DDS in every ROS terminal. Do not mix rmw_cyclonedds_cpp between
  the Jazzy host and the Humble container.

Clean start order
=================

Stop the previous Gazebo, loader, manager, ros_gz_bridge and Python bridge
with Ctrl+C first. A restart is required after changing the SDF because Gazebo
only reads kinematic/gravity settings while loading the world.

If a previous Docker terminal was closed without Ctrl+C, clean its orphaned
processes first (this also releases the Groot ZMQ port 5555):

  ./stop_school_hunav_container.sh hunavsim_gz_fortress

Install/update the container files once:

  cd ~/nav_ws/src/custom_corridor/worlds/school_arena
  ./setup_school_hunav_container.sh hunavsim_gz_fortress

For every host ROS terminal use:

  source /opt/ros/jazzy/setup.bash
  source ~/nav_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export ROS_DOMAIN_ID=0
  export GZ_PARTITION=school_hunav

For every Docker ROS terminal use:

  source /opt/ros/humble/setup.bash
  source /home/hunav_gz_fortress_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  export ROS_DOMAIN_ID=0

Start these long-running commands in separate terminals:

1. Host - Gazebo:

  cd ~/nav_ws/src/custom_corridor/worlds/school_arena
  ./run_school_arena.sh

2. Docker - loader:

  ros2 run hunav_agent_manager hunav_loader --ros-args --params-file \
    /home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/school_agents.yaml

3. Docker - manager:

  ros2 run hunav_agent_manager hunav_agent_manager --ros-args \
    -p use_sim_time:=false -p publish_tf:=false -p publish_sfm_forces:=false

4. Host - Gazebo/ROS bridge + local pose applier:

  cd ~/nav_ws/src/custom_corridor/worlds/school_arena
  ./run_school_gz_bridge.sh

5. Host - spawn robot (this command exits after success):

  cd ~/nav_ws/src/custom_corridor/worlds/school_arena
  ./spawn_school_robot_camera.sh

6. Docker - school HuNav adapter:

  python3 /tmp/school_arena/hunav_gz8_school_bridge.py \
    --scenario /home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/school_agents.yaml \
    --sdf /tmp/school_arena/school_arena_hunav.sdf --ros-args \
    -p world_name:=school_arena -p robot_name:=robot -p update_hz:=10.0 \
    -p human_z:=0.0 -p route_half_width:=0.45

7. Host - keyboard teleoperation (optional):

  cd ~/nav_ws/src/custom_corridor/worlds/school_arena
  ./run_school_teleop.sh

Keep this terminal focused while driving. The standard keys are i/j/l/, for
forward/left/right/backward and k to stop (the backward key is comma `,`).

8. Host - camera view with direct keyboard control (recommended instead of 7):

  cd ~/nav_ws/src/custom_corridor/worlds/school_arena
  ./run_robot_camera_teleop.sh

Click the camera window, then use W/S to move, A/D to turn, X or K to stop and
Q or Escape to close it. Do not use Space: it is Gazebo's global pause key.
