#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source "${HOME}/nav_ws/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
export GZ_PARTITION=school_hunav
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec ros2 run ros_gz_sim create \
  -world school_arena \
  -name robot \
  -file "$SCRIPT_DIR/turtlebot3_burger_camera.sdf" \
  -x -13.5 -y -8.0 -z 0.01 -Y 0.0
