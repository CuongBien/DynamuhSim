#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source "${HOME}/nav_ws/install/setup.bash"

set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
