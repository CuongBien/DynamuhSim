#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
if [ -f "/home/cuongbien/DynamuhSim/install/setup.bash" ]; then
  source /home/cuongbien/DynamuhSim/install/setup.bash
elif [ -f "${HOME}/nav_ws/install/setup.bash" ]; then
  source "${HOME}/nav_ws/install/setup.bash"
fi

set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel -p stamped:=true
