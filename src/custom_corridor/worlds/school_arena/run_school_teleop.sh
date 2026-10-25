#!/usr/bin/env bash
set -eo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/school_runtime.sh"
if [[ -f /home/cuongbien/DynamuhSim/install/setup.bash ]]; then
  school_ros /home/cuongbien/DynamuhSim/install/setup.bash
elif [[ -f "${HOME}/nav_ws/install/setup.bash" ]]; then
  school_ros "${HOME}/nav_ws/install/setup.bash"
else
  school_ros
fi

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel -p stamped:=true
