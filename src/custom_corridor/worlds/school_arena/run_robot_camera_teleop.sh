#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source "${HOME}/nav_ws/install/setup.bash"
set -u

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0
export GZ_PARTITION=school_hunav

# Gazebo uses Space as a global pause shortcut. Recover a world that was
# accidentally paused before the camera window starts.
gz service -s /world/school_arena/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req 'pause: false' >/dev/null 2>&1 || true

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec python3 "$SCRIPT_DIR/robot_camera_teleop.py"
