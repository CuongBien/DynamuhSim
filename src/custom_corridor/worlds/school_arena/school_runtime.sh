#!/usr/bin/env bash
# Shared setup and process ownership for the School Arena launchers.
school_ros() {
  source /opt/ros/jazzy/setup.bash
  if [[ -n "${1:-}" ]]; then source "$1"; fi
  set -u
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=0
}

# The installer places both helpers side by side inside the HuNav container.
if [[ -f "$(dirname "${BASH_SOURCE[0]}")/process_runtime.sh" ]]; then
  source "$(dirname "${BASH_SOURCE[0]}")/process_runtime.sh"
else
  source "$(dirname "${BASH_SOURCE[0]}")/../../../../tools/process_runtime.sh"
fi
