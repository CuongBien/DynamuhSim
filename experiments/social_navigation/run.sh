#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../.." && pwd)"
TASK_MODE="${1:-warehouse}"
if (($#)); then shift; fi
case "$TASK_MODE" in
  warehouse)
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$TASK_ROOT/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    export GZ_PARTITION="${GZ_PARTITION:-dynamuhsim_social_warehouse}"
    exec ros2 launch "$TASK_DIR/warehouse/warehouse.launch.py" "$@"
    ;;
  *) echo 'Usage: bash run.sh warehouse [launch arguments]' >&2; exit 2;;
esac
