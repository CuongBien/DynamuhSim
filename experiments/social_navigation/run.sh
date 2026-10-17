#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../.." && pwd)"
TASK_MODE="${1:-warehouse}"
if (($#)); then shift; fi
case "$TASK_MODE" in
  warehouse|verify)
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$TASK_ROOT/install/setup.bash"
    set -u
    export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-73}"
    export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    export GZ_PARTITION="${GZ_PARTITION:-dynamuhsim_social_warehouse}"
    if [[ "$TASK_MODE" == warehouse ]]; then
      exec ros2 launch "$TASK_DIR/warehouse/warehouse.launch.py" "$@"
    else
      exec python3 "$TASK_DIR/verify_sim.py" "$@"
    fi
    ;;
  socnavgym)
    export SDL_AUDIODRIVER=dummy
    exec "$TASK_DIR/.venv-socnavgym/bin/python" "$TASK_DIR/run_socnavgym.py" "$@"
    ;;
  *) echo 'Usage: bash run.sh {warehouse|verify|socnavgym} [arguments]' >&2; exit 2;;
esac

