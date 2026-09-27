#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 generated/ep_xxxxxx [ros2 launch options]" >&2
  exit 2
fi
EPISODE="$(realpath "$1")"
shift
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO="$(dirname "$HERE")"
WS_ROOT="$(cd "$HERE/../../../../../.." && pwd)"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WS_ROOT/install/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 Jazzy setup not found: $ROS_SETUP" >&2
  exit 1
fi
if [[ ! -f "$WS_SETUP" ]]; then
  echo "Workspace setup not found: $WS_SETUP" >&2
  echo "Build custom_corridor first:" >&2
  echo "  cd $WS_ROOT" >&2
  echo "  colcon build --symlink-install --packages-select custom_corridor" >&2
  exit 1
fi
set +u
source "$ROS_SETUP"
source "$WS_SETUP"
set -u
if ! ros2 pkg prefix custom_corridor >/dev/null 2>&1; then
  echo "custom_corridor is not built/available." >&2
  echo "Run:" >&2
  echo "  cd $WS_ROOT" >&2
  echo "  colcon build --symlink-install --packages-select custom_corridor" >&2
  exit 1
fi
read -r ROBOT_X ROBOT_Y ROBOT_YAW < <(python3 - "$EPISODE/metadata.yaml" <<'PY'
import sys
import yaml
with open(sys.argv[1], encoding='utf-8') as stream:
    pose = yaml.safe_load(stream)['robot']['start_pose']
print(pose['x'], pose['y'], pose['heading'])
PY
)
exec ros2 launch "$DEMO/school_hunav_demo.launch.py" \
  "episode_world:=$EPISODE/school_floor.world" \
  "nav_params_file:=$EPISODE/nav2_school.yaml" \
  "robot_x:=$ROBOT_X" "robot_y:=$ROBOT_Y" "robot_yaw:=$ROBOT_YAW" "$@"
