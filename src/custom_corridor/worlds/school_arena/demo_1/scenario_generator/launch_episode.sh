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
