#!/usr/bin/env bash
set -eo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/school_runtime.sh"
school_ros "${HOME}/nav_ws/install/setup.bash"
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
