#!/usr/bin/env bash
set -euo pipefail

BASE="$(cd "$(dirname "$0")" && pwd)"
BRIDGE_CONFIG="$BASE/school_gz8_bridge.yaml"
source "$BASE/school_runtime.sh"
school_processes

school_start python3 "$BASE/hunav_gz8_pose_applier.py" \
  --ros-args \
  -p world_name:=school_arena \
  -p target_topic:=/school_hunav/target_poses \
  -p apply_hz:=20.0 \
  -p ground_z:=0.0

school_start ros2 run ros_gz_bridge parameter_bridge \
  --ros-args \
  -p config_file:="$BRIDGE_CONFIG"

school_wait
