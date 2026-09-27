#!/usr/bin/env bash
set -euo pipefail

BASE="$(cd "$(dirname "$0")" && pwd)"
BRIDGE_CONFIG="$BASE/school_gz8_bridge.yaml"

python3 "$BASE/hunav_gz8_pose_applier.py" \
  --ros-args \
  -p world_name:=school_arena \
  -p target_topic:=/school_hunav/target_poses \
  -p apply_hz:=20.0 \
  -p ground_z:=0.0 &
APPLIER_PID=$!

ros2 run ros_gz_bridge parameter_bridge \
  --ros-args \
  -p config_file:="$BRIDGE_CONFIG" &
BRIDGE_PID=$!

cleanup() {
  kill "$APPLIER_PID" "$BRIDGE_PID" 2>/dev/null || true
  wait "$APPLIER_PID" "$BRIDGE_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

wait -n "$APPLIER_PID" "$BRIDGE_PID"
