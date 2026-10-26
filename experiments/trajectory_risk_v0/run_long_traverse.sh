#!/usr/bin/env bash
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
EXP="$WS/experiments/trajectory_risk_v0"
SCENARIO="$EXP/scenarios_v1/scenario_014.json"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=$EXP/data_long_traverse/episode_${STAMP}
TIMEOUT_SEC=${TIMEOUT_SEC:-120}
GOAL_X=${GOAL_X:-4.5}
GOAL_Y=${GOAL_Y:-0.0}
mkdir -p "$OUT"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source "$WS/tools/process_runtime.sh"
process_init
PROCESS_GRACE_STEPS=200  # Allow rosbag up to 20s to flush before escalation.
cd "$EXP"
echo '[1/5] Publish + start human scenario'
python3 scripts/publish_gz_scenario.py "$SCENARIO"
echo '[2/5] Start rosbag2'
process_start ros2 bag record -o "$OUT/rosbag" /scan /odom /cmd_vel /tf /tf_static /clock >"$OUT/rosbag.log" 2>&1
echo '[3/5] Start CSV frame logger'
process_start python3 scripts/log_frame_csv.py --output "$OUT/frames.csv" >"$OUT/csv_logger.log" 2>&1
cp "$SCENARIO" "$OUT/scenario.json"
sleep 1
echo '[4/5] Send ONE full-room NavigateToPose goal'
set +e
process_start bash -c '
  timeout --foreground "$1" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "$2" --feedback | tee "$3"
  exit "${PIPESTATUS[0]}"
' long-traverse "${TIMEOUT_SEC}s" \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  "$OUT/navigation.log"
wait "$PROCESS_PID"
NAV_RC=$?
set -e
echo "$NAV_RC" > "$OUT/navigation_return_code.txt"
echo '[5/5] Stop human motion'
gz topic -t /multi_human/command -m gz.msgs.StringMsg -p 'data: "stop"' || true
echo "Episode output: $OUT"
