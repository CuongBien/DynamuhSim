#!/usr/bin/env bash
set -eo pipefail
WS=/home/thaonhi/nav_ws
EXP=$WS/experiments/trajectory_risk_v0
SCENARIO=$EXP/scenarios_v1/scenario_014.json
STAMP=$(date +%Y%m%d_%H%M%S)
OUT=$EXP/data_long_traverse/episode_${STAMP}
TIMEOUT_SEC=${TIMEOUT_SEC:-120}
GOAL_X=${GOAL_X:-4.5}
GOAL_Y=${GOAL_Y:-0.0}
TIMEOUT_SEC=${TIMEOUT_SEC:-120}
mkdir -p "$OUT"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd "$EXP"
echo '[1/5] Publish + start human scenario'
python3 scripts/publish_gz_scenario.py "$SCENARIO"
echo '[2/5] Start rosbag2'
ros2 bag record -o "$OUT/rosbag" /scan /odom /cmd_vel /tf /tf_static /clock >"$OUT/rosbag.log" 2>&1 &
BAG_PID=$!
echo '[3/5] Start CSV frame logger'
python3 scripts/log_frame_csv.py --output "$OUT/frames.csv" >"$OUT/csv_logger.log" 2>&1 &
CSV_PID=$!
cleanup(){
  kill -INT "$CSV_PID" 2>/dev/null || true
  kill -INT "$BAG_PID" 2>/dev/null || true
  wait "$CSV_PID" 2>/dev/null || true
  wait "$BAG_PID" 2>/dev/null || true
}
trap cleanup EXIT
cp "$SCENARIO" "$OUT/scenario.json"
sleep 1
echo '[4/5] Send ONE full-room NavigateToPose goal'
set +e
timeout "${TIMEOUT_SEC}s" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $GOAL_X, y: $GOAL_Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}" \
  --feedback | tee "$OUT/navigation.log"
NAV_RC=${PIPESTATUS[0]}
set -e
echo "$NAV_RC" > "$OUT/navigation_return_code.txt"
echo '[5/5] Stop human motion'
gz topic -t /multi_human/command -m gz.msgs.StringMsg -p 'data: "stop"' || true
echo "Episode output: $OUT"
