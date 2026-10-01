#!/usr/bin/env bash
set -e
SCENARIO_ID="${1:-1}"
EPISODE_ID="${2:-$SCENARIO_ID}"
printf -v SID "%03d" "$SCENARIO_ID"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WS"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd "$WS/experiments/trajectory_risk_v0"
python3 scripts/collect_episode_v2.py \
  --episode "$EPISODE_ID" \
  --world arena_dataset \
  --model burger \
  --scenario "scenarios/scenario_${SID}.json"
