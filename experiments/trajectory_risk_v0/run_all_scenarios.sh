#!/usr/bin/env bash
set -e
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd ~/nav_ws/experiments/trajectory_risk_v0
for n in $(seq 1 13); do
  printf -v sid "%03d" "$n"
  echo "============================================================"
  echo "Running scenario_${sid}.json | episode=${n}"
  echo "============================================================"
  python3 scripts/collect_episode_v2.py \
    --episode "$n" \
    --world arena_dataset \
    --model burger \
    --scenario "scenarios_v1/scenarios/scenario_${sid}.json"
done
