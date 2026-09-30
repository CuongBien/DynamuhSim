#!/usr/bin/env bash
set -e

SCENARIO="${1:-/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/school_agents.yaml}"
SDF_WORLD="/tmp/school_arena/demo_1_school_floor.world"

source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=0

echo "[1/4] Cleaning up previous HuNav processes..."
pkill -9 -x hunav_loader 2>/dev/null || true
pkill -9 -x hunav_agent_manager 2>/dev/null || true
pkill -9 -f "python3.*hunav_gz8_school_bridge" 2>/dev/null || true
sleep 1

echo "[2/4] Starting hunav_loader with: ${SCENARIO}..."
ros2 run hunav_agent_manager hunav_loader --ros-args --params-file "${SCENARIO}" > /tmp/hunav_loader.log 2>&1 &
LOADER_PID=$!
sleep 2

echo "[3/4] Starting hunav_agent_manager..."
ros2 run hunav_agent_manager hunav_agent_manager --ros-args -p use_sim_time:=false -p publish_tf:=false -p publish_sfm_forces:=false > /tmp/hunav_agent_mgr.log 2>&1 &
MGR_PID=$!

cleanup() {
  echo ""
  echo "Shutting down HuNav processes..."
  kill -9 "$LOADER_PID" "$MGR_PID" 2>/dev/null || true
  pkill -9 -x hunav_loader 2>/dev/null || true
  pkill -9 -x hunav_agent_manager 2>/dev/null || true
  exit 0
}
trap cleanup EXIT INT TERM

echo "Waiting for /compute_agents service..."
READY=0
for i in {1..20}; do
  if ros2 service list | grep -q '/compute_agents'; then
    echo "HuNav /compute_agents is READY!"
    READY=1
    break
  fi
  sleep 1
done

if [[ "$READY" -ne 1 ]]; then
  echo "ERROR: /compute_agents service timed out!"
  echo "--- hunav_loader.log ---"
  cat /tmp/hunav_loader.log
  echo "--- hunav_agent_mgr.log ---"
  cat /tmp/hunav_agent_mgr.log
  exit 1
fi

echo "[4/4] Starting hunav_gz8_school_bridge (Human motion active!)..."
python3 /tmp/school_arena/hunav_gz8_school_bridge.py \
  --scenario "${SCENARIO}" \
  --sdf "${SDF_WORLD}" \
  --ros-args \
  -p world_name:=school_arena \
  -p robot_name:=robot \
  -p update_hz:=10.0 \
  -p human_z:=0.0 \
  -p route_half_width:=0.45 2>&1 | grep --line-buffered -v "sequence size exceeds remaining buffer"
