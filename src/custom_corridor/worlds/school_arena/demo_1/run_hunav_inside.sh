#!/usr/bin/env bash
set -eo pipefail
SCENARIO="${1:-/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/school_agents.yaml}"
SDF_WORLD="/tmp/school_arena/demo_1_school_floor.world"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$HERE/school_runtime.sh"
source /opt/ros/humble/setup.bash
source /home/ros2_ws/install/setup.bash
source /home/hunav_gz_fortress_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=0

# Refuse duplicate manual/managed sessions instead of killing other users' nodes.
exec 9>"$HERE/hunav_manual.lock"
flock -n 9 || { echo "ERROR: a manual HuNav session is already running" >&2; exit 1; }
if pgrep -f '[r]os2 run hunav_agent_manager (hunav_loader|hunav_agent_manager)|[h]unav_agent_manager/(hunav_loader|hunav_agent_manager)|[h]unav_gz8_school_bridge.py' >/dev/null; then
  echo "ERROR: HuNav nodes are already running; stop their owning terminal first" >&2
  exit 1
fi
LOG_DIR="$(mktemp -d /tmp/school_hunav.XXXXXX)"
echo "HuNav logs: $LOG_DIR"
school_processes
school_start ros2 run hunav_agent_manager hunav_loader --ros-args --params-file "$SCENARIO" > "$LOG_DIR/loader.log" 2>&1
sleep 2
school_start ros2 run hunav_agent_manager hunav_agent_manager --ros-args -p use_sim_time:=false -p publish_tf:=false -p publish_sfm_forces:=false > "$LOG_DIR/manager.log" 2>&1

READY=0
for _ in {1..20}; do
  for pid in "${SCHOOL_PIDS[@]}"; do
    kill -0 "$pid" 2>/dev/null || { wait "$pid" || exit $?; exit 1; }
  done
  if ros2 service list | grep -qx /compute_agents; then READY=1; break; fi
  sleep 1
done
if [[ "$READY" != 1 ]]; then
  echo "ERROR: /compute_agents service timed out!" >&2
  cat "$LOG_DIR/loader.log" "$LOG_DIR/manager.log"
  exit 1
fi
echo "HuNav /compute_agents is READY!"
school_start python3 /tmp/school_arena/hunav_gz8_school_bridge.py \
  --scenario "${SCENARIO}" \
  --sdf "${SDF_WORLD}" \
  --ros-args \
  -p world_name:=school_arena \
  -p robot_name:=robot \
  -p update_hz:=10.0 \
  -p human_z:=0.0 \
  -p route_half_width:=0.45 > >(sed -u '/sequence size exceeds remaining buffer/d') 2>&1
school_wait
