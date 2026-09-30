#!/usr/bin/env bash
set -e

SCENARIO_ARG="${1:-school_agents}"
CONTAINER="hunavsim_gz_fortress"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Start container if not running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
  echo "[INFO] Starting container ${CONTAINER}..."
  docker start "${CONTAINER}"
fi

# 2. Sync files into container
"${HERE}/setup_demo_hunav_container.sh" "${CONTAINER}" >/dev/null 2>&1

# 3. Determine scenario path inside container
if [[ "${SCENARIO_ARG}" == "demo_1" || "${SCENARIO_ARG}" == "demo_1_hunav" ]]; then
  SCENARIO_PATH="/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/demo_1_hunav.yaml"
elif [[ "${SCENARIO_ARG}" == "social" || "${SCENARIO_ARG}" == "demo_1_social" ]]; then
  SCENARIO_PATH="/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/demo_1_social.yaml"
else
  SCENARIO_PATH="/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios/school_agents.yaml"
fi

echo "=========================================================="
echo " Starting HuNav Simulation for: ${SCENARIO_PATH}"
echo " Press Ctrl+C anytime to stop HuNav."
echo "=========================================================="

exec docker exec -it "${CONTAINER}" /tmp/school_arena/run_hunav_inside.sh "${SCENARIO_PATH}"
