#!/usr/bin/env bash
set -euo pipefail
CONTAINER=${1:-hunavsim_gz_fortress}
BASE="$(cd "$(dirname "$0")" && pwd)"
SCEN_DST=/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios
BT_DST=/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/behavior_trees
RUNTIME_DST=/tmp/school_arena

docker cp "$BASE/school_agents.yaml" "$CONTAINER:$SCEN_DST/school_agents.yaml"
for f in "$BASE"/school_hunav_bt/*.xml; do
  docker cp "$f" "$CONTAINER:$BT_DST/"
done
docker exec "$CONTAINER" mkdir -p "$RUNTIME_DST"
docker cp "$BASE/hunav_gz8_school_bridge.py" "$CONTAINER:$RUNTIME_DST/hunav_gz8_school_bridge.py"
docker cp "$BASE/school_arena_hunav.sdf" "$CONTAINER:$RUNTIME_DST/school_arena_hunav.sdf"

echo "Installed school HuNav scenario, BT files and runtime bridge into $CONTAINER"
