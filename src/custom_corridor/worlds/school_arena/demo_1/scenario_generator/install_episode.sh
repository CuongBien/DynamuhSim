#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 generated/ep_xxxxxx [container_name]" >&2
  exit 2
fi

EPISODE="$(realpath "$1")"
CONTAINER="${2:-hunavsim_gz_fortress}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO="$(dirname "$HERE")"
PARENT="$(dirname "$DEMO")"
SCENARIOS=/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios
BT=/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/behavior_trees
RUNTIME=/tmp/school_arena
ID="$(basename "$EPISODE")"

for name in metadata.yaml humans.yaml scenario.yaml school_floor.world nav2_school.yaml; do
  [[ -f "$EPISODE/$name" ]] || { echo "Missing $EPISODE/$name" >&2; exit 1; }
done

docker exec "$CONTAINER" mkdir -p "$SCENARIOS" "$BT" "$RUNTIME"
docker cp "$EPISODE/humans.yaml" "$CONTAINER:$SCENARIOS/$ID.yaml"
docker cp "$EPISODE/school_floor.world" "$CONTAINER:$RUNTIME/$ID.world"
docker cp "$PARENT/hunav_gz8_school_bridge.py" "$CONTAINER:$RUNTIME/"
docker cp "$PARENT/school_hunav_bt/BTRegularNav.xml" "$CONTAINER:$BT/"
docker cp "$PARENT/school_hunav_bt/BTSchoolInteractive.xml" "$CONTAINER:$BT/"
for file in "$EPISODE"/*__agent_*_bt.xml; do
  [[ -e "$file" ]] || continue
  docker cp "$file" "$CONTAINER:$BT/"
done
echo "Installed $ID into $CONTAINER"
