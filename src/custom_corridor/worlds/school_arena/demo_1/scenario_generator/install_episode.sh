#!/usr/bin/env bash
set -euo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 episode_dir|--school|--demo [container_name]" >&2
  exit 2
fi
MODE="$1"
CONTAINER="${2:-hunavsim_gz_fortress}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO="$(dirname "$HERE")"
PARENT="$(dirname "$DEMO")"
SCENARIOS=/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios
BT=/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/behavior_trees
RUNTIME=/tmp/school_arena
if [[ "$MODE" != --school && "$MODE" != --demo ]]; then
  EPISODE="$(realpath "$MODE")"
  ID="$(basename "$EPISODE")"
  for name in metadata.yaml humans.yaml scenario.yaml school_floor.world nav2_school.yaml; do
    [[ -f "$EPISODE/$name" ]] || { echo "Missing $EPISODE/$name" >&2; exit 1; }
  done
fi

docker exec "$CONTAINER" mkdir -p "$SCENARIOS" "$BT" "$RUNTIME"
case "$MODE" in
  --school)
    docker cp "$PARENT/school_agents.yaml" "$CONTAINER:$SCENARIOS/school_agents.yaml"
    docker cp "$PARENT/school_arena_hunav.sdf" "$CONTAINER:$RUNTIME/school_arena_hunav.sdf"
    BT_FILES=("$PARENT"/school_hunav_bt/*.xml)
    LABEL="school HuNav scenario, BT files and runtime bridge" ;;
  --demo)
    docker cp "$DEMO/hunav_one_human.yaml" "$CONTAINER:$SCENARIOS/demo_1_hunav.yaml"
    docker cp "$DEMO/hunav_social_test.yaml" "$CONTAINER:$SCENARIOS/demo_1_social.yaml"
    docker cp "$DEMO/school_floor.world" "$CONTAINER:$RUNTIME/demo_1_school_floor.world"
    docker cp "$PARENT/school_runtime.sh" "$CONTAINER:$RUNTIME/"
    source "$PARENT/school_runtime.sh"
    docker cp "$SCHOOL_PROCESS_RUNTIME" "$CONTAINER:$RUNTIME/"
    docker cp "$DEMO/run_hunav_inside.sh" "$CONTAINER:$RUNTIME/"
    docker exec "$CONTAINER" chmod +x "$RUNTIME/run_hunav_inside.sh"
    BT_FILES=("$DEMO"/demo_1_{hunav__agent_1,social__agent_1,social__agent_2}_bt.xml
              "$PARENT"/school_hunav_bt/{BTRegularNav,BTSchoolInteractive}.xml)
    LABEL="demo_1 HuNav scenarios" ;;
  *)
    docker cp "$EPISODE/humans.yaml" "$CONTAINER:$SCENARIOS/$ID.yaml"
    docker cp "$EPISODE/school_floor.world" "$CONTAINER:$RUNTIME/$ID.world"
    BT_FILES=("$PARENT"/school_hunav_bt/{BTRegularNav,BTSchoolInteractive}.xml)
    for file in "$EPISODE"/*__agent_*_bt.xml; do
      [[ -e "$file" ]] || continue
      BT_FILES+=("$file")
    done
    LABEL="$ID" ;;
esac
docker cp "$PARENT/hunav_gz8_school_bridge.py" "$CONTAINER:$RUNTIME/"
for file in "${BT_FILES[@]}"; do
  docker cp "$file" "$CONTAINER:$BT/"
done
echo "Installed $LABEL into $CONTAINER"
