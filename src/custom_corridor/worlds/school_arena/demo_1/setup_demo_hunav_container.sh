#!/usr/bin/env bash
set -euo pipefail
CONTAINER=${1:-hunavsim_gz_fortress}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARENT="$(dirname "$HERE")"
SCENARIOS=/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/scenarios
BT=/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/behavior_trees
RUNTIME=/tmp/school_arena

docker exec "$CONTAINER" mkdir -p "$SCENARIOS" "$BT" "$RUNTIME"
docker cp "$HERE/hunav_one_human.yaml" "$CONTAINER:$SCENARIOS/demo_1_hunav.yaml"
docker cp "$HERE/hunav_social_test.yaml" "$CONTAINER:$SCENARIOS/demo_1_social.yaml"
docker cp "$HERE/demo_1_hunav__agent_1_bt.xml" "$CONTAINER:$BT/"
docker cp "$HERE/demo_1_social__agent_1_bt.xml" "$CONTAINER:$BT/"
docker cp "$HERE/demo_1_social__agent_2_bt.xml" "$CONTAINER:$BT/"
docker cp "$PARENT/school_hunav_bt/BTRegularNav.xml" "$CONTAINER:$BT/"
docker cp "$PARENT/school_hunav_bt/BTSchoolInteractive.xml" "$CONTAINER:$BT/"
docker cp "$PARENT/hunav_gz8_school_bridge.py" "$CONTAINER:$RUNTIME/"
docker cp "$HERE/school_floor.world" "$CONTAINER:$RUNTIME/demo_1_school_floor.world"
echo "Installed demo_1 HuNav scenarios into $CONTAINER"

