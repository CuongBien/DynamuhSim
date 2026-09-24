#!/usr/bin/env bash
set -euo pipefail
CONTAINER=${1:-hunavsim_gz_fortress}
SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
DST=/home/hunav_gz_fortress_ws/src/hunav_gazebo_fortress_wrapper/behavior_trees
for f in "$SRC_DIR"/*.xml; do
  docker cp "$f" "$CONTAINER:$DST/"
done
echo "Copied school interactive BT files to $CONTAINER:$DST"
