#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# This is the HuNav-controlled world.  The legacy school_arena.sdf contains
# TrajectoryFollower controllers and must not run at the same time.
export GZ_PARTITION="${GZ_PARTITION:-school_hunav}"

# The Gazebo server (not the later spawn command) resolves model:// mesh URIs.
# Put TurtleBot3's model parent on its resource path before the world starts.
TB3_MODEL_PATH=/opt/ros/jazzy/share/turtlebot3_gazebo/models
if [[ -d "$TB3_MODEL_PATH" ]]; then
  export GZ_SIM_RESOURCE_PATH="${TB3_MODEL_PATH}${GZ_SIM_RESOURCE_PATH:+:${GZ_SIM_RESOURCE_PATH}}"
fi

exec gz sim -r -v "${GZ_VERBOSITY:-3}" \
  "${SCRIPT_DIR}/school_arena_hunav.sdf"
