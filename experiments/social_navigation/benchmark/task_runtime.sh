#!/usr/bin/env bash
# Sourced by task entrypoints; each task keeps its command table local.
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)"
TASK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi

task_ros() {
  set +u
  source /opt/ros/jazzy/setup.bash
  if [[ "${1:-}" == workspace ]]; then source "$TASK_ROOT/install/setup.bash"; fi
  set -u
}

task_cmake() {
  cmake -S "$TASK_DIR" -B "$1" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$1" "-j$2"
}
