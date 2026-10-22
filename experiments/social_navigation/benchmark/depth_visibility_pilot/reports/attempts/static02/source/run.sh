#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../../../.." && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 build|capture)
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$TASK_ROOT/install/setup.bash"
  set -u
  if [[ "$TASK_MODE" == build ]]; then
   cmake -S "$TASK_DIR" -B "$TASK_DIR/generated/build" -DCMAKE_BUILD_TYPE=Release
   cmake --build "$TASK_DIR/generated/build" -j1
  else exec python3 "$TASK_DIR/capture.py" "$@"; fi ;;
 analyze|audit|test) exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {build|capture|analyze|audit|test}' ;;
esac
