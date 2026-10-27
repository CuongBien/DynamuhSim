#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../../../.." && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi
case "$TASK_MODE" in
 build)
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  cmake -S "$TASK_DIR" -B "$TASK_DIR/build" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$TASK_DIR/build" -j2
  ;;
 live)
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$TASK_ROOT/install/setup.bash"
  set -u
  export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
  exec python3 "$TASK_DIR/verify_live.py" "$@"
  ;;
 transport) exec python3 "$TASK_DIR/transport.py" "$@" ;;
 analyze) exec "$TASK_DIR/../../.venv-socnavgym/bin/python" "$TASK_DIR/analyze.py" "$@" ;;
 *) echo 'Usage: bash run.sh {build|transport|live|analyze} [arguments]' ;;
esac
