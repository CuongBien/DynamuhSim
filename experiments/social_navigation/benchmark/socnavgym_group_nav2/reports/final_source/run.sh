#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../../../.." && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi
case "$TASK_MODE" in
 live)
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$TASK_ROOT/install/setup.bash"
  set -u
  export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
  exec python3 "$TASK_DIR/trial.py" "$@"
  ;;
 gate) exec python3 "$TASK_DIR/test_gate.py" "$@" ;;
 analyze) exec "$TASK_DIR/../../.venv-socnavgym/bin/python" "$TASK_DIR/analyze.py" "$@" ;;
 *) echo 'Usage: bash run.sh {live|gate|analyze} [arguments]' ;;
esac
