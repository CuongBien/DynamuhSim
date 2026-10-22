#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_ROOT="$(cd "$TASK_DIR/../../../.." && pwd)"
TASK_MODE="${1:-help}"
if (($#)); then shift; fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
case "$TASK_MODE" in
 prepare) exec python3 "$TASK_DIR/prepare.py" "$@" ;;
 finalize) exec python3 "$TASK_DIR/finalize.py" "$@" ;;
 smoke) exec python3 -I "$TASK_DIR/smoke.py" "$@" ;;
 build|audit)
  set +u
  source /opt/ros/jazzy/setup.bash
  source "$TASK_ROOT/install/setup.bash"
  set -u
  exec python3 "$TASK_DIR/$TASK_MODE.py" "$@" ;;
 *) echo 'Usage: bash run.sh {prepare|build|finalize|audit|smoke} [arguments]' ;;
esac
