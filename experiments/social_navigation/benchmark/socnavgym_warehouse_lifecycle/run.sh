#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
case "$TASK_MODE" in
  build)
    task_ros
    task_cmake "$TASK_DIR/build" 2 ;;
  offline)
    exec "$TASK_DIR/../../.venv-socnavgym/bin/python" "$TASK_DIR/offline.py" "$@" ;;
  live|live-seed)
    python3 "$TASK_DIR/check_preflight.py"
    task_ros workspace
    if [[ "$TASK_MODE" == live-seed ]]; then exec python3 "$TASK_DIR/verify_live_seed.py" "$@"; fi
    exec python3 "$TASK_DIR/verify_live.py" "$@" ;;
  analyze)
    exec "$TASK_DIR/../../.venv-socnavgym/bin/python" "$TASK_DIR/analyze.py" "$@" ;;
  *) echo 'Usage: run.sh build|offline|live|live-seed|analyze [arguments]' >&2; exit 2 ;;
esac
