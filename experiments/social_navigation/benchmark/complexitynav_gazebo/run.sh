#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../task_runtime.sh"
case "$TASK_MODE" in
  build)
    task_ros
    task_cmake "$TASK_DIR/build" 2
    ;;
  live)
    python3 "$TASK_DIR/preflight.py"
    task_ros workspace
    exec python3 "$TASK_DIR/verify_live.py" "$@"
    ;;
  analyze) exec "$TASK_DIR/../../.venv-socnavgym/bin/python" "$TASK_DIR/analyze.py" "$@" ;;
  transport)
    python3 "$TASK_DIR/preflight.py"
    exec python3 "$TASK_DIR/test_transport.py" "$@"
    ;;
  *) echo 'Usage: bash run.sh {build|live|analyze|transport} [arguments]' ;;
esac
