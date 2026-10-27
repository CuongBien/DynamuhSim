#!/usr/bin/env bash
set -euo pipefail
if (($# == 0)); then set -- demo; fi
source "$(dirname "${BASH_SOURCE[0]}")/../benchmark/task_runtime.sh"
case "$TASK_MODE" in
  build) task_cmake "$TASK_DIR/build" 2 ;;
  demo)
    if [[ ! -f "$TASK_DIR/build/libsocnavgym_adapter.so" ]]; then
      echo "Run $TASK_DIR/run.sh build first" >&2; exit 1
    fi
    task_ros workspace
    exec python3 "$TASK_DIR/run.py" "$@"
    ;;
  *) echo 'Usage: run.sh build | demo [--mode walking|conversation] [--gui] [--rviz] [--nav]' >&2; exit 2 ;;
esac
