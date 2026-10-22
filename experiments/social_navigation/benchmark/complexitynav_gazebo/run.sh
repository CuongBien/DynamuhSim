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
    python3 "$TASK_DIR/preflight.py"
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$TASK_ROOT/install/setup.bash"
    set -u
    exec python3 "$TASK_DIR/verify_live.py" "$@"
    ;;
  analyze) exec "$TASK_DIR/../../.venv-socnavgym/bin/python" "$TASK_DIR/analyze.py" "$@" ;;
  transport)
    python3 "$TASK_DIR/preflight.py"
    exec python3 "$TASK_DIR/test_transport.py" "$@"
    ;;
  *) echo 'Usage: bash run.sh {build|live|analyze|transport} [arguments]' ;;
esac
