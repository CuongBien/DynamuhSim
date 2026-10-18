#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
case "${1:-}" in
  trial|matrix|geometry-matrix|avoidance-matrix)
    MODE="$1"; shift
    if [[ "$MODE" == matrix ]]; then exec python3 "$HERE/matrix.py" "$@"; fi
    if [[ "$MODE" == geometry-matrix ]]; then exec python3 "$HERE/geometry_matrix.py" "$@"; fi
    if [[ "$MODE" == avoidance-matrix ]]; then exec python3 "$HERE/avoidance_matrix.py" "$@"; fi
    exec python3 "$HERE/trial.py" "$@" ;;
  *) echo 'Usage: run.sh trial --backend empty|rvo2|socnavgym [--gui] [--report-dir DIR] | matrix|geometry-matrix|avoidance-matrix' >&2; exit 2 ;;
esac
