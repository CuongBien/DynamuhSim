#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/task_runtime.sh"
task_ros workspace
case "$TASK_MODE" in
  trial|matrix|geometry-matrix|avoidance-matrix|crowdsim-matrix|sfm-matrix|individual-goals-matrix|crossing-matrix|bottleneck-matrix)
    exec python3 "$TASK_DIR/${TASK_MODE//-/_}.py" "$@" ;;
  *) echo 'Usage: run.sh trial --backend empty|rvo2|socnavgym|crowdsimplus [--human-policy orca_plus|sfm] [--goal-layout shared|individual] [--gui] [--report-dir DIR] | matrix|geometry-matrix|avoidance-matrix|crowdsim-matrix|sfm-matrix|individual-goals-matrix|crossing-matrix|bottleneck-matrix' >&2; exit 2 ;;
esac
