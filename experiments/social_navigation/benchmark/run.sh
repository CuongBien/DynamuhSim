#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/task_runtime.sh"
case "$TASK_MODE" in
  trial)
    task_ros workspace
    exec python3 "$TASK_DIR/trial.py" "$@" ;;
  *) echo 'Usage: run.sh trial --backend empty|rvo2|socnavgym|crowdsimplus [--human-policy orca_plus|sfm] [--goal-layout shared|individual] [--gui] [--report-dir DIR]' >&2; exit 2 ;;
esac
