#!/usr/bin/env bash
# Use local /usr/bin/python3 so ROS2/Jazzy workflows cannot accidentally pick Miniconda.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_RESTORED="$HOME/nav_ws/wm_restore/wm_episode_batch/generated/dataset01"
DEFAULT_IN_REPO="$HOME/nav_ws/experiments/social_navigation/benchmark/wm_episode_batch/generated/dataset01"
if [[ $# -ge 1 ]]; then
  DATASET_ROOT="$1"
elif [[ -f "$DEFAULT_RESTORED/dataset.json" ]]; then
  DATASET_ROOT="$DEFAULT_RESTORED"
elif [[ -f "$DEFAULT_IN_REPO/dataset.json" ]]; then
  DATASET_ROOT="$DEFAULT_IN_REPO"
else
  echo "ERROR: dataset.json not found in expected folders." >&2
  echo "Usage: bash run_all.sh /absolute/path/to/dataset01 [output_dir]" >&2
  exit 2
fi
OUTPUT_DIR="${2:-$HOME/nav_ws/wm_dataset_report}"
# Inventory the complete restored backup (raw MCAP, runtime, reports, sample windows),
# while profiling the normalized portable dataset01 specifically.
if [[ "$DATASET_ROOT" == "$HOME/nav_ws/wm_restore/wm_episode_batch/generated/dataset01" ]]; then
  DEFAULT_INDEX_ROOT="$HOME/nav_ws/wm_restore"
elif [[ "$DATASET_ROOT" == */generated/dataset01 ]]; then
  DEFAULT_INDEX_ROOT="${DATASET_ROOT%/generated/dataset01}"
else
  DEFAULT_INDEX_ROOT="$DATASET_ROOT"
fi
INDEX_ROOT="${3:-$DEFAULT_INDEX_ROOT}"
PY=/usr/bin/python3
printf 'Inventory root: %s\nPortable dataset: %s\nOutput: %s\n' "$INDEX_ROOT" "$DATASET_ROOT" "$OUTPUT_DIR"
"$PY" "$SCRIPT_DIR/01_inventory.py" --root "$INDEX_ROOT" --out "$OUTPUT_DIR"
"$PY" "$SCRIPT_DIR/02_profile_dataset.py" --root "$DATASET_ROOT" --out "$OUTPUT_DIR" --sample-windows 120 --depth-frames 12
printf '\nDone. AI prompt package:\n  %s/DATASET_PROFILE.md\n  %s/PROMPT_FOR_AI.md\n  %s/DATASET_CONTEXT.json\n  %s/FIELD_SCHEMA.csv\n  %s/ALL_PATHS.csv\n' "$OUTPUT_DIR" "$OUTPUT_DIR" "$OUTPUT_DIR" "$OUTPUT_DIR" "$OUTPUT_DIR"
