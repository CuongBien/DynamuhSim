#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_MODE="${1:-demo}"
if (($#)); then shift; fi
TASK_IMAGE=dynamuhsim-complexitynav:round3
case "$TASK_MODE" in
  build)
    if ! docker image inspect dynamuhsim-crowdsimplus:round2 >/dev/null 2>&1; then
      bash "$TASK_DIR/../round2/run.sh" build
    fi
    exec docker build -t "$TASK_IMAGE" "$TASK_DIR"
    ;;
  demo)
    exec docker run --rm --network none --cpus 4 --memory 4g --user "$(id -u):$(id -g)" \
      -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 -v "$TASK_DIR:/work" "$TASK_IMAGE" \
      python /work/native_entrypoint.py --policy vecmpc --config /work/compatible_demo_config.py \
      --phase test --visualize --video_dir /work/reports --video_file compatible_native_demo.mp4 "$@"
    ;;
  scene|matrix)
    TASK_SCRIPT=run_complexitynav.py
    if [[ "$TASK_MODE" == matrix ]]; then TASK_SCRIPT=run_matrix.py; fi
    exec docker run --rm --network none --cpus 4 --memory 4g --user "$(id -u):$(id -g)" \
      -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 -v "$TASK_DIR:/work" "$TASK_IMAGE" \
      python "/work/$TASK_SCRIPT" "$@"
    ;;
  *) echo 'Usage: bash run.sh {build|demo|scene|matrix} [arguments]' >&2; exit 2;;
esac
