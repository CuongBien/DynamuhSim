#!/usr/bin/env bash
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_MODE="${1:-demo}"
if (($#)); then shift; fi
TASK_IMAGE=dynamuhsim-crowdsimplus:round2
case "$TASK_MODE" in
  build)
    exec docker build -t "$TASK_IMAGE" "$TASK_DIR"
    ;;
  demo)
    mkdir -p "$TASK_DIR/reports/official_run"
    exec docker run --rm --network none --cpus 4 --memory 4g \
      --user "$(id -u):$(id -g)" -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
      -v "$TASK_DIR:/work" -w /work/reports/official_run "$TASK_IMAGE" \
      python /opt/CrowdSimPlus/sicnav/simple_test.py --policy dwa \
      --env_config /opt/CrowdSimPlus/sicnav/configs/env.config \
      --policy_config /opt/CrowdSimPlus/sicnav/configs/policy.config --test_case 0 "$@"
    ;;
  scene|matrix)
    TASK_SCRIPT=run_crowdsimplus.py
    if [[ "$TASK_MODE" == matrix ]]; then TASK_SCRIPT=run_matrix.py; fi
    exec docker run --rm --network none --cpus 4 --memory 4g \
      --user "$(id -u):$(id -g)" -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1 \
      -v "$TASK_DIR:/work" "$TASK_IMAGE" python "/work/$TASK_SCRIPT" "$@"
    ;;
  *) echo 'Usage: bash run.sh {build|demo|scene|matrix} [arguments]' >&2; exit 2;;
esac
