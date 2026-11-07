#!/usr/bin/env bash
# Local Task 01: small dependency builds, installed launch refresh, one new episode.
set -eo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TASK_ROOT"
source /opt/ros/jazzy/setup.bash
docker image inspect dynamuhsim-crowdsimplus:round2 --format '{{.Id}}' >/dev/null
cmake -S experiments/social_navigation/navigation/socnavgym_response_fix \
  -B experiments/social_navigation/navigation/socnavgym_response_fix/build --log-level=WARNING
cmake --build experiments/social_navigation/navigation/socnavgym_response_fix/build -j2
mkdir -p python/dynamuhsim/runtime/native/generated
if [[ ! -x python/dynamuhsim/runtime/native/generated/check_friction || \
      python/dynamuhsim/runtime/native/check_friction.cc -nt python/dynamuhsim/runtime/native/generated/check_friction ]]; then
  c++ -std=c++17 python/dynamuhsim/runtime/native/check_friction.cc \
    -o python/dynamuhsim/runtime/native/generated/check_friction $(pkg-config --cflags --libs sdformat14)
fi
python3 - <<'PY'
import importlib.util
from pathlib import Path
p = Path('experiments/social_navigation/navigation/shared_nav2_pilot/runtime.py')
spec = importlib.util.spec_from_file_location('task01_backend', p)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
print('Native ORCA+ kernel:', m.prepare_capture_kernel())
PY
colcon build --packages-select custom_corridor --symlink-install
source install/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
exec python3 tools/capture_episode.py pilot "$@"
