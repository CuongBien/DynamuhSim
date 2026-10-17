#!/usr/bin/env bash
# All Python packages are installed in this experiment's venv only.
set -euo pipefail
TASK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TASK_VENDOR="$TASK_DIR/vendor"
TASK_PY="$TASK_DIR/.venv-socnavgym/bin/python"
mkdir -p "$TASK_VENDOR" "$TASK_DIR/reports"
if [[ ! -d "$TASK_VENDOR/SocNavGym/.git" ]]; then
  git clone https://github.com/gnns4hri/SocNavGym.git "$TASK_VENDOR/SocNavGym"
fi
if [[ ! -d "$TASK_VENDOR/Python-RVO2/.git" ]]; then
  git clone https://github.com/sybrenstuvel/Python-RVO2.git "$TASK_VENDOR/Python-RVO2"
fi
# Do not reset a modified vendor checkout.
for TASK_REPO in SocNavGym Python-RVO2; do
  if [[ -n "$(git -C "$TASK_VENDOR/$TASK_REPO" status --porcelain)" ]]; then
    echo "Vendor checkout $TASK_REPO has changes; refusing to overwrite." >&2
    exit 1
  fi
done
git -C "$TASK_VENDOR/SocNavGym" checkout --detach 95fbc9cebe4357201145a7d0a28c7736c0efe6cf
git -C "$TASK_VENDOR/Python-RVO2" checkout --detach c2c46ba8d59556aa10faf03479293236efea154d
if [[ ! -x "$TASK_PY" ]]; then
  python3 -m venv --without-pip "$TASK_DIR/.venv-socnavgym"
fi
if ! "$TASK_PY" -m pip --version >/dev/null 2>&1; then
  curl -fL https://bootstrap.pypa.io/get-pip.py -o "$TASK_VENDOR/get-pip.py"
  "$TASK_PY" "$TASK_VENDOR/get-pip.py"
fi
"$TASK_PY" -m pip install -r "$TASK_DIR/requirements-socnavgym.txt"
"$TASK_PY" -m pip install 'torch==2.14.1+cpu' --index-url https://download.pytorch.org/whl/cpu
"$TASK_PY" -m pip install --no-build-isolation "$TASK_VENDOR/Python-RVO2"
"$TASK_PY" -m pip install --no-build-isolation --no-deps -e "$TASK_VENDOR/SocNavGym"
"$TASK_PY" -m pip check
"$TASK_PY" -m pip freeze > "$TASK_DIR/reports/environment_freeze.txt"
echo 'Ready. Run: bash experiments/social_navigation/run.sh socnavgym --scenario original --show'
