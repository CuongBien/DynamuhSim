#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

if [[ ! -x .venv/bin/python ]]; then
    printf 'Missing humans/.venv. Follow the setup in %s/README.md.\n' "$PWD" >&2
    exit 2
fi

mkdir -p tmp
export TMPDIR="$PWD/tmp"
export XDG_CACHE_HOME="$PWD/tmp/cache"
export XDG_CONFIG_HOME="$PWD/tmp/config"
export XDG_DATA_HOME="$PWD/tmp/data"
export MESA_SHADER_CACHE_DIR="$PWD/tmp/mesa"
export PYTHONPATH="$PWD/../src"

exec .venv/bin/python -m human_visual.demo "$@"
