#!/usr/bin/env bash
# Rebuild from the pinned source archive and install only into a new venv.
set -euo pipefail

if [[ $# != 2 ]]; then
    echo "usage: bash $0 jupedsim-1.4.2.tar.gz new-build-directory" >&2
    exit 64
fi
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
archive="$(realpath -- "$1")"
build_root="$(realpath -m -- "$2")"
stock_python="$repo/.venv/bin/python"
patch_dir="$repo/experiments/social_navigation/patches"
base_patch="$patch_dir/jupedsim_v1_4_2_task04s.patch"
patch_file="$patch_dir/jupedsim_v1_4_2_task04u.patch"

printf '%s  %s\n' a61ebf4f535474c0e9838bea98f0996f58d87bfc8a5ddb0ca60fb01816a7ec44 "$archive" | sha256sum -c
printf '%s  %s\n' 0d79faa084acf5b863a3944258499837b46f7f9ccae56d8119b3a555bab710e4 "$base_patch" | sha256sum -c
mkdir -- "$build_root"
mkdir -- "$build_root/source"
tar -xf "$archive" --strip-components=1 -C "$build_root/source"
patch --batch -p1 --directory="$build_root/source" < "$base_patch"
patch --batch -p1 --directory="$build_root/source" < "$patch_file"
for model in SocialForceModel AnticipationVelocityModel CollisionFreeSpeedModelV2; do
    source="libsimulator/src/$model.cpp"
    tar -xOf "$archive" "jupedsim-1.4.2/$source" | cmp - "$build_root/source/$source"
done

env -u PYTHONPATH "$stock_python" -m venv --without-pip "$build_root/venv"
probe_python="$build_root/venv/bin/python"
stock_site="$(env -u PYTHONPATH "$stock_python" -B -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
probe_site="$(env -u PYTHONPATH "$probe_python" -B -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
printf '%s\n' "$stock_site" "$repo/src" > "$probe_site/locked-repo-deps.pth"

cmake -S "$build_root/source" -B "$build_root/build" -G 'Unix Makefiles' \
    -DCMAKE_CXX_COMPILER=/usr/bin/c++ -DPython_EXECUTABLE="$probe_python" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_UNITY_BUILD=ON -DCMAKE_UNITY_BUILD_BATCH_SIZE=4 \
    -DBUILD_TESTS=OFF -DBUILD_BENCHMARKS=OFF 2>&1 | tee "$build_root/configure.log"
cmake --build "$build_root/build" --target py_jupedsim --parallel 3 2>&1 | tee "$build_root/build.log"

extension=("$build_root"/build/lib/py_jupedsim*.so)
wheel="$(env -u PYTHONPATH "$probe_python" -B "$repo/scripts/diagnostics/package_jupedsim_04u.py" \
    "$build_root/source" "${extension[0]}" "$base_patch" "$patch_file" "$build_root")"
env -u PYTHONPATH "$stock_python" -B -m pip --python "$probe_python" install \
    --no-index --no-deps --ignore-installed --no-compile "$wheel"
{
    "$probe_python" -VV
    cmake --version
    /usr/bin/c++ --version
    sha256sum "$archive" "$base_patch" "$patch_file" "$repo/uv.lock" "$wheel" "${extension[0]}"
    env -u PYTHONPATH "$probe_python" -B -c '
import jupedsim
from importlib.metadata import version
from pathlib import Path
from jupedsim.native import py_jps
print(version("jupedsim"), jupedsim.__file__, py_jps.__file__)
print(Path(jupedsim.__file__).with_name("_task04u.json").read_text())
print(jupedsim.get_build_info())
'
} | tee "$build_root/provenance.log"
