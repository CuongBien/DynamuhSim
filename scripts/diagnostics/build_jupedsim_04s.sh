#!/usr/bin/env bash
# Build only into a new directory. The official environment is read-only.
set -euo pipefail

if [[ $# != 2 ]]; then
    echo "usage: bash $0 jupedsim-1.4.2.tar.gz new-build-directory" >&2
    exit 64
fi

repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
archive="$(realpath -- "$1")"
build_root="$(realpath -m -- "$2")"
patch_file="$repo/experiments/social_navigation/patches/jupedsim_v1_4_2_task04s.patch"
stock_python="$repo/.venv/bin/python"

printf '%s  %s\n' a61ebf4f535474c0e9838bea98f0996f58d87bfc8a5ddb0ca60fb01816a7ec44 "$archive" | sha256sum -c
mkdir -- "$build_root"
mkdir -- "$build_root/source"
tar -xf "$archive" --strip-components=1 -C "$build_root/source"
patch --batch -p1 --directory="$build_root/source" < "$patch_file"

sfm_source="$build_root/source/libsimulator/src/SocialForceModel.cpp"
printf '%s  %s\n' 53569b2b310f84a855fd01ee2702792352acef6c553a39c356eed8650b44a2b3 "$sfm_source" | sha256sum -c
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

cp -a "$build_root/source/python_modules/jupedsim/jupedsim" "$probe_site/"
cp -- "$build_root"/build/lib/py_jupedsim*.so "$probe_site/jupedsim/"
mkdir -- "$probe_site/jupedsim-1.4.2+task04s.dist-info"
printf 'Metadata-Version: 2.1\nName: jupedsim\nVersion: 1.4.2+task04s\n' \
    > "$probe_site/jupedsim-1.4.2+task04s.dist-info/METADATA"

cd -- "$repo"
{
    printf 'JuPedSim release commit: %s\n' 80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe
    cmake --version
    /usr/bin/c++ --version
    "$probe_python" -VV
    sha256sum "$archive" "$patch_file" "$sfm_source" "$repo/uv.lock" \
        "$probe_site"/jupedsim/py_jupedsim*.so "$build_root"/build/lib/*.a
    ldd "$probe_site"/jupedsim/py_jupedsim*.so
    env -u PYTHONPATH "$probe_python" -B -c '
from importlib.metadata import version
from experiments.social_navigation.probe_jupedsim_external_actor import require_experimental_build
print(require_experimental_build())
for package in ("mujoco", "shapely", "numpy", "hydra-core", "pyrvo2", "pytest"):
    print(package, version(package))
'
} | tee "$build_root/provenance.log"
