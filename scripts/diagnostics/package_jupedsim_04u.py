"""Package the audited headless CMake build as a local, platform-tagged wheel."""

import argparse
import base64
import csv
import hashlib
import io
import json
import sys
import sysconfig
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile, ZipInfo

VERSION = "1.4.2+task04u"
COMMIT = "80d0b3b1ac5fb523b76e3d8b07f9b540b1f8dcbe"


def digest(data):
    return hashlib.sha256(data).hexdigest()


def package(source, binary, patches, destination):
    tag = f"cp{sys.version_info.major}{sys.version_info.minor}"
    if not binary.name.endswith(sysconfig.get_config_var("EXT_SUFFIX")):
        raise ValueError("extension ABI does not match the packaging interpreter")
    platform = sysconfig.get_platform().replace("-", "_").replace(".", "_")
    tag = f"{tag}-{tag}-{platform}"
    package_dir = source / "python_modules/jupedsim/jupedsim"
    files = {
        f"jupedsim/{path.relative_to(package_dir).as_posix()}": path.read_bytes()
        for path in package_dir.rglob("*.py")
    }
    extension = binary.read_bytes()
    files[f"jupedsim/{binary.name}"] = extension
    files["jupedsim/_task04u.json"] = (
        json.dumps(
            {
                "source_commit": COMMIT,
                "patch_sha256": digest(patches[1].read_bytes()),
                "base_patch_sha256": digest(patches[0].read_bytes()),
                "binary_sha256": digest(extension),
                "wheel_tag": tag,
            },
            sort_keys=True,
        )
        + "\n"
    ).encode()
    info = f"jupedsim-{VERSION}.dist-info"
    files[f"{info}/licenses/LICENSE"] = (source / "LICENSE").read_bytes()
    files[f"{info}/METADATA"] = (
        f"Metadata-Version: 2.4\nName: jupedsim\nVersion: {VERSION}\n"
        "Summary: JuPedSim headless native build for TASK 04U\n"
        "License-Expression: LGPL-3.0-or-later\nLicense-File: LICENSE\n"
        "Requires-Python: >=3.12,<3.13\nRequires-Dist: numpy~=2.2\n"
        "Requires-Dist: shapely~=2.0\nRequires-Dist: deprecated~=1.2.18\n\n"
    ).encode()
    files[f"{info}/WHEEL"] = (
        f"Wheel-Version: 1.0\nGenerator: DynamuhSim TASK 04U\n"
        f"Root-Is-Purelib: false\nTag: {tag}\n"
    ).encode()
    record = io.StringIO(newline="")
    writer = csv.writer(record, lineterminator="\n")
    for name, data in sorted(files.items()):
        sha = (
            base64.urlsafe_b64encode(hashlib.sha256(data).digest())
            .rstrip(b"=")
            .decode()
        )
        writer.writerow((name, f"sha256={sha}", len(data)))
    writer.writerow((f"{info}/RECORD", "", ""))
    files[f"{info}/RECORD"] = record.getvalue().encode()
    wheel = destination / f"jupedsim-{VERSION}-{tag}.whl"
    with ZipFile(wheel, "w") as output:
        for name, data in sorted(files.items()):
            entry = ZipInfo(name, (2020, 1, 1, 0, 0, 0))
            entry.external_attr = 0o100644 << 16
            output.writestr(entry, data, compress_type=ZIP_DEFLATED)
    return wheel


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("binary", type=Path)
    parser.add_argument("base_patch", type=Path)
    parser.add_argument("patch", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    print(
        package(
            args.source, args.binary, (args.base_patch, args.patch), args.destination
        )
    )


if __name__ == "__main__":
    main()
