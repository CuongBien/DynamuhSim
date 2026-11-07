"""Fetch and verify the pinned CC0 Quaternius archives and exact FBX members."""

import json
from hashlib import sha256
from io import BytesIO
from pathlib import Path
from urllib.request import urlopen
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parent


def checked(content, expected, label):
    if sha256(content).hexdigest() != expected:
        raise ValueError(f"SHA-256 mismatch for {label}")
    return content


def main():
    selection = json.loads((ROOT / "assets/selection.json").read_text(encoding="utf-8"))
    output = ROOT / "assets/source"
    output.mkdir(parents=True, exist_ok=True)
    for character in selection["characters"]:
        source = character["source"]
        targets = (
            (source["member"], output / source["file"], source["sha256"]),
            ("License.txt", output / source["license_file"], source["license_sha256"]),
        )
        for _, path, digest in targets:
            if path.exists():
                checked(path.read_bytes(), digest, path)
        if not all(path.exists() for _, path, _ in targets):
            with urlopen(source["archive_url"], timeout=60) as response:
                archive = checked(
                    response.read(), source["archive_sha256"], source["archive_url"]
                )
            with ZipFile(BytesIO(archive)) as package:
                files = [
                    (path, checked(package.read(member), digest, member))
                    for member, path, digest in targets
                ]
            for path, content in files:
                if not path.exists():
                    path.write_bytes(content)
        for _, path, digest in targets:
            print(f"{digest}  {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
