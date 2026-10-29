"""Publish an immutable, complete worker handshake before consumers see its path."""
import json
import os
from pathlib import Path
import tempfile


def publish_ready(runtime, payload):
    """Atomically create ready.json once; never replace a prior handshake.

    The closed staging file and final hard link share a filesystem. Readers of
    ready.json see either no file or all metadata. This is an IPC visibility
    guarantee, not a crash-durability/fsync guarantee.
    """
    runtime = Path(runtime)
    encoded = json.dumps(payload, allow_nan=False) + "\n"
    descriptor, name = tempfile.mkstemp(prefix=".ready-", suffix=".json", dir=runtime)
    staging = Path(name)
    try:
        with os.fdopen(descriptor, "w") as stream:
            stream.write(encoded)
        os.link(staging, runtime / "ready.json")
    finally:
        staging.unlink(missing_ok=True)
