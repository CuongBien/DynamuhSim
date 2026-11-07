"""Independent deterministic random streams as specified by episode schema v1."""

from hashlib import sha256
from random import Random
from typing import Literal

from .errors import ContractError
from .validation import identifier, require_type, uint64

SeedStream = Literal["spawn", "sensor", "robot_policy", "human_backend", "engine"]
_STREAMS = ("spawn", "sensor", "robot_policy", "human_backend", "engine")


def derive_seed(episode_seed: int, stream: SeedStream, scope: str = "global") -> int:
    uint64(episode_seed)
    if stream not in _STREAMS:
        raise ContractError(f"unknown seed stream {stream!r}")
    require_type(scope, str, "scope")
    if scope != "global":
        kind, _, scope_id = scope.partition(".")
        if kind not in ("entity", "sensor"):
            raise ContractError("seed scope must be global, entity.<id> or sensor.<id>")
        identifier(scope_id, "scope ID")
    material = f"narrow_nav/v1/{episode_seed}/{stream}/{scope}".encode("ascii")
    return int.from_bytes(sha256(material).digest()[:8], "big")


def make_rng(episode_seed: int, stream: SeedStream, scope: str = "global") -> Random:
    return Random(derive_seed(episode_seed, stream, scope))
