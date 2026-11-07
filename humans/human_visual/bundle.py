"""Validate and read normalized visual-only character bundles."""

import json
from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path

import numpy as np

from .animation import Clip


class AssetError(ValueError):
    """Unavailable or malformed source/generated assets."""


def file_sha256(path):
    return sha256(Path(path).read_bytes()).hexdigest()


@dataclass(frozen=True)
class SkinPart:
    vertices: np.ndarray
    faces: np.ndarray
    uv: np.ndarray
    weights: np.ndarray
    rgba: tuple
    texture: bytes | None


@dataclass(frozen=True)
class Character:
    identifier: str
    metadata: dict
    parents: np.ndarray
    levels: tuple
    rest_p: np.ndarray
    rest_q: np.ndarray
    parts: tuple[SkinPart, ...]
    clips: dict[str, Clip]
    foot_indices: tuple[tuple[int, np.ndarray], ...]


def hierarchy_levels(parents):
    depth = []
    for index, parent in enumerate(parents):
        if not -1 <= parent < index:
            raise AssetError("rig must be parent-first and acyclic")
        depth.append(0 if parent < 0 else depth[parent] + 1)
    if not depth:
        raise AssetError("rig has no bones")
    return tuple(
        np.flatnonzero(np.asarray(depth) == level) for level in range(max(depth) + 1)
    )


def _check_quaternions(values, shape):
    if values.shape != shape or not np.allclose(
        np.linalg.norm(values, axis=-1), 1, rtol=0, atol=2e-5
    ):
        raise AssetError("missing or unnormalized bone quaternion")


def load_character(directory):
    directory = Path(directory)
    if not (directory / "manifest.json").is_file():
        raise AssetError(
            f"missing character bundle at {directory}; prepare real Quaternius assets first"
        )
    manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
    source = manifest["source"]
    if manifest["schema_version"] != 1 or manifest["gender"] not in ("male", "female"):
        raise AssetError("unsupported visual bundle schema or character gender")
    if (
        source["license"] != "CC0-1.0"
        or len(source["sha256"]) != 64
        or not source["url"].startswith("https://")
    ):
        raise AssetError("missing source provenance or supported license")
    parents = np.asarray(manifest["parents"], dtype=np.int64)
    levels = hierarchy_levels(parents)
    bones = manifest["bone_names"]
    n = len(parents)
    if len(bones) != n or len(set(bones)) != n:
        raise AssetError("bone names must be unique and match the hierarchy")
    if file_sha256(directory / "character.npz") != manifest["data_sha256"]:
        raise AssetError("character arrays do not match the manifest hash")
    with np.load(directory / "character.npz", allow_pickle=False) as data:
        rest_p, rest_q = data["rest_p"], data["rest_q"]
        if rest_p.shape != (n, 3) or not np.all(np.isfinite(rest_p)):
            raise AssetError("invalid rest translations")
        _check_quaternions(rest_q, (n, 4))
        parts = []
        feet = []
        for index, part in enumerate(manifest["parts"]):
            verts, faces, uv, weights = (
                data[f"{key}_{index}"] for key in ("verts", "faces", "uv", "weights")
            )
            count = len(verts)
            if not count or verts.shape != (count, 3) or not np.all(np.isfinite(verts)):
                raise AssetError("invalid character vertices")
            if (
                faces.ndim != 2
                or faces.shape[1] != 3
                or not len(faces)
                or faces.min() < 0
                or faces.max() >= count
            ):
                raise AssetError("invalid character triangle indices")
            if uv.shape != (count, 2) or not np.all(np.isfinite(uv)):
                raise AssetError("invalid skin UV coordinates")
            if (
                weights.shape != (count, n)
                or np.any(weights < 0)
                or not np.allclose(weights.sum(axis=1), 1, rtol=0, atol=1e-5)
            ):
                raise AssetError("skin weights must be nonnegative and normalized")
            texture = None
            if part["texture"]:
                path = directory / Path(part["texture"]).name
                if file_sha256(path) != part["texture_sha256"]:
                    raise AssetError("texture does not match the manifest hash")
                texture = path.read_bytes()
            parts.append(
                SkinPart(verts, faces, uv, weights, tuple(part["rgba"]), texture)
            )
            feet.append((index, data[f"feet_{index}"]))
        clips = {}
        for name, clip in manifest["clips"].items():
            index = clip["index"]
            times, positions, rotations = (
                data[f"{key}_{index}"] for key in ("times", "pos", "rot")
            )
            if (
                times.ndim != 1
                or len(times) < 2
                or times[0] != 0
                or not np.all(np.isfinite(times))
                or np.any(np.diff(times) <= 0)
                or positions.shape != (len(times), n, 3)
                or not np.all(np.isfinite(positions))
            ):
                raise AssetError(f"invalid sample data for {name}")
            _check_quaternions(rotations, (len(times), n, 4))
            reference = clip["reference_speed_mps"]
            if reference is not None and (not np.isfinite(reference) or reference <= 0):
                raise AssetError(f"invalid reference speed for {name}")
            clips[name] = Clip(
                times,
                positions,
                rotations,
                clip["loop"],
                reference,
                clip["seam_blend_s"],
            )
    if not parts or not {"idle", "walk"}.issubset(clips):
        raise AssetError("character needs a skinned mesh and real idle and walk clips")
    return Character(
        manifest["identifier"],
        manifest,
        parents,
        levels,
        rest_p,
        rest_q,
        tuple(parts),
        clips,
        tuple(feet),
    )
