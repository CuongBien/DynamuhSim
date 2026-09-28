"""Episode-level deterministic split and immutable Dataset V1 fingerprint."""
from __future__ import annotations

import csv
import hashlib
import os
import shutil
import tempfile
from collections import Counter, defaultdict
from pathlib import Path

import yaml

from .plan import ProductionConfig, read_plan
from scenario_generator.src.scenario_generator import FAMILIES

SPLITS = ("train", "val", "test")
TERMINAL = {"PASS", "FAIL_INFRASTRUCTURE"}


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _split_counts(size: int, ratios: dict[str, float]) -> dict[str, int]:
    raw = {name: size * ratios[name] for name in SPLITS}
    counts = {name: int(raw[name]) for name in SPLITS}
    for name in sorted(SPLITS, key=lambda n: (-(raw[n] - counts[n]), SPLITS.index(n)))[
        :size - sum(counts.values())
    ]:
        counts[name] += 1
    return counts


def make_splits(rows: list[dict[str, str]], config: ProductionConfig) -> dict[str, list[str]]:
    if any(row["status"] not in TERMINAL for row in rows):
        raise ValueError("Cannot freeze while production has unfinished episodes")
    accepted = [row for row in rows if row["status"] == "PASS"]
    counts = Counter(row["scenario_family"] for row in accepted)
    for index, family in enumerate(FAMILIES):
        target = config.target_episodes[f"S{index:02d}"]
        if counts[family] < target:
            raise ValueError(f"Cannot freeze: {family} has {counts[family]} accepted episodes")
    groups: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in accepted:
        groups[(row["scenario_family"], row["density"])].append(row)
    result = {name: [] for name in SPLITS}
    for key in sorted(groups):
        group = sorted(groups[key], key=lambda row: hashlib.sha256(
            f"{config.split_seed}:{row['scenario_family']}:{row['seed']}:{row['density']}".encode()
        ).hexdigest())
        allocation = _split_counts(len(group), config.split_ratios)
        offset = 0
        for name in SPLITS:
            take = allocation[name]
            result[name].extend(row["episode_id"] for row in group[offset:offset+take])
            offset += take
    for ids in result.values():
        ids.sort()
    all_ids = [episode_id for ids in result.values() for episode_id in ids]
    if len(all_ids) != len(set(all_ids)) or set(all_ids) != {r["episode_id"] for r in accepted}:
        raise ValueError("Split episode leakage or omission")
    digest_by_id = {row["episode_id"]: row["scenario_sha256"] for row in accepted}
    all_configs = [digest_by_id[episode_id] for episode_id in all_ids]
    if len(all_configs) != len(set(all_configs)):
        raise ValueError("Configuration leakage across split files")
    return result


def _manifest_ids(path: Path) -> set[str]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if "episode_id" not in (reader.fieldnames or []):
            raise ValueError("Manifest lacks episode_id")
        ids = [row["episode_id"] for row in reader]
    if len(ids) != len(set(ids)):
        raise ValueError("Manifest has duplicate episode IDs")
    return set(ids)


def freeze(config: ProductionConfig) -> dict:
    from .audit import audit

    if audit(config, write=False)["state"] != "COMPLETE":
        raise ValueError("Cannot freeze: accepted episode integrity/audit is incomplete")
    root = config.output_dir
    rows = read_plan(root / "production_plan.csv")
    split_ids = make_splits(rows, config)
    accepted = {row["episode_id"] for row in rows if row["status"] == "PASS"}
    manifest = root / "manifest.csv"
    if _manifest_ids(manifest) != accepted:
        raise ValueError("Manifest episodes differ from accepted production plan")
    copied_config = root / "configs" / "dataset_v1.yaml"
    copied_config.parent.mkdir(parents=True, exist_ok=True)
    if copied_config.exists():
        if copied_config.read_bytes() != config.path.read_bytes():
            raise ValueError("Frozen dataset config differs from production config")
    else:
        copied_config.write_bytes(config.path.read_bytes())
    split_dir = root / "splits"
    expected_bytes = {name: ("\n".join(split_ids[name]) +
                             ("\n" if split_ids[name] else "")).encode("utf-8")
                      for name in SPLITS}
    if split_dir.exists():
        if any((split_dir / f"{name}.txt").read_bytes() != expected_bytes[name]
               for name in SPLITS):
            raise ValueError("Frozen split files differ from deterministic assignment")
    else:
        staging = Path(tempfile.mkdtemp(prefix=".splits_", dir=root))
        try:
            for name in SPLITS:
                (staging / f"{name}.txt").write_bytes(expected_bytes[name])
            os.replace(staging, split_dir)
        finally:
            if staging.exists():
                shutil.rmtree(staging)
    fingerprint = {
        "dataset_version": config.dataset_version,
        "episode_count": len(accepted),
        "manifest_sha256": _digest(manifest),
        "production_plan_sha256": _digest(root / "production_plan.csv"),
        "train_sha256": _digest(split_dir / "train.txt"),
        "val_sha256": _digest(split_dir / "val.txt"),
        "test_sha256": _digest(split_dir / "test.txt"),
        "config_sha256": _digest(copied_config),
    }
    destination = root / "dataset_fingerprint.yaml"
    if destination.exists():
        if yaml.safe_load(destination.read_text(encoding="utf-8")) != fingerprint:
            raise ValueError("Frozen dataset fingerprint no longer matches its files")
    else:
        destination.write_text(yaml.safe_dump(fingerprint, sort_keys=False), encoding="utf-8")
    return fingerprint
