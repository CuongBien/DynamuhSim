"""Verify a frozen NarrowNav research release without writing to its artifacts."""

import argparse
import csv
import hashlib
import json
import sys
import tarfile
from collections import Counter
from pathlib import Path, PurePosixPath
from zipfile import BadZipFile, ZipFile

RAW_CHANNELS = {
    "actions_executed.jsonl",
    "actions_limited.jsonl",
    "actions_requested.jsonl",
    "events.jsonl",
    "gt_10hz.jsonl",
    "intent.json",
    "manifest.json",
    "resolved_config.yaml",
    "trace.jsonl",
    "validity.json",
}
CARD_IDS = {
    "FA_Corridor_Passing",
    "ND_Doorway_Passing",
    "BC_Corner_Turn",
    "IN_Crossing_Paths",
    "OT_Same_Direction",
    "MG_Small_Group",
}
BACKENDS = {"avm_jupedsim", "csm_jupedsim", "orca", "sfm_jupedsim"}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load(path):
    return json.loads(path.read_text())


def read_csv(path):
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def sha256(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def fingerprint(value):
    payload = json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode()).hexdigest()


def artifact_path(root, name):
    path = PurePosixPath(name)
    require(
        path.parts and not path.is_absolute() and ".." not in path.parts,
        f"invalid relative path {name!r}",
    )
    resolved = root.joinpath(*path.parts).resolve()
    require(resolved.is_relative_to(root), f"artifact escapes the root: {name}")
    return resolved


def verify_hashes(root, spec):
    checksum_path = artifact_path(root, spec["path"])
    require(
        sha256(checksum_path) == spec["sha256"], "checksum inventory digest mismatch"
    )
    entries = {}
    for line in checksum_path.read_text().splitlines():
        expected, name = line.split("  ", 1)
        require(
            name not in entries and len(expected) == 64, f"invalid checksum row: {name}"
        )
        path = artifact_path(root, name)
        require(
            path.is_file() and sha256(path) == expected,
            f"missing or changed artifact: {name}",
        )
        entries[name] = expected
    require(len(entries) == spec["entries"], "checksum inventory count mismatch")
    return entries


def verify_design(phases, protocol, lock):
    require(protocol["phase_totals"] == [48, 540, 900], "unsupported phase allocation")
    rosters = []
    for phase, manifest in enumerate(phases):
        require(
            manifest["schema_version"] == 1 and manifest["phase"] == phase,
            f"invalid Phase {phase} schema",
        )
        require(
            manifest["plan_id"] == lock["plan_id"],
            f"Phase {phase} has a different plan ID",
        )
        payload = {k: v for k, v in manifest.items() if k != "manifest_sha256"}
        require(
            fingerprint(payload) == manifest["manifest_sha256"],
            f"Phase {phase} canonical fingerprint mismatch",
        )
        roster = {s["slot_id"]: s for s in manifest["slots"]}
        require(
            len(roster) == len(manifest["slots"]) == protocol["phase_totals"][phase],
            f"Phase {phase} slot count or uniqueness mismatch",
        )
        rosters.append(roster)
    zero, one, two = rosters
    require(
        zero.keys() < one.keys() < two.keys(), "phases do not preserve slot membership"
    )
    require(
        Counter(s["branch"] for s in two.values()) == {"a": 720, "b": 180},
        "branch counts differ",
    )
    require(
        {s["backend"] for s in two.values()} == BACKENDS, "backend coverage differs"
    )
    require({s["card_id"] for s in two.values()} == CARD_IDS, "Card coverage differs")
    critical = Counter(
        (s["backend"], s["card_id"], s["seed"])
        for s in two.values()
        if s["branch"] == "a" and s["level"] == "critical" and s["action"] == "advance"
    )
    expected = {
        (b, c, seed): 1
        for b in BACKENDS
        for c in CARD_IDS
        for seed in range(1000, 1020)
    }
    require(critical == expected, "Critical seed coverage differs")
    volatile = {"is_new", "reused_from_phase"}
    for key, slot in two.items():
        phase = 0 if key in zero else 1 if key in one else 2
        require(
            slot["introduced"] == phase
            and slot["is_new"] is (phase == 2)
            and not slot["blocked"],
            f"invalid origin or blocked slot: {key}",
        )
        stable = {k: v for k, v in slot.items() if k not in volatile}
        for roster in rosters[phase:]:
            require(
                {k: v for k, v in roster[key].items() if k not in volatile} == stable,
                f"retained slot changed: {key}",
            )
        if phase == 2:
            require(
                slot["branch"] == "a"
                and slot["level"] == "critical"
                and slot["action"] == "advance"
                and 1005 <= slot["seed"] <= 1019,
                f"unexpected addition: {key}",
            )
    return rosters


def verify_native(paths, native, runtime):
    with ZipFile(paths["jupedsim_wheel"]) as wheel:
        meta = json.loads(wheel.read("jupedsim/_task04u.json"))
        (native_entry,) = (
            name
            for name in wheel.namelist()
            if name.startswith("jupedsim/py_jupedsim.") and name.endswith(".so")
        )
        require(
            hashlib.sha256(wheel.read(native_entry)).hexdigest()
            == meta["binary_sha256"]
            == native["jupedsim_binary_sha256"],
            "JuPedSim native hash mismatch",
        )
        for field in (
            "source_commit",
            "base_patch_sha256",
            "patch_sha256",
            "wheel_tag",
        ):
            require(
                meta[field] == native["jupedsim_build"][field],
                f"JuPedSim {field} mismatch",
            )
    with tarfile.open(paths["rvo2_binary_archive"], "r:gz") as archive:
        (binary,) = (
            entry
            for entry in archive
            if entry.name.startswith("rvo2.") and entry.name.endswith(".so")
        )
        with archive.extractfile(binary) as stream:
            value = hashlib.file_digest(stream, "sha256").hexdigest()
        require(
            value == native["rvo2_binary_sha256"] == runtime["rvo2_binary_sha256"],
            "RVO2 native hash mismatch",
        )
        with archive.extractfile("pyrvo2-0.0.0.dist-info/METADATA") as stream:
            require(
                b"Version: 0.0.0\n" in stream.read(),
                "RVO2 distribution version mismatch",
            )
    require(
        runtime["git_dirty"] is False
        and runtime["packages"]["jupedsim"] == "1.4.2+task04u"
        and runtime["packages"]["pyrvo2"] == "0.0.0"
        and runtime["packages"]["mujoco"] == "4.0.0",
        "unexpected research runtime",
    )


def verify_episodes(root, raw, paths, entries, rosters, lock, release):
    slots = rosters[-1]
    inventory = read_csv(paths["episode_inventory"])
    by_key = {r["slot_id"]: r for r in inventory}
    require(
        len(inventory) == len(by_key) == 900 and by_key.keys() == slots.keys(),
        "episode inventory does not cover 900 unique slots",
    )
    require(
        {p.name for p in (raw / "attempts").iterdir()} == slots.keys(),
        "raw slot directories differ from Phase 2",
    )
    used, episode_ids = set(), set()
    versions = {backend: set() for backend in BACKENDS}
    for key, row in by_key.items():
        slot = slots[key]
        folder = raw / "attempts" / key
        attempt = folder / "attempt-0001"
        require(
            list(folder.iterdir()) == [attempt]
            and row["attempt"] == attempt.name
            and artifact_path(root, row["raw_attempt"]) == attempt,
            f"attempt mismatch: {key}",
        )
        require(
            {p.name for p in attempt.iterdir()} == RAW_CHANNELS,
            f"raw channel set differs: {key}",
        )
        require(
            all(
                (attempt / f).relative_to(root).as_posix() in entries
                for f in RAW_CHANNELS
            ),
            f"raw channel is missing from checksums: {key}",
        )
        intent, m, v = (
            load(attempt / f) for f in ("intent.json", "manifest.json", "validity.json")
        )
        require(
            all(m.get(k) == val for k, val in intent.items())
            and m["schema_version"] == v.get("schema_version", 1) == 1
            and m["error"] is None
            and v["infrastructure_valid"] is True,
            f"invalid attempt provenance: {key}",
        )
        phase = slot["introduced"]
        require(
            m["slot"] == rosters[phase][key]
            and m["run_provenance"] == lock["runtime"]
            and m["phase"] == phase,
            f"slot binding or runtime mismatch: {key}",
        )
        execution = fingerprint(
            (
                lock,
                key,
                m["backend_version"],
                m["config_sha256"],
                m["model_sha256"],
                m["asset_hashes"],
                slot["binding"],
            )
        )
        require(
            m["execution_key"] == row["execution_key"] == execution
            and execution not in used,
            f"invalid or duplicate execution key: {key}",
        )
        require(
            m["episode_id"] == row["episode_id"] and m["episode_id"] not in episode_ids,
            f"invalid or duplicate episode ID: {key}",
        )
        used.add(execution)
        episode_ids.add(m["episode_id"])
        versions[slot["backend"]].add(m["backend_version"])
        config_name = (attempt / "resolved_config.yaml").relative_to(root).as_posix()
        require(
            entries[config_name] == row["config_sha256"] == m["config_sha256"]
            and row["model_sha256"] == m["model_sha256"]
            and row["backend_version"] == m["backend_version"]
            and json.loads(row["asset_hashes"]) == m["asset_hashes"],
            f"fingerprint mismatch: {key}",
        )
        for name, digest in m["asset_hashes"].items():
            asset = (
                PurePosixPath(release["artifact_roots"]["snapshot"])
                / "sim/mujoco"
                / name
            ).as_posix()
            require(
                entries.get(asset) == digest, f"asset fingerprint mismatch: {asset}"
            )
        require(
            row["outcome"] == v["outcome"]
            and row["safe_task_pass"] == str(v["episode_task_pass"])
            and row["label"]
            == (
                ""
                if v["scenario_label_valid"] is None
                else str(v["scenario_label_valid"])
            ),
            f"inventory result differs from raw: {key}",
        )
    require(
        {k: sorted(v) for k, v in versions.items()} == release["backend_versions"],
        "observed backend versions differ",
    )
    return by_key


def verify_reports(paths, by_key, release):
    counts = release["summary"]
    qa = read_csv(paths["final_qa"])
    require(
        len(qa) == 900 and {r["slot_id"] for r in qa} == by_key.keys(),
        "final QA slot set differs",
    )
    for r in qa:
        row = by_key[r["slot_id"]]
        for field in (
            "episode_id",
            "execution_key",
            "outcome",
            "label",
            "infrastructure_valid",
        ):
            require(
                r[field] == row[field],
                f"final QA differs from inventory: {r['slot_id']}",
            )
    require(
        sum(r["safe_task_pass"] == "True" for r in by_key.values())
        == counts["safe_task"]
        and sum(r["label"] == "True" for r in by_key.values()) == counts["label_pass"],
        "release outcome summary differs from raw",
    )
    paired = read_csv(paths["paired_comparison"])
    require(
        len(paired) == 1350
        and len({(r["left_slot_id"], r["right_slot_id"]) for r in paired}) == 1350,
        "backend pair coverage differs",
    )
    for p in paired:
        a, b = (by_key[p[f"{side}_slot_id"]] for side in ("left", "right"))
        task = a["label"] == b["label"] == "True"
        motion = task and a["safe_task_pass"] == b["safe_task_pass"] == "True"
        require(
            a["paired_key"] == b["paired_key"] == p["paired_key"]
            and p["task_eligible"] == str(task)
            and p["motion_eligible"] == str(motion),
            "backend pair or eligibility differs from inventory",
        )
    action = read_csv(paths["action_contrasts"])
    require(
        len(action) == 180
        and len({r["action_slot_id"] for r in action}) == 180
        and sum(
            r["prefix_verified"] == r["intervention_verified"] == "True" for r in action
        )
        == 180
        and sum(r["task_eligible"] == "True" for r in action) == 150
        and sum(r["motion_eligible"] == "True" for r in action) == 83,
        "action pair coverage differs",
    )
    replay = load(paths["replay_results"])
    require(
        replay["schema_version"] == 1
        and replay["status"] == "PASS"
        and len(replay["cases"]) == 9,
        "replay does not satisfy the release contract",
    )
    require(
        {r["backend"] for r in replay["cases"]} == BACKENDS
        and {r["card"] for r in replay["cases"]} == CARD_IDS,
        "replay lacks model or Card coverage",
    )
    require(
        all(
            r["command_exit_code"] == 0
            and not r["differences"]
            and r["source_episode_id"] == by_key[r["slot_id"]]["episode_id"]
            and r["outcome"] == by_key[r["slot_id"]]["outcome"]
            for r in replay["cases"]
        ),
        "replay results disagree with the frozen run",
    )


def verify(manifest_path, root):
    release = load(manifest_path)
    require(
        release["schema_version"] == 1
        and release["summary"]["unique_valid_slots"] == 900,
        "unsupported release schema or result count",
    )
    entries = verify_hashes(root, release["checksums"])
    paths = {
        name: artifact_path(root, value) for name, value in release["files"].items()
    }
    require(
        all(name in entries for name in release["files"].values()),
        "release artifact lacks a checksum",
    )
    raw = artifact_path(root, release["artifact_roots"]["raw"])
    lock = load(paths["run_lock"])
    require(
        lock["schema_version"] == 1 and lock["runtime"] == release["runtime"],
        "run lock differs from release provenance",
    )
    phases = [load(paths[f"phase_{i}"]) for i in range(3)]
    rosters = verify_design(phases, release["protocol"], lock)
    verify_native(paths, release["native"], lock["runtime"])
    by_key = verify_episodes(root, raw, paths, entries, rosters, lock, release)
    verify_reports(paths, by_key, release)
    print(
        f"PASS {release['release_id']}: 900 unique slots, {len(entries)} checksums, 9 matched replays"
    )
    print(f"release_manifest_sha256={sha256(manifest_path)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument(
        "--root", required=True, type=Path, help="Workspace or restored package root"
    )
    args = parser.parse_args()
    try:
        verify(args.manifest, args.root.resolve())
    except (
        ValueError,
        TypeError,
        KeyError,
        OSError,
        tarfile.TarError,
        BadZipFile,
    ) as exc:
        print(f"FAILED: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
