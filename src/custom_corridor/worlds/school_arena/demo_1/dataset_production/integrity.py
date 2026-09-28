"""Check accepted episode artifacts before audit/freeze can call them complete."""
from __future__ import annotations

import csv
import math
import sqlite3
import shutil
import subprocess
import tempfile
from contextlib import contextmanager
from pathlib import Path

import yaml

from dataset_runner.episode_loader import EpisodeLoader

REQUIRED_TOPICS = ("/odom", "/scan", "/robot/camera/image_raw")
OUTCOMES = {"success", "collision", "timeout", "stuck", "nav_failure"}


def _mapping(path: Path) -> dict:
    if not path.is_file():
        raise ValueError(f"missing {path.name}")
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"invalid {path.name}")
    return value


@contextmanager
def readable_bag(folder: Path):
    """Present file-compressed rosbag2 storage to readers without changing raw data."""
    document = _mapping(folder / "metadata.yaml")
    metadata = document.get("rosbag2_bagfile_information")
    if not isinstance(metadata, dict):
        raise ValueError("invalid rosbag metadata")
    if str(metadata.get("compression_mode", "")).lower() != "file":
        yield folder
        return
    if metadata.get("compression_format") != "zstd":
        raise ValueError("unsupported rosbag file compression")
    with tempfile.TemporaryDirectory(prefix="dataset_bag_read_") as temp:
        readable = Path(temp)
        decompressed = []
        for relative in metadata.get("relative_file_paths", []):
            if not relative.endswith(".zstd") or Path(relative).is_absolute() or ".." in Path(relative).parts:
                raise ValueError("invalid compressed rosbag path")
            source = folder / relative
            if not source.is_file():
                raise ValueError(f"missing compressed rosbag file {relative}")
            target_name = relative[:-5]
            target = readable / target_name
            target.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(["zstd", "-d", "-q", "-f", "-o", str(target), str(source)],
                           check=True, capture_output=True)
            decompressed.append(target_name)
        copy = dict(document)
        copy["rosbag2_bagfile_information"] = dict(metadata,
            relative_file_paths=decompressed, compression_mode="NONE", compression_format="")
        (readable / "metadata.yaml").write_text(yaml.safe_dump(copy), encoding="utf-8")
        yield readable


def _bag(folder: Path) -> dict[str, int]:
    metadata = _mapping(folder / "metadata.yaml").get("rosbag2_bagfile_information")
    if not isinstance(metadata, dict):
        raise ValueError("invalid rosbag metadata")
    paths = metadata.get("relative_file_paths")
    if not isinstance(paths, list) or not paths:
        raise ValueError("rosbag has no data files")
    for relative in paths:
        if not isinstance(relative, str) or Path(relative).is_absolute() or ".." in Path(relative).parts:
            raise ValueError("unsafe rosbag data path")
        file = folder / relative
        if not file.is_file() or file.stat().st_size == 0:
            raise ValueError(f"missing/empty rosbag data file {relative}")
        if file.suffix == ".db3":
            with sqlite3.connect(f"file:{file}?mode=ro", uri=True) as connection:
                if connection.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
                    raise ValueError(f"corrupted rosbag sqlite file {relative}")
    counts = {}
    for entry in metadata.get("topics_with_message_count", []):
        topic = entry.get("topic_metadata", {}).get("name")
        count = entry.get("message_count")
        if isinstance(topic, str) and isinstance(count, int):
            counts[topic] = count
    missing = [topic for topic in REQUIRED_TOPICS if counts.get(topic, 0) <= 0]
    if missing:
        raise ValueError(f"required topics absent/empty: {missing}")
    try:
        import rosbag2_py
        with readable_bag(folder) as readable:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(readable),
                                          storage_id=metadata.get("storage_identifier", "sqlite3")),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"),
            )
            observed = {topic: 0 for topic in REQUIRED_TOPICS}
            previous = None
            while reader.has_next():
                topic, _data, timestamp = reader.read_next()
                if previous is not None and timestamp < previous:
                    raise ValueError("rosbag timestamps are not monotonic")
                previous = timestamp
                if topic in observed:
                    observed[topic] += 1
            if any(value == 0 for value in observed.values()):
                raise ValueError("required rosbag topics unreadable")
    except ImportError as exc:
        raise ValueError("rosbag2_py unavailable for bag readability check") from exc
    except RuntimeError as exc:
        raise ValueError(f"rosbag reader failed: {exc}") from exc
    return counts


def _structured(path: Path) -> int:
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError("missing/empty structured samples.csv")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if "timestamp_ns" not in (reader.fieldnames or []):
            raise ValueError("structured samples missing timestamp_ns")
        count = 0
        last = None
        for row in reader:
            timestamp = int(row["timestamp_ns"])
            if last is not None and timestamp <= last:
                raise ValueError("structured timestamps not strictly monotonic")
            last = timestamp
            count += 1
    if count == 0:
        raise ValueError("structured samples contain no rows")
    return count


def check_accepted(episode_dir: Path, row: dict[str, str]) -> dict:
    """Verify traceability, execution, raw bag, structured samples and QA evidence."""
    context = EpisodeLoader().load(episode_dir)
    if (context.episode_id != row["episode_id"] or context.scenario_family != row["scenario_family"]
            or context.seed != int(row["seed"]) or context.metadata.get("density") != row["density"]):
        raise ValueError("episode metadata does not match production plan")
    execution = _mapping(episode_dir / "execution.yaml")
    outcome = execution.get("outcome")
    duration = execution.get("duration_sec")
    if (outcome != row["outcome"] or outcome not in OUTCOMES or
            isinstance(duration, bool) or not isinstance(duration, (float, int)) or
            not math.isfinite(duration) or duration <= 0):
        raise ValueError("invalid execution outcome/duration")
    _bag(episode_dir / "raw_bag")
    sample_count = _structured(episode_dir / "structured" / "samples.csv")
    qa = _mapping(episode_dir / "qa.yaml")
    if qa.get("status") != "PASS" or qa.get("sample_count") != sample_count:
        raise ValueError("QA report missing PASS or sample count differs")
    ratio = qa.get("missing_ratio")
    if isinstance(ratio, bool) or not isinstance(ratio, (int, float)) or not 0 <= ratio <= 1:
        raise ValueError("invalid QA missing ratio")
    return {"duration_sec": float(duration), "sample_count": sample_count,
            "missing_ratio": float(ratio), "outcome": outcome}
