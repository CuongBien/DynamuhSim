"""Local B3B adapter: record, build, QA and manifest around EpisodeRunner."""
from __future__ import annotations

import csv
import math
import os
import shutil
import signal
import subprocess
import tempfile
import time
from bisect import bisect_left
from pathlib import Path

import cv2
import numpy as np
import yaml

from dataset_runner import (EpisodeHooks, EpisodeMonitor, EpisodeRunner, HuNavManager,
                            ReadinessChecker, SimulatorManager)
from dataset_runner.nav2_manager import Nav2Manager
from dataset_runner.hunav_manager import DockerHuNavRuntime, HuNavConfig
from dataset_runner.episode_types import EpisodeResult
from .classification import ProductionDecision, classify
from .integrity import check_accepted, readable_bag

TOPICS = ("/odom", "/scan", "/robot/camera/image_raw", "/tf", "/tf_static", "/cmd_vel")
MANIFEST_FIELDS = ("episode_id", "scenario_family", "seed", "density", "outcome",
                   "duration_sec", "sample_count", "missing_ratio", "qa_status",
                   "bag_path", "structured_path")


def _write_yaml(path: Path, value: dict) -> None:
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")


def _reader(folder: Path):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    metadata = yaml.safe_load((folder / "metadata.yaml").read_text(encoding="utf-8"))[
        "rosbag2_bagfile_information"]
    reader.open(rosbag2_py.StorageOptions(uri=str(folder), storage_id=metadata["storage_identifier"]),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                            output_serialization_format="cdr"))
    return reader


def _nearest(times: list[int], target: int) -> int | None:
    if not times:
        return None
    index = bisect_left(times, target)
    candidates = [i for i in (index - 1, index) if 0 <= i < len(times)]
    return min(candidates, key=lambda i: abs(times[i] - target))


def _rgb(message) -> np.ndarray:
    if message.encoding not in {"rgb8", "bgr8", "rgba8", "bgra8", "mono8"}:
        raise ValueError(f"Unsupported RGB encoding {message.encoding}")
    channels = 1 if message.encoding == "mono8" else 4 if "a8" in message.encoding else 3
    data = np.frombuffer(message.data, dtype=np.uint8)
    image = data.reshape(message.height, message.step)[:, :message.width * channels]
    image = image.reshape(message.height, message.width, channels)
    if message.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if message.encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if message.encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image


def build_structured(folder: Path) -> dict:
    """Create synchronized 5 Hz RGB/odom/scan samples from a readable bag."""
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image, LaserScan
    from rclpy.serialization import deserialize_message

    bag = folder / "raw_bag"
    odom = []
    scans = []
    with readable_bag(bag) as readable:
        reader = _reader(readable)
        while reader.has_next():
            topic, blob, timestamp = reader.read_next()
            if topic == "/odom":
                message = deserialize_message(blob, Odometry)
                pose = message.pose.pose.position
                odom.append((timestamp, float(pose.x), float(pose.y)))
            elif topic == "/scan":
                message = deserialize_message(blob, LaserScan)
                finite = [x for x in message.ranges if math.isfinite(x) and x >= message.range_min]
                scans.append((timestamp, min(finite) if finite else float("nan")))
    if not odom or not scans:
        raise ValueError("Cannot build structured samples without odom and scan")
    odom.sort()
    scans.sort()
    odom_times = [x[0] for x in odom]
    scan_times = [x[0] for x in scans]
    structured = folder / "structured"
    if structured.exists():
        shutil.rmtree(structured)
    (structured / "rgb").mkdir(parents=True)
    target_interval_ns = 200_000_000
    last_timestamp = None
    sample_count = 0
    missing = 0
    with readable_bag(bag) as readable, (structured / "samples.csv").open("w", newline="", encoding="utf-8") as stream:
        reader = _reader(readable)
        writer = csv.DictWriter(stream, fieldnames=("timestamp_ns", "rgb_path", "odom_x", "odom_y",
                                                  "scan_min_m", "odom_delta_ns", "scan_delta_ns"))
        writer.writeheader()
        while reader.has_next():
            topic, blob, timestamp = reader.read_next()
            if topic != "/robot/camera/image_raw":
                continue
            if last_timestamp is not None and timestamp - last_timestamp < target_interval_ns:
                continue
            if last_timestamp is not None and timestamp <= last_timestamp:
                continue
            message = deserialize_message(blob, Image)
            if message.height <= 0 or message.width <= 0:
                continue
            frame = _rgb(message)
            filename = f"rgb/{sample_count:06d}.jpg"
            if not cv2.imwrite(str(structured / filename), frame, [cv2.IMWRITE_JPEG_QUALITY, 90]):
                raise ValueError("JPEG image write failed")
            oi = _nearest(odom_times, timestamp)
            si = _nearest(scan_times, timestamp)
            odom_delta = abs(odom[oi][0] - timestamp)
            scan_delta = abs(scans[si][0] - timestamp)
            if odom_delta > 500_000_000 or scan_delta > 500_000_000:
                missing += 1
            writer.writerow({"timestamp_ns": timestamp, "rgb_path": filename,
                             "odom_x": odom[oi][1], "odom_y": odom[oi][2],
                             "scan_min_m": scans[si][1], "odom_delta_ns": odom_delta,
                             "scan_delta_ns": scan_delta})
            sample_count += 1
            last_timestamp = timestamp
    if sample_count == 0:
        raise ValueError("No RGB samples in raw bag")
    ratio = missing / sample_count
    return {"sample_count": sample_count, "missing_ratio": ratio,
            "status": "PASS" if ratio <= 0.20 else "FAIL"}


class RecordingHooks(EpisodeHooks):
    def __init__(self, folder: Path):
        self.folder = folder
        self.process: subprocess.Popen | None = None
        self.error: str | None = None
        self.recording_ok = False
        self.structured_ok = False
        self.qa_pass = False

    def on_episode_start(self, context):
        bag = self.folder / "raw_bag"
        if bag.exists():
            shutil.rmtree(bag)
        command = ["ros2", "bag", "record", "-s", "sqlite3", "-o", str(bag),
                   "--compression-mode", "file", "--compression-format", "zstd",
                   "--topics", *TOPICS]
        log = (self.folder / "recorder.log").open("w", encoding="utf-8")
        self.process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                        start_new_session=True)
        log.close()
        time.sleep(2)
        if self.process.poll() is not None:
            raise RuntimeError(f"ros2 bag record exited {self.process.returncode}")

    def on_episode_end(self, context, result: EpisodeResult):
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=45)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=10)
                self.error = "recorder shutdown timeout"
        if context is None:
            return
        _write_yaml(self.folder / "execution.yaml", {
            "episode_id": context.episode_id, "seed": context.seed,
            "outcome": result.status.value, "duration_sec": result.duration_sec,
            "termination_reason": result.termination_reason,
        })
        if self.error:
            return
        try:
            from .integrity import _bag
            _bag(self.folder / "raw_bag")
            self.recording_ok = True
            qa = build_structured(self.folder)
            self.structured_ok = True
            self.qa_pass = qa["status"] == "PASS"
            _write_yaml(self.folder / "qa.yaml", qa)
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            _write_yaml(self.folder / "qa.yaml", {"status": "FAIL", "reason": self.error})


def _manifest(folder: Path, result: EpisodeResult, qa: dict) -> None:
    root = folder.parent.parent
    path = root / "manifest.csv"
    records = {}
    if path.exists():
        with path.open(newline="", encoding="utf-8") as stream:
            for row in csv.DictReader(stream):
                records[row["episode_id"]] = row
    context = result.context
    records[context.episode_id] = {
        "episode_id": context.episode_id, "scenario_family": context.scenario_family,
        "seed": context.seed, "density": context.metadata["density"],
        "outcome": result.status.value, "duration_sec": result.duration_sec,
        "sample_count": qa["sample_count"], "missing_ratio": qa["missing_ratio"],
        "qa_status": "PASS", "bag_path": f"episodes/{context.episode_id}/raw_bag",
        "structured_path": f"episodes/{context.episode_id}/structured/samples.csv",
    }
    with tempfile.NamedTemporaryFile("w", newline="", encoding="utf-8", dir=root,
                                     prefix=".manifest_", suffix=".csv", delete=False) as stream:
        temporary = Path(stream.name)
        writer = csv.DictWriter(stream, fieldnames=MANIFEST_FIELDS)
        writer.writeheader()
        writer.writerows(records[key] for key in sorted(records))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def run(folder: Path) -> ProductionDecision:
    """Production backend callable for produce_dataset_v1.py."""
    folder = Path(folder)
    hooks = RecordingHooks(folder)
    log_dir = folder / "runtime_logs"
    log_dir.mkdir(exist_ok=True)
    hunav_config = HuNavConfig.from_yaml()
    simulator = SimulatorManager(log_dir=log_dir)
    hunav = HuNavManager(config=hunav_config,
                         runtime=DockerHuNavRuntime(hunav_config, log_dir=log_dir))
    nav2 = Nav2Manager()
    readiness = ReadinessChecker()
    monitor = EpisodeMonitor(nav2_manager=nav2, simulator_manager=simulator, hunav_manager=hunav)
    runner = EpisodeRunner(simulator_manager=simulator, hunav_manager=hunav,
                           nav2_manager=nav2, readiness_checker=readiness,
                           monitor=monitor, hooks=hooks)
    result = runner.run(folder)
    decision = classify(result, recording_ok=hooks.recording_ok,
                        structured_ok=hooks.structured_ok, qa_pass=hooks.qa_pass,
                        reason=hooks.error or "")
    if decision.status == "PASS":
        row = {"episode_id": result.context.episode_id,
               "scenario_family": result.context.scenario_family,
               "seed": str(result.context.seed),
               "density": result.context.metadata["density"],
               "outcome": result.status.value}
        check_accepted(folder, row)
        qa = yaml.safe_load((folder / "qa.yaml").read_text(encoding="utf-8"))
        _manifest(folder, result, qa)
    return decision
