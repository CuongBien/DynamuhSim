"""Ground-truth trajectory parsing and interpolation utilities."""
from __future__ import annotations

import bisect
import csv
import math
import sys
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


def read_ground_truth(path: Path) -> List[Tuple[float, float, float]]:
    """Read a CSV with flexible column names for time, x, y.

    Returns a list of (time_s, x_m, y_m) tuples sorted chronologically.
    """
    if not path.is_file():
        raise FileNotFoundError(f"Ground truth CSV file not found: {path}")

    try:
        with path.open("r", newline="", encoding="utf-8-sig") as f:
            sample = f.read(4096)
            f.seek(0)
            dialect = csv.Sniffer().sniff(sample, delimiters=",;\t")
            reader = csv.DictReader(f, dialect=dialect)
            if not reader.fieldnames:
                raise ValueError("Ground truth CSV has no header row")

            names = {n.strip().lower().replace(" ", "_"): n for n in reader.fieldnames if n}

            def choose(candidates: Sequence[str]) -> Optional[str]:
                return next((names[x] for x in candidates if x in names), None)

            t_col = choose(("time", "t", "sim_time", "timestamp", "stamp", "time_sec", "seconds"))
            x_col = choose(("x", "pose_x", "position_x", "obstacle_x", "world_x"))
            y_col = choose(("y", "pose_y", "position_y", "obstacle_y", "world_y"))

            if not all((t_col, x_col, y_col)):
                raise ValueError(
                    f"Need time/x/y columns in ground truth CSV; found: {', '.join(reader.fieldnames)}"
                )

            rows: List[Tuple[float, float, float]] = []
            for line_no, row in enumerate(reader, start=2):
                try:
                    t = float(row[t_col])
                    x = float(row[x_col])
                    y = float(row[y_col])

                    # Convert integer nanoseconds if present
                    if abs(t) > 1e12:
                        t *= 1e-9

                    if all(math.isfinite(v) for v in (t, x, y)):
                        rows.append((t, x, y))
                except (TypeError, ValueError):
                    print(f"Warning: Ignored malformed ground-truth row {line_no}", file=sys.stderr)

    except OSError as exc:
        raise RuntimeError(f"Cannot read ground-truth CSV {path}: {exc}") from exc

    rows.sort(key=lambda r: r[0])
    if len(rows) < 2:
        raise RuntimeError("Ground-truth CSV requires at least two valid time/x/y rows")
    return rows


def interpolate_xy(
    rows: Sequence[Tuple[float, float, float]],
    t: float,
    times: Optional[Sequence[float]] = None,
) -> Optional[Tuple[float, float]]:
    """Linear interpolation of (x, y) at time t.

    Returns None if t is strictly outside the coverage window.
    """
    if not rows:
        return None
    if t < rows[0][0] or t > rows[-1][0]:
        return None

    times = times if times is not None else [r[0] for r in rows]
    idx = bisect.bisect_left(times, t)
    if idx == 0:
        return rows[0][1], rows[0][2]
    if idx == len(rows):
        return rows[-1][1], rows[-1][2]

    t0, x0, y0 = rows[idx - 1]
    t1, x1, y1 = rows[idx]
    if t1 == t0:
        return x0, y0

    alpha = (t - t0) / (t1 - t0)
    return x0 + alpha * (x1 - x0), y0 + alpha * (y1 - y0)


def compute_ground_truth_speed_series(
    samples: Sequence[Tuple[float, float, float]],
    min_dt: float = 0.001,
    max_plausible_speed: float = 5.0,
) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]], Dict[str, int]]:
    """Compute raw and quality-filtered speeds for the ground truth trajectory."""
    raw: List[Tuple[float, float]] = []
    filtered: List[Tuple[float, float]] = []
    quality = {"nonpositive_or_tiny_dt": 0, "speed_outlier": 0, "kept": 0}

    for (t0, x0, y0), (t1, x1, y1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0:
            quality["nonpositive_or_tiny_dt"] += 1
            continue
        speed = math.hypot(x1 - x0, y1 - y0) / dt
        raw.append((t1, speed))
        if dt < min_dt:
            quality["nonpositive_or_tiny_dt"] += 1
        elif speed > max_plausible_speed:
            quality["speed_outlier"] += 1
        else:
            filtered.append((t1, speed))
            quality["kept"] += 1

    return raw, filtered, quality
