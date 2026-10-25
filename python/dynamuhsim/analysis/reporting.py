"""Measured recording selection and table/series data shared by report scripts."""
import gzip
import json
from pathlib import Path

from .warehouse import pose_intervals, robot_human_gaps


def recorded_poses(report, *, assessment_end=None, population=6):
    """Read recorded poses with the legacy population and assessment-window rules."""
    with gzip.open(Path(report) / "gazebo_poses.jsonl.gz", "rt") as stream:
        poses = [json.loads(line) for line in stream]
    if assessment_end is None:
        return [row for row in poses if len(row["poses"]) == population+1]
    return [row for row in poses if row["time"] <= assessment_end+1e-8 and len(row["poses"]) == population+1]


def gap_table_row(cells, gaps, lidar_fraction):
    """Keep the existing column order, four gap decimals and percent precision."""
    return ("|" + "|".join(cells) + f"|{gaps['pair']:.4f}m|{gaps['robot']:.4f}m|"
            f"{gaps['static']:.4f}m|{100*lidar_fraction:.2f}%|")


def robot_gap_series(poses, proxies):
    """Interval end timestamps and signed gaps from the same measurement as audit."""
    if len(poses) < 2:
        return [], []
    intervals = pose_intervals(poses, proxies)
    return [row["time"] for row in poses[1:]], robot_human_gaps(*intervals).tolist()
