"""Timestamp alignment with explicit tolerances and no silent extrapolation."""
from bisect import bisect_left
import math


def ordered(stamps, *, strict=False):
    if any(not math.isfinite(t) for t in stamps) or any(b <= a if strict else b < a for a, b in zip(stamps, stamps[1:])):
        raise ValueError("Expected finite ordered timestamps")


def nearest_index(stamps, query, tolerance):
    """Earlier sample wins an equal-distance tie. None means unavailable."""
    if tolerance < 0 or not math.isfinite(tolerance) or not math.isfinite(query):
        raise ValueError("Invalid timestamp query/tolerance")
    if not stamps:
        return None
    index = bisect_left(stamps, query)
    candidates = [i for i in [index - 1, index] if 0 <= i < len(stamps)]
    nearest = min(candidates, key=lambda i: (abs(stamps[i] - query), i))
    return nearest if abs(stamps[nearest] - query) <= tolerance else None


def interpolate_se2(stamps, poses, query, max_gap):
    """XY linear interpolation and shortest yaw arc; invalid gaps return None."""
    ordered(stamps, strict=True)
    if len(stamps) != len(poses) or max_gap < 0 or not math.isfinite(max_gap) or not math.isfinite(query):
        raise ValueError("Invalid SE2 timeline")
    if not stamps or query < stamps[0] or query > stamps[-1]:
        return None
    high = bisect_left(stamps, query)
    if stamps[high] == query:
        return tuple(poses[high])
    low = high - 1
    gap = stamps[high] - stamps[low]
    if gap > max_gap:
        return None
    weight = (query - stamps[low]) / gap
    a, b = poses[low], poses[high]
    angle = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
    yaw = a[2] + weight * angle
    return a[0] + weight * (b[0] - a[0]), a[1] + weight * (b[1] - a[1]), math.atan2(math.sin(yaw), math.cos(yaw))
