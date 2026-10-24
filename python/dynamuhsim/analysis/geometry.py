"""Geometric measurements; acceptance thresholds belong to audit policies."""
import math


def pose_errors(pose, goal):
    xy = math.hypot(pose["x"] - goal[0], pose["y"] - goal[1])
    delta = pose["yaw"] - goal[2]
    return xy, abs(math.atan2(math.sin(delta), math.cos(delta)))


def swept_disc_gap(a0, a1, radius_a, b0, b1, radius_b):
    """Signed closest gap over one linear interval, including endpoints."""
    if radius_a < 0 or radius_b < 0:
        raise ValueError("Negative disc radius")
    p = (a0[0] - b0[0], a0[1] - b0[1])
    d = (a1[0] - a0[0] - b1[0] + b0[0], a1[1] - a0[1] - b1[1] + b0[1])
    den = d[0]**2 + d[1]**2
    fraction = min(1., max(0., -(p[0]*d[0] + p[1]*d[1])/den)) if den > 1e-25 else 0.
    return math.hypot(p[0] + fraction*d[0], p[1] + fraction*d[1]) - radius_a - radius_b
