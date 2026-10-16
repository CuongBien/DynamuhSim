"""
Geometric utilities and footprint-aware predictive wall safety filter
for Mecanum robot (0.24m x 0.20m) in corridor environments.
"""

import math
from typing import Tuple
import numpy as np


def compute_rect_circle_distance(
    rx: float, ry: float, ryaw: float,
    hx: float, hy: float,
    rw: float = 0.20, rl: float = 0.24, hr: float = 0.28
) -> Tuple[float, float]:
    """Computes Euclidean center distance and footprint edge-to-edge clearance."""
    dx = hx - rx
    dy = hy - ry
    d_center = math.hypot(dx, dy)

    cos_y = math.cos(ryaw)
    sin_y = math.sin(ryaw)
    x_rel = dx * cos_y + dy * sin_y
    y_rel = -dx * sin_y + dy * cos_y

    half_l = rl / 2.0  # 0.12 m
    half_w = rw / 2.0  # 0.10 m

    x_cl = max(-half_l, min(half_l, x_rel))
    y_cl = max(-half_w, min(half_w, y_rel))
    d_rect = math.hypot(x_rel - x_cl, y_rel - y_cl)
    d_clear = d_rect - hr

    return d_center, d_clear


def check_footprint_wall_contact(
    ry: float, ryaw: float,
    corridor_half_width: float = 0.60,
    half_l: float = 0.12, half_w: float = 0.10
) -> bool:
    """Checks if any corner of the rotated rectangular footprint touches or exceeds the corridor walls."""
    cos_yaw = math.cos(ryaw)
    sin_yaw = math.sin(ryaw)
    corners_y = [
        ry + dx * sin_yaw + dy * cos_yaw
        for dx, dy in [(-half_l, -half_w), (-half_l, half_w), (half_l, -half_w), (half_l, half_w)]
    ]
    return max(abs(cy) for cy in corners_y) >= corridor_half_width


def apply_footprint_wall_safety_filter(
    rx: float, ry: float, ryaw: float,
    cmd_vx: float, cmd_vy: float, cmd_wz: float,
    corridor_half_width: float = 0.60,
    margin_safe: float = 0.055
) -> Tuple[float, float, float]:
    """
    Rigorously accounts for the rectangular footprint (0.24m x 0.20m) and its rotation.
    1. Strict yaw limit: within +/- 45 deg (+/- 0.7854 rad) around corridor axis (yaw = 0).
    2. Clamps vx based on global yaw to prevent driving into walls.
    3. Active lateral push-back towards corridor centerline with freeze avoidance.
    """
    vx = float(cmd_vx)
    vy = float(cmd_vy)
    wz = float(cmd_wz)

    # 1. Yaw constraint: strictly within +/- 45 deg (+/- 0.7854 rad) around corridor axis (yaw = 0)
    max_yaw = math.pi / 4.0  # 0.7854 rad (45 deg)
    yaw_margin = 0.10  # start steering back at ~39 deg (0.68 rad)

    if ryaw > (max_yaw - yaw_margin):
        wz = min(wz, -1.0 * (ryaw - (max_yaw - yaw_margin)) / yaw_margin - 0.2)
    elif ryaw < -(max_yaw - yaw_margin):
        wz = max(wz, 1.0 * (-ryaw - (max_yaw - yaw_margin)) / yaw_margin + 0.2)

    pred_yaw = ryaw + wz * 0.10
    if pred_yaw > max_yaw:
        wz = min(wz, (max_yaw - ryaw) / 0.10)
    elif pred_yaw < -max_yaw:
        wz = max(wz, (-max_yaw - ryaw) / 0.10)

    wz = float(np.clip(wz, -1.20, 1.20))

    # 2. Footprint corners check
    l, w = 0.12, 0.10
    body_corners = [(-l, -w), (-l, w), (l, -w), (l, w)]
    max_allowed_corner_y = corridor_half_width - margin_safe  # 0.60 - 0.055 = 0.545 m

    horizons = [0.05, 0.10, 0.20, 0.35]

    for _ in range(6):
        violates_left = False
        violates_right = False

        for dt in horizons:
            yp = ryaw + wz * dt
            vx_w = vx * math.cos(ryaw) - vy * math.sin(ryaw)
            vy_w = vx * math.sin(ryaw) + vy * math.cos(ryaw)
            x_pred = rx + vx_w * dt
            y_pred = ry + vy_w * dt

            cos_yp = math.cos(yp)
            sin_yp = math.sin(yp)

            for cx, cy in body_corners:
                cp_y = y_pred + cx * sin_yp + cy * cos_yp
                if cp_y > max_allowed_corner_y:
                    violates_left = True
                if cp_y < -max_allowed_corner_y:
                    violates_right = True

        if violates_left or ry > 0.35:
            # Pushing towards left wall (+y):
            # If yaw > 0, forward vx drives towards wall -> clamp vx!
            if ryaw > 0.05:
                vx = min(vx, 0.05)
                wz = min(wz, -0.80)
            else:
                wz = min(wz, -0.30)
            # Active lateral push back towards center
            vy = min(vy, -0.20)
            # When aligned along corridor, maintain forward drive to avoid freezing
            if abs(ryaw) < 0.25 and ry < 0.40:
                vx = max(vx, 0.20)
        elif violates_right or ry < -0.35:
            # Pushing towards right wall (-y):
            if ryaw < -0.05:
                vx = min(vx, 0.05)
                wz = max(wz, 0.80)
            else:
                wz = max(wz, 0.30)
            vy = max(vy, 0.20)
            if abs(ryaw) < 0.25 and ry > -0.40:
                vx = max(vx, 0.20)
        else:
            break

    # Actuator limits
    vx = float(np.clip(vx, -0.10, 0.50))
    vy = float(np.clip(vy, -0.35, 0.35))
    wz = float(np.clip(wz, -1.20, 1.20))

    # Longitudinal safety bounds
    if rx >= -6.3 and vx > 0:
        vx = 0.0
    elif rx <= -14.2 and vx < 0:
        vx = 0.0

    return (float(vx), float(vy), float(wz))
