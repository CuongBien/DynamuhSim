#!/usr/bin/env python3
"""
Unit test for P4 Footprint-Aware Predictive Wall Safety Filter using core.geometry.
Simulates 50 episodes of 40s (800 steps each = 40,000 steps) in a 1.20m corridor.
"""

import os
import sys
import math
import numpy as np

BENCH_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if BENCH_DIR not in sys.path:
    sys.path.insert(0, BENCH_DIR)

from core import (
    apply_footprint_wall_safety_filter,
    check_footprint_wall_contact,
)


def main():
    rng = np.random.RandomState(42)
    wall_contacts = 0
    freezes = 0
    total_episodes = 50

    for ep in range(total_episodes):
        rx = -13.0
        ry = 0.0
        ryaw = 0.0
        dt = 0.05

        current_cmd = (0.25, 0.0, 0.0)
        next_resample = 0.0

        ep_wall_contact = False
        ep_frozen_dur = 0.0
        ep_frozen = False

        for t_step in range(800):
            t = t_step * dt
            if t >= next_resample:
                next_resample = t + rng.uniform(0.5, 1.5)
                vx_cmd = rng.uniform(-0.10, 0.50)
                vy_cmd = rng.uniform(-0.30, 0.30)
                wz_cmd = rng.uniform(-1.00, 1.00)
                current_cmd = (vx_cmd, vy_cmd, wz_cmd)

            vx_c, vy_c, wz_c = current_cmd
            vx, vy, wz = apply_footprint_wall_safety_filter(rx, ry, ryaw, vx_c, vy_c, wz_c)

            # Integrate motion
            ryaw += wz * dt
            vx_w = vx * math.cos(ryaw) - vy * math.sin(ryaw)
            vy_w = vx * math.sin(ryaw) + vy * math.cos(ryaw)
            rx += vx_w * dt
            ry += vy_w * dt

            if check_footprint_wall_contact(ry, ryaw, 0.60):
                ep_wall_contact = True

            # Check freeze: v < 0.05 for >= 3.0s (60 steps)
            spd = math.hypot(vx, vy)
            if spd < 0.05:
                ep_frozen_dur += dt
                if ep_frozen_dur >= 3.0:
                    ep_frozen = True
            else:
                ep_frozen_dur = 0.0

        if ep_wall_contact:
            wall_contacts += 1
        if ep_frozen:
            freezes += 1

    print(f"Total episodes: {total_episodes}")
    print(f"Wall contacts: {wall_contacts} ({wall_contacts/total_episodes*100:.1f}%)")
    print(f"Freezes: {freezes} ({freezes/total_episodes*100:.1f}%)")


if __name__ == "__main__":
    main()
