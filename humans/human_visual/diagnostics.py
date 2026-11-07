"""Foot probe diagnostics on actual skinned vertices."""

from dataclasses import dataclass, field
from math import cos, sin

import numpy as np


@dataclass
class FootDiagnostics:
    min_sole_z_m: float = float("inf")
    stance_speeds: list[float] = field(default_factory=list)
    near_ground_speeds: list[float] = field(default_factory=list)
    _previous: dict = field(default_factory=dict)

    def update(self, actor_id, side, vertices, speed, dt, root_xy, heading):
        lowest = float(vertices[:, 2].min())
        self.min_sole_z_m = min(self.min_sole_z_m, lowest)
        center = vertices.mean(axis=0)
        x, y = center[:2] - root_xy
        forward = cos(heading) * x + sin(heading) * y
        key = actor_id, side
        previous = self._previous.get(key)
        self._previous[key] = lowest, center, forward
        # Measure only moving-root stance frames; require consecutive near-ground samples.
        if (
            previous is not None
            and speed >= 0.2
            and dt > 0
            and max(lowest, previous[0]) <= 0.04
        ):
            delta = center - previous[1]
            if abs(delta[2] / dt) <= 0.12:
                world_speed = float(np.linalg.norm(delta[:2]) / dt)
                self.near_ground_speeds.append(world_speed)
                if (forward - previous[2]) / dt < -0.05:
                    self.stance_speeds.append(world_speed)

    def summary(self):
        speed = self.stance_speeds
        return {
            "min_sole_z_m": self.min_sole_z_m,
            "max_penetration_m": max(0.0, -self.min_sole_z_m),
            "stance_samples": len(speed),
            "stance_sliding_median_mps": float(np.median(speed)) if speed else None,
            "stance_sliding_p95_mps": float(np.percentile(speed, 95))
            if speed
            else None,
            "near_ground_samples": len(self.near_ground_speeds),
            "near_ground_sliding_p95_mps": (
                float(np.percentile(self.near_ground_speeds, 95))
                if self.near_ground_speeds
                else None
            ),
        }
