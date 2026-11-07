"""Authored deterministic root tracks for the animation test fixture."""

from dataclasses import dataclass
from math import cos, pi, sin

import numpy as np


@dataclass(frozen=True)
class Command:
    human_id: str
    x: float
    y: float
    heading: float
    speed: float
    state: str


class Replay:
    def __init__(self, spec, duration, agents, seed):
        self.spec = spec
        self.duration = duration
        self.ids = tuple(f"human_{index + 1:02d}" for index in range(agents))
        self.position = np.zeros((agents, 2))
        rng = np.random.default_rng(seed)
        self.position[:2, 0] = (-0.20, 0.20)[:agents]
        self.position[:2, 1] = (-0.72, 0.72)[:agents]
        self._ensemble_jitter = rng.uniform(
            -spec["ensemble_speed_jitter_mps"],
            spec["ensemble_speed_jitter_mps"],
            agents,
        )
        self._offset = rng.uniform(-0.35, 0.35, agents)
        self._speed_keys = np.asarray(spec["speed_keys"]).T
        self._heading_keys = np.asarray(spec["heading_keys_rad"]).T
        self.stage = -1
        self._elapsed = 0.0

    def update(self, seconds, dt):
        if dt < 0 or seconds < self._elapsed or seconds > self.duration:
            raise ValueError(
                "fixture time must advance monotonically within its duration"
            )
        self._elapsed = seconds
        reference = seconds * 150 / self.duration
        stage = min(4, int(reference / 30))
        local = reference - 30 * stage
        heading = 0.0
        speed = self.spec["straight_speed_mps"]
        state = "walk"
        if stage == 1:
            speed = float(np.interp(local, *self._speed_keys))
            state = "slow_walk" if speed < 0.8 else "walk"
        elif stage == 2:
            heading = float(np.interp(local, *self._heading_keys))
            state = "turn"
        elif stage == 3:
            cycle = local % 10
            if local < 0.9:
                speed, state = speed * (1 - local / 0.9), "walk_stop"
            elif cycle < 2.0 or cycle >= 8.0:
                speed, state = 0.0, "idle"
            elif cycle < 3.0:
                speed, state = speed * (cycle - 2.0), "walk_start"
            elif cycle >= 7.0:
                speed, state = speed * (8.0 - cycle), "walk_stop"
        if stage == 4 and self.stage != stage and len(self.ids) > 2:
            center_x, center_y = self.position[:2].mean(axis=0)
            for index in range(2, len(self.ids)):
                side = 1 if index % 2 else -1
                lane = 2 + (index - 2) // 2
                self.position[index] = (
                    center_x + self._offset[index],
                    center_y + side * lane * self.spec["lane_spacing_m"],
                )
        self.stage = stage
        active = len(self.ids) if stage == 4 else min(2, len(self.ids))
        commands = []
        for index in range(active):
            velocity = (
                speed
                if stage != 4
                else self.spec["ensemble_speed_mps"] + self._ensemble_jitter[index]
            )
            self.position[index] += (
                dt * velocity * cos(heading),
                dt * velocity * sin(heading),
            )
            x, y = self.position[index]
            commands.append(
                Command(
                    self.ids[index], float(x), float(y), heading, float(velocity), state
                )
            )
        return tuple(commands)


class CorridorReplay:
    """Constant-speed visual fixture along two lanes joined by semicircles."""

    straight_m = 7.6
    turn_radius_m = 0.95
    speed_mps = 1.15
    stage = 0

    def __init__(self, agents, seed):
        self.ids = tuple(f"human_{index + 1:02d}" for index in range(agents))
        self._turn_m = pi * self.turn_radius_m
        self._lap_m = 2 * (self.straight_m + self._turn_m)
        spacing = self._lap_m / agents
        phase = np.random.default_rng(seed).uniform(0, spacing)
        self._offsets = tuple(phase + index * spacing for index in range(agents))

    def update(self, seconds, _dt):
        commands = []
        for human_id, offset in zip(self.ids, self._offsets, strict=True):
            distance = (offset + self.speed_mps * seconds) % self._lap_m
            # The second half is the first half rotated 180 degrees about the origin.
            direction = 1 if distance < self._lap_m / 2 else -1
            distance %= self._lap_m / 2
            half_straight = self.straight_m / 2
            radius = self.turn_radius_m
            heading = 0.0 if direction == 1 else pi
            if distance < self.straight_m:
                x, y, state = distance - half_straight, -radius, "walk"
            else:
                angle = (distance - self.straight_m) / radius
                x = half_straight + radius * sin(angle)
                y = -radius * cos(angle)
                heading += angle
                state = "turn"
            commands.append(
                Command(
                    human_id,
                    direction * x,
                    direction * y,
                    heading,
                    self.speed_mps,
                    state,
                )
            )
        return tuple(commands)
