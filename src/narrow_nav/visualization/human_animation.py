"""Quaternion sampling, speed-aware playback and externally positioned actors."""

from dataclasses import dataclass
from math import cos, isfinite, sin

import numpy as np


def normalize(quaternions):
    norms = np.linalg.norm(quaternions, axis=-1, keepdims=True)
    if not np.all(np.isfinite(norms) & (norms > 1e-12)):
        raise ValueError("invalid quaternion")
    return quaternions / norms


def multiply(a, b):
    aw, av = a[..., :1], a[..., 1:]
    bw, bv = b[..., :1], b[..., 1:]
    return np.concatenate(
        (
            aw * bw - np.sum(av * bv, axis=-1, keepdims=True),
            aw * bv + bw * av + np.cross(av, bv),
        ),
        axis=-1,
    )


def rotate(q, vector):
    uv = 2 * np.cross(q[..., 1:], vector)
    return vector + q[..., :1] * uv + np.cross(q[..., 1:], uv)


def slerp(a, b, fraction):
    """Shortest-arc SLERP for unit wxyz quaternions, including antipodes."""
    dot = np.sum(a * b, axis=-1, keepdims=True)
    b = np.where(dot < 0, -b, b)
    angle = np.arccos(np.clip(np.abs(dot), 0.0, 1.0))
    sine = np.sin(angle)
    linear = angle < 1e-5
    divisor = np.where(linear, 1.0, sine)
    left = np.where(linear, 1 - fraction, np.sin((1 - fraction) * angle) / divisor)
    right = np.where(linear, fraction, np.sin(fraction * angle) / divisor)
    return normalize(left * a + right * b)


def smoothstep(fraction):
    return fraction * fraction * (3 - 2 * fraction)


@dataclass(frozen=True)
class Clip:
    times: np.ndarray
    positions: np.ndarray
    rotations: np.ndarray
    loop: bool
    reference_speed_mps: float | None
    seam_blend_s: float = 0.12

    @property
    def duration(self):
        return float(self.times[-1])

    def sample(self, seconds):
        phase = (
            seconds % self.duration
            if self.loop
            else min(max(0, seconds), self.duration)
        )
        left = int(
            np.clip(
                np.searchsorted(self.times, phase, side="right") - 1,
                0,
                len(self.times) - 2,
            )
        )
        fraction = (phase - self.times[left]) / (
            self.times[left + 1] - self.times[left]
        )
        position = (1 - fraction) * self.positions[left] + fraction * self.positions[
            left + 1
        ]
        rotation = slerp(self.rotations[left], self.rotations[left + 1], fraction)
        seam = min(self.seam_blend_s, self.duration / 4)
        if self.loop and seam > 0 and phase > self.duration - seam:
            mix = smoothstep((phase - self.duration + seam) / seam)
            position = (1 - mix) * position + mix * self.positions[0]
            rotation = slerp(rotation, self.rotations[0], mix)
        return position, rotation


@dataclass(frozen=True)
class VisualPose:
    human_id: str
    root_xyz: tuple[float, float, float]
    heading: float
    bone_positions: np.ndarray
    bone_quaternions: np.ndarray


@dataclass
class _Actor:
    character: object
    clip_name: str | None = None
    phase: float = 0.0
    blend_age: float = 0.0
    blend_from: tuple[np.ndarray, np.ndarray] | None = None
    local_pose: tuple[np.ndarray, np.ndarray] | None = None


_STATES = {
    "idle",
    "walk",
    "slow_walk",
    "run",
    "turn",
    "walk_start",
    "walk_stop",
    "backward_walk",
}


def forward_kinematics(positions, quaternions, parents, levels, origin, heading):
    world_p = np.empty_like(positions, dtype=np.float64)
    world_q = np.empty_like(quaternions, dtype=np.float64)
    yaw = np.array([cos(heading / 2), 0.0, 0.0, sin(heading / 2)])
    root = levels[0]
    world_p[root] = rotate(yaw, positions[root]) + origin
    world_q[root] = multiply(yaw, quaternions[root])
    for indices in levels[1:]:
        p = parents[indices]
        world_p[indices] = world_p[p] + rotate(world_q[p], positions[indices])
        world_q[indices] = multiply(world_q[p], quaternions[indices])
    return world_p, normalize(world_q)


class HumanAnimator:
    """Stateful visual controller. All coordinates are metres/radians, Z up and +X forward."""

    def __init__(self, characters_by_id, transition_s=0.25):
        if not isfinite(transition_s) or transition_s < 0:
            raise ValueError("transition_s must be finite and nonnegative")
        self._actors = {
            actor_id: _Actor(character)
            for actor_id, character in characters_by_id.items()
        }
        self._transition_s = transition_s

    def update(self, human_id, x, y, heading, speed, animation_state, dt):
        if not all(map(isfinite, (x, y, heading, speed, dt))) or min(speed, dt) < 0:
            raise ValueError(
                "pose and dt must be finite; speed and dt must be nonnegative"
            )
        if animation_state not in _STATES:
            raise ValueError(f"unsupported animation state {animation_state!r}")
        actor = self._actors[human_id]
        clips = actor.character.clips
        fallback = {
            "slow_walk": "walk",
            "walk_start": "walk",
            "walk_stop": "idle",
            "turn": "walk" if speed > 0.05 else "idle",
        }
        name = (
            animation_state
            if animation_state in clips
            else fallback.get(animation_state)
        )
        if name not in clips:
            raise ValueError(
                f"{actor.character.identifier}: missing animation {animation_state!r}"
            )
        clip = clips[name]
        if name != actor.clip_name:
            actor.blend_from = actor.local_pose
            actor.clip_name = name
            actor.phase = actor.blend_age = 0.0
        rate = (
            min(2.5, max(0.35, speed / clip.reference_speed_mps))
            if clip.reference_speed_mps
            else 1.0
        )
        actor.phase += dt * rate
        actor.blend_age += dt
        position, rotation = clip.sample(actor.phase)
        if actor.blend_from is not None and actor.blend_age < self._transition_s:
            mix = smoothstep(actor.blend_age / self._transition_s)
            before_p, before_q = actor.blend_from
            position = (1 - mix) * before_p + mix * position
            rotation = slerp(before_q, rotation, mix)
        else:
            actor.blend_from = None
        actor.local_pose = position, rotation
        origin = (x, y, 0.0)
        p, q = forward_kinematics(
            position,
            rotation,
            actor.character.parents,
            actor.character.levels,
            origin,
            heading,
        )
        return VisualPose(human_id, origin, heading, p, q)
