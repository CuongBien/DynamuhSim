"""Bind visual bones to simulator-owned actor poses and time."""

from math import atan2

import mujoco
import numpy as np

from narrow_nav.visualization.human_animation import HumanAnimator
from narrow_nav.visualization.human_skin import SkinActor, bone_name


class HumanVisuals:
    def __init__(self, model: mujoco.MjModel, actors: tuple[SkinActor, ...]) -> None:
        self._actors = tuple(
            (
                actor,
                model.body(actor.actor_id).id,
                np.array(
                    [
                        model.body(bone_name(actor.actor_id, i)).mocapid.item()
                        for i in range(len(actor.character.parents))
                    ],
                    dtype=np.intp,
                ),
            )
            for actor in actors
            if not actor.fixed
        )
        if any(np.any(indices < 0) for _, _, indices in self._actors):
            raise ValueError("human skin bones must be mocap bodies")
        self._animator = None

    def reset_animation(self) -> None:
        self._animator = HumanAnimator(
            {actor.actor_id: actor.character for actor, _, _ in self._actors}
        )

    def update(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        time_s: float,
        *,
        speeds_mps: dict[str, float] | None = None,
        dt_s: float = 0.0,
    ) -> None:
        if not self._actors:
            return
        for actor, body_id, indices in self._actors:
            x, y = data.xpos[body_id, :2]
            orientation = data.xmat[body_id]
            yaw = atan2(orientation[3], orientation[0])
            if speeds_mps is None:
                positions, rotations = actor.sample_idle(x, y, yaw, time_s)
            else:
                speed = speeds_mps[actor.actor_id]
                pose = self._animator.update(
                    actor.actor_id,
                    x,
                    y,
                    yaw,
                    speed,
                    "walk" if speed > 0.05 else "idle",
                    dt_s,
                )
                positions, rotations = pose.bone_positions, pose.bone_quaternions
            data.mocap_pos[indices] = positions
            data.mocap_quat[indices] = rotations
        mujoco.mj_kinematics(model, data)
