"""Bind visual bones to simulator-owned actor poses and time."""

from math import atan2

import mujoco
import numpy as np

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

    def update(self, model: mujoco.MjModel, data: mujoco.MjData, time_s: float) -> None:
        if not self._actors:
            return
        for actor, body_id, indices in self._actors:
            x, y = data.xpos[body_id, :2]
            orientation = data.xmat[body_id]
            yaw = atan2(orientation[3], orientation[0])
            positions, rotations = actor.sample_idle(x, y, yaw, time_s)
            data.mocap_pos[indices] = positions
            data.mocap_quat[indices] = rotations
        mujoco.mj_kinematics(model, data)
