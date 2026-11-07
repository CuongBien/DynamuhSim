"""Bind and inspect the shared MuJoCo human skins in the standalone demo."""

import xml.etree.ElementTree as ET
from dataclasses import dataclass

import mujoco
import numpy as np

from narrow_nav.visualization.human_skin import append_visuals, bind_pose, bone_name

from .animation import HumanAnimator, rotate


class MujocoVisuals:
    """Bind a standalone animator to its named mocap bodies in an existing MjModel/MjData."""

    def __init__(self, model, data, actors, transition_s=0.25):
        self.model, self.data = model, data
        self.animator = HumanAnimator(actors, transition_s=transition_s)
        self._indices = {}
        for actor_id, character in actors.items():
            indices = np.array(
                [
                    model.body(bone_name(actor_id, bone)).mocapid.item()
                    for bone in range(len(character.parents))
                ]
            )
            if np.any(indices < 0) or len(np.unique(indices)) != len(indices):
                raise ValueError(f"invalid mocap binding for {actor_id}")
            self._indices[actor_id] = indices

    def update(self, human_id, x, y, heading, speed, animation_state, dt):
        pose = self.animator.update(human_id, x, y, heading, speed, animation_state, dt)
        indices = self._indices[human_id]
        self.data.mocap_pos[indices] = pose.bone_positions
        self.data.mocap_quat[indices] = pose.bone_quaternions
        return pose

    def forward(self):
        mujoco.mj_kinematics(self.model, self.data)


@dataclass(frozen=True)
class SoleProbe:
    bone: np.ndarray
    local: np.ndarray
    weight: np.ndarray
    vertex: np.ndarray
    size: int

    def position(self, pose):
        values = pose.bone_positions[self.bone] + rotate(
            pose.bone_quaternions[self.bone], self.local
        )
        result = np.zeros((self.size, 3))
        np.add.at(result, self.vertex, values * self.weight[:, None])
        return result


def sole_probes(character):
    """Use the mesh's weighted sole vertices, with the same linear skinning as MuJoCo."""
    bind_p, bind_q = bind_pose(character)
    probes = []
    for side in range(2):
        points, weights = [], []
        for part_index, labels in character.foot_indices:
            part = character.parts[part_index]
            mask = labels == side
            points.extend(part.vertices[mask])
            weights.extend(part.weights[mask])
        if not points:
            raise ValueError(
                f"{character.identifier}: no sole vertices for foot {side}"
            )
        points, weights = np.asarray(points), np.asarray(weights)
        vertex, bone = np.nonzero(weights > 0)
        inverse = bind_q[bone].copy()
        inverse[:, 1:] *= -1
        local = rotate(inverse, points[vertex] - bind_p[bone])
        probes.append(
            SoleProbe(bone, local, weights[vertex, bone], vertex, len(points))
        )
    return tuple(probes)


def build_preview(actors, width=640, height=480, *, scene_path=None):
    """Compile the supplied visual scene, or the default studio floor."""
    root = (
        ET.parse(scene_path).getroot()
        if scene_path is not None
        else ET.fromstring(
            """<mujoco model="human_visual_preview">
      <statistic center="0 0 0.8" extent="5"/>
      <visual>
        <global offwidth="640" offheight="480"/>
        <headlight ambient="0.65 0.65 0.65" diffuse="0.55 0.55 0.55" specular="0.1 0.1 0.1"/>
      </visual>
      <worldbody>
        <geom type="plane" size="0 0 0.1" rgba="0.29 0.32 0.37 1" contype="0" conaffinity="0"/>
      </worldbody>
    </mujoco>"""
        )
    )
    root.find("visual/global").set("offwidth", str(width))
    root.find("visual/global").set("offheight", str(height))
    vfs = append_visuals(root, actors)
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"), vfs)
    data = mujoco.MjData(model)
    return MujocoVisuals(model, data, actors)
