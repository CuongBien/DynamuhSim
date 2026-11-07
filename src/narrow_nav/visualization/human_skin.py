"""Assemble visual-only MJCF skins from normalized character bundles."""

import re
from dataclasses import dataclass
from hashlib import sha256
from math import cos, sin, sqrt
from xml.etree import ElementTree as ET

import numpy as np

from .human_animation import forward_kinematics, multiply, normalize, rotate
from .human_bundle import Character


@dataclass(frozen=True, slots=True)
class SkinActor:
    actor_id: str
    character: Character
    initial_pose: tuple[float, float, float]
    fixed: bool = False
    seat_height_m: float | None = None
    idle_offset_s: float = 0.0

    def initial_bones(self):
        if self.seat_height_m is None:
            return self.sample_idle(*self.initial_pose, 0.0)
        return _seated_bones(self)

    def sample_idle(self, x: float, y: float, yaw: float, time_s: float):
        position, rotation = self.character.clips["idle"].sample(
            time_s + self.idle_offset_s
        )
        return forward_kinematics(
            position,
            rotation,
            self.character.parents,
            self.character.levels,
            (x, y, 0.0),
            yaw,
        )


def _align_segment(previous, target):
    a, b = previous / np.linalg.norm(previous), target / np.linalg.norm(target)
    return normalize(np.array([1 + np.dot(a, b), *np.cross(a, b)]))


def _seated_bones(actor: SkinActor):
    """Pose the supported Quaternius rig on a bench, keeping the shoes grounded."""
    positions, rotations = actor.sample_idle(0, 0, 0, 0)
    original = positions.copy()
    bone = {
        name: index for index, name in enumerate(actor.character.metadata["bone_names"])
    }
    hip_z = (positions[bone["UpperLeg.L"], 2] + positions[bone["UpperLeg.R"], 2]) / 2
    positions += (0.08, 0, actor.seat_height_m + 0.07 - hip_z)
    for side in ("L", "R"):
        upper, lower, ankle, shoe, toe = (
            bone[f"{name}.{side}{suffix}"]
            for name, suffix in (
                ("UpperLeg", ""),
                ("LowerLeg", ""),
                ("LowerLeg", "_end"),
                ("Foot", ""),
                ("Foot", "_end"),
            )
        )
        thigh = original[lower] - original[upper]
        shin = original[ankle] - original[lower]
        thigh_target = np.linalg.norm(thigh) * np.array([cos(0.54), 0, -sin(0.54)])
        positions[lower] = positions[upper] + thigh_target
        length = np.linalg.norm(shin)
        drop = np.clip(positions[lower, 2] - original[ankle, 2], 0, length)
        shin_target = np.array([sqrt(max(0, np.dot(shin, shin) - drop**2)), 0, -drop])
        positions[ankle] = positions[lower] + shin_target
        positions[shoe] = positions[ankle] + original[shoe] - original[ankle]
        positions[toe] = positions[shoe] + original[toe] - original[shoe]
        rotations[upper] = multiply(
            _align_segment(thigh, thigh_target), rotations[upper]
        )
        shin_rotation = _align_segment(shin, shin_target)
        rotations[[lower, ankle]] = multiply(shin_rotation, rotations[[lower, ankle]])
    x, y, yaw = actor.initial_pose
    heading = np.array([cos(yaw / 2), 0, 0, sin(yaw / 2)])
    return rotate(heading, positions) + (x, y, 0), normalize(
        multiply(heading, rotations)
    )


def _numbers(array):
    return " ".join(format(value, ".8g") for value in np.asarray(array).flat)


def bone_name(actor_id, bone):
    return f"hv/{actor_id}/bone_{bone}"


def _section(root, name):
    section = root.find(name)
    return section if section is not None else ET.SubElement(root, name)


def bind_pose(character):
    return forward_kinematics(
        character.rest_p,
        character.rest_q,
        character.parents,
        character.levels,
        (0.0, 0.0, 0.0),
        0.0,
    )


def append_visuals(root, actors, *, initial_poses=None):
    """Append namespaced MJCF skins. Pass the returned VFS files when compiling the model."""
    world = _section(root, "worldbody")
    assets = _section(root, "asset")
    deformable = _section(root, "deformable")
    materials = {}
    textures = {}
    vfs = {}
    for actor_id, character in actors.items():
        if not isinstance(actor_id, str) or not re.fullmatch(
            r"[A-Za-z0-9_-]+", actor_id
        ):
            raise ValueError(
                "human_id must be a nonempty identifier using letters, digits, _ or -"
            )
        bind_pos, bind_rot = bind_pose(character)
        positions, rotations = (
            initial_poses[actor_id]
            if initial_poses is not None
            else (bind_pos, bind_rot)
        )
        for bone, (pos, quat) in enumerate(zip(positions, rotations, strict=True)):
            ET.SubElement(
                world,
                "body",
                name=bone_name(actor_id, bone),
                mocap="true",
                pos=_numbers(pos),
                quat=_numbers(quat),
            )
        for part_index, part in enumerate(character.parts):
            texture_name = ""
            if part.texture:
                digest = sha256(part.texture).hexdigest()
                if digest not in textures:
                    texture_name = f"hv_texture_{len(textures)}"
                    textures[digest] = texture_name
                    file_name = f"{texture_name}.png"
                    vfs[file_name] = part.texture
                    ET.SubElement(
                        assets, "texture", name=texture_name, file=file_name, type="2d"
                    )
                texture_name = textures[digest]
            material_key = (part.rgba, texture_name)
            if material_key not in materials:
                name = f"hv_material_{len(materials)}"
                materials[material_key] = name
                attributes = {
                    "name": name,
                    "rgba": _numbers(part.rgba),
                    "specular": "0.08",
                }
                if texture_name:
                    attributes["texture"] = texture_name
                ET.SubElement(assets, "material", attributes)
            skin = ET.SubElement(
                deformable,
                "skin",
                name=f"hv/{actor_id}/skin_{part_index}",
                material=materials[material_key],
                vertex=_numbers(part.vertices),
                face=_numbers(part.faces),
                texcoord=_numbers(part.uv),
                group="0",
            )
            for bone, column in enumerate(part.weights.T):
                indices = np.flatnonzero(column > 0)
                if len(indices):
                    ET.SubElement(
                        skin,
                        "bone",
                        body=bone_name(actor_id, bone),
                        bindpos=_numbers(bind_pos[bone]),
                        bindquat=_numbers(bind_rot[bone]),
                        vertid=_numbers(indices),
                        vertweight=_numbers(column[indices]),
                    )
    return vfs
