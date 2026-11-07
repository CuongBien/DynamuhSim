"""Offline Blender FBX/GLTF extraction. The generated runtime has no Blender dependency."""

import json
import sys
from argparse import ArgumentParser
from hashlib import sha256
from math import atan2, pi
from pathlib import Path
from tempfile import TemporaryDirectory

import bpy
import numpy as np
from mathutils import Quaternion, Vector

ROOT = Path(__file__).resolve().parent
_MISSING_STATES = ("slow_walk", "turn", "walk_start", "walk_stop", "backward_walk")


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def load_source(path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    if path.suffix.lower() == ".fbx":
        bpy.ops.import_scene.fbx(
            filepath=str(path), use_anim=True, use_image_search=True
        )
    elif path.suffix.lower() in (".glb", ".gltf"):
        bpy.ops.import_scene.gltf(filepath=str(path))
    else:
        raise ValueError(f"expected FBX, GLB or GLTF, got {path.name}")
    armatures = [obj for obj in bpy.context.scene.objects if obj.type == "ARMATURE"]
    if len(armatures) != 1:
        raise ValueError("expected exactly one source armature")
    armature = armatures[0]
    meshes = [
        obj
        for obj in bpy.context.scene.objects
        if obj.type == "MESH"
        and (
            obj.parent == armature
            or any(
                mod.type == "ARMATURE" and mod.object == armature
                for mod in obj.modifiers
            )
        )
    ]
    if not meshes:
        raise ValueError("no original skinned mesh found on the source armature")
    return armature, meshes


def local_transforms(points, rotations, parents):
    position, orientation = [], []
    for index, parent in enumerate(parents):
        if parent < 0:
            position.append(tuple(points[index]))
            orientation.append(tuple(rotations[index].normalized()))
            continue
        inverse = rotations[parent].inverted()
        position.append(tuple(inverse @ (points[index] - points[parent])))
        orientation.append(tuple((inverse @ rotations[index]).normalized()))
    return np.asarray(position), np.asarray(orientation)


def material_data(material, stage, index):
    color = tuple(material.diffuse_color) if material else (0.7, 0.7, 0.7, 1.0)
    texture = None
    if material and material.use_nodes:
        shader = next(
            (
                node
                for node in material.node_tree.nodes
                if node.type == "BSDF_PRINCIPLED"
            ),
            None,
        )
        if shader:
            base = shader.inputs["Base Color"]
            color = tuple(base.default_value)
            if base.is_linked:
                node = base.links[0].from_node
                if node.type != "TEX_IMAGE" or not node.image:
                    raise ValueError(
                        f"unsupported base-color shader on {material.name}"
                    )
                if node.image.size[0] == 0:
                    raise ValueError(f"missing source texture on {material.name}")
                texture = f"texture_{index}.png"
                node.image.filepath_raw = str(stage / texture)
                node.image.file_format = "PNG"
                node.image.save()
                color = (1.0, 1.0, 1.0, 1.0)
    return {
        "name": material.name if material else "Default",
        "rgba": [min(1.0, max(0.0, float(value))) for value in color],
        "texture": texture,
        "texture_sha256": digest(stage / texture) if texture else None,
    }


def mesh_parts(mesh, normalized_vertices, names, foot_columns, arrays, parts, stage):
    data = mesh.data
    if len(data.color_attributes):
        raise ValueError(f"{mesh.name}: vertex-color baking is not supported")
    data.calc_loop_triangles()
    groups = {group.index: names.get(group.name) for group in mesh.vertex_groups}
    weights = np.zeros((len(data.vertices), len(names)), dtype=np.float32)
    for vertex in data.vertices:
        for group in vertex.groups:
            bone = groups.get(group.group)
            if bone is not None:
                weights[vertex.index, bone] += group.weight
    for material_index in sorted(
        {triangle.material_index for triangle in data.loop_triangles}
    ):
        if mesh.material_slots:
            material = mesh.material_slots[material_index].material
        else:
            material = None
        index = len(parts)
        parts.append(material_data(material, stage, index))
        uv_layer = data.uv_layers.active
        if parts[-1]["texture"] and uv_layer is None:
            raise ValueError(f"{mesh.name}: source texture has no UV coordinates")
        vertices, texcoords, original, faces, lookup = [], [], [], [], {}
        for triangle in data.loop_triangles:
            if triangle.material_index != material_index:
                continue
            face = []
            for loop, vertex in zip(triangle.loops, triangle.vertices, strict=True):
                uv = tuple(uv_layer.data[loop].uv) if uv_layer else (0.0, 0.0)
                key = (vertex, uv)
                if key not in lookup:
                    lookup[key] = len(vertices)
                    vertices.append(tuple(normalized_vertices[vertex]))
                    texcoords.append(uv)
                    original.append(vertex)
                face.append(lookup[key])
            faces.append(face)
        points = np.asarray(vertices, dtype=np.float32)
        influences = weights[original]
        totals = influences.sum(axis=1, keepdims=True)
        if np.any(totals <= 1e-8):
            raise ValueError(
                f"{mesh.name}: unweighted source vertices in material {parts[-1]['name']}"
            )
        influences /= totals
        foot_weights = np.column_stack(
            [influences[:, cols].sum(axis=1) for cols in foot_columns]
        )
        feet = np.where(
            (points[:, 2] < 0.08) & (foot_weights.max(axis=1) >= 0.20),
            foot_weights.argmax(axis=1),
            -1,
        )
        for name, array in (
            ("verts", points),
            ("faces", np.asarray(faces, dtype=np.int32)),
            ("uv", np.asarray(texcoords, dtype=np.float32)),
            ("weights", influences),
            ("feet", feet.astype(np.int8)),
        ):
            arrays[f"{name}_{index}"] = array


def animation_data(
    armature,
    bones,
    parents,
    frame_basis,
    normalize_point,
    rest_points,
    rest_rot,
    config,
    arrays,
):
    clips = {}
    scene = bpy.context.scene
    fps = scene.render.fps / scene.render.fps_base
    available = list(bpy.data.actions)
    anchor = next(
        index for index, bone in enumerate(bones) if bone.name == config["motion_bone"]
    )
    anim = armature.animation_data_create()
    anim.use_nla = False
    for name, action_name in config["clips"].items():
        matches = [
            action
            for action in available
            if action.name == action_name or action.name.split("|")[-1] == action_name
        ]
        if len(matches) != 1:
            raise ValueError(f"expected one source action named {action_name!r}")
        action = matches[0]
        anim.action = action
        if len(action.slots) != 1:
            raise ValueError(
                f"{action.name}: exactly one armature action slot is required"
            )
        anim.action_slot = action.slots[0]
        start, end = action.frame_range
        frames = np.arange(start, end, 1.0).tolist() + [end]
        if len(frames) < 2:
            raise ValueError(f"{action.name} has no duration")
        positions, rotations, root_removed = [], [], []
        for frame in frames:
            whole = int(np.floor(frame))
            scene.frame_set(whole, subframe=frame - whole)
            matrices = [
                armature.matrix_world @ armature.pose.bones[bone.name].matrix
                for bone in bones
            ]
            points = [normalize_point(matrix.translation) for matrix in matrices]
            quats = [frame_basis @ matrix.to_quaternion() for matrix in matrices]
            # Remove XY and heading from the designated root only; preserve vertical and joint motion.
            delta = points[anchor] - rest_points[anchor]
            delta.z = 0
            root_removed.append((delta.x, delta.y))
            relative = quats[anchor] @ rest_rot[anchor].inverted()
            yaw = 2 * atan2(relative.z, relative.w)
            correction = Quaternion((0, 0, 1), -yaw)
            pivot = points[anchor] - delta
            points = [pivot + correction @ (point - points[anchor]) for point in points]
            quats = [correction @ value for value in quats]
            p, q = local_transforms(points, quats, parents)
            positions.append(p)
            rotations.append(q)
        index = len(clips)
        times = (np.asarray(frames) - start) / fps
        arrays[f"times_{index}"] = times
        sampled_p = np.asarray(positions, dtype=np.float32)
        sampled_q = np.asarray(rotations, dtype=np.float32)
        arrays[f"pos_{index}"] = sampled_p
        arrays[f"rot_{index}"] = sampled_q
        closed = np.allclose(
            sampled_p[0], sampled_p[-1], atol=1e-5, rtol=0
        ) and np.allclose(
            np.abs(np.sum(sampled_q[0] * sampled_q[-1], axis=1)), 1, atol=1e-6, rtol=0
        )
        clips[name] = {
            "index": index,
            "source_action": action.name,
            "fps": fps,
            "duration_s": float(times[-1]),
            "loop": name not in ("walk_start", "walk_stop"),
            "seam_blend_s": 0.0 if closed else 0.12,
            "reference_speed_mps": config.get("reference_speeds_mps", {}).get(name),
            "max_removed_xy_m": float(np.max(np.linalg.norm(root_removed, axis=1))),
        }
    return clips, [action.name for action in available]


def convert(config, path, license_path, output):
    source = dict(config["source"])
    for file, field in ((path, "sha256"), (license_path, "license_sha256")):
        if digest(file) != source[field]:
            raise ValueError(
                f"unexpected SHA-256 for {file}; verify provenance before preparing"
            )
    source["file"] = path.name
    if output.exists():
        raise ValueError(f"{output} already exists; choose a new output path")
    armature, meshes = load_source(path)
    bones = list(armature.data.bones)
    names = {bone.name: index for index, bone in enumerate(bones)}
    parents = [names[bone.parent.name] if bone.parent else -1 for bone in bones]
    if any(parent >= index for index, parent in enumerate(parents)):
        raise ValueError("expected parent-first Blender bone hierarchy")
    armature.animation_data_clear()
    bpy.context.view_layer.update()
    source_vertices = [
        [mesh.matrix_world @ vertex.co for vertex in mesh.data.vertices]
        for mesh in meshes
    ]
    z_min = min(v.z for vertices in source_vertices for v in vertices)
    z_max = max(v.z for vertices in source_vertices for v in vertices)
    scale = config["height_m"] / (z_max - z_min)
    if not 0 < scale < 100:
        raise ValueError("character has invalid source dimensions")
    angles = {"+X": 0.0, "-X": pi, "+Y": -pi / 2, "-Y": pi / 2}
    basis = Quaternion((0, 0, 1), angles[config["facing"]])
    matrices = [armature.matrix_world @ bone.matrix_local for bone in bones]
    motion_origin = matrices[names[config["motion_bone"]]].translation
    ground = Vector((motion_origin.x, motion_origin.y, z_min))

    def normalize_point(point):
        return (basis @ (point - ground)) * scale

    rest_points = [normalize_point(matrix.translation) for matrix in matrices]
    rest_rot = [basis @ matrix.to_quaternion() for matrix in matrices]
    rest_p, rest_q = local_transforms(rest_points, rest_rot, parents)
    arrays = {"rest_p": rest_p, "rest_q": rest_q}
    foot_columns = []
    for foot in config["foot_bones"]:
        side = foot.rsplit(".", 1)[-1]
        foot_columns.append(
            [names[foot]]
            + [
                i
                for name, i in names.items()
                if name in (f"{foot}_end", f"LowerLeg.{side}")
            ]
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    with TemporaryDirectory(dir=output.parent, prefix=".prepare-") as temp:
        stage = Path(temp) / "bundle"
        stage.mkdir()
        parts = []
        for mesh, points in zip(meshes, source_vertices, strict=True):
            mesh_parts(
                mesh,
                [normalize_point(point) for point in points],
                names,
                foot_columns,
                arrays,
                parts,
                stage,
            )
        clips, actions = animation_data(
            armature,
            bones,
            parents,
            basis,
            normalize_point,
            rest_points,
            rest_rot,
            config,
            arrays,
        )
        np.savez_compressed(stage / "character.npz", **arrays)
        manifest = {
            "schema_version": 1,
            "identifier": config["identifier"],
            "gender": config["gender"],
            "source": source,
            "rig_version": config["rig_version"],
            "bone_names": list(names),
            "parents": parents,
            "motion_bone": config["motion_bone"],
            "foot_bones": config["foot_bones"],
            "height_m": config["height_m"],
            "metres_per_source_unit": scale,
            "source_facing": config["facing"],
            "coordinates": "Z up, +X forward, feet at z=0 in bind pose",
            "blender_version": bpy.app.version_string,
            "parts": parts,
            "clips": clips,
            "source_actions": actions,
            "missing_clips": [name for name in _MISSING_STATES if name not in clips],
            "data_sha256": digest(stage / "character.npz"),
        }
        (stage / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )
        stage.rename(output)
    print(
        f"{config['identifier']}: {len(bones)} bones, {len(parts)} material parts, {list(clips)} -> {output}"
    )


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--selection", type=Path, default=ROOT / "assets/selection.json"
    )
    parser.add_argument("--source-dir", type=Path, default=ROOT / "assets/source")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "assets/generated")
    args = parser.parse_args(
        sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    )
    output = args.output_dir.resolve()
    if not output.is_relative_to(ROOT):
        parser.error("generated output must stay inside humans/")
    selection = json.loads(args.selection.read_text(encoding="utf-8"))
    for config in selection["characters"]:
        source = config["source"]
        target = (output / config["identifier"]).resolve()
        if target.parent != output:
            parser.error("character identifier must be a single directory name")
        convert(
            config,
            (args.source_dir / source["file"]).resolve(),
            (args.source_dir / source["license_file"]).resolve(),
            target,
        )


if __name__ == "__main__":
    main()
