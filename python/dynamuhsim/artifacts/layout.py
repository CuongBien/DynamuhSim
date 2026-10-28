"""Explicit compatibility paths for archived immutable benchmark reports."""
import os
from pathlib import Path, PurePosixPath
from dynamuhsim.workspace import source_path, logical_source_name, SOCIAL, STAGE_GROUPS

BENCHMARK = PurePosixPath("experiments/social_navigation/benchmark")
ARCHIVE = PurePosixPath("experiments/archive/social_navigation/reports")
SUPPORT = PurePosixPath("experiments/archive/social_navigation/support")
SUPPORT_STAGES = frozenset(["gpu_runtime", "shutdown_fix"])
REPORT_STAGES = frozenset([
    'camera_height_ab',
    'common_gazebo',
    'common_response',
    'dataset_pilot',
    'depth_identity_guard',
    'depth_perception_corpus',
    'depth_person_classifier',
    'depth_proxy_ablation',
    'depth_proxy_lifecycle',
    'depth_seed43_validation',
    'depth_surface_guard',
    'depth_track_refinement',
    'depth_visibility_pilot',
    'early_avoidance_pilot',
    'lidar_person_pilot',
    'perception_target_pilot',
    'portable_episode',
    'shared_nav2_pilot',
    'socnavgym_group_nav2',
    'socnavgym_nav2_pilot',
    'socnavgym_response_fix',
    'wm_dataset_replay',
    'wm_episode_batch',
])


def report_target(checkout, relative):
    """Recognize only exact migration aliases; all other symlinks stay forbidden."""
    relative = PurePosixPath(logical_source_name(str(relative)))
    parts = relative.parts
    alias = destination = suffix = None
    if parts[:3] == BENCHMARK.parts and len(parts) >= 4:
        if parts[3] in SUPPORT_STAGES:
            alias = checkout / BENCHMARK / parts[3]
            destination = checkout / SUPPORT / parts[3]
            suffix = parts[4:]
        elif len(parts) >= 5 and parts[3] in REPORT_STAGES and parts[4] == "reports":
            alias = source_path(checkout, str(BENCHMARK / parts[3] / "reports"))
            if not (checkout / SOCIAL / STAGE_GROUPS[parts[3]] / parts[3]).exists():
                alias = checkout / BENCHMARK / parts[3] / "reports"
            destination = checkout / ARCHIVE / parts[3]
            suffix = parts[5:]
    if alias is not None and alias.is_symlink():
        if os.readlink(alias) != os.path.relpath(destination, alias.parent):
            raise ValueError("Unrecognized archive symlink")
        for parent in alias.parents:
            if parent == checkout: break
            if parent.is_symlink(): raise ValueError("Archive alias ancestor contains a symlink")
        return destination.joinpath(*suffix)
    if parts[:3] == BENCHMARK.parts and len(parts) >= 4 and parts[3] in STAGE_GROUPS:
        if not (checkout / SOCIAL / STAGE_GROUPS[parts[3]] / parts[3]).exists():
            return checkout.joinpath(*relative.parts)
    return source_path(checkout, str(relative))


def logical_report_name(name):
    """Normalize an archive read for the existing read-only probe restrictions."""
    path = PurePosixPath(logical_source_name(name))
    if path.parts[:4] == ARCHIVE.parts and len(path.parts) >= 5 and path.parts[4] in REPORT_STAGES:
        return str(BENCHMARK / path.parts[4] / "reports" / PurePosixPath(*path.parts[5:]))
    return str(path)
