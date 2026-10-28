"""Physical capability paths and exact legacy source IDs for recorded receipts."""
from pathlib import Path, PurePosixPath
SOCIAL = Path('experiments/social_navigation')
LEGACY = SOCIAL / 'benchmark'
NAVIGATION_FILES = frozenset(['amcl_observer.py', 'check_crowd_sensors.py', 'circle_contact.py', 'diagnostics.py', 'footprint_geometry.py', 'replay_contact_geometry.py', 'robot_orca.py', 'run.sh', 'trial.py', 'validate_circle_contact.py', 'validate_robot_orca.py', 'warehouse.launch.py', 'worker.py'])
STAGE_GROUPS = {'camera_height_ab': 'perception',
 'common_gazebo': 'navigation',
 'common_response': 'navigation',
 'complexitynav_contract': 'navigation',
 'complexitynav_geometry': 'navigation',
 'dataset_pilot': 'datasets',
 'depth_identity_guard': 'perception',
 'depth_perception_corpus': 'perception',
 'depth_person_classifier': 'perception',
 'depth_proxy_ablation': 'perception',
 'depth_proxy_lifecycle': 'perception',
 'depth_seed43_validation': 'perception',
 'depth_surface_guard': 'perception',
 'depth_track_refinement': 'perception',
 'depth_visibility_pilot': 'perception',
 'early_avoidance_pilot': 'navigation',
 'lidar_person_pilot': 'perception',
 'localization_trials': 'navigation',
 'navigation_diagnostics': 'navigation',
 'perception_target_pilot': 'datasets',
 'portable_episode': 'datasets',
 'shared_nav2_pilot': 'navigation',
 'socnavgym_group_nav2': 'navigation',
 'socnavgym_lifecycle_fix': 'navigation',
 'socnavgym_nav2_pilot': 'navigation',
 'socnavgym_response_fix': 'navigation',
 'socnavgym_warehouse_lifecycle': 'navigation',
 'wm_dataset_replay': 'replay',
 'wm_episode_batch': 'datasets'}


ARCHIVE_ONLY_STAGES = frozenset(['dataset_pilot', 'depth_proxy_lifecycle', 'early_avoidance_pilot'])
CURRENT_STAGE_GROUPS = {name: group for name, group in STAGE_GROUPS.items() if name not in ARCHIVE_ONLY_STAGES}
RELOCATED_FILES = {
    'experiments/social_navigation/benchmark/wm_episode_batch/tools/legacy/dataset_core.py': 'python/dynamuhsim/datasets/normalization.py',
    'experiments/social_navigation/benchmark/wm_episode_batch/tools/legacy/contract.json': 'python/dynamuhsim/datasets/normalization_contract.json',
    'experiments/social_navigation/benchmark/dataset_pilot/contract.json': 'python/dynamuhsim/datasets/normalization_contract.json',
    'experiments/social_navigation/benchmark/dataset_pilot/qos.yaml': 'configs/capture/qos.yaml',
    'experiments/social_navigation/benchmark/dataset_pilot/storage.yaml': 'configs/capture/storage.yaml',
    'experiments/social_navigation/benchmark/early_avoidance_pilot/check_friction.cc': 'python/dynamuhsim/runtime/native/check_friction.cc',
    'experiments/social_navigation/benchmark/early_avoidance_pilot/generated/check_friction': 'python/dynamuhsim/runtime/native/generated/check_friction',
    'experiments/social_navigation/benchmark/early_avoidance_pilot/protocol.json': 'configs/history/early_avoidance_pilot.json',
    'experiments/social_navigation/benchmark/depth_proxy_lifecycle/protocol.json': 'configs/history/depth_proxy_lifecycle.json',
}


def stage_directory(repository, stage):
    group = STAGE_GROUPS[stage]
    repository = Path(repository)
    current, legacy = repository / SOCIAL / group / stage, repository / LEGACY / stage
    # A current stage always wins, even if a requested child is missing.
    return legacy if not current.exists() and legacy.exists() else current


def source_path(repository, relative):
    """Resolve only declared stage IDs; supports recorded legacy checkouts too."""
    name = PurePosixPath(relative)
    if name.is_absolute() or '..' in name.parts: raise ValueError('Expected repository-relative source path')
    parts = name.parts
    relocated = RELOCATED_FILES.get(logical_source_name(str(name)))
    if relocated is not None:
        current = Path(repository) / relocated
        if current.exists() or (Path(repository) / 'python/dynamuhsim/datasets/__init__.py').is_file():
            return current
    if parts[:2] == ('experiments', 'social_navigation') and len(parts) >= 4:
        group, stage = parts[2:4]
        if stage in STAGE_GROUPS and group in ('benchmark', STAGE_GROUPS[stage]):
            return stage_directory(repository, stage).joinpath(*parts[4:])
    if parts[:3] == ('experiments','social_navigation','benchmark') and len(parts)==4 and parts[3] in NAVIGATION_FILES:
        current=Path(repository)/SOCIAL/'navigation'/parts[3]
        return current if current.exists() else Path(repository).joinpath(*parts)
    return Path(repository).joinpath(*parts)


def logical_source_name(relative):
    name = PurePosixPath(relative); parts = name.parts
    if parts[:2] == ('experiments', 'social_navigation') and len(parts) >= 4:
        group, stage = parts[2:4]
        if STAGE_GROUPS.get(stage) == group: return str(PurePosixPath(LEGACY) / stage / PurePosixPath(*parts[4:]))
    if parts[:3]==('experiments','social_navigation','navigation') and len(parts)==4 and parts[3] in NAVIGATION_FILES:
        return str(PurePosixPath(LEGACY)/parts[3])
    return str(name)
