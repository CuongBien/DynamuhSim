"""Compose and validate repository YAML without starting a simulation."""

from argparse import ArgumentParser
from dataclasses import dataclass
from math import hypot
from pathlib import Path
from threading import Lock
from typing import TypeVar
from xml.etree import ElementTree as ET

from hydra import compose, initialize_config_dir
from hydra.errors import HydraException
from omegaconf import DictConfig, ListConfig, OmegaConf
from omegaconf.errors import OmegaConfBaseException

from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.metadata import EpisodeConfig
from narrow_nav.core.types import Point2D, Pose2D
from narrow_nav.core.validation import (
    identifier,
    ordered_ids,
    positive,
    require_type,
    unique,
)

from .robot_controller import DriveProfile
from .timing import Timing
from .world_assets import MJCF_ROOT, WorldAsset, load_world_asset
from .world_geometry import WorldCollision, WorldSpec

T = TypeVar("T")
_HYDRA_LOCK = Lock()


def _required(config: DictConfig, field: str) -> object:
    value = OmegaConf.select(config, field, throw_on_missing=True)
    if value is None:
        raise ConfigurationError(f"missing required config field: {field}")
    return value


def _required_as(config: DictConfig, field: str, expected: type[T]) -> T:
    return require_type(_required(config, field), expected, field)


def timing_from_config(config: DictConfig, episode: EpisodeConfig) -> Timing:
    return Timing(
        episode.physics_dt_ns,
        episode.control_dt_ns,
        OmegaConf.select(
            config, "simulation.human_control_dt_ns", default=episode.control_dt_ns
        ),
        OmegaConf.select(
            config, "simulation.observation_dt_ns", default=episode.control_dt_ns
        ),
    )


def drive_from_config(config: DictConfig) -> DriveProfile:
    robot = _required_as(config, "robots", DictConfig)
    return DriveProfile(
        drive=_required_as(robot, "drive", str),
        wheel_radius_m=_required(robot, "wheel_radius_m"),
        wheel_separation_m=_required(robot, "wheel_separation_m"),
        max_linear_speed_mps=_required(robot, "max_linear_speed_mps"),
        max_angular_speed_radps=_required(robot, "max_angular_speed_radps"),
        max_linear_acceleration_mps2=_required(robot, "max_linear_acceleration_mps2"),
        max_angular_acceleration_radps2=_required(
            robot, "max_angular_acceleration_radps2"
        ),
        wheel_speed_limit_radps=OmegaConf.select(
            robot, "wheel_speed_limit_radps", default=22.0
        ),
        wheelbase_m=OmegaConf.select(robot, "wheelbase_m", default=0.0),
    )


def mecanum_sensor_heights(config: DictConfig) -> tuple[float, float]:
    """Resolve nominal floor-relative LiDAR and depth camera heights in metres."""
    lidar = "robots.lidar_height_m"
    camera = "robots.depth_camera_height_m"
    return (
        positive(OmegaConf.select(config, lidar, default=0.20), lidar),
        positive(OmegaConf.select(config, camera, default=0.50), camera),
    )


def _coordinates(actor: DictConfig, field: str, dimensions: int) -> ListConfig:
    values = _required_as(actor, field, ListConfig)
    if len(values) != dimensions:
        raise ConfigurationError(f"{field} must have {dimensions} coordinates")
    return values


@dataclass(frozen=True, slots=True)
class Spawn:
    entity_id: str
    pose: Pose2D
    goal: Point2D
    radius_m: float


@dataclass(frozen=True, slots=True)
class HumanProp:
    entity_id: str
    pose: Pose2D
    seat_height_m: float | None = None


@dataclass(frozen=True, slots=True)
class SceneGeometry:
    spec: WorldSpec | WorldAsset
    collision: WorldCollision
    robot: Spawn
    humans: tuple[Spawn, ...]
    human_props: tuple[HumanProp, ...] = ()


def _read_actor(actor: DictConfig, radius: float, collision: WorldCollision) -> Spawn:
    actor_id = identifier(_required(actor, "id"), "actor ID")
    if actor_id == "world":
        raise ConfigurationError("actor ID 'world' is reserved for the scene")
    pose = Pose2D(*_coordinates(actor, "initial_pose", 3))
    goal = Point2D(*_coordinates(actor, "goal", 2))
    for field, point in (("initial_pose", pose), ("goal", goal)):
        if not collision.contains(point.x, point.y, radius):
            raise ConfigurationError(
                f"{actor_id}.{field} is outside collision-free space"
            )
    return Spawn(actor_id, pose, goal, radius)


def _world_spec(world: DictConfig, mjcf_root: Path) -> WorldSpec | WorldAsset:
    if "geometry_id" in world:
        if set(world) - {"id", "geometry_id", "human_props"}:
            raise ConfigurationError(
                "worlds.geometry_id cannot be combined with parametric world fields"
            )
        geometry_id = _required_as(world, "geometry_id", str)
        return load_world_asset(mjcf_root, geometry_id)
    return WorldSpec(
        layout=OmegaConf.select(world, "layout", default="corridor"),
        corridor_length_m=positive(_required(world, "length_m"), "worlds.length_m"),
        corridor_width_m=positive(_required(world, "width_m"), "worlds.width_m"),
        wall_height_m=positive(
            _required(world, "wall_height_m"), "worlds.wall_height_m"
        ),
        wall_thickness_m=positive(
            _required(world, "wall_thickness_m"), "worlds.wall_thickness_m"
        ),
        turn_length_m=OmegaConf.select(world, "turn_length_m"),
        passing_bay_width_m=OmegaConf.select(world, "passing_bay_width_m"),
        passing_bay_length_m=OmegaConf.select(world, "passing_bay_length_m"),
        passing_bay_center_x_m=OmegaConf.select(world, "passing_bay_center_x_m"),
    )


def _human_props(
    world: DictConfig, spec: WorldSpec | WorldAsset
) -> tuple[HumanProp, ...]:
    if "human_props" not in world:
        return ()
    if not isinstance(spec, WorldAsset):
        raise ConfigurationError("worlds.human_props requires authored geometry")
    geoms = {node.get("name"): node for node in ET.fromstring(spec.worldbody)}
    props = []
    for entry in _required_as(world, "human_props", ListConfig):
        prop = require_type(entry, DictConfig, "human prop")
        geom_id = identifier(_required(prop, "geom"), "human prop geom")
        geom = geoms.get(geom_id)
        if (
            geom is None
            or geom.get("class") != "wall"
            or geom.get("type") != "cylinder"
        ):
            raise ConfigurationError(
                f"human prop {geom_id!r} must name a collision cylinder"
            )
        x, y, _ = map(float, geom.attrib["pos"].split())
        pose = Pose2D(x, y, _required(prop, "yaw_rad"))
        seat_height = None
        if "seat" in prop:
            seat = geoms.get(_required_as(prop, "seat", str))
            if seat is None or seat.get("class") != "wall" or seat.get("type") != "box":
                raise ConfigurationError(f"human prop {geom_id!r} must name a seat box")
            sx, sy, sz = map(float, seat.attrib["pos"].split())
            hx, hy, hz = map(float, seat.attrib["size"].split())
            if abs(x - sx) > hx or abs(y - sy) > hy:
                raise ConfigurationError(
                    f"human prop {geom_id!r} must be placed over its seat"
                )
            seat_height = sz + hz
        props.append(HumanProp(geom_id, pose, seat_height))
    return tuple(props)


def _scene_geometry(config: DictConfig, mjcf_root: Path) -> SceneGeometry:
    """Resolve the layout and validate the configured footprints in world XY."""
    world = _required_as(config, "worlds", DictConfig)
    spec = _world_spec(world, mjcf_root)
    collision = spec.collision if isinstance(spec, WorldAsset) else spec.collision()
    robot_radius = positive(_required(config, "robots.radius_m"), "robots.radius_m")
    human_radius = positive(
        _required(config, "scenarios.human_profile.radius_m"),
        "scenarios.human_profile.radius_m",
    )
    robot = _read_actor(
        _required_as(config, "scenarios.robot", DictConfig), robot_radius, collision
    )
    humans = tuple(
        _read_actor(
            require_type(actor, DictConfig, "human actor"), human_radius, collision
        )
        for actor in _required_as(config, "scenarios.humans", ListConfig)
    )
    ordered_ids(tuple(human.entity_id for human in humans), "scenarios.humans")
    actors = (robot, *humans)
    props = _human_props(world, spec)
    unique(
        (actor.entity_id for actor in (*actors, *props)),
        "scene actor and human prop IDs",
    )
    for index, actor in enumerate(actors):
        for other in actors[index + 1 :]:
            distance = hypot(actor.pose.x - other.pose.x, actor.pose.y - other.pose.y)
            if distance <= actor.radius_m + other.radius_m:
                raise ConfigurationError(
                    f"initial footprints overlap: {actor.entity_id}, {other.entity_id}"
                )
    return SceneGeometry(spec, collision, robot, humans, props)


def validate_scene_config(
    config: DictConfig, *, mjcf_root: Path = MJCF_ROOT
) -> tuple[EpisodeConfig, SceneGeometry]:
    """Validate configuration and resolve its engine-independent scene geometry."""
    try:
        episode = EpisodeConfig(
            scenario_id=_required_as(config, "scenarios.id", str),
            episode_seed=_required_as(config, "seed", int),
            control_dt_ns=_required_as(config, "simulation.control_dt_ns", int),
            physics_dt_ns=_required_as(config, "simulation.physics_dt_ns", int),
            duration_ns=_required_as(config, "experiments.duration_ns", int),
            schema_version=_required_as(config, "schema_version", int),
        )
        timing_from_config(config, episode)
        if _required(config, "simulation.engine") != "mujoco":
            raise ConfigurationError("simulation.engine must be mujoco")
        if OmegaConf.select(
            config, "simulation.human_visuals", default="proxy"
        ) not in (
            "quaternius",
            "proxy",
        ):
            raise ConfigurationError(
                "simulation.human_visuals must be quaternius or proxy"
            )
        world_id = identifier(_required(config, "worlds.id"), "worlds.id")
        if _required(config, "scenarios.world_id") != world_id:
            raise ConfigurationError("scenarios.world_id must match the selected world")

        for group in ("robots", "humans", "experiments"):
            identifier(_required(config, f"{group}.id"), f"{group}.id")
        backend = identifier(_required(config, "humans.backend"), "humans.backend")
        if backend == "scripted":
            positive(_required(config, "humans.speed_mps"), "humans.speed_mps")
        drive = _required(config, "robots.drive")
        if drive not in ("differential", "mecanum"):
            raise ConfigurationError("robots.drive must be differential or mecanum")
        if drive == "mecanum":
            mecanum_sensor_heights(config)
        _required_as(config, "experiments.terminate_on_contact", bool)

        for field in (
            "robots.max_linear_speed_mps",
            "robots.max_angular_speed_radps",
            "robots.max_linear_acceleration_mps2",
            "robots.max_angular_acceleration_radps2",
            "scenarios.human_profile.max_speed_mps",
            "scenarios.human_profile.max_acceleration_mps2",
        ):
            positive(_required(config, field), field)
        scene = _scene_geometry(config, mjcf_root)
    except ConfigurationError:
        raise
    except (ContractError, OmegaConfBaseException) as exc:
        raise ConfigurationError(str(exc)) from exc
    return episode, scene


def validate_config(config: DictConfig) -> EpisodeConfig:
    """Check required fields and return the engine-independent episode settings."""
    episode, _ = validate_scene_config(config)
    return episode


def compose_config(
    config_dir: Path | str = "configs", overrides: tuple[str, ...] | list[str] = ()
) -> DictConfig:
    """Compose actual Hydra config groups and return a read-only resolved result."""
    try:
        # Hydra's Compose API temporarily changes its process-wide state.
        with (
            _HYDRA_LOCK,
            initialize_config_dir(
                version_base="1.3", config_dir=str(Path(config_dir).resolve())
            ),
        ):
            resolved = compose(config_name="config", overrides=list(overrides))
            OmegaConf.resolve(resolved)
            OmegaConf.to_container(resolved, throw_on_missing=True)
    except (HydraException, OmegaConfBaseException) as exc:
        raise ConfigurationError(str(exc)) from exc
    validate_config(resolved)
    OmegaConf.set_readonly(resolved, True)
    return resolved


def main() -> None:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path, default=Path("configs"))
    parser.add_argument("overrides", nargs="*", help="Hydra overrides such as seed=123")
    args = parser.parse_args()
    try:
        config = compose_config(args.config_dir, args.overrides)
    except ConfigurationError as exc:
        parser.error(str(exc))
    print(OmegaConf.to_yaml(config, sort_keys=True), end="")


if __name__ == "__main__":
    main()
