"""Build self-contained MJCF from a resolved NarrowNav configuration."""

from copy import deepcopy
from dataclasses import dataclass
from hashlib import sha256
from math import cos, hypot, sin
from pathlib import Path
from xml.etree import ElementTree as ET

from omegaconf import DictConfig

from narrow_nav.core.errors import ConfigurationError, ContractError
from narrow_nav.core.types import NS_PER_SECOND
from narrow_nav.core.validation import positive

from .configuration import (
    SceneGeometry,
    Spawn,
    _required,
    drive_from_config,
    validate_scene_config,
)
from .robot_controller import DriveProfile
from .world_geometry import Rectangle

MJCF_ROOT = Path(__file__).resolve().parents[3] / "sim" / "mujoco"


@dataclass(frozen=True, slots=True)
class BuiltWorld:
    mjcf: str
    scene: SceneGeometry
    source_sha256: tuple[tuple[str, str], ...] = ()


def _read_mjcf(root: Path, path: str, sources: list[tuple[str, str]]) -> ET.Element:
    content = (root / path).read_bytes()
    sources.append((path, sha256(content).hexdigest()))
    return ET.fromstring(content)


def _numbers(*values: float) -> str:
    return " ".join(format(value, ".17g") for value in values)


def _add_box(
    parent: ET.Element,
    name: str,
    kind: str,
    rect: Rectangle,
    z_min: float,
    z_max: float,
) -> None:
    ET.SubElement(
        parent,
        "geom",
        name=name,
        attrib={"class": kind},
        pos=_numbers(
            (rect.x_min + rect.x_max) / 2,
            (rect.y_min + rect.y_max) / 2,
            (z_min + z_max) / 2,
        ),
        size=_numbers(
            (rect.x_max - rect.x_min) / 2,
            (rect.y_max - rect.y_min) / 2,
            (z_max - z_min) / 2,
        ),
    )


def _add_environment(worldbody: ET.Element, scene: SceneGeometry) -> None:
    for index, floor in enumerate(scene.collision.floors):
        _add_box(worldbody, f"floor_{index}", "floor", floor, -0.1, 0)
    for index, wall in enumerate(scene.collision.walls):
        _add_box(worldbody, f"wall_{index}", "wall", wall, 0, scene.spec.wall_height_m)
    for actor in (scene.robot, *scene.humans):
        for marker, point in (("start", actor.pose), ("goal", actor.goal)):
            ET.SubElement(
                worldbody,
                "site",
                name=f"{actor.entity_id}/{marker}",
                pos=_numbers(point.x, point.y, 0.005),
                size="0.045 0.005",
                type="cylinder",
                rgba="0.2 0.7 0.4 0.6" if marker == "goal" else "0.2 0.4 0.8 0.6",
            )


def _place_actor(body: ET.Element, spawn: Spawn, z: float) -> None:
    body.set("pos", _numbers(spawn.pose.x, spawn.pose.y, z))
    half_yaw = spawn.pose.yaw / 2
    body.set("quat", _numbers(cos(half_yaw), 0, 0, sin(half_yaw)))
    for node in body.iter():
        if "name" in node.attrib:
            node.set("name", f"{spawn.entity_id}/{node.attrib['name']}")
    body.set("name", spawn.entity_id)


def _add_robot(
    worldbody: ET.Element,
    root: ET.Element,
    spawn: Spawn,
    config: DictConfig,
    mjcf_root: Path,
    sources: list[tuple[str, str]],
) -> None:
    drive = drive_from_config(config)
    mass = positive(_required(config, "robots.mass_kg"), "robots.mass_kg")
    wheel_mass = positive(
        _required(config, "robots.wheel_mass_kg"), "robots.wheel_mass_kg"
    )
    wheel_width = positive(
        _required(config, "robots.wheel_width_m"), "robots.wheel_width_m"
    )
    template = "mobile_robot" if drive.drive == "differential" else "mecanum_robot"
    robot = _read_mjcf(mjcf_root, f"models/robots/{template}/robot.xml", sources)
    body = robot.find("worldbody/body")
    if drive.drive == "differential":
        _configure_differential(
            body, spawn.radius_m, drive, mass, wheel_mass, wheel_width
        )
    else:
        _configure_mecanum(body, spawn.radius_m, drive, mass, wheel_mass, wheel_width)

    _place_actor(body, spawn, drive.wheel_radius_m)
    worldbody.append(body)
    actuators = robot.find("actuator")
    for actuator in actuators:
        actuator.set(
            "ctrlrange",
            _numbers(-drive.wheel_speed_limit_radps, drive.wheel_speed_limit_radps),
        )
        for attribute in ("name", "joint"):
            actuator.set(attribute, f"{spawn.entity_id}/{actuator.attrib[attribute]}")
    root.append(actuators)


def _configure_differential(
    body: ET.Element,
    radius: float,
    drive: DriveProfile,
    mass: float,
    wheel_mass: float,
    wheel_width: float,
) -> None:
    wheel_radius, separation = drive.wheel_radius_m, drive.wheel_separation_m
    if mass <= 2 * wheel_mass:
        raise ConfigurationError("robots.mass_kg must exceed the total wheel mass")
    if (
        separation <= wheel_width
        or hypot(wheel_radius, (separation + wheel_width) / 2) > radius
    ):
        raise ConfigurationError(
            "robot wheels must be separated and fit inside the robot footprint"
        )

    chassis = body.find("geom[@name='chassis_collision']")
    chassis.set("size", _numbers(radius, 0.055))
    chassis.set("mass", _numbers(mass - 2 * wheel_mass))
    body.find("geom[@name='chassis_visual']").set("size", _numbers(0.9 * radius, 0.05))

    for side, direction in (("left", 1), ("right", -1)):
        wheel = body.find(f"body[@name='{side}_wheel']")
        wheel.set("pos", _numbers(0, direction * separation / 2, 0))
        tire = wheel.find("geom")
        tire.set("size", _numbers(wheel_radius, wheel_width / 2))
        tire.set("mass", _numbers(wheel_mass))
    support_radius = min(0.018, 0.15 * radius)
    for name, direction in (("front_support", 1), ("rear_support", -1)):
        support = body.find(f"geom[@name='{name}']")
        support.set("size", _numbers(support_radius))
        support.set(
            "pos", _numbers(direction * 0.6 * radius, 0, support_radius - wheel_radius)
        )


def _configure_mecanum(
    body: ET.Element,
    radius: float,
    drive: DriveProfile,
    mass: float,
    wheel_mass: float,
    wheel_width: float,
) -> None:
    wheel_radius, separation = drive.wheel_radius_m, drive.wheel_separation_m
    chassis = body.find("inertial")
    mounts_mass = sum(
        float(body.find(f"body[@name='{name}']/inertial").get("mass"))
        for name in ("imu_link", "base_scan", "camera_link")
    )
    chassis_mass = mass - 4 * wheel_mass - mounts_mass
    if chassis_mass <= 0:
        raise ConfigurationError(
            "robots.mass_kg must exceed the total wheel and mount mass"
        )
    roller_radius = 0.15 * wheel_radius
    half_projection = wheel_width / 4
    wheel_envelope = (
        hypot(wheel_radius - roller_radius, half_projection) + roller_radius
    )
    if (
        separation <= wheel_width
        or drive.wheelbase_m <= 2 * wheel_envelope
        or max(
            hypot(0.12, 0.10),
            hypot(
                drive.wheelbase_m / 2 + wheel_envelope, separation / 2 + wheel_width / 2
            ),
        )
        > radius
    ):
        raise ConfigurationError(
            "mecanum chassis and separated wheels must fit inside the robot footprint"
        )
    chassis.set("mass", _numbers(chassis_mass))
    chassis.set(
        "diaginertia",
        _numbers(*(value * chassis_mass / 3.5 for value in (0.01598, 0.02473, 0.037))),
    )
    body.find("site[@name='base_footprint']").set("pos", _numbers(0, 0, -wheel_radius))
    for name, x_sign, y_sign, roller_sign in zip(
        drive.wheels, (1, 1, -1, -1), (1, -1, 1, -1), (-1, 1, 1, -1), strict=True
    ):
        wheel = body.find(f"body[@name='{name}_wheel']")
        wheel.set(
            "pos", _numbers(x_sign * drive.wheelbase_m / 2, y_sign * separation / 2, 0)
        )
        hub = wheel.find("inertial")
        hub.set("mass", _numbers(0.98 * wheel_mass))
        scale = wheel_mass / 0.25 * (wheel_radius / 0.04) ** 2
        hub.set(
            "diaginertia",
            _numbers(*(value * scale for value in (0.00012, 0.00020, 0.00012))),
        )
        wheel.find("geom").set("size", _numbers(wheel_radius, wheel_width / 2))
        roller = wheel.find("replicate/body")
        roller.set("pos", _numbers(0, 0, roller_radius - wheel_radius))
        geom = roller.find("geom")
        geom.set("size", _numbers(roller_radius))
        geom.set(
            "mass",
            _numbers(0.02 * wheel_mass / int(wheel.find("replicate").get("count"))),
        )
        geom.set(
            "fromto",
            _numbers(
                -half_projection,
                -roller_sign * half_projection,
                0,
                half_projection,
                roller_sign * half_projection,
                0,
            ),
        )


def _add_humans(
    worldbody: ET.Element,
    humans: tuple[Spawn, ...],
    mjcf_root: Path,
    sources: list[tuple[str, str]],
) -> None:
    if not humans:
        return
    pedestrian = _read_mjcf(mjcf_root, "models/humans/pedestrian.xml", sources).find(
        "worldbody/body"
    )
    for spawn in humans:
        body = deepcopy(pedestrian)
        body.find("geom[@name='collision']").set("size", _numbers(spawn.radius_m, 0.85))
        body.find("geom[@name='visual']").set(
            "size", _numbers(0.8 * spawn.radius_m, 0.78)
        )
        _place_actor(body, spawn, 0)
        worldbody.append(body)


def build_world(config: DictConfig, *, mjcf_root: Path = MJCF_ROOT) -> BuiltWorld:
    """Generate a scene without opening a renderer or mutating the input files."""
    episode, scene = validate_scene_config(config)
    sources: list[tuple[str, str]] = []
    root = ET.Element("mujoco", model=config.worlds.id)
    defaults = _read_mjcf(mjcf_root, "templates/world_defaults.xml", sources)
    defaults.find("option").set(
        "timestep", _numbers(episode.physics_dt_ns / NS_PER_SECOND)
    )
    root.extend(defaults)
    worldbody = ET.SubElement(root, "worldbody")
    _add_environment(worldbody, scene)
    try:
        _add_robot(worldbody, root, scene.robot, config, mjcf_root, sources)
        _add_humans(worldbody, scene.humans, mjcf_root, sources)
    except ConfigurationError:
        raise
    except ContractError as exc:
        raise ConfigurationError(str(exc)) from exc
    return BuiltWorld(ET.tostring(root, encoding="unicode"), scene, tuple(sources))
