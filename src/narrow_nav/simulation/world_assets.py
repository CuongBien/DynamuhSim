"""Resolve the restricted, static MJCF geometry used by authored worlds."""

from dataclasses import dataclass
from fnmatch import fnmatchcase
from hashlib import sha256
from pathlib import Path
from xml.etree import ElementTree as ET

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.validation import finite, identifier, positive

from .corridor_width import CorridorResize
from .world_geometry import GEOMETRY_TOLERANCE_M, Circle, Rectangle, WorldCollision

MJCF_ROOT = Path(__file__).resolve().parents[3] / "sim" / "mujoco"
DEFAULTS_INCLUDE = "../../templates/world_defaults.xml"


@dataclass(frozen=True, slots=True)
class WorldAsset:
    relative_path: str
    fingerprint: str
    worldbody: bytes
    collision: WorldCollision


def _numbers(node: ET.Element, attribute: str, length: int) -> tuple[float, ...]:
    values = tuple(float(token) for token in node.get(attribute, "").split())
    if len(values) != length:
        raise ValueError(f"{attribute} requires {length} values")
    return tuple(finite(value, attribute) for value in values)


def load_world_asset(
    root: Path,
    geometry_id: str,
    *,
    resize: CorridorResize | None = None,
    offsets: tuple[tuple[str, tuple[float, float]], ...] = (),
) -> WorldAsset:
    """Read one standalone world and validate geometry before using it for spawns."""
    identifier(geometry_id, "worlds.geometry_id")
    relative_path = f"worlds/{geometry_id}/main.xml"
    path = (root / relative_path).resolve()
    if not path.is_relative_to(root.resolve() / "worlds"):
        raise ConfigurationError(
            f"geometry escapes the worlds directory: {relative_path}"
        )
    try:
        content = path.read_bytes()
        xml = ET.fromstring(content)
        if (
            xml.tag != "mujoco"
            or xml.attrib != {"model": geometry_id}
            or [node.tag for node in xml] != ["include", "worldbody"]
        ):
            raise ValueError("expected a mujoco root with include and worldbody")
        include, body = xml
        if include.attrib != {"file": DEFAULTS_INCLUDE} or len(include) or body.attrib:
            raise ValueError(
                "expected the shared world defaults and an unmodified worldbody"
            )

        names = set()
        floors, walls = [], []
        for node in body:
            if (
                node.tag != "geom"
                or len(node)
                or set(node.attrib) - {"name", "class", "type", "pos", "size", "rgba"}
            ):
                raise ValueError("worldbody accepts only static geom primitives")
            name = identifier(node.get("name"), "geom name")
            if name in names:
                raise ValueError(f"duplicate geom: {name}")
            names.add(name)
            kind, shape = node.get("class"), node.get("type")
            x, y, z = _numbers(node, "pos", 3)
            if "rgba" in node.attrib and not all(
                0 <= value <= 1 for value in _numbers(node, "rgba", 4)
            ):
                raise ValueError(f"invalid geom color: {name}")
            if kind not in ("floor", "wall") or shape not in ("box", "cylinder"):
                raise ValueError(f"invalid geom class or type: {name}")
            size = _numbers(node, "size", 3 if shape == "box" else 2)
            for half_size in size:
                positive(half_size, "geom half-size")
            if kind == "floor" and (
                shape != "box" or abs(z + size[-1]) > GEOMETRY_TOLERANCE_M
            ):
                raise ValueError("the floor must be a box with top at Z=0")
            if resize is not None and kind == "wall":
                for pattern, (dx, dy) in offsets:
                    if fnmatchcase(name, pattern):
                        x, y = x + dx, y + dy
                        break
                if shape == "box":
                    x0, x1 = resize.x.map(x - size[0]), resize.x.map(x + size[0])
                    y0, y1 = resize.y.map(y - size[1]), resize.y.map(y + size[1])
                    x, y = (x0 + x1) / 2, (y0 + y1) / 2
                    size = ((x1 - x0) / 2, (y1 - y0) / 2, size[2])
                    node.set("size", " ".join(format(value, ".17g") for value in size))
                else:
                    x, y = resize.x.map(x), resize.y.map(y)
                node.set("pos", " ".join(format(value, ".17g") for value in (x, y, z)))
            footprint = (
                Rectangle(x - size[0], x + size[0], y - size[1], y + size[1])
                if shape == "box"
                else Circle(x, y, size[0])
            )
            if kind == "floor":
                floors.append(footprint)
            elif z + size[-1] > 0:
                walls.append(footprint)
            else:
                raise ValueError(f"wall geom must extend above the floor: {name}")
        if len(floors) != 1 or not walls:
            raise ValueError("expected exactly one floor and at least one wall")
    except (OSError, ET.ParseError, ValueError) as exc:
        raise ConfigurationError(f"{relative_path}: {exc}") from exc
    return WorldAsset(
        relative_path,
        sha256(content).hexdigest(),
        ET.tostring(body),
        WorldCollision(tuple(floors), tuple(walls)),
    )
