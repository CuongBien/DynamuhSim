"""Checked input model for the existing demo_1 semantic and HuNav files."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class ConfigError(ValueError):
    """Input data cannot be used to generate valid episodes."""


def read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ConfigError(f"Missing input: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ConfigError(f"Expected YAML mapping: {path}")
    return data


@dataclass(frozen=True)
class Node:
    id: str
    x: float
    y: float
    type: str
    zone: str


@dataclass(frozen=True)
class Zone:
    id: str
    type: str
    polygon: tuple[tuple[float, float], ...]


@dataclass
class Inputs:
    config_path: Path
    config: dict[str, Any]
    paths: dict[str, Path]
    zones: dict[str, Zone]
    nodes: dict[str, Node]
    edges: list[dict[str, Any]]
    hunav: dict[str, Any]
    map_config: dict[str, Any]


def _interval(value: Any, label: str, *, minimum: float = 0.0) -> None:
    if (not isinstance(value, list) or len(value) != 2
            or any(not isinstance(v, (int, float)) for v in value)
            or value[0] > value[1] or value[0] < minimum):
        raise ConfigError(f"Invalid interval {label}: {value}")


def load_inputs(config_path: Path) -> Inputs:
    config_path = config_path.resolve()
    cfg = read_yaml(config_path)
    required = {"zones", "graph", "map", "hunav", "world"}
    if not isinstance(cfg.get("source"), dict) or set(cfg["source"]) != required:
        raise ConfigError(f"source must contain exactly {sorted(required)}")
    paths = {key: (config_path.parent / value).resolve()
             for key, value in cfg["source"].items()}
    for key, value in paths.items():
        if not value.is_file():
            raise ConfigError(f"Missing {key}: {value}")
    zone_doc, graph, hunav, map_cfg = (
        read_yaml(paths[key]) for key in ("zones", "graph", "hunav", "map")
    )
    if zone_doc.get("frame_id") != "map" or graph.get("frame_id") != "map":
        raise ConfigError("zones and graph must use map frame")
    if graph.get("policy", {}).get("random_xy_allowed") is not False:
        raise ConfigError("Graph must forbid random XY")
    zones = {}
    for item in zone_doc["zones"]:
        zone = Zone(item["id"], item["type"],
                    tuple((float(x), float(y)) for x, y in item["polygon"]))
        if zone.id in zones or len(zone.polygon) < 3:
            raise ConfigError(f"Invalid/duplicate zone {zone.id}")
        zones[zone.id] = zone
    nodes = {}
    for item in graph["nodes"]:
        node = Node(item["id"], float(item["x"]), float(item["y"]),
                    item["type"], item["zone"])
        if node.id in nodes:
            raise ConfigError(f"Duplicate graph node {node.id}")
        nodes[node.id] = node
    edges = graph["edges"]
    for edge in edges:
        if edge["from"] not in nodes or edge["to"] not in nodes:
            raise ConfigError(f"Edge refers to missing node: {edge}")
    if "hunav_loader" not in hunav or "ros__parameters" not in hunav["hunav_loader"]:
        raise ConfigError("HuNav loader parameters missing")
    for name, prob in cfg["behavior_probability"].items():
        if name not in {"regular", "impassive", "surprised", "scared", "curious", "threatening"} or prob < 0:
            raise ConfigError(f"Invalid behavior probability {name}={prob}")
    if abs(sum(cfg["behavior_probability"].values()) - 1.0) > 1e-8:
        raise ConfigError("Behavior probabilities must sum to 1")
    for label, item in cfg["density"].items():
        _interval(item["background_humans"], f"density.{label}")
        if any(int(v) != v for v in item["background_humans"]):
            raise ConfigError(f"Density bounds must be integers: {label}")
    _interval(cfg["human"]["speed"], "human.speed")
    _interval(cfg["human"]["start_delay"], "human.start_delay")
    if (not isinstance(cfg.get("max_attempts"), int)
            or cfg["max_attempts"] < 1):
        raise ConfigError("max_attempts must be a positive integer")
    image_path = (paths["map"].parent / map_cfg["image"]).resolve()
    if not image_path.is_file():
        raise ConfigError(f"Missing map image: {image_path}")
    paths["map_image"] = image_path
    return Inputs(config_path, cfg, paths, zones, nodes, edges, hunav, map_cfg)
