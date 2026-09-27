"""Read generated episode files without changing their scenario configuration."""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from .episode_types import EpisodeContext, Pose2D


class EpisodeValidationError(ValueError):
    """A generated episode is missing or internally inconsistent."""


def _read_mapping(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise EpisodeValidationError(f"Missing YAML file: {path}")
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise EpisodeValidationError(f"Cannot read YAML {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise EpisodeValidationError(f"Expected YAML mapping: {path}")
    return value


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise EpisodeValidationError(f"Missing/invalid mapping: {label}")
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EpisodeValidationError(f"Missing/invalid number: {label}")
    number = float(value)
    if not math.isfinite(number):
        raise EpisodeValidationError(f"Non-finite number: {label}")
    return number


class EpisodeLoader:
    def __init__(self, graph_path: Path | None = None) -> None:
        self.graph_path = Path(graph_path) if graph_path is not None else (
            Path(__file__).resolve().parents[1] / "human_navigation_graph.yaml"
        )

    def load(self, episode_dir: Path) -> EpisodeContext:
        episode_dir = Path(episode_dir).resolve()
        metadata = _read_mapping(episode_dir / "metadata.yaml")
        scenario = _read_mapping(episode_dir / "scenario.yaml")
        humans = _read_mapping(episode_dir / "humans.yaml")
        graph = _read_mapping(self.graph_path)

        episode_id = metadata.get("episode_id")
        if not isinstance(episode_id, str) or episode_id != episode_dir.name:
            raise EpisodeValidationError("metadata.episode_id does not match episode directory")
        family = metadata.get("scenario_family")
        if not isinstance(family, str) or not family or family != scenario.get("scenario_family"):
            raise EpisodeValidationError("Scenario family missing or inconsistent")
        seed = metadata.get("seed")
        if isinstance(seed, bool) or not isinstance(seed, int) or seed != scenario.get("seed"):
            raise EpisodeValidationError("Seed missing or inconsistent")

        meta_robot = _mapping(metadata.get("robot"), "metadata.robot")
        robot = _mapping(scenario.get("robot"), "scenario.robot")
        start_data = _mapping(robot.get("start_pose"), "scenario.robot.start_pose")
        start_pose = Pose2D(*(
            _number(start_data.get(key), f"robot.start_pose.{key}")
            for key in ("x", "y", "heading")
        ))
        if meta_robot.get("start_pose") != start_data:
            raise EpisodeValidationError("Robot start pose differs between metadata and scenario")
        start_node = robot.get("start_node")
        if (not isinstance(start_node, str) or not start_node
                or meta_robot.get("start_node") != start_node):
            raise EpisodeValidationError("Robot start node missing or inconsistent")
        goal_node = robot.get("goal_node")
        if not isinstance(goal_node, str) or not goal_node or meta_robot.get("goal_node") != goal_node:
            raise EpisodeValidationError("Robot goal node missing or inconsistent")
        if graph.get("frame_id") != "map" or not isinstance(graph.get("nodes"), list):
            raise EpisodeValidationError("Navigation graph must contain map-frame nodes")
        nodes = {}
        for item in graph["nodes"]:
            item = _mapping(item, "graph.nodes[]")
            name = item.get("id")
            if not isinstance(name, str) or not name or name in nodes:
                raise EpisodeValidationError(f"Invalid or duplicate graph node: {name}")
            nodes[name] = item
        if start_node not in nodes:
            raise EpisodeValidationError(f"Unknown robot start node: {start_node}")
        if goal_node not in nodes:
            raise EpisodeValidationError(f"Unknown robot goal node: {goal_node}")

        route = robot.get("route")
        if not isinstance(route, list) or not route or any(
            not isinstance(node, str) or node not in nodes for node in route
        ):
            raise EpisodeValidationError("Robot route missing or contains unknown nodes")
        if route[0] != start_node or route[-1] != goal_node:
            raise EpisodeValidationError("Robot route endpoints do not match start/goal nodes")
        goal = nodes[goal_node]
        goal_x = _number(goal.get("x"), f"graph.{goal_node}.x")
        goal_y = _number(goal.get("y"), f"graph.{goal_node}.y")
        # Goal orientation follows the final graph edge already chosen by the
        # generator. A single-node route retains its stored robot heading.
        if len(route) > 1:
            previous = nodes[route[-2]]
            dx = goal_x - _number(previous.get("x"), f"graph.{route[-2]}.x")
            dy = goal_y - _number(previous.get("y"), f"graph.{route[-2]}.y")
            if dx == 0 and dy == 0:
                raise EpisodeValidationError("Final route edge has zero length")
            goal_heading = math.atan2(dy, dx)
        else:
            goal_heading = _number(robot.get("heading"), "robot.heading")
        goal_pose = Pose2D(goal_x, goal_y, goal_heading)

        scenario_humans = scenario.get("humans")
        params = _mapping(_mapping(humans.get("hunav_loader"), "humans.hunav_loader").get(
            "ros__parameters"), "humans.hunav_loader.ros__parameters")
        agent_names = params.get("agents", [])
        count = _mapping(metadata.get("humans"), "metadata.humans").get("total")
        if (not isinstance(scenario_humans, list) or not isinstance(agent_names, list)
                or isinstance(count, bool) or not isinstance(count, int)
                or count != len(scenario_humans) or count != len(agent_names)
                or [item.get("name") if isinstance(item, dict) else None
                    for item in scenario_humans] != agent_names):
            raise EpisodeValidationError("Human counts/names differ between episode files")

        return EpisodeContext(
            episode_dir=episode_dir, episode_id=episode_id, scenario_family=family,
            seed=seed, metadata=metadata, scenario=scenario, humans=humans,
            start_pose=start_pose, goal_node=goal_node, goal_pose=goal_pose,
        )
