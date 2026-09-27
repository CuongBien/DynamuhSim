"""Generate reproducible school social navigation episodes from Step 1 data."""
from __future__ import annotations

import argparse
import copy
import math
import random
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from .human_sampler import HumanSampler
from .models import Inputs, load_inputs, read_yaml
from .route_sampler import RouteSampler
from .validators import OccupancyMap, ValidationError, validate_episode
from .zone_sampler import ZoneSampler, contains

ROOT = Path(__file__).resolve().parents[1]
FAMILIES = ("empty", "head_on", "same_direction", "crossing", "exit_room")
TEMPLATE_FILES = {family: ROOT / "templates" / f"S{i:02d}_{family}.yaml"
                  for i, family in enumerate(FAMILIES)}


class GenerationError(RuntimeError):
    pass


class ScenarioGenerator:
    def __init__(self, config_path: Path = ROOT / "configs/generator.yaml"):
        self.inputs = load_inputs(config_path)
        self.occupancy = OccupancyMap(self.inputs)
        self.templates = {family: read_yaml(path)
                          for family, path in TEMPLATE_FILES.items()}
        for family, template in self.templates.items():
            if template.get("family") != family:
                raise GenerationError(f"Template family mismatch: {family}")

    def _components(self, seed: int):
        rng = random.Random(seed)
        routes = RouteSampler(self.inputs, rng)
        return rng, routes, ZoneSampler(self.inputs, rng), HumanSampler(self.inputs, routes, rng)

    def _robot(self, route: list[str], routes: RouteSampler, corridor: str) -> dict:
        a, b = (self.inputs.nodes[route[0]], self.inputs.nodes[route[-1]])
        return {"start_zone": corridor, "goal_zone": corridor,
                "start_node": a.id, "goal_node": b.id,
                "start_pose": {"x": a.x, "y": a.y, "heading": round(routes.heading(route), 6)},
                "route": route, "heading": round(routes.heading(route), 6)}

    def _corridor_event(self, family: str, routes: RouteSampler,
                        humans: HumanSampler, rng: random.Random) -> tuple[dict, list[dict], dict]:
        zone, chain = rng.choice(routes.corridor_chains())
        direction = rng.choice((1, -1))
        if direction < 0:
            chain = chain[::-1]
        if family == "empty":
            start = rng.randrange(0, max(1, len(chain)-2))
            route = chain[start:]
            return self._robot(route, routes, zone), [], {"corridor": zone}
        if family == "head_on":
            start = rng.randrange(0, max(1, len(chain)-3))
            route = chain[start:]
            human_route = route[::-1]
            lateral = rng.uniform(*self.inputs.config["human"]["lateral_offset"])
            event = humans.agent("event_agent_1", "event_agent", human_route,
                                 lateral=lateral)
            return self._robot(route, routes, zone), [event], {"corridor": zone}
        if family == "same_direction":
            start = rng.randrange(0, max(1, len(chain)-3))
            route = chain[start:]
            gap = self.templates[family]["min_gap_nodes"]
            if len(route) < gap + 2:
                raise ValidationError("Corridor too short for same direction")
            human_route = route[gap:]
            speed = rng.uniform(*self.templates[family]["human_speed"])
            lateral = rng.uniform(*self.inputs.config["human"]["lateral_offset"])
            event = humans.agent("event_agent_1", "event_agent", human_route,
                                 speed=speed, lateral=lateral)
            return self._robot(route, routes, zone), [event], {"corridor": zone,
                    "initial_gap_m": round(routes.length(route[:gap+1]), 3)}
        raise GenerationError(f"Not a corridor event: {family}")

    def _room_anchors(self, routes: RouteSampler) -> list[tuple[str, str, str, list[str]]]:
        # A classroom approach joins a corridor graph node. Pair classrooms
        # sharing that node for door crossings, without inventing trajectories.
        anchors = {}
        for zone in self.inputs.zones.values():
            if zone.type != "classroom":
                continue
            approach = f"{zone.id}_approach"
            if approach not in self.inputs.nodes:
                continue
            for neighbor in routes.adj[approach]:
                if self.inputs.nodes[neighbor].type == "corridor" and neighbor.startswith("corridor_"):
                    anchors.setdefault(neighbor, []).append(zone.id)
        result = []
        for corridor, chain in routes.corridor_chains():
            for anchor in chain:
                for room in sorted(anchors.get(anchor, [])):
                    result.append((room, anchor, corridor, chain))
        return result

    def _timed_event(self, family: str, routes: RouteSampler,
                     humans: HumanSampler, rng: random.Random) -> tuple[dict, list[dict], dict]:
        candidates = self._room_anchors(routes)
        if not candidates:
            raise GenerationError("No classroom-to-corridor anchors")
        room, anchor, corridor, chain = rng.choice(candidates)
        idx = chain.index(anchor)
        sides = [side for side in (-1, 1) if 0 <= idx+side < len(chain)]
        if not sides:
            raise ValidationError("No robot approach to doorway")
        side = rng.choice(sides)
        # Starting two edges away usually leaves enough time for a human to
        # reach the door; where the chain ends, one edge is still valid.
        span = 2 if 0 <= idx+2*side < len(chain) else 1
        robot_start = chain[idx+span*side]
        robot_goal = chain[idx-side] if 0 <= idx-side < len(chain) else anchor
        robot_route = routes.shortest(robot_start, robot_goal)
        if anchor not in robot_route or robot_route.index(anchor) == 0:
            raise ValidationError("Robot does not approach conflict point")
        if family == "crossing":
            others = sorted(r for r, a, _, _ in candidates if a == anchor and r != room)
            if not others:
                raise ValidationError("No room across corridor at anchor")
            human_goal = f"{rng.choice(others)}_center"
        else:
            # Continue along the corridor after exiting the classroom.
            destination = chain[idx-side] if 0 <= idx-side < len(chain) else chain[idx+side]
            human_goal = destination
        human_route = routes.shortest(f"{room}_center", human_goal)
        if anchor not in human_route:
            raise ValidationError("Human route misses corridor conflict")
        speed = rng.uniform(*self.inputs.config["human"]["speed"])
        robot_time = routes.length(robot_route[:robot_route.index(anchor)+1]) / self.inputs.config["robot"]["nominal_speed"]
        human_time = routes.length(human_route[:human_route.index(anchor)+1]) / speed
        low, high = self.templates[family]["start_delay"]
        delay = robot_time - human_time + rng.uniform(-0.4, 0.4)
        if not low <= delay <= high:
            raise ValidationError("Cannot synchronize door crossing within delay range")
        event = humans.agent("event_agent_1", "event_agent", human_route,
                             speed=speed, delay=delay)
        detail = {"room": room, "doorway": f"{room}_door",
                  "corridor": corridor, "conflict_node": anchor,
                  "time_tolerance": self.templates[family]["time_tolerance"]}
        if family == "crossing":
            detail["crossing_mode"] = self.templates[family]["crossing_mode"]
        return self._robot(robot_route, routes, corridor), [event], detail

    def _background(self, count: int, existing: list[dict], robot: dict,
                    routes: RouteSampler, humans: HumanSampler,
                    rng: random.Random) -> list[dict]:
        if count == 0:
            return []
        starts = sorted(n.id for n in self.inputs.nodes.values()
                        if n.id.endswith("_center") and n.type == "classroom")
        rng.shuffle(starts)
        goals = sorted(n.id for n in self.inputs.nodes.values()
                       if n.type == "classroom" and n.id.endswith("_center"))
        selected = []
        occupied_nodes = {robot["start_node"]} | {h["start_node"] for h in existing}
        for start in starts:
            if start in occupied_nodes:
                continue
            choices = [goal for goal in goals if goal != start]
            goal = rng.choice(choices)
            try:
                route = routes.shortest(start, goal)
            except ValueError:
                continue
            selected.append(humans.agent(f"background_agent_{len(selected)+1}",
                                         "background_agent", route, delay=rng.uniform(0, 5)))
            occupied_nodes.add(start)
            if len(selected) == count:
                return selected
        raise ValidationError(f"Only found {len(selected)} safe background spawns; need {count}")

    def sample(self, family: str, seed: int, density: str = "low") -> dict:
        if family not in FAMILIES:
            raise GenerationError(f"Unknown scenario {family}")
        if density not in self.inputs.config["density"]:
            raise GenerationError(f"Unknown density {density}")
        rng, routes, _, humans = self._components(seed)
        attempts = self.inputs.config["max_attempts"]
        errors = []
        for _ in range(attempts):
            try:
                if family in {"empty", "head_on", "same_direction"}:
                    robot, event_agents, detail = self._corridor_event(family, routes, humans, rng)
                else:
                    robot, event_agents, detail = self._timed_event(family, routes, humans, rng)
                limits = self.inputs.config["density"][density]["background_humans"]
                background_count = 0 if family == "empty" else rng.randint(*limits)
                all_humans = event_agents + self._background(background_count, event_agents,
                                                               robot, routes, humans, rng)
                scenario = {"dataset_version": self.inputs.config["dataset_version"],
                            "seed": seed, "scenario_family": family, "density": density,
                            "map": self.inputs.config["map_name"],
                            "robot": robot, "event": detail, "humans": all_humans}
                validate_episode(self.inputs, routes, self.occupancy, scenario)
                return scenario
            except (ValidationError, ValueError) as exc:
                errors.append(str(exc))
        raise GenerationError(f"No valid {family} episode after {attempts} attempts; last: {errors[-1] if errors else 'unknown'}")

    def write(self, scenario: dict, output_dir: Path, episode_id: str) -> Path:
        output_dir.mkdir(parents=True, exist_ok=False)
        scenario = copy.deepcopy(scenario)
        humans = scenario["humans"]
        yaml_base_name = f"scenario_{scenario['scenario_family']}_{scenario['density']}_{scenario['seed']}"
        _, routes, _, human_sampler = self._components(scenario["seed"])
        hunav = human_sampler.hunav_yaml(humans, yaml_base_name)
        metadata = {"episode_id": episode_id,
                    "dataset_version": scenario["dataset_version"],
                    "seed": scenario["seed"], "scenario_family": scenario["scenario_family"],
                    "density": scenario["density"], "map": scenario["map"],
                    "robot": {key: scenario["robot"][key]
                              for key in ("start_zone", "goal_zone", "start_node", "goal_node", "start_pose")},
                    "humans": {"total": len(humans),
                               "event_agents": sum(h["role"] == "event_agent" for h in humans),
                               "background_agents": sum(h["role"] == "background_agent" for h in humans)}}
        for filename, doc in (("metadata.yaml", metadata), ("scenario.yaml", scenario),
                              ("humans.yaml", hunav)):
            (output_dir / filename).write_text(yaml.safe_dump(doc, sort_keys=False), encoding="utf-8")
        self._write_world(output_dir / "school_floor.world", humans)
        nav_cfg = read_yaml(self.inputs.paths["world"].parent / "nav2_school.yaml")
        pose = scenario["robot"]["start_pose"]
        nav_cfg["amcl"]["ros__parameters"]["initial_pose"].update(
            x=pose["x"], y=pose["y"], yaw=pose["heading"]
        )
        (output_dir / "nav2_school.yaml").write_text(
            yaml.safe_dump(nav_cfg, sort_keys=False), encoding="utf-8"
        )
        bt_source = self.inputs.paths["world"].parent / "demo_1_hunav__agent_1_bt.xml"
        for index in range(1, len(humans)+1):
            (output_dir / f"{yaml_base_name}__agent_{index}_bt.xml").write_bytes(bt_source.read_bytes())
        return output_dir

    def _write_world(self, destination: Path, humans: list[dict]) -> None:
        tree = ET.parse(self.inputs.paths["world"])
        world = tree.getroot().find("world")
        models = list(world.findall("model"))
        proxies = [m for m in models if m.get("name") in {"demo_primary", "demo_crossing"}]
        if not proxies:
            raise GenerationError("Source world has no HuNav proxy model")
        insert_at = list(world).index(proxies[0])
        for proxy in proxies:
            world.remove(proxy)
        for index, human in enumerate(humans):
            proxy = copy.deepcopy(proxies[index % len(proxies)])
            proxy.set("name", human["name"])
            pose = proxy.find("pose")
            pose.text = f"{human['spawn']['x']:.5f} {human['spawn']['y']:.5f} 0.000 0 0 {human['spawn']['heading']:.6f}"
            world.insert(insert_at+index, proxy)
        tree.write(destination, encoding="utf-8", xml_declaration=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=(*FAMILIES, "all"), required=True)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--seed-start", type=int)
    parser.add_argument("--num-episodes", type=int, default=1)
    parser.add_argument("--density", choices=("low", "medium", "high"), default="low")
    parser.add_argument("--config", type=Path, default=ROOT / "configs/generator.yaml")
    parser.add_argument("--output", type=Path, default=ROOT / "generated")
    args = parser.parse_args(argv)
    if args.num_episodes < 1:
        parser.error("--num-episodes must be positive")
    if args.seed is not None and args.seed_start is not None:
        parser.error("Use --seed or --seed-start, not both")
    generator = ScenarioGenerator(args.config)
    root_seed = args.seed_start if args.seed_start is not None else args.seed
    if root_seed is None:
        root_seed = random.SystemRandom().randrange(2**32)
    args.output.mkdir(parents=True, exist_ok=True)
    existing = [int(match.group(1)) for p in args.output.iterdir()
                if p.is_dir() and (match := re.fullmatch(r"ep_(\d{6})", p.name))]
    next_id = max(existing, default=0) + 1
    for index in range(args.num_episodes):
        family = FAMILIES[index % len(FAMILIES)] if args.scenario == "all" else args.scenario
        episode = generator.sample(family, root_seed+index, args.density)
        episode_id = f"ep_{next_id+index:06d}"
        output = generator.write(episode, args.output / episode_id, episode_id)
        print(f"{output} seed={root_seed+index} scenario={family}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
