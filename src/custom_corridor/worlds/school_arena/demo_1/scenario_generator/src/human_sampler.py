"""Sample HuNav agents while preserving the demo loader's schema."""
from __future__ import annotations

import copy
import math
import random

from .models import Inputs
from .route_sampler import RouteSampler


class HumanSampler:
    def __init__(self, inputs: Inputs, routes: RouteSampler, rng: random.Random):
        self.inputs = inputs
        self.routes = routes
        self.rng = rng

    def behavior(self) -> str:
        weights = self.inputs.config["behavior_probability"]
        return self.rng.choices(list(weights), weights=list(weights.values()), k=1)[0]

    def agent(self, name: str, role: str, route: list[str], *, speed: float | None = None,
              delay: float | None = None, lateral: float = 0.0,
              behavior: str | None = None) -> dict:
        if not self.routes.valid(route):
            raise ValueError(f"Invalid human route: {route}")
        cfg = self.inputs.config["human"]
        if speed is None:
            speed = self.rng.uniform(*cfg["speed"])
        if delay is None:
            delay = self.rng.uniform(*cfg["start_delay"])
        heading = self.routes.heading(route)
        start = self.inputs.nodes[route[0]]
        x = start.x - math.sin(heading) * lateral
        y = start.y + math.cos(heading) * lateral
        return {
            "name": name, "role": role,
            "start_node": route[0], "goal_node": route[-1],
            "route": route,
            "spawn": {"x": round(x, 5), "y": round(y, 5), "heading": round(heading, 6)},
            "speed": round(speed, 4), "start_delay": round(delay, 4),
            "lateral_offset": round(lateral, 5),
            "behavior": behavior or self.behavior(), "group_id": -1,
        }

    def hunav_yaml(self, humans: list[dict], episode_id: str) -> dict:
        params = copy.deepcopy(self.inputs.hunav["hunav_loader"]["ros__parameters"])
        prototype = copy.deepcopy(params[params["agents"][0]])
        for old_name in params["agents"]:
            params.pop(old_name, None)
        params["agents"] = []
        params["global_goals"] = {}
        params["yaml_base_name"] = episode_id
        goal_id = 1
        labels = {"regular": "Regular", "impassive": "Impassive",
                  "surprised": "Surprised", "scared": "Scared",
                  "curious": "Curious", "threatening": "Threatening"}
        for index, human in enumerate(humans, 1):
            name = human["name"]
            params["agents"].append(name)
            agent = copy.deepcopy(prototype)
            agent.update(id=index, group_id=human["group_id"],
                         skin=(index-1) % 4, max_vel=human["speed"],
                         radius=self.inputs.config["human"]["radius"],
                         goal_radius=self.inputs.config["human"]["goal_radius"],
                         cyclic_goals=False, start_delay=human["start_delay"])
            agent["init_pose"] = {"x": human["spawn"]["x"],
                                  "y": human["spawn"]["y"], "z": 0.0,
                                  "h": human["spawn"]["heading"]}
            agent["behavior"]["type"] = labels[human["behavior"]]
            agent["goals"] = []
            for node_id in human["route"][1:]:
                node = self.inputs.nodes[node_id]
                params["global_goals"][goal_id] = {"x": node.x, "y": node.y}
                agent["goals"].append(goal_id)
                goal_id += 1
            params[name] = agent
        if not humans:
            # ROS 2 params files reject empty map/sequence parameter values.
            # HuNav loader defaults both omitted parameters to empty.
            params.pop("global_goals")
            params.pop("agents")
        return {"hunav_loader": {"ros__parameters": params}}
