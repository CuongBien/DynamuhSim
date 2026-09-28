"""Graph-backed group and mixed-traffic events (S16–S23)."""
from __future__ import annotations

import math
import random

from .human_sampler import HumanSampler
from .route_sampler import RouteSampler
from .validators import ValidationError


def _agent_on_edge(humans: HumanSampler, name: str, route: list[str],
                   along: float, *, delay: float | None = None,
                   lateral: float = 0.0, group_id: int = 1,
                   speed: float | None = None) -> dict:
    """Place a human on the first declared edge, within its start zone.

    The graph node remains the route origin; ``along`` only spreads agents
    within the semantic zone rather than inventing a path through the map.
    """
    a, b = (humans.inputs.nodes[node] for node in route[:2])
    length = math.hypot(b.x - a.x, b.y - a.y)
    if not 0 <= along < length:
        raise ValidationError("Group does not fit on its starting graph edge")
    unit_x, unit_y = (b.x-a.x)/length, (b.y-a.y)/length
    agent = humans.agent(name, "event_agent", route, delay=delay,
                         speed=speed, lateral=lateral)
    agent["spawn"]["x"] = round(a.x + along*unit_x - lateral*unit_y, 5)
    agent["spawn"]["y"] = round(a.y + along*unit_y + lateral*unit_x, 5)
    agent["group_id"] = group_id
    return agent


def _column(humans: HumanSampler, route: list[str], size: int,
            rng: random.Random, *, delay: float | None = None,
            group_id: int = 1, speed: float | None = None) -> list[dict]:
    gap = rng.uniform(0.58, 0.64)
    if speed is None:
        speed = rng.uniform(*humans.inputs.config["human"]["speed"])
    return [_agent_on_edge(humans, f"event_agent_{i+1}", route,
                           i*gap, delay=delay, group_id=group_id,
                           speed=speed, lateral=rng.uniform(-0.025, 0.025))
            for i in range(size)]


def _size(family: str, density: str, rng: random.Random) -> int:
    bounds = {
        "walking_group": {"low": (2, 3), "medium": (2, 4), "high": (3, 4)},
        "opposing_group": {"low": (2, 3), "medium": (2, 4), "high": (3, 4)},
        "crossing_group": {"low": (2, 2), "medium": (2, 3), "high": (3, 4)},
        "conversation_group": {"low": (2, 2), "medium": (3, 3), "high": (4, 4)},
    }
    return rng.randint(*bounds[family][density])


def _corridor_group(generator, family: str, routes: RouteSampler,
                    humans: HumanSampler, rng: random.Random,
                    density: str) -> tuple[dict, list[dict], dict]:
    size = _size(family, density, rng)
    candidates = []
    for corridor, chain in routes.corridor_chains():
        for direction in (chain, chain[::-1]):
            for index in range(1, len(direction)-1):
                segment = direction[index:index+2]
                if routes.length(segment) < 0.68*(size-1)+0.3:
                    continue
                if family == "walking_group":
                    robot_route = direction[index-1:]
                    gap = routes.length(robot_route[:2])
                    catch_distance = gap*0.55/(0.55-0.40)
                    if routes.length(robot_route) < catch_distance+0.5:
                        continue
                elif routes.length(direction[index-1:index+1]) < 0.68*(size-1)+0.3:
                    continue
                candidates.append((corridor, direction, index))
    if not candidates:
        raise ValidationError("No corridor edge can hold a group")
    corridor, chain, index = rng.choice(candidates)
    if family == "walking_group":
        robot_route = chain[index-1:]
        human_route = chain[index:]
    else:
        robot_route = chain
        human_route = chain[index::-1]
    if family == "opposing_group":
        # The first edge for the reversed group route is checked independently.
        if routes.length(human_route[:2]) < 0.68*(size-1)+0.3:
            raise ValidationError("Opposing group start edge too short")
    delay = rng.uniform(0.0, 0.8)
    speed = rng.uniform(0.30, 0.40) if family == "walking_group" else None
    agents = _column(humans, human_route, size, rng, delay=delay, speed=speed)
    return generator._robot(robot_route, routes, corridor), agents, {
        "corridor": corridor, "group_id": 1, "group_size": size,
        "formation": "column", "interaction_node": chain[index],
    }


def _crossing_group(generator, routes: RouteSampler, humans: HumanSampler,
                    rng: random.Random, density: str) -> tuple[dict, list[dict], dict]:
    candidates = [(room, anchor, corridor, chain)
                  for room, anchor, corridor, chain in generator._room_anchors(routes)
                  if any(other != room and other_anchor == anchor
                         for other, other_anchor, _, _ in generator._room_anchors(routes))]
    if not candidates:
        raise ValidationError("No paired classrooms for crossing group")
    room, anchor, corridor, chain = rng.choice(candidates)
    opposite = sorted(other for other, other_anchor, _, _ in candidates
                      if other_anchor == anchor and other != room)
    goal_room = rng.choice(opposite)
    human_route = routes.shortest(f"{room}_center", f"{goal_room}_center")
    index = chain.index(anchor)
    sides = [side for side in (-1, 1) if 0 <= index+side < len(chain)]
    side = rng.choice(sides)
    span = 2 if 0 <= index+2*side < len(chain) else 1
    robot_start = chain[index+span*side]
    robot_goal = chain[index-side] if 0 <= index-side < len(chain) else anchor
    robot_route = routes.shortest(robot_start, robot_goal)
    size = _size("crossing_group", density, rng)
    speed = rng.uniform(*humans.inputs.config["human"]["speed"])
    robot_time = (routes.length(robot_route[:robot_route.index(anchor)+1])
                  / generator.inputs.config["robot"]["nominal_speed"])
    human_time = routes.length(human_route[:human_route.index(anchor)+1]) / speed
    delay = robot_time-human_time+rng.uniform(-0.25, 0.25)
    if delay < 0:
        raise ValidationError("Crossing group cannot synchronize with robot")
    agents = _column(humans, human_route, size, rng, delay=delay, speed=speed)
    return generator._robot(robot_route, routes, corridor), agents, {
        "corridor": corridor, "group_id": 1, "group_size": size,
        "source_room": room, "goal_room": goal_room,
        "conflict_node": anchor, "time_tolerance": 2.5,
    }


def _conversation_group(generator, routes: RouteSampler, humans: HumanSampler,
                        rng: random.Random, density: str) -> tuple[dict, list[dict], dict]:
    # The waiting area is represented in zones.yaml and contains the graph's
    # room12_approach node. Use that node's route for every waiting person.
    waiting = generator.inputs.zones.get("waiting_area_north")
    start = generator.inputs.nodes.get("room12_approach")
    if waiting is None or start is None:
        raise ValidationError("Waiting area or entry node missing")
    route = routes.shortest(start.id, "corridor_f_north")
    size = _size("conversation_group", density, rng)
    robot_route = routes.shortest("corridor_f_lower_before_bin", "corridor_f_north")
    center_y = (min(y for _, y in waiting.polygon) +
                max(y for _, y in waiting.polygon)) / 2
    spacing = rng.uniform(0.56, 0.60)
    agents = []
    for i in range(size):
        # Delayed movement makes the cluster a social obstacle while the robot
        # reaches this corridor. Each spawn remains inside the waiting zone.
        agent = humans.agent(f"event_agent_{i+1}", "event_agent", route,
                             delay=rng.uniform(35.0, 45.0),
                             speed=rng.uniform(*humans.inputs.config["human"]["speed"]))
        agent["spawn"]["x"] = round(start.x + rng.uniform(-0.025, 0.025), 5)
        agent["spawn"]["y"] = round(center_y + (i-(size-1)/2)*spacing, 5)
        agent["group_id"] = 1
        agents.append(agent)
    return generator._robot(robot_route, routes, "corridor_f"), agents, {
        "corridor": "corridor_f", "group_id": 1, "group_size": size,
        "social_obstacle_node": start.id, "waiting_zone": waiting.id,
    }


def _bidirectional(generator, routes: RouteSampler, humans: HumanSampler,
                   rng: random.Random, density: str) -> tuple[dict, list[dict], dict]:
    corridor, chain = next((z, c) for z, c in routes.corridor_chains()
                           if z == "corridor_a")
    if rng.choice((True, False)):
        chain = chain[::-1]
    count = {"low": 4, "medium": 5, "high": 6}[density]
    indices = sorted(rng.sample(range(1, len(chain)-1), count))
    split = rng.randint(2, count-2)
    forward_indices = set(rng.sample(indices, split))
    agents = []
    for i, index in enumerate(indices, 1):
        forward = index in forward_indices
        route = chain[index:] if forward else chain[index::-1]
        if len(route) < 2:
            raise ValidationError("Flow agent at corridor endpoint")
        agents.append(humans.agent(f"event_agent_{i}", "event_agent", route,
                                   delay=rng.uniform(0, 3)))
    return generator._robot(chain, routes, corridor), agents, {
        "corridor": corridor,
        "flow_counts": {"forward": split, "reverse": count-split},
    }


def _multi_crossing(generator, routes: RouteSampler, humans: HumanSampler,
                    rng: random.Random, density: str) -> tuple[dict, list[dict], dict]:
    corridor, chain = next((z, c) for z, c in routes.corridor_chains()
                           if z == "corridor_a")
    pairs = []
    for room, anchor, _, _ in generator._room_anchors(routes):
        if anchor not in chain[1:-1]:
            continue
        other = sorted(r for r, a, _, _ in generator._room_anchors(routes)
                       if a == anchor and r != room)
        if other:
            pairs.append((room, other[0], anchor))
    anchors = sorted(set(anchor for _, _, anchor in pairs), key=chain.index)
    count = min({"low": 2, "medium": 3, "high": 4}[density], len(anchors))
    selected = sorted(rng.sample(anchors, count), key=chain.index)
    if rng.choice((True, False)):
        chain = chain[::-1]
    robot = generator._robot(chain, routes, corridor)
    agents = []
    first_side = rng.choice((-1, 1))
    for i, anchor in enumerate(selected, 1):
        side = first_side if i % 2 else -first_side
        anchor_y = generator.inputs.nodes[anchor].y
        choices = sorted((room, goal) for room, goal, at in pairs if at == anchor
                         and (generator.inputs.nodes[f"{room}_center"].y - anchor_y) * side > 0)
        if not choices:
            raise ValidationError("No classroom on requested crossing side")
        room, goal = rng.choice(choices)
        route = routes.shortest(f"{room}_center", f"{goal}_center")
        speed = rng.uniform(*humans.inputs.config["human"]["speed"])
        robot_time = routes.length(chain[:chain.index(anchor)+1]) / generator.inputs.config["robot"]["nominal_speed"]
        human_time = routes.length(route[:route.index(anchor)+1]) / speed
        delay = robot_time-human_time+rng.uniform(-0.3, 0.3)
        if delay < 0:
            raise ValidationError("Cannot time a crossing from this robot start")
        agents.append(humans.agent(f"event_agent_{i}", "event_agent", route,
                                   speed=speed, delay=delay))
    return robot, agents, {"corridor": corridor,
                           "conflict_nodes": selected, "source_rooms": [
                               a["start_node"].removesuffix("_center") for a in agents],
                           "time_tolerance": 1.5}


def _class_burst(generator, routes: RouteSampler, humans: HumanSampler,
                 rng: random.Random, density: str) -> tuple[dict, list[dict], dict]:
    corridor, chain = next((z, c) for z, c in routes.corridor_chains()
                           if z == "corridor_a")
    if rng.choice((True, False)):
        chain = chain[::-1]
    rooms = sorted({room for room, anchor, c, _ in generator._room_anchors(routes)
                    if c == corridor and anchor in chain})
    count = {"low": 3, "medium": 5, "high": 7}[density]
    selected = rng.sample(rooms, count)
    # One early pedestrian followed by a compact wave from distinct classes.
    delays = [rng.uniform(0, 1.0)] + [rng.uniform(4.0, 7.0)
                                       for _ in range(count-1)]
    agents = []
    for i, (room, delay) in enumerate(zip(selected, delays), 1):
        approach = f"{room}_approach"
        endpoint = max((chain[0], chain[-1]),
                       key=lambda node: math.hypot(
                           generator.inputs.nodes[node].x-generator.inputs.nodes[approach].x,
                           generator.inputs.nodes[node].y-generator.inputs.nodes[approach].y))
        route = routes.shortest(f"{room}_center", endpoint)
        agents.append(humans.agent(f"event_agent_{i}", "event_agent", route,
                                   delay=delay))
    return generator._robot(chain, routes, corridor), agents, {
        "corridor": corridor, "source_rooms": selected,
        "exit_delays": [agent["start_delay"] for agent in agents],
        "burst_window": [4.0, 7.0],
    }


def _mixed_interaction(generator, routes: RouteSampler, humans: HumanSampler,
                       rng: random.Random) -> tuple[dict, list[dict], dict]:
    robot, agents, detail = generator._corridor_event("head_on", routes, humans, rng)
    return robot, agents, {
        "corridor": detail["corridor"],
        "primary_interaction": "head_on",
        "conflict_node": agents[0]["route"][len(agents[0]["route"])//2],
        "minimum_background": 2,
    }


def build_event(generator, family: str, routes: RouteSampler,
                humans: HumanSampler, rng: random.Random,
                density: str = "low") -> tuple[dict, list[dict], dict]:
    """Return robot, event humans, and semantic detail for one sampled event."""
    if family in {"walking_group", "opposing_group"}:
        return _corridor_group(generator, family, routes, humans, rng, density)
    if family == "crossing_group":
        return _crossing_group(generator, routes, humans, rng, density)
    if family == "conversation_group":
        return _conversation_group(generator, routes, humans, rng, density)
    if family == "bidirectional_flow":
        return _bidirectional(generator, routes, humans, rng, density)
    if family == "multi_crossing":
        return _multi_crossing(generator, routes, humans, rng, density)
    if family == "class_change_burst":
        return _class_burst(generator, routes, humans, rng, density)
    if family == "mixed_interaction":
        return _mixed_interaction(generator, routes, humans, rng)
    raise ValueError(f"Not a group or mixed traffic scenario: {family}")
