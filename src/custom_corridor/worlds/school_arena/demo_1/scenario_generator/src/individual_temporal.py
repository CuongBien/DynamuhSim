"""Graph-grounded S05–S15 event sampling for the school floor."""
from __future__ import annotations

import math

from .validators import ValidationError
from .zone_sampler import contains


FAMILIES = (
    "enter_room", "merge", "diverge", "overtake_robot", "robot_overtake",
    "stop_resume", "waiting_person", "sudden_entry", "blind_corner",
    "door_bottleneck", "narrow_passing",
)


def _length_to(routes, route: list[str], node: str) -> float:
    if node not in route or route.index(node) == 0:
        raise ValidationError(f"Route does not approach {node}")
    return routes.length(route[:route.index(node) + 1])


def _timed(generator, routes, rng, robot_route, human_route, conflict,
           human_speed, delay_range, tolerance=1.2):
    robot_t = _length_to(routes, robot_route, conflict) / generator.inputs.config["robot"]["nominal_speed"]
    human_t = _length_to(routes, human_route, conflict) / human_speed
    delay = robot_t - human_t + rng.uniform(-0.35, 0.35)
    if not delay_range[0] <= delay <= delay_range[1]:
        raise ValidationError(f"Cannot synchronize at {conflict} within delay interval")
    return round(delay, 4), round(tolerance, 3)


def _anchor_candidates(generator, routes, min_left=1, min_right=1):
    return [(room, anchor, corridor, chain, chain.index(anchor))
            for room, anchor, corridor, chain in generator._room_anchors(routes)
            if chain.index(anchor) >= min_left and len(chain) - chain.index(anchor) - 1 >= min_right]


def _room_event(generator, family, routes, humans, rng):
    if family in {"enter_room", "merge", "diverge"}:
        candidates = _anchor_candidates(generator, routes, 2, 2)
    else:
        candidates = _anchor_candidates(generator, routes, 1, 1)
    if not candidates:
        raise ValidationError("No suitable doorway anchor")
    room, anchor, corridor, chain, idx = rng.choice(candidates)
    door = f"{room}_door"
    template = generator.templates[family]
    delay_bounds = template.get("start_delay", [0.0, 20.0])
    speed = rng.uniform(*generator.inputs.config["human"]["speed"])
    detail = {"room": room, "doorway": door, "corridor": corridor}

    if family == "enter_room":
        robot_route = routes.shortest(chain[idx-2], chain[idx+1])
        human_route = routes.shortest(chain[idx+1], f"{room}_center")
        delay, tolerance = _timed(generator, routes, rng, robot_route, human_route,
                                  anchor, speed, delay_bounds, template["time_tolerance"])
        detail.update(conflict_node=anchor, time_tolerance=tolerance)
    elif family == "merge":
        robot_route = routes.shortest(chain[idx-2], chain[idx+2])
        human_route = routes.shortest(f"{room}_center", chain[idx+2])
        delay, tolerance = _timed(generator, routes, rng, robot_route, human_route,
                                  anchor, speed, delay_bounds, template["time_tolerance"])
        detail.update(merge_node=anchor, shared_downstream=chain[idx+1:idx+3],
                      time_tolerance=tolerance)
    elif family == "diverge":
        robot_route = routes.shortest(chain[idx-2], chain[idx+2])
        human_route = routes.shortest(chain[idx-1], f"{room}_center")
        speed = rng.uniform(*template["human_speed"])
        delay = rng.uniform(0.0, 1.0)
        detail.update(diverge_node=anchor, shared_upstream=chain[idx-1:idx+1])
    elif family == "waiting_person":
        robot_route = routes.shortest(chain[idx-1], chain[idx+1])
        human_route = routes.shortest(f"{room}_predoor", chain[idx+1])
        delay = rng.uniform(*template["wait_duration_sec"])
        detail.update(waiting_node=f"{room}_predoor", wait_duration_sec=round(delay, 4))
    elif family == "sudden_entry":
        robot_route = routes.shortest(chain[idx-1], chain[idx+1])
        human_route = routes.shortest(f"{room}_predoor", chain[idx+1])
        delay, tolerance = _timed(generator, routes, rng, robot_route, human_route,
                                  anchor, speed, delay_bounds, template["time_tolerance"])
        detail.update(conflict_node=anchor, time_tolerance=tolerance,
                      occluded_start_node=f"{room}_predoor")
    elif family == "door_bottleneck":
        robot_route = routes.shortest(chain[idx-1], f"{room}_center")
        human_route = routes.shortest(f"{room}_center", chain[idx+1])
        delay, tolerance = _timed(generator, routes, rng, robot_route, human_route,
                                  door, speed, delay_bounds, template["time_tolerance"])
        detail.update(conflict_node=door, time_tolerance=tolerance)
    else:
        raise ValidationError(f"Unknown room event: {family}")

    event = humans.agent("event_agent_1", "event_agent", human_route,
                         speed=speed, delay=delay)
    return generator._robot(robot_route, routes, corridor), [event], detail


def _overtake(generator, family, routes, humans, rng):
    candidates = [(zone, chain) for zone, chain in routes.corridor_chains() if len(chain) >= 7]
    if not candidates:
        raise ValidationError("No sufficiently long corridor for overtake")
    corridor, chain = rng.choice(candidates)
    if rng.choice((False, True)):
        chain = chain[::-1]
    start = rng.choice((1, 2))
    robot_route = chain[start:]
    if family == "overtake_robot":
        human_route = chain[start-1:]
        speed = rng.uniform(*generator.templates[family]["human_speed"])
        gap = routes.length(chain[start-1:start+1])
        catch = gap * generator.inputs.config["robot"]["nominal_speed"] / (speed - generator.inputs.config["robot"]["nominal_speed"])
    else:
        human_route = chain[start+1:]
        speed = rng.uniform(*generator.templates[family]["human_speed"])
        gap = routes.length(chain[start:start+2])
        catch = gap * generator.inputs.config["robot"]["nominal_speed"] / (generator.inputs.config["robot"]["nominal_speed"] - speed)
    if catch > routes.length(robot_route):
        raise ValidationError("Overtake occurs beyond robot goal")
    lateral = rng.uniform(*generator.inputs.config["human"]["lateral_offset"])
    event = humans.agent("event_agent_1", "event_agent", human_route,
                         speed=speed, lateral=lateral, delay=rng.uniform(0, 0.4))
    detail = {"corridor": corridor, "initial_gap_m": round(gap, 3),
              "catch_distance_m": round(catch, 3)}
    return generator._robot(robot_route, routes, corridor), [event], detail


def _stop_resume(generator, routes, humans, rng):
    corridor, chain = rng.choice([(z, c) for z, c in routes.corridor_chains() if len(c) >= 5])
    if rng.choice((False, True)):
        chain = chain[::-1]
    robot_route = chain
    human_route = chain[1:]
    speed = rng.uniform(*generator.inputs.config["human"]["speed"])
    pause_node = human_route[1]
    pause_after = routes.length(human_route[:2]) / speed
    pause_duration = rng.uniform(*generator.templates["stop_resume"]["pause_duration_sec"])
    event = humans.agent("event_agent_1", "event_agent", human_route,
                         speed=speed, delay=rng.uniform(0, 0.5))
    event["pause_after_sec"] = round(pause_after, 4)
    event["pause_duration_sec"] = round(pause_duration, 4)
    detail = {"corridor": corridor, "pause_node": pause_node,
              "pause_after_sec": event["pause_after_sec"],
              "pause_duration_sec": event["pause_duration_sec"]}
    return generator._robot(robot_route, routes, corridor), [event], detail


def _blind_corner(generator, routes, humans, rng):
    corners = []
    for zone in generator.inputs.zones.values():
        if zone.type != "blind_corner":
            continue
        for node in generator.inputs.nodes.values():
            if node.type != "junction" or not contains(zone, node.x, node.y):
                continue
            neighbors = [n for n in routes.adj[node.id]
                         if generator.inputs.nodes[n].type in {"junction", "corridor"}]
            for robot_in in neighbors:
                for human_in in neighbors:
                    if robot_in == human_in:
                        continue
                    a, c, b = (generator.inputs.nodes[k] for k in (robot_in, node.id, human_in))
                    va = (a.x-c.x, a.y-c.y)
                    vb = (b.x-c.x, b.y-c.y)
                    cosine = (va[0]*vb[0]+va[1]*vb[1]) / (math.hypot(*va)*math.hypot(*vb))
                    if abs(cosine) < 0.5:
                        corners.append((zone.id, node.id, robot_in, human_in))
    if not corners:
        raise ValidationError("No orthogonal blind corner approaches")
    zone, corner, robot_in, human_in = rng.choice(corners)
    # One predecessor gives room for positive start delay when the other branch
    # is longer. The graph determines the predecessor; no world XY is sampled.
    predecessors = [n for n in routes.adj[robot_in] if n != corner
                    and generator.inputs.nodes[n].type in {"junction", "corridor"}]
    robot_start = rng.choice(predecessors) if predecessors else robot_in
    robot_route = routes.shortest(robot_start, human_in)
    human_route = routes.shortest(human_in, robot_in)
    if corner not in robot_route or corner not in human_route:
        raise ValidationError("Blind corner route bypasses corner")
    speed = rng.uniform(*generator.inputs.config["human"]["speed"])
    template = generator.templates["blind_corner"]
    delay, tolerance = _timed(generator, routes, rng, robot_route, human_route,
                              corner, speed, template["start_delay"], template["time_tolerance"])
    event = humans.agent("event_agent_1", "event_agent", human_route,
                         speed=speed, delay=delay)
    detail = {"corner": corner, "corner_zone": zone,
              "robot_incoming": robot_in, "human_incoming": human_in,
              "conflict_node": corner, "time_tolerance": tolerance}
    return generator._robot(robot_route, routes, zone), [event], detail


def _narrow_passing(generator, routes, humans, rng):
    candidates = []
    for corridor, chain in routes.corridor_chains():
        for index in range(1, len(chain)-1):
            node = generator.inputs.nodes[chain[index]]
            for zone in generator.inputs.zones.values():
                if zone.type == "bottleneck" and contains(zone, node.x, node.y):
                    candidates.append((corridor, chain, index, zone.id))
    if not candidates:
        raise ValidationError("No graph node inside bottleneck zone")
    corridor, chain, index, zone = rng.choice(candidates)
    robot_route = chain[index-1:index+2]
    human_route = robot_route[::-1]
    event = humans.agent("event_agent_1", "event_agent", human_route,
                         delay=rng.uniform(0, 1),
                         lateral=rng.uniform(*generator.inputs.config["human"]["lateral_offset"]))
    detail = {"corridor": corridor, "bottleneck_zone": zone,
              "bottleneck_node": chain[index]}
    return generator._robot(robot_route, routes, corridor), [event], detail


def build_event(generator, family: str, routes, humans, rng):
    """Return ``robot, [event_agent], event_detail`` for one S05–S15 family."""
    if family in {"enter_room", "merge", "diverge", "waiting_person",
                  "sudden_entry", "door_bottleneck"}:
        return _room_event(generator, family, routes, humans, rng)
    if family in {"overtake_robot", "robot_overtake"}:
        return _overtake(generator, family, routes, humans, rng)
    if family == "stop_resume":
        return _stop_resume(generator, routes, humans, rng)
    if family == "blind_corner":
        return _blind_corner(generator, routes, humans, rng)
    if family == "narrow_passing":
        return _narrow_passing(generator, routes, humans, rng)
    raise ValueError(f"Unknown individual/temporal family: {family}")
