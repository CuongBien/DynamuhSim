"""Offline footprint, graph and social event checks before writing an episode."""
from __future__ import annotations

import math
from pathlib import Path

from .models import Inputs
from .route_sampler import RouteSampler
from .zone_sampler import contains


class ValidationError(ValueError):
    pass


class OccupancyMap:
    def __init__(self, inputs: Inputs):
        raw = inputs.paths["map_image"].read_bytes()
        import io
        stream = io.BytesIO(raw)
        if stream.readline().strip() != b"P5":
            raise ValidationError("Only binary P5 occupancy maps are supported")
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        self.width, self.height = map(int, line.split())
        if int(stream.readline()) != 255:
            raise ValidationError("Invalid PGM depth")
        self.pixels = stream.read()
        if len(self.pixels) != self.width * self.height:
            raise ValidationError("Invalid PGM size")
        self.resolution = float(inputs.map_config["resolution"])
        self.ox, self.oy = map(float, inputs.map_config["origin"][:2])
        self._edge_cache = {}

    def occupied(self, x: float, y: float) -> bool:
        col = int(math.floor((x-self.ox)/self.resolution))
        row = self.height - 1 - int(math.floor((y-self.oy)/self.resolution))
        return (not (0 <= col < self.width and 0 <= row < self.height)
                or self.pixels[row*self.width+col] < 128)

    def footprint_free(self, x: float, y: float, radius: float) -> bool:
        cells = int(math.ceil(radius/self.resolution))
        col = int(math.floor((x-self.ox)/self.resolution))
        row = int(math.floor((y-self.oy)/self.resolution))
        for dc in range(-cells, cells+1):
            for dr in range(-cells, cells+1):
                dx = max(abs(dc)-0.5, 0.0)*self.resolution
                dy = max(abs(dr)-0.5, 0.0)*self.resolution
                if math.hypot(dx, dy) > radius + 0.5*self.resolution:
                    continue
                wx = self.ox + (col+dc+0.5)*self.resolution
                wy = self.oy + (row+dr+0.5)*self.resolution
                if self.occupied(wx, wy):
                    return False
        return True

    def edge_free(self, a, b, radius: float) -> bool:
        key = (a.id, b.id, radius)
        if key not in self._edge_cache:
            distance = math.hypot(b.x-a.x, b.y-a.y)
            steps = max(1, math.ceil(distance/(0.5*self.resolution)))
            self._edge_cache[key] = all(
                self.footprint_free(a.x+(b.x-a.x)*i/steps,
                                    a.y+(b.y-a.y)*i/steps, radius)
                for i in range(steps+1)
            )
        return self._edge_cache[key]


def _angle_gap(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def _directed_edges(route: list[str]) -> set[tuple[str, str]]:
    return set(zip(route, route[1:]))


def _shared_forward(a: list[str], b: list[str]) -> bool:
    return bool(_directed_edges(a) & _directed_edges(b))


def _shared_opposing(a: list[str], b: list[str]) -> bool:
    return bool(_directed_edges(a) & {(v, u) for u, v in _directed_edges(b)})


def _arrival(routes: RouteSampler, route: list[str], node: str,
             speed: float, delay: float = 0.0) -> float:
    if node not in route or speed <= 0:
        raise ValidationError(f"Cannot arrive at {node}")
    index = route.index(node)
    return delay + (routes.length(route[:index + 1]) / speed if index else 0.0)


def _room_transition(inputs: Inputs, route: list[str], *, exiting: bool) -> str | None:
    endpoint = route[0] if exiting else route[-1]
    room = inputs.nodes[endpoint].zone
    if room not in inputs.zones or inputs.zones[room].type != "classroom":
        return None
    names = [f"{room}_{suffix}" for suffix in ("predoor", "door", "approach")]
    if not all(name in route for name in names):
        return None
    indices = [route.index(name) for name in names]
    if exiting and not indices[0] < indices[1] < indices[2]:
        return None
    if not exiting and not indices[2] < indices[1] < indices[0]:
        return None
    return room


def _group_spacing(humans: list[dict], *, maximum: float = 3.0) -> bool:
    points = [(h["spawn"]["x"], h["spawn"]["y"]) for h in humans]
    return all(math.hypot(x - u, y - v) <= maximum
               for x, y in points for u, v in points)


def validate_episode(inputs: Inputs, routes: RouteSampler, occupancy: OccupancyMap,
                     scenario: dict) -> None:
    robot = scenario["robot"]
    robot_route = robot["route"]
    if not routes.valid(robot_route):
        raise ValidationError("Robot route is not on graph")
    robot_node = inputs.nodes[robot_route[0]]
    for a, b in zip(robot_route, robot_route[1:]):
        if not occupancy.edge_free(inputs.nodes[a], inputs.nodes[b], inputs.config["robot"]["radius"]):
            raise ValidationError(f"Robot route edge crosses occupancy: {a}->{b}")
    if not occupancy.footprint_free(robot_node.x, robot_node.y,
                                    inputs.config["robot"]["radius"]):
        raise ValidationError("Robot footprint intersects wall")
    if not any(contains(z, robot_node.x, robot_node.y)
               for z in inputs.zones.values() if z.type != "stair"):
        raise ValidationError("Robot spawn outside allowed semantic zone")
    spawn_points = [(robot_node.x, robot_node.y, inputs.config["robot"]["radius"], "robot")]
    for human in scenario["humans"]:
        route = human["route"]
        if not routes.valid(route) or route[0] != human["start_node"] or route[-1] != human["goal_node"]:
            raise ValidationError(f"Invalid route for {human['name']}")
        for a, b in zip(route, route[1:]):
            if not occupancy.edge_free(inputs.nodes[a], inputs.nodes[b], inputs.config["human"]["radius"]):
                raise ValidationError(f"Human route edge crosses occupancy: {a}->{b}")
        start = human["spawn"]
        point = (start["x"], start["y"])
        node = inputs.nodes[route[0]]
        possible = [z for z in inputs.zones.values() if contains(z, node.x, node.y)]
        if not possible or not any(contains(z, *point) for z in possible):
            raise ValidationError(f"Spawn outside semantic zone: {human['name']}")
        radius = inputs.config["human"]["radius"]
        if not occupancy.footprint_free(*point, radius):
            raise ValidationError(f"Spawn intersects wall: {human['name']}")
        if not occupancy.footprint_free(inputs.nodes[route[-1]].x,
                                        inputs.nodes[route[-1]].y, radius):
            raise ValidationError(f"Goal intersects wall: {human['name']}")
        for x, y, other_radius, other in spawn_points:
            if math.hypot(point[0]-x, point[1]-y) < radius + other_radius + 0.05:
                raise ValidationError(f"Spawn overlap {human['name']} / {other}")
        spawn_points.append((*point, radius, human["name"]))
    family = scenario["scenario_family"]
    events = [h for h in scenario["humans"] if h["role"] == "event_agent"]
    if family == "empty":
        if scenario["humans"]:
            raise ValidationError("EMPTY must have no humans")
        return
    if family not in {"head_on", "same_direction", "crossing", "exit_room"}:
        _validate_new_family(inputs, routes, scenario, events)
        return
    if len(events) != 1:
        raise ValidationError("Expected one event agent")
    event = events[0]
    overlap = set(robot_route) & set(event["route"])
    if family == "head_on":
        if (scenario["event"]["corridor"] not in inputs.zones
                or len(overlap) < 2
                or _angle_gap(robot["heading"], event["spawn"]["heading"]) < 2.3):
            raise ValidationError("HEAD_ON corridor/heading/route constraint failed")
    elif family == "same_direction":
        robot_speed = inputs.config["robot"]["nominal_speed"]
        gap = scenario["event"]["initial_gap_m"]
        catch_distance = gap * robot_speed / max(robot_speed-event["speed"], 1e-6)
        if (len(overlap) < 2 or _angle_gap(robot["heading"], event["spawn"]["heading"]) > 0.7
                or event["speed"] >= robot_speed
                or catch_distance > routes.length(robot_route)):
            raise ValidationError("SAME_DIRECTION constraint failed")
    elif family in {"crossing", "exit_room"}:
        room = scenario["event"]["room"]
        if (inputs.nodes[event["start_node"]].zone != room
                or inputs.zones[room].type != "classroom"
                or f"{room}_door" not in event["route"]
                or f"{room}_approach" not in event["route"]
                or not overlap):
            raise ValidationError(f"{family} room/door/corridor constraint failed")
        if family == "exit_room" and inputs.nodes[event["goal_node"]].type != "corridor":
            raise ValidationError("EXIT_ROOM human goal must be in corridor")
        if family == "crossing" and inputs.nodes[event["goal_node"]].type != "classroom":
            raise ValidationError("CROSSING human goal must be a classroom")
        conflict = scenario["event"]["conflict_node"]
        if conflict not in overlap:
            raise ValidationError("Trajectories do not intersect at conflict node")
        def arrival(route, speed):
            return routes.length(route[:route.index(conflict)+1])/speed
        robot_t = arrival(robot_route, inputs.config["robot"]["nominal_speed"])
        human_t = event["start_delay"] + arrival(event["route"], event["speed"])
        if abs(robot_t-human_t) > scenario["event"]["time_tolerance"]:
            raise ValidationError("Trajectories do not intersect in time")
    else:
        raise ValidationError(f"Unknown scenario {family}")


def _validate_new_family(inputs: Inputs, routes: RouteSampler,
                         scenario: dict, events: list[dict]) -> None:
    family = scenario["scenario_family"]
    detail = scenario["event"]
    robot = scenario["robot"]
    rr = robot["route"]
    if family not in {"walking_group", "opposing_group", "crossing_group",
                      "conversation_group", "bidirectional_flow", "multi_crossing",
                      "class_change_burst"} and len(events) != 1:
        raise ValidationError(f"{family} requires one event agent")
    if not events:
        raise ValidationError(f"{family} requires event agents")
    event = events[0]
    hr = event["route"]

    if family in {"enter_room", "sudden_entry", "door_bottleneck"}:
        room = detail.get("room")
        door = detail.get("doorway")
        conflict = detail.get("conflict_node")
        if room not in inputs.zones or inputs.zones[room].type != "classroom":
            raise ValidationError(f"{family} needs a classroom")
        if door != f"{room}_door" or door not in hr:
            raise ValidationError(f"{family} misses named doorway")
        if conflict not in rr or conflict not in hr:
            raise ValidationError(f"{family} routes miss conflict node")
        if family == "enter_room":
            if _room_transition(inputs, hr, exiting=False) != room:
                raise ValidationError("ENTER_ROOM must enter the named classroom")
        elif _room_transition(inputs, hr, exiting=True) != room:
            raise ValidationError(f"{family} must exit the named classroom")
        if family == "door_bottleneck" and door not in rr:
            raise ValidationError("DOOR_BOTTLENECK robot must traverse doorway")
        if family == "sudden_entry" and (detail.get("occluded_start_node") != hr[0]
                                         or inputs.nodes[hr[0]].type != "classroom"):
            raise ValidationError("SUDDEN_ENTRY needs an occluded classroom start")
        tolerance = detail.get("time_tolerance")
        if (not isinstance(tolerance, (int, float)) or tolerance <= 0
                or abs(_arrival(routes, rr, conflict, inputs.config["robot"]["nominal_speed"])
                       - _arrival(routes, hr, conflict, event["speed"], event["start_delay"])) > tolerance):
            raise ValidationError(f"{family} trajectories miss conflict in time")
    elif family == "merge":
        merge = detail.get("merge_node")
        if (_room_transition(inputs, hr, exiting=True) != detail.get("room")
                or merge not in rr or merge not in hr
                or rr[0] == hr[0] or not _shared_forward(rr, hr)
                or not _shared_forward(rr[rr.index(merge):], hr[hr.index(merge):])):
            raise ValidationError("MERGE needs different sources and shared downstream edge")
        if detail.get("shared_downstream") is not None:
            downstream = detail["shared_downstream"]
            if not isinstance(downstream, list) or not set(_directed_edges(downstream)) <= (_directed_edges(rr) & _directed_edges(hr)):
                raise ValidationError("MERGE downstream annotation is inconsistent")
    elif family == "diverge":
        fork = detail.get("diverge_node")
        if (_room_transition(inputs, hr, exiting=False) != detail.get("room")
                or fork not in rr or fork not in hr or not _shared_forward(rr, hr)
                or not _shared_forward(rr[:rr.index(fork)+1], hr[:hr.index(fork)+1])
                or rr[rr.index(fork)+1:rr.index(fork)+2] == hr[hr.index(fork)+1:hr.index(fork)+2]):
            raise ValidationError("DIVERGE needs shared upstream edge and distinct branches")
    elif family in {"overtake_robot", "robot_overtake"}:
        robot_speed = inputs.config["robot"]["nominal_speed"]
        gap = detail.get("initial_gap_m")
        if (not _shared_forward(rr, hr) or not isinstance(gap, (int, float)) or gap <= 0
                or _angle_gap(robot["heading"], event["spawn"]["heading"]) > 0.7):
            raise ValidationError(f"{family} needs aligned motion with initial gap")
        if family == "overtake_robot":
            if event["speed"] <= robot_speed or rr[0] not in hr or hr.index(rr[0]) == 0:
                raise ValidationError("OVERTAKE_ROBOT human must start behind and move faster")
            catch = gap * robot_speed / (event["speed"] - robot_speed)
        else:
            if event["speed"] >= robot_speed or hr[0] not in rr or rr.index(hr[0]) == 0:
                raise ValidationError("ROBOT_OVERTAKE human must start ahead and move slower")
            catch = gap * robot_speed / (robot_speed - event["speed"])
        if (catch > routes.length(rr)
                or abs(catch - detail.get("catch_distance_m", -1)) > 0.1):
            raise ValidationError(f"{family} catch point is beyond robot route")
    elif family == "stop_resume":
        pause = detail.get("pause_node")
        duration = detail.get("pause_duration_sec")
        after = detail.get("pause_after_sec")
        if (pause not in hr or pause == hr[0] or pause == hr[-1]
                or not _shared_forward(rr, hr)
                or not isinstance(duration, (int, float)) or duration <= 0
                or not isinstance(after, (int, float)) or after < 0
                or event.get("pause_duration_sec") != duration
                or event.get("pause_after_sec") != after
                or abs(after - routes.length(hr[:hr.index(pause)+1]) / event["speed"]) > 0.01):
            raise ValidationError("STOP_RESUME needs a positive pause on the interaction route")
    elif family == "waiting_person":
        node = detail.get("waiting_node")
        duration = detail.get("wait_duration_sec")
        room = detail.get("room")
        if (node not in hr or room not in inputs.zones
                or detail.get("doorway") != f"{room}_door"
                or _room_transition(inputs, hr, exiting=True) != room
                or f"{room}_approach" not in hr
                or not set(hr) & set(rr)
                or not isinstance(duration, (int, float)) or duration <= 0
                or abs(event["start_delay"] - duration) > 0.01):
            raise ValidationError("WAITING_PERSON needs a door-side wait near robot route")
    elif family == "blind_corner":
        corner = detail.get("corner")
        zone = inputs.zones.get(detail.get("corner_zone"))
        if (corner not in rr or corner not in hr or zone is None or zone.type != "blind_corner"
                or not contains(zone, inputs.nodes[corner].x, inputs.nodes[corner].y)
                or rr.index(corner) == 0 or hr.index(corner) == 0):
            raise ValidationError("BLIND_CORNER needs both routes approaching a blind corner")
        ri, hi = rr[rr.index(corner)-1], hr[hr.index(corner)-1]
        if (ri == hi or detail.get("robot_incoming") != ri
                or detail.get("human_incoming") != hi):
            raise ValidationError("BLIND_CORNER needs different incoming branches")
        c = inputs.nodes[corner]
        r, h = inputs.nodes[ri], inputs.nodes[hi]
        dot = ((r.x-c.x)*(h.x-c.x) + (r.y-c.y)*(h.y-c.y))
        lengths = math.hypot(r.x-c.x, r.y-c.y)*math.hypot(h.x-c.x, h.y-c.y)
        if lengths == 0 or abs(dot/lengths) > 0.5:
            raise ValidationError("BLIND_CORNER branches must meet at an angle")
        tolerance = detail.get("time_tolerance")
        if (not isinstance(tolerance, (int, float)) or tolerance <= 0
                or abs(_arrival(routes, rr, corner, inputs.config["robot"]["nominal_speed"])
                       - _arrival(routes, hr, corner, event["speed"], event["start_delay"])) > tolerance):
            raise ValidationError("BLIND_CORNER arrival times do not overlap")
    elif family == "narrow_passing":
        zone = inputs.zones.get(detail.get("bottleneck_zone"))
        node = detail.get("bottleneck_node")
        if (zone is None or zone.type != "bottleneck" or node not in rr or node not in hr
                or not _shared_opposing(rr, hr)):
            raise ValidationError("NARROW_PASSING needs opposing routes through bottleneck")
        n = inputs.nodes[node]
        if not contains(zone, n.x, n.y):
            raise ValidationError("NARROW_PASSING node is outside bottleneck zone")
    else:
        _validate_multi_family(inputs, routes, scenario, events)


def _validate_multi_family(inputs: Inputs, routes: RouteSampler,
                           scenario: dict, events: list[dict]) -> None:
    family = scenario["scenario_family"]
    detail = scenario["event"]
    robot_route = scenario["robot"]["route"]
    if family in {"walking_group", "opposing_group", "crossing_group", "conversation_group"}:
        group_id = detail.get("group_id")
        if (not 2 <= len(events) <= 4 or not isinstance(group_id, int) or group_id < 0
                or detail.get("group_size") != len(events)
                or any(h.get("group_id") != group_id for h in events)
                or not _group_spacing(events)):
            raise ValidationError(f"{family} needs a coherent group of 2–4")
        if family != "conversation_group" and len({h["goal_node"] for h in events}) != 1:
            raise ValidationError(f"{family} members need compatible goals")
        if family == "walking_group":
            if not all(_shared_forward(robot_route, h["route"]) for h in events):
                raise ValidationError("WALKING_GROUP must share robot corridor direction")
        elif family == "opposing_group":
            if not all(_shared_opposing(robot_route, h["route"]) for h in events):
                raise ValidationError("OPPOSING_GROUP must move against robot")
        elif family == "crossing_group":
            conflict = detail.get("conflict_node")
            if (conflict not in robot_route
                    or len({_room_transition(inputs, h["route"], exiting=True) for h in events}) != 1
                    or not all(conflict in h["route"] for h in events)
                    or not all(_room_transition(inputs, h["route"], exiting=False) == detail.get("goal_room")
                               for h in events)):
                raise ValidationError("CROSSING_GROUP must cross the same robot conflict point")
        else:
            obstacle = detail.get("social_obstacle_node")
            zone = inputs.zones.get(detail.get("waiting_zone"))
            if (zone is None or zone.type != "waiting_area" or obstacle not in inputs.nodes
                    or not contains(zone, inputs.nodes[obstacle].x, inputs.nodes[obstacle].y)
                    or not all(obstacle in h["route"] and contains(zone, h["spawn"]["x"], h["spawn"]["y"])
                               and h["start_delay"] >= 15.0 for h in events)
                    or not any(contains(zone, inputs.nodes[a].x, inputs.nodes[a].y)
                               or contains(zone, inputs.nodes[b].x, inputs.nodes[b].y)
                               or (min(inputs.nodes[a].y, inputs.nodes[b].y) <= inputs.nodes[obstacle].y
                                   <= max(inputs.nodes[a].y, inputs.nodes[b].y))
                               for a, b in zip(robot_route, robot_route[1:]))):
                raise ValidationError("CONVERSATION_GROUP must wait in area crossed by robot")
    elif family == "bidirectional_flow":
        counts = detail.get("flow_counts")
        if (len(events) < 2 or not isinstance(counts, dict)
                or counts.get("forward", 0) < 1 or counts.get("reverse", 0) < 1
                or counts.get("forward", 0) + counts.get("reverse", 0) != len(events)):
            raise ValidationError("BIDIRECTIONAL_FLOW needs humans in both directions")
        forward = sum(_shared_forward(robot_route, h["route"]) for h in events)
        reverse = sum(_shared_opposing(robot_route, h["route"]) for h in events)
        if forward != counts["forward"] or reverse != counts["reverse"]:
            raise ValidationError("BIDIRECTIONAL_FLOW direction counts mismatch")
    elif family == "multi_crossing":
        conflicts = detail.get("conflict_nodes")
        sides = {1 if inputs.nodes[h["start_node"]].y > inputs.nodes[conflict].y else -1
                 for h, conflict in zip(events, conflicts or [])}
        if (len(events) < 2 or not isinstance(conflicts, list) or len(set(conflicts)) < 2
                or len(conflicts) != len(events) or sides != {-1, 1}
                or not set(conflicts) <= set(robot_route)
                or not all(conflict in h["route"] for h, conflict in zip(events, conflicts))
                or len({h["start_node"] for h in events}) < 2):
            raise ValidationError("MULTI_CROSSING needs multiple sources and crossings")
    elif family == "class_change_burst":
        source_rooms = detail.get("source_rooms")
        delays = detail.get("exit_delays")
        actual_rooms = [_room_transition(inputs, h["route"], exiting=True) for h in events]
        actual_delays = [h["start_delay"] for h in events]
        if (len(events) < 3 or not isinstance(source_rooms, list)
                or not isinstance(delays, list) or len(source_rooms) != len(events)
                or len(delays) != len(events) or actual_rooms != source_rooms
                or any(room is None for room in actual_rooms)
                or len(set(actual_rooms)) < 2 or len(set(actual_delays)) < 2
                or min(actual_delays) > 1.5
                or sum(4.0 <= d <= 7.5 for d in actual_delays) < 2
                or any(abs(a-b) > 1e-4 for a, b in zip(delays, actual_delays))
                or not all(any(inputs.nodes[n].type == "corridor" and
                               inputs.nodes[n].zone.startswith("corridor") for n in h["route"])
                           for h in events)):
            raise ValidationError("CLASS_CHANGE_BURST needs staggered exits from multiple rooms")
    elif family == "mixed_interaction":
        conflict = detail.get("conflict_node")
        backgrounds = [h for h in scenario["humans"] if h["role"] == "background_agent"]
        minimum = detail.get("minimum_background")
        if (len(events) != 1 or not isinstance(minimum, int) or minimum < 1
                or len(backgrounds) < minimum or conflict not in robot_route
                or conflict not in events[0]["route"]
                or detail.get("primary_interaction") != "head_on"
                or not _shared_opposing(robot_route, events[0]["route"])):
            raise ValidationError("MIXED_INTERACTION needs opposing primary event plus background traffic")
    else:
        raise ValidationError(f"Unknown scenario {family}")
