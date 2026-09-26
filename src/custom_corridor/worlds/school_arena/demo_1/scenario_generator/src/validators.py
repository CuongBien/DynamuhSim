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
