"""Route only along edges declared in human_navigation_graph.yaml."""
from __future__ import annotations

import collections
import math
import random

from .models import Inputs, Node


class RouteError(ValueError):
    pass


class RouteSampler:
    def __init__(self, inputs: Inputs, rng: random.Random):
        self.inputs = inputs
        self.rng = rng
        self.adj = collections.defaultdict(list)
        for edge in inputs.edges:
            self.adj[edge["from"]].append(edge["to"])
            if edge.get("bidirectional", False):
                self.adj[edge["to"]].append(edge["from"])
        for neighbors in self.adj.values():
            neighbors.sort()

    def shortest(self, start: str, goal: str) -> list[str]:
        if start not in self.inputs.nodes or goal not in self.inputs.nodes or start == goal:
            raise RouteError(f"Invalid route endpoints {start} -> {goal}")
        queue = collections.deque([start])
        previous = {start: None}
        while queue:
            current = queue.popleft()
            if current == goal:
                result = []
                while current is not None:
                    result.append(current)
                    current = previous[current]
                return result[::-1]
            for nxt in self.adj[current]:
                if nxt not in previous:
                    previous[nxt] = current
                    queue.append(nxt)
        raise RouteError(f"No graph route {start} -> {goal}")

    def valid(self, route: list[str]) -> bool:
        return (len(route) >= 2 and len(set(route)) == len(route)
                and all(a in self.inputs.nodes and b in self.adj[a]
                        for a, b in zip(route, route[1:])))

    def length(self, route: list[str]) -> float:
        if not self.valid(route):
            raise RouteError(f"Invalid graph route: {route}")
        nodes = self.inputs.nodes
        return sum(math.hypot(nodes[b].x - nodes[a].x, nodes[b].y - nodes[a].y)
                   for a, b in zip(route, route[1:]))

    def heading(self, route: list[str]) -> float:
        a, b = (self.inputs.nodes[k] for k in route[:2])
        return math.atan2(b.y - a.y, b.x - a.x)

    def corridor_chains(self) -> list[tuple[str, list[str]]]:
        # These are graph node sequences inside the two long, straight corridor
        # polygons in the existing school layout. Discover from zone geometry,
        # then order along the long axis instead of storing coordinates.
        chains = []
        for zone in self.inputs.zones.values():
            if zone.type != "corridor":
                continue
            xs = [p[0] for p in zone.polygon]
            ys = [p[1] for p in zone.polygon]
            if max(max(xs)-min(xs), max(ys)-min(ys)) < 8:
                continue
            from .zone_sampler import contains
            nodes = [n for n in self.inputs.nodes.values()
                     if n.type == "corridor" and n.id.startswith(zone.id + "_")
                     and contains(zone, n.x, n.y)]
            axis = "x" if max(xs)-min(xs) > max(ys)-min(ys) else "y"
            nodes.sort(key=lambda n: getattr(n, axis))
            chain = [n.id for n in nodes]
            if len(chain) >= 3 and self.valid(chain):
                chains.append((zone.id, chain))
        return sorted(chains)
