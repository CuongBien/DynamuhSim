"""Use semantic polygons and approved graph nodes as spawn candidates."""
from __future__ import annotations

import math
import random

from .models import Inputs, Node, Zone


def contains(zone: Zone, x: float, y: float, tolerance: float = 1e-6) -> bool:
    poly = zone.polygon
    inside = False
    for (ax, ay), (bx, by) in zip(poly, poly[1:] + poly[:1]):
        cross = (x - ax) * (by - ay) - (y - ay) * (bx - ax)
        if abs(cross) <= tolerance and min(ax, bx) - tolerance <= x <= max(ax, bx) + tolerance and min(ay, by) - tolerance <= y <= max(ay, by) + tolerance:
            return True
        if (ay > y) != (by > y) and x < (bx - ax) * (y - ay) / (by - ay) + ax:
            inside = not inside
    return inside


class ZoneSampler:
    def __init__(self, inputs: Inputs, rng: random.Random):
        self.inputs = inputs
        self.rng = rng

    def containing_zones(self, node: Node, zone_type: str | None = None) -> list[Zone]:
        return [z for z in self.inputs.zones.values()
                if (zone_type is None or z.type == zone_type)
                and contains(z, node.x, node.y)]

    def nodes(self, *, node_type: str | None = None,
              zone_id: str | None = None) -> list[Node]:
        zone = self.inputs.zones.get(zone_id) if zone_id else None
        if zone_id and zone is None:
            raise ValueError(f"Unknown zone {zone_id}")
        return [n for n in self.inputs.nodes.values()
                if (node_type is None or n.type == node_type)
                and (zone is None or contains(zone, n.x, n.y))]

    def choose_node(self, *, node_type: str | None = None,
                    zone_id: str | None = None) -> Node:
        candidates = self.nodes(node_type=node_type, zone_id=zone_id)
        if not candidates:
            raise ValueError(f"No graph nodes for type={node_type}, zone={zone_id}")
        return self.rng.choice(sorted(candidates, key=lambda n: n.id))

    def choose_zone(self, zone_type: str) -> Zone:
        candidates = sorted((z for z in self.inputs.zones.values()
                             if z.type == zone_type), key=lambda z: z.id)
        if not candidates:
            raise ValueError(f"No zone of type {zone_type}")
        return self.rng.choice(candidates)


def distance(a: Node | tuple[float, float], b: Node | tuple[float, float]) -> float:
    ax, ay = (a.x, a.y) if isinstance(a, Node) else a
    bx, by = (b.x, b.y) if isinstance(b, Node) else b
    return math.hypot(ax - bx, ay - by)
