#!/usr/bin/env python3
"""Offline acceptance tests for demo_1 geometry, semantics and route topology."""
from __future__ import annotations

import argparse
import collections
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
REQUIRED = [
    "school_floor.world",
    "map.pgm",
    "map.yaml",
    "zones.yaml",
    "human_navigation_graph.yaml",
]
ZONE_TYPES = {
    "corridor", "classroom", "doorway", "intersection",
    "blind_corner", "bottleneck", "waiting_area", "stair",
}


def read_pgm(path: Path):
    with path.open("rb") as stream:
        if stream.readline().strip() != b"P5":
            raise AssertionError("map.pgm must be binary P5")
        line = stream.readline()
        while line.startswith(b"#"):
            line = stream.readline()
        width, height = map(int, line.split())
        assert int(stream.readline()) == 255
        pixels = stream.read()
    assert len(pixels) == width * height
    return width, height, pixels


def shortest(graph, start, goal):
    nodes = {item["id"]: item for item in graph["nodes"]}
    adj = collections.defaultdict(list)
    for edge in graph["edges"]:
        adj[edge["from"]].append(edge["to"])
        if edge.get("bidirectional", False):
            adj[edge["to"]].append(edge["from"])
    queue = collections.deque([(start, [start])])
    seen = {start}
    while queue:
        node, path = queue.popleft()
        if node == goal:
            return path
        for nxt in adj[node]:
            if nxt not in seen:
                seen.add(nxt)
                queue.append((nxt, path + [nxt]))
    raise AssertionError(f"No route {start} -> {goal}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--write-report", action="store_true")
    args = parser.parse_args()

    for name in REQUIRED:
        assert (HERE / name).is_file(), f"Missing {name}"

    root = ET.parse(HERE / "school_floor.world").getroot()
    world = root.find("world")
    assert world is not None and world.get("name") == "school_arena"
    names = {model.get("name", "") for model in world.findall("model")}
    classrooms = {name.split("_")[0] + "_" + name.split("_")[1]
                   for name in names if name.startswith("classroom_")}
    # Original implementation has 15 classrooms plus stair/waiting/corridor
    # functional spaces, i.e. roughly 20 school areas.
    assert len(classrooms) >= 15, f"Only {len(classrooms)} classrooms"
    for feature in ("stair_step_01", "bench_01", "locker_01", "notice_board"):
        assert feature in names, f"Missing Gazebo feature {feature}"
    assert {"demo_primary", "demo_crossing"} <= names

    map_cfg = yaml.safe_load((HERE / "map.yaml").read_text())
    width, height, pixels = read_pgm(HERE / map_cfg["image"])
    resolution = float(map_cfg["resolution"])
    ox, oy, _ = map_cfg["origin"]
    assert math.isclose(resolution, 0.05)
    assert width == 730 and height == 580
    assert 0 in pixels and 254 in pixels

    zones = yaml.safe_load((HERE / "zones.yaml").read_text())
    found_types = {zone["type"] for zone in zones["zones"]}
    assert ZONE_TYPES <= found_types, f"Missing zone types: {ZONE_TYPES - found_types}"
    assert len([z for z in zones["zones"] if z["type"] == "classroom"]) == 15

    graph = yaml.safe_load((HERE / "human_navigation_graph.yaml").read_text())
    assert graph["frame_id"] == "map"
    assert graph["policy"]["random_xy_allowed"] is False
    node_by_id = {node["id"]: node for node in graph["nodes"]}
    assert len(node_by_id) == len(graph["nodes"])

    def occupied(x, y):
        col = int((x - ox) / resolution)
        image_row = height - 1 - int((y - oy) / resolution)
        if not (0 <= col < width and 0 <= image_row < height):
            return True
        return pixels[image_row * width + col] < 128

    def footprint_is_free(x, y, radius=0.22):
        col = int(math.floor((x - ox) / resolution))
        row = int(math.floor((y - oy) / resolution))
        cells = int(math.ceil(radius / resolution))
        for dc in range(-cells, cells + 1):
            for dr in range(-cells, cells + 1):
                dx = max(abs(dc) - 0.5, 0.0) * resolution
                dy = max(abs(dr) - 0.5, 0.0) * resolution
                if math.hypot(dx, dy) > radius + 0.5 * resolution:
                    continue
                wx = ox + (col + dc + 0.5) * resolution
                wy = oy + (row + dr + 0.5) * resolution
                if occupied(wx, wy):
                    return False
        return True

    # Check the whole HuNav footprint every half map cell. A point-only test
    # previously passed routes whose body still overlapped a wall or prop.
    for edge in graph["edges"]:
        a, b = node_by_id[edge["from"]], node_by_id[edge["to"]]
        length = math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        samples = max(2, int(math.ceil(length / (0.5 * resolution))))
        blocked = []
        for i in range(samples + 1):
            t = i / samples
            x = a["x"] + t * (b["x"] - a["x"])
            y = a["y"] + t * (b["y"] - a["y"])
            if not footprint_is_free(x, y):
                blocked.append((round(x, 2), round(y, 2)))
                break
        assert not blocked, f"Edge {edge['from']}->{edge['to']} lacks footprint clearance at {blocked[0]}"

    # Negative controls prove the guard rejects tempting wall-crossing shortcuts.
    forbidden_shortcuts = [
        ((-12.55, -10.60), (-12.55, -8.15)),  # Room01 wall, away from its door
        ((7.00, -4.00), (15.00, 0.00)),       # diagonal through stair core
    ]
    for start, goal in forbidden_shortcuts:
        samples = int(math.ceil(math.dist(start, goal) / (0.5 * resolution)))
        assert any(
            not footprint_is_free(
                start[0] + i / samples * (goal[0] - start[0]),
                start[1] + i / samples * (goal[1] - start[1]),
            )
            for i in range(samples + 1)
        ), f"Wall guard failed to reject shortcut {start}->{goal}"

    results = []
    labels = {
        "robot_a_to_b": "Robot A → B",
        "human_room01_to_room12": "Human Room01 → Room12",
        "human_stair_to_room05": "Human Stair → Room05",
        "human_corridor_to_room": "Human Corridor → Room",
    }
    for test in graph["test_routes"]:
        path = shortest(graph, test["start"], test["goal"])
        assert len(path) >= 2
        results.append((labels[test["id"]], path))

    scenario = yaml.safe_load((HERE / "hunav_one_human.yaml").read_text())
    params = scenario["hunav_loader"]["ros__parameters"]
    assert params["agents"] == ["demo_primary"]
    human = params["demo_primary"]
    assert human["behavior"]["obstacle_force_factor"] > 0
    assert human["behavior"]["social_force_factor"] > 0
    assert human["behavior"]["other_force_factor"] > 0
    assert len(human["goals"]) >= 3

    output = [
        "# Demo 1 — Step 1 acceptance report",
        "",
        "Static geometry/map/topology validation: **PASS**",
        "",
    ]
    for label, path in results:
        print(f"PASS: {label} ({len(path)} nodes)")
        output.append(f"- PASS: {label} ({len(path)} graph nodes)")
    output.extend([
        "",
        "- PASS: full 0.22 m HuNav footprint on every graph edge",
        "- PASS: Room01 wall-crossing shortcut rejected",
        "- PASS: stair-core diagonal shortcut rejected",
        "",
        "> PASS here proves the world/map/topology are consistent and route footprints do",
        "> not intersect occupied map cells. Run nav2_plan_clearance_test.py and",
        "> nav2_baseline_test.py for live Gazebo/Nav2 acceptance.",
        "",
    ])
    if args.write_report:
        (HERE / "STEP1_TEST_REPORT.md").write_text("\n".join(output), encoding="utf-8")
    print("All demo_1 offline acceptance tests PASS")


if __name__ == "__main__":
    main()

