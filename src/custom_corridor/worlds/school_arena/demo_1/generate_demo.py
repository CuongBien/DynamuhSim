#!/usr/bin/env python3
"""Generate the self-contained School Arena demo assets.

The Gazebo geometry is inherited from ../school_arena_hunav.sdf. Occupancy,
semantic zones and the pedestrian graph are generated from one coordinate
model so they stay aligned.
"""
from __future__ import annotations

import copy
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
SOURCE_WORLD = HERE.parent / "school_arena_hunav.sdf"
WORLD = HERE / "school_floor.world"
RESOLUTION = 0.05
ORIGIN = (-16.0, -14.0)
MAXIMUM = (20.5, 15.0)
WIDTH = round((MAXIMUM[0] - ORIGIN[0]) / RESOLUTION)
HEIGHT = round((MAXIMUM[1] - ORIGIN[1]) / RESOLUTION)

# Room bounds and the first valid door center on the corridor-facing wall.
ROOMS: dict[str, dict] = {}
for i in range(6):
    x0 = -15.0 + i * 4.9
    x1 = x0 + 4.9
    ROOMS[f"room{i + 1:02d}"] = {
        "bounds": [x0, -12.6, x1, -8.6],
        "center": [(x0 + x1) / 2, -10.6],
        "door": [x0 + 0.30 * 4.9, -8.6],
        "front": "N",
    }
for i in range(4):
    x0 = -15.0 + i * 4.9
    x1 = x0 + 4.9
    ROOMS[f"room{i + 7:02d}"] = {
        "bounds": [x0, -7.4, x1, -3.4],
        "center": [(x0 + x1) / 2, -5.4],
        "door": [x0 + 0.30 * 4.9, -7.4],
        "front": "S",
    }
east_step = (14.0 - 0.6) / 3
for i in range(3):
    y0 = 0.6 + i * east_step
    y1 = y0 + east_step
    ROOMS[f"room{i + 11:02d}"] = {
        "bounds": [15.6, y0, 19.6, y1],
        "center": [17.6, (y0 + y1) / 2],
        "door": [15.6, y0 + 0.30 * east_step],
        "front": "W",
    }
west_step = (14.0 - 4.6) / 2
for i in range(2):
    y0 = 4.6 + i * west_step
    y1 = y0 + west_step
    ROOMS[f"room{i + 14:02d}"] = {
        "bounds": [10.4, y0, 14.4, y1],
        "center": [12.4, (y0 + y1) / 2],
        "door": [14.4, y0 + 0.30 * west_step],
        "front": "E",
    }

CORRIDOR_NODES = {
    "corridor_a_west": [-14.0, -8.15],
    "corridor_a_1": [-10.0, -7.85],
    "corridor_a_2": [-5.0, -8.15],
    "corridor_a_backpack_south": [-2.0, -8.18],
    "corridor_a_3": [0.0, -7.85],
    "corridor_a_locker_south": [2.6, -8.15],
    "corridor_a_4": [5.0, -8.15],
    "corridor_a_5": [10.0, -8.15],
    "junction_ab": [15.0, -8.15],
    "junction_bc": [15.0, -4.0],
    "junction_cd": [7.0, -4.0],
    "stair_entrance": [7.0, -2.0],
    "junction_de": [7.0, 0.0],
    "junction_ef": [15.0, 0.0],
    # Clearance waypoints steer around the trash bins and backpack in the
    # narrow east corridor. A single x=15.0 centreline was point-free but
    # not footprint-free, so HuNav agents could overlap those obstacles.
    "corridor_f_lower_before_bin": [15.0, 2.55],
    "corridor_f_lower_after_bin": [15.0, 3.50],
    "corridor_f_mid": [15.0, 8.00],
    "corridor_f_before_upper_bin": [15.0, 9.80],
    "corridor_f_after_upper_bin": [15.0, 10.65],
    "corridor_f_upper": [15.0, 12.30],
    "corridor_f_north": [15.0, 13.50],
}


def polygon(x0: float, y0: float, x1: float, y1: float) -> list[list[float]]:
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def apply_clearance_fixes(tree: ET.ElementTree) -> None:
    """Move loose clutter away from doors and merge it with a corridor wall.

    The source arena intentionally contains bottlenecks, but four props left
    less than one HuNav diameter of free space. Keeping them there made any
    collision-free graph mathematically impossible.
    """
    world = tree.getroot().find("world")
    if world is None:
        raise RuntimeError("Source SDF has no world")
    replacements = {
        "bench_01": (-10.50, -8.43),
        "bench_02": (-0.30, -8.43),
        "backpack_01": (-2.00, -7.65),
        "trash_bin_01": (14.28, 3.00),
        "trash_bin_02": (15.72, 10.30),
        "backpack_02": (14.28, 8.30),
    }
    for name, (x, y) in replacements.items():
        model = next((item for item in world.findall("model") if item.get("name") == name), None)
        if model is None:
            raise RuntimeError(f"Missing clearance prop: {name}")
        pose = [float(v) for v in model.findtext("pose", "0 0 0 0 0 0").split()]
        pose += [0.0] * (6 - len(pose))
        pose[0], pose[1] = x, y
        model.find("pose").text = " ".join(f"{value:.4f}" for value in pose)


def add_demo_human_proxies(tree: ET.ElementTree) -> None:
    world = tree.getroot().find("world")
    if world is None:
        raise RuntimeError("Source SDF has no world")
    template = world.find("./model[@name='student_head_on_A']")
    if template is None:
        raise RuntimeError("Source SDF has no HuNav proxy template")
    for name, pose in (
        ("demo_primary", (*ROOMS["room01"]["center"], 0.0)),
        ("demo_crossing", (-1.0, -8.0, 0.0)),
    ):
        if world.find(f"./model[@name='{name}']") is not None:
            continue
        model = copy.deepcopy(template)
        model.set("name", name)
        pose_tag = model.find("pose")
        if pose_tag is not None:
            pose_tag.text = f"{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} 0 0 0"
        world.append(model)


def add_demo_camera(tree: ET.ElementTree) -> None:
    """Open Gazebo looking at the orange robot spawn instead of the origin."""
    world = tree.getroot().find("world")
    if world is None:
        raise RuntimeError("Source SDF has no world")
    old_gui = world.find("gui")
    if old_gui is not None:
        world.remove(old_gui)
    gui = ET.SubElement(world, "gui", {"fullscreen": "0"})
    camera = ET.SubElement(gui, "camera", {"name": "school_demo_camera"})
    ET.SubElement(camera, "pose").text = "-16.0 -8.15 5.5 0 0.68 0"
    ET.SubElement(camera, "view_controller").text = "orbit"
    ET.SubElement(camera, "projection_type").text = "perspective"


def build_world() -> None:
    tree = ET.parse(SOURCE_WORLD)
    apply_clearance_fixes(tree)
    add_demo_human_proxies(tree)
    add_demo_camera(tree)
    ET.indent(tree, space="  ")
    tree.write(WORLD, encoding="utf-8", xml_declaration=True)

    baseline = copy.deepcopy(tree)
    baseline_world = baseline.getroot().find("world")
    for model in list(baseline_world.findall("model")):
        if model.findtext("static", "false").strip().lower() != "true":
            baseline_world.remove(model)
    baseline.write(HERE / "school_floor_baseline.world", encoding="utf-8", xml_declaration=True)


def world_to_grid() -> bytearray:
    # PGM convention: 0 occupied, 254 free. Row zero is the map's north edge.
    grid = bytearray([254]) * (WIDTH * HEIGHT)
    root = ET.parse(WORLD).getroot()
    world = root.find("world")
    if world is None:
        raise RuntimeError("Generated world has no <world>")

    def mark_box(cx: float, cy: float, sx: float, sy: float) -> None:
        # Half-cell padding preserves thin 0.12 m walls at 0.05 m resolution.
        pad = RESOLUTION / 2
        x0, x1 = cx - sx / 2 - pad, cx + sx / 2 + pad
        y0, y1 = cy - sy / 2 - pad, cy + sy / 2 + pad
        c0 = max(0, int(math.floor((x0 - ORIGIN[0]) / RESOLUTION)))
        c1 = min(WIDTH - 1, int(math.ceil((x1 - ORIGIN[0]) / RESOLUTION)))
        r0 = max(0, int(math.floor((MAXIMUM[1] - y1) / RESOLUTION)))
        r1 = min(HEIGHT - 1, int(math.ceil((MAXIMUM[1] - y0) / RESOLUTION)))
        for row in range(r0, r1 + 1):
            start = row * WIDTH + c0
            grid[start:start + c1 - c0 + 1] = bytes([0]) * (c1 - c0 + 1)

    for model in world.findall("model"):
        if model.findtext("static", "false").strip().lower() != "true":
            continue
        if model.get("name") == "ground":
            continue
        mp = [float(v) for v in model.findtext("pose", "0 0 0 0 0 0").split()]
        mp += [0.0] * (6 - len(mp))
        for collision in model.findall("./link/collision"):
            cp = [float(v) for v in collision.findtext("pose", "0 0 0 0 0 0").split()]
            cp += [0.0] * (6 - len(cp))
            geometry = collision.find("geometry")
            if geometry is None:
                continue
            box = geometry.find("box")
            cylinder = geometry.find("cylinder")
            if box is not None:
                size = [float(v) for v in box.findtext("size").split()]
                mark_box(mp[0] + cp[0], mp[1] + cp[1], size[0], size[1])
            elif cylinder is not None:
                radius = float(cylinder.findtext("radius"))
                mark_box(mp[0] + cp[0], mp[1] + cp[1], 2 * radius, 2 * radius)

    # Occupied map border prevents planners from escaping around the building.
    for col in range(WIDTH):
        grid[col] = 0
        grid[(HEIGHT - 1) * WIDTH + col] = 0
    for row in range(HEIGHT):
        grid[row * WIDTH] = 0
        grid[row * WIDTH + WIDTH - 1] = 0
    return grid


def build_map() -> None:
    grid = world_to_grid()
    with (HERE / "map.pgm").open("wb") as stream:
        stream.write(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode())
        stream.write(grid)
    map_yaml = {
        "image": "map.pgm",
        "mode": "trinary",
        "resolution": RESOLUTION,
        "origin": [ORIGIN[0], ORIGIN[1], 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    (HERE / "map.yaml").write_text(
        yaml.safe_dump(map_yaml, sort_keys=False), encoding="utf-8"
    )


def build_zones() -> None:
    zones = []
    for name, room in ROOMS.items():
        zones.append({
            "id": name,
            "type": "classroom",
            "polygon": polygon(*room["bounds"]),
            "attributes": {"capacity": 24, "speed_limit": 0.45},
        })
        dx, dy = room["door"]
        if room["front"] in ("N", "S"):
            door_poly = polygon(dx - 0.45, dy - 0.30, dx + 0.45, dy + 0.30)
        else:
            door_poly = polygon(dx - 0.30, dy - 0.45, dx + 0.30, dy + 0.45)
        zones.append({
            "id": f"{name}_door",
            "type": "doorway",
            "polygon": door_poly,
            "attributes": {"width": 0.9, "yield_required": True},
        })

    zones.extend([
        {"id": "corridor_a", "type": "corridor", "polygon": polygon(-15, -8.6, 15.6, -7.4)},
        {"id": "corridor_b", "type": "corridor", "polygon": polygon(14.4, -7.4, 15.6, -3.4)},
        {"id": "corridor_c", "type": "corridor", "polygon": polygon(6.4, -4.6, 15.6, -3.4)},
        {"id": "corridor_d", "type": "corridor", "polygon": polygon(6.4, -4.6, 7.6, 0.6)},
        {"id": "corridor_e", "type": "corridor", "polygon": polygon(6.4, -0.6, 15.6, 0.6)},
        {"id": "corridor_f", "type": "corridor", "polygon": polygon(14.4, 0.6, 15.6, 14.0)},
        {"id": "turn_ab", "type": "intersection", "polygon": polygon(14.4, -8.6, 15.6, -7.4)},
        {"id": "turn_bc", "type": "blind_corner", "polygon": polygon(14.4, -4.6, 15.6, -3.4)},
        {"id": "turn_cd", "type": "blind_corner", "polygon": polygon(6.4, -4.6, 7.6, -3.4)},
        {"id": "turn_de", "type": "intersection", "polygon": polygon(6.4, -0.6, 7.6, 0.6)},
        {"id": "turn_ef", "type": "blind_corner", "polygon": polygon(14.4, -0.6, 15.6, 0.6)},
        {"id": "bench_bottleneck_west", "type": "bottleneck", "polygon": polygon(-10.4, -8.6, -8.6, -7.4),
         "attributes": {"effective_width": 0.88}},
        {"id": "locker_bottleneck", "type": "bottleneck", "polygon": polygon(-6.0, -8.1, -4.0, -7.4),
         "attributes": {"effective_width": 0.96}},
        {"id": "waiting_area_north", "type": "waiting_area", "polygon": polygon(14.4, 5.3, 15.6, 7.5)},
        {"id": "stair_core", "type": "stair", "polygon": polygon(7.6, -3.4, 14.4, -0.6),
         "attributes": {"robot_access": False, "human_entry_node": "stair_entrance"}},
    ])
    data = {
        "schema_version": 1,
        "frame_id": "map",
        "units": "meters",
        "source_world": "school_floor.world",
        "zones": zones,
    }
    (HERE / "zones.yaml").write_text(
        yaml.safe_dump(data, sort_keys=False), encoding="utf-8"
    )


def nearest_corridor(point: list[float]) -> str:
    return min(
        CORRIDOR_NODES,
        key=lambda key: math.dist(point, CORRIDOR_NODES[key]),
    )


def build_graph() -> dict:
    nodes = []
    edges = []

    def add_node(node_id: str, xy: list[float], node_type: str, zone: str) -> None:
        nodes.append({"id": node_id, "x": round(xy[0], 4), "y": round(xy[1], 4),
                      "type": node_type, "zone": zone})

    def add_edge(a: str, b: str, width: float = 1.2, behavior: str = "walk") -> None:
        lookup = {n["id"]: n for n in nodes}
        length = math.hypot(lookup[a]["x"] - lookup[b]["x"], lookup[a]["y"] - lookup[b]["y"])
        edges.append({"from": a, "to": b, "bidirectional": True,
                      "length": round(length, 4), "width": width,
                      "behavior": behavior})

    for node_id, xy in CORRIDOR_NODES.items():
        node_type = "stair" if node_id == "stair_entrance" else (
            "junction" if node_id.startswith("junction") else "corridor"
        )
        add_node(node_id, xy, node_type, node_id)
    spine = list(CORRIDOR_NODES)
    for a, b in zip(spine, spine[1:]):
        add_edge(a, b, 1.2, "yield_at_corner" if "junction" in a + b else "walk")

    for room_id, room in ROOMS.items():
        room_node = f"{room_id}_center"
        door_node = f"{room_id}_door"
        add_node(room_node, room["center"], "classroom", room_id)
        add_node(door_node, room["door"], "doorway", f"{room_id}_door")
        # Approach the opening perpendicularly. A direct diagonal from a room
        # centre can clip the end of a wall even though both endpoints are free.
        dx, dy = room["door"]
        inward = {"N": [dx, dy - 0.30], "S": [dx, dy + 0.30],
                  "W": [dx + 0.30, dy], "E": [dx - 0.30, dy]}[room["front"]]
        predoor_node = f"{room_id}_predoor"
        add_node(predoor_node, inward, "classroom", room_id)
        add_edge(room_node, predoor_node, 0.9, "approach_door")
        add_edge(predoor_node, door_node, 0.9, "door_transition")
        if room["front"] in ("N", "S"):
            approach = [room["door"][0], -8.15]
        else:
            approach = [15.0, room["door"][1]]
        approach_node = f"{room_id}_approach"
        add_node(approach_node, approach, "corridor", "corridor_merge")
        add_edge(door_node, approach_node, 0.9, "cross_doorway")
        anchor = nearest_corridor(approach)
        add_edge(approach_node, anchor, 1.2, "merge_to_corridor")

    graph = {
        "schema_version": 1,
        "frame_id": "map",
        "directed": False,
        "policy": {
            "random_xy_allowed": False,
            "route_rule": "classroom -> doorway -> corridor -> junction -> doorway -> classroom/stair",
            "wall_crossing": "forbidden",
        },
        "nodes": nodes,
        "edges": edges,
        "test_routes": [
            {"id": "robot_a_to_b", "actor": "robot", "start": "corridor_a_west", "goal": "room12_center"},
            {"id": "human_room01_to_room12", "actor": "human", "start": "room01_center", "goal": "room12_center"},
            {"id": "human_stair_to_room05", "actor": "human", "start": "stair_entrance", "goal": "room05_center"},
            {"id": "human_corridor_to_room", "actor": "human", "start": "corridor_a_3", "goal": "room10_center"},
        ],
    }
    (HERE / "human_navigation_graph.yaml").write_text(
        yaml.safe_dump(graph, sort_keys=False), encoding="utf-8"
    )
    return graph


def route_nodes(graph: dict, start: str, goal: str) -> list[str]:
    nodes = {node["id"] for node in graph["nodes"]}
    adj = {node: [] for node in nodes}
    for edge in graph["edges"]:
        adj[edge["from"]].append(edge["to"])
        adj[edge["to"]].append(edge["from"])
    queue = [(start, [start])]
    seen = {start}
    while queue:
        current, route = queue.pop(0)
        if current == goal:
            return route
        for nxt in adj[current]:
            if nxt not in seen:
                seen.add(nxt)
                queue.append((nxt, route + [nxt]))
    raise RuntimeError(f"No graph route: {start} -> {goal}")


def build_hunav(graph: dict) -> None:
    lookup = {node["id"]: node for node in graph["nodes"]}
    route = route_nodes(graph, "room01_center", "room10_center")
    # Remove the initial node: init_pose already represents it.
    goals = {index + 1: {"x": lookup[node]["x"], "y": lookup[node]["y"]}
             for index, node in enumerate(route[1:])}
    primary = {
        "id": 1, "group_id": -1, "skin": 0, "max_vel": 0.85,
        "radius": 0.22, "goal_radius": 0.25, "cyclic_goals": False,
        "init_pose": {"x": lookup[route[0]]["x"], "y": lookup[route[0]]["y"],
                      "z": 0.0, "h": 0.0},
        "behavior": {
            "type": "Regular", "configuration": 0,
            "goal_force_factor": 2.0, "obstacle_force_factor": 10.0,
            "social_force_factor": 5.0, "other_force_factor": 20.0,
        },
        "goals": list(goals),
    }
    scenario = {
        "hunav_loader": {
            "ros__parameters": {
                "yaml_base_name": "demo_1_hunav",
                "simulator": "Gazebo Fortress",
                "map": "school_arena",
                "publish_people": True,
                "global_goals": goals,
                "agents": ["demo_primary"],
                "demo_primary": primary,
            }
        }
    }
    (HERE / "hunav_one_human.yaml").write_text(
        yaml.safe_dump(scenario, sort_keys=False), encoding="utf-8"
    )

    # A second optional scenario adds a crossing pedestrian so social-force
    # human-human avoidance can be observed, which is impossible with one agent.
    social = copy.deepcopy(scenario)
    params = social["hunav_loader"]["ros__parameters"]
    params["yaml_base_name"] = "demo_1_social"
    next_goal = max(goals) + 1
    params["global_goals"][next_goal] = {"x": 5.0, "y": -8.0}
    params["global_goals"][next_goal + 1] = {"x": -5.0, "y": -8.0}
    params["agents"].append("demo_crossing")
    params["demo_crossing"] = {
        "id": 2, "group_id": -1, "skin": 1, "max_vel": 0.9,
        "radius": 0.22, "goal_radius": 0.25, "cyclic_goals": True,
        "init_pose": {"x": -5.0, "y": -8.0, "z": 0.0, "h": 0.0},
        "behavior": copy.deepcopy(primary["behavior"]),
        "goals": [next_goal, next_goal + 1],
    }
    (HERE / "hunav_social_test.yaml").write_text(
        yaml.safe_dump(social, sort_keys=False), encoding="utf-8"
    )


def main() -> None:
    build_world()
    build_map()
    build_zones()
    graph = build_graph()
    build_hunav(graph)
    print(f"Generated {WORLD.name}, map.pgm, map.yaml, zones.yaml, human_navigation_graph.yaml")
    print(f"Map: {WIDTH}x{HEIGHT} at {RESOLUTION} m/cell")


if __name__ == "__main__":
    main()

