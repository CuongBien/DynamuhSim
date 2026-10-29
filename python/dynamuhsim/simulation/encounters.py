"""Explicit, opt-in initial conditions for close-encounter group regression.

Only initialization changes. Subsequent native human goals/lifecycle, robot
Nav2 control, ORCA and collision projection continue normally.
"""
import json
import math
from pathlib import Path

PROFILES = ("head_on", "crossing")

def fixture(profile):
    if profile == "head_on":
        centers = [[-24.5, .35], [-20.5, -.35]]
        goals = [[-28.2, .35], [-27.5, -.35]]
        headings = [math.pi, math.pi]
    elif profile == "crossing":
        centers = [[-25.1, -3.5], [-24.6, 4.5]]
        goals = [[-25.1, 4.5], [-24.6, -3.5]]
        headings = [math.pi / 2, -math.pi / 2]
    else:
        raise ValueError("Unknown encounter profile")
    return dict(schema="dynamuhsim_encounter/1", profile=profile,
                formation_radius_m=.72, human_radius_m=.36,
                robot_start=[-28., 0.], robot_response_radius_m=.25,
                groups=[dict(members=members, center=center, goal=goal, yaw=yaw)
                        for members, center, goal, yaw in zip([[1,2,3],[4,5,6]], centers, goals, headings)])


def validate_fixture(value):
    if not isinstance(value, dict) or value != fixture(value.get("profile")):
        raise ValueError("Undeclared encounter fixture mutation")
    return fixture(value["profile"])


def initial_humans(spec):
    validate_fixture(spec)
    return [dict(id=hid, x=g["center"][0] + spec["formation_radius_m"] * math.cos(i * 2 * math.pi / 3),
                 y=g["center"][1] + spec["formation_radius_m"] * math.sin(i * 2 * math.pi / 3),
                 yaw=g["yaw"]) for g in spec["groups"] for i, hid in enumerate(g["members"])]


def initial_clearance(spec, boxes):
    humans=initial_humans(spec)
    static=pair=robot=math.inf
    radius=spec["human_radius_m"]
    for i,h in enumerate(humans):
        for box in boxes:
            dx=abs(h["x"]-box["x"])-box["sx"]/2
            dy=abs(h["y"]-box["y"])-box["sy"]/2
            static=min(static, math.hypot(max(dx,0), max(dy,0)) + min(max(dx,dy),0) - radius)
        robot=min(robot,math.hypot(h["x"]-spec["robot_start"][0], h["y"]-spec["robot_start"][1])-radius-spec["robot_response_radius_m"])
        for other in humans[:i]:
            pair=min(pair,math.hypot(h["x"]-other["x"],h["y"]-other["y"])-2*radius)
    if min(static,pair,robot) <= .002:
        raise ValueError("Initial encounter intersects native geometry")
    return dict(human_static_gap_m=static,human_human_gap_m=pair,human_robot_gap_m=robot)


def initialize(kernel, spec):
    validate_fixture(spec)
    if kernel.origin != (0.,0.):
        raise ValueError("Encounter fixture requires factory world coordinates")
    initial_clearance(spec,kernel.boxes)
    e=kernel.env
    if e.static_interactions or e.static_humans or e.dynamic_humans or e.h_l_interactions:
        raise ValueError("Encounter fixture requires two initial moving groups")
    groups={tuple(sorted(int(h.id) for h in g.humans)):g for g in e.moving_interactions}
    if set(groups) != {(1,2,3),(4,5,6)}:
        raise ValueError("Encounter membership differs from fixture")
    positions={h["id"]:h for h in initial_humans(spec)}
    for row in spec["groups"]:
        g=groups[tuple(row["members"])]
        g.x,g.y=row["center"]
        for h in g.humans:
            p=positions[int(h.id)]
            h.x=h.initial_x=h.prev_x=p["x"]
            h.y=h.initial_y=h.prev_y=p["y"]
            h.orientation=h.initial_orientation=h.prev_orientation=p["yaw"]
            h.speed=0.;h.position_history.clear()
        g.set_goal(*row["goal"])
        for h in g.humans:
            e.goals[h.id].x,e.goals[h.id].y=row["goal"]
    kernel.motion.velocities={}
    kernel.initial=kernel.humans()
    return dict(profile=spec["profile"],initial_clearance=initial_clearance(spec,kernel.boxes),initial_groups=group_goals(kernel))


def group_goals(kernel):
    return [dict(members=sorted(int(h.id) for h in g.humans),goal=[float(g.goal_x),float(g.goal_y)]) for g in kernel.env.moving_interactions]


def load_fixture(kernel,runtime):
    path=Path(runtime)/"encounter.json"
    return initialize(kernel,json.loads(path.read_text())) if path.exists() else None
