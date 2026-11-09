"""Scenario labels and fixed schedules over offline world-frame observations."""

from bisect import bisect_right
from itertools import pairwise
from math import cos, degrees, dist, isfinite, pi, sin


def schedule(entries, dt_ns, duration_ns):
    if not isinstance(entries, list) or not entries:
        raise ValueError("a fixed robot schedule is required")
    times, commands = [], []
    for entry in entries:
        time, body = entry["at_ns"], entry["body"]
        if type(time) is not int or not 0 <= time < duration_ns or time % dt_ns:
            raise ValueError(
                "robot command must fall on a control tick before episode end"
            )
        if len(body) != 3 or any(not isfinite(value) for value in body):
            raise ValueError("robot command must be a finite body-frame twist")
        wait = entry.get("intentional_wait", False)
        if type(wait) is not bool or (wait and any(body)):
            raise ValueError("intentional waiting must be an explicit zero command")
        times.append(time)
        commands.append((tuple(body), wait))
    if times[0] != 0 or sorted(set(times)) != times or commands[-1][1]:
        raise ValueError(
            "schedule must start at zero, be ordered, and resume after waiting"
        )
    return times, commands


def command_at(schedule_data, time_ns):
    times, commands = schedule_data
    return commands[bisect_right(times, time_ns) - 1]


def _heading(a, b):
    return abs(degrees((a["yaw"] - b["yaw"] + pi) % (2 * pi) - pi))


def _crosses(path, center, axis, width):
    lateral = 1 - axis
    for before, after in pairwise(path):
        a, b = before[axis] - center[axis], after[axis] - center[axis]
        if a != b and a * b <= 0:
            value = before[lateral] + (after[lateral] - before[lateral]) * (
                -a / (b - a)
            )
            if abs(value - center[lateral]) <= width / 2:
                return True
    return False


def scenario_label(card, binding, rows, visible=None):
    if card is None or not rows or not rows[0]["humans"]:
        return None
    robot, people = rows[0]["robot"], list(rows[0]["humans"].values())
    rule, card_id = card["labels"], card["id"][:2]
    headings = [_heading(robot, human) for human in people]
    if card_id == "FA":
        return all(
            abs(heading - rule["heading_deg"]) <= rule["heading_tolerance_deg"]
            and dist(robot["xy"], human["xy"]) >= rule["min_initial_separation_m"]
            for heading, human in zip(headings, people, strict=True)
        )
    if card_id == "OT":
        human = people[0]
        offset = tuple(h - r for h, r in zip(human["xy"], robot["xy"], strict=True))
        forward = offset[0] * cos(robot["yaw"]) + offset[1] * sin(robot["yaw"])
        subtype = binding["subtype"]
        return (
            headings[0] <= rule["max_heading_difference_deg"]
            and dist(robot["xy"], human["xy"]) <= rule["max_initial_separation_m"]
            and (forward < 0 if subtype == "pedestrian_overtaking" else forward > 0)
        )
    if card_id == "MG":
        return any(
            sum(
                dist(r["robot"]["xy"], h["xy"]) <= rule["robot_radius_m"]
                for h in r["humans"].values()
            )
            >= rule["min_simultaneous_humans"]
            for r in rows
        )
    context = binding.get("label_context", {})
    center = context.get("center_xy_m")
    if center is None:
        return None
    if card_id == "ND":
        axis = context.get("crossing_axis")
        if axis not in (0, 1):
            return None
        width = binding["width_m"]
        radius = rule["both_within_door_center_m"]
        return _crosses([r["robot"]["xy"] for r in rows], center, axis, width) and any(
            _crosses([r["humans"][key]["xy"] for r in rows], center, axis, width)
            and any(
                dist(r["robot"]["xy"], center) <= radius
                and dist(r["humans"][key]["xy"], center) <= radius
                for r in rows
            )
            for key in rows[0]["humans"]
        )
    if card_id == "BC":
        if visible is None:
            return None
        if not all(abs(heading - 90) <= 1e-6 for heading in headings):
            return False
        key = next(iter(rows[0]["humans"]))
        if visible(robot["xy"], rows[0]["humans"][key]["xy"]):
            return False
        first = next(
            (r for r in rows[1:] if visible(r["robot"]["xy"], r["humans"][key]["xy"])),
            None,
        )
        return first is not None and all(
            dist(actor["xy"], center) <= rule["occluded_until_both_within_corner_m"]
            for actor in (first["robot"], first["humans"][key])
        )
    speeds = context.get("approach_speed_mps")
    if speeds is None:
        return None
    robot_eta = dist(robot["xy"], center) / speeds["robot"]
    return all(
        abs(heading - rule["heading_deg"]) <= rule["heading_tolerance_deg"]
        and abs(dist(human["xy"], center) / speeds[key] - robot_eta)
        <= rule["max_predicted_arrival_difference_s"]
        for heading, (key, human) in zip(
            headings, rows[0]["humans"].items(), strict=True
        )
    )
