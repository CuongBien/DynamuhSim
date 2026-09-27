"""Container-side HuNav process ownership and ROS readiness probe.

Copied to /tmp/school_arena by HuNavManager. ROS imports stay inside the
Humble container; the Jazzy host does not need hunav_msgs or people_msgs.
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

TOKEN_KEY = b"PBL6_HUNAV_TOKEN="


def owned_processes(token: str) -> list[dict]:
    expected = TOKEN_KEY + token.encode("ascii")
    found = []
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            fields = (Path(entry.path) / "stat").read_text().rsplit(")", 1)[1].split()
            if fields[0] in ("Z", "X"):
                continue
            environment = (Path(entry.path) / "environ").read_bytes().split(b"\0")
            if expected not in environment:
                continue
            found.append({"pid": int(entry.name), "pgid": int(fields[2])})
        except (OSError, IndexError, ValueError):
            continue
    return found


def ros_probe(mode: str, args: list[str]) -> dict:
    import rclpy
    from rclpy.node import Node

    required = {args[0]} if mode == "wait_service" else {
        "/get_parameters", "/compute_agents"
    }
    expected = json.loads(args[0]) if mode == "observe" else None
    timeout = float(args[1])
    rclpy.init()
    node = Node("pbl6_hunav_probe")
    last_names = None

    if mode == "observe":
        from hunav_msgs.msg import Agents

        def on_agents(message):
            nonlocal last_names
            last_names = [agent.name for agent in message.agents]

        node.create_subscription(Agents, "/human_states", on_agents, 10)

    deadline = time.monotonic() + timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.2, remaining))
            available = {name for name, _ in node.get_service_names_and_types()}
            if not required <= available:
                continue
            if mode == "wait_service":
                return {"status": "ready", "services": sorted(available & required)}
            if last_names is not None:
                if len(last_names) != len(expected) or sorted(last_names) != sorted(expected):
                    return {"status": "human_count_mismatch", "names": last_names,
                            "expected": expected}
                return {"status": "ready", "count": len(last_names),
                        "names": last_names, "services": sorted(available & required)}
        return {"status": "timeout", "services": sorted(available & required),
                "names": last_names}
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv: list[str]) -> int:
    if not argv:
        return 2
    command, *args = argv
    if command == "status":
        print(json.dumps({"processes": owned_processes(args[0])}))
    elif command == "signal":
        token, pgid_text, signal_text = args
        pgid = int(pgid_text)
        if pgid not in {item["pgid"] for item in owned_processes(token)}:
            print(json.dumps({"sent": False}))
            return 1
        os.killpg(pgid, int(signal_text))
        print(json.dumps({"sent": True}))
    elif command in ("wait_service", "observe"):
        print(json.dumps(ros_probe(command, args)))
    else:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
