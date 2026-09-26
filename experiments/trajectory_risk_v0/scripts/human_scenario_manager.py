#!/usr/bin/env python3
"""
human_scenario_manager.py

Multi-human scenario manager for Gazebo Sim / ROS 2 project.

Controls synchronized pairs:
    human_01_visual + human_01_proxy
    human_02_visual + human_02_proxy
    human_03_visual + human_03_proxy
    human_04_visual + human_04_proxy

Supported modes:
    stationary
    autonomous
    scripted

Design principle:
    ONE manager owns the world pose of every human.
    visual actor and physical proxy always receive the SAME x, y, yaw.

Recommended world:
    arena_dataset

The visual actors in arena_dataset.sdf should have:
    <auto_start>false</auto_start>

The proxies should be kinematic collision models.

Scenario file format:
    JSON is always supported.
    YAML is supported when python3-yaml / PyYAML is installed.

Example:
{
  "episode_id": 1,
  "humans": [
    {
      "id": 1,
      "mode": "scripted",
      "behavior": "head_on",
      "start": {"x": -11.4, "y": 0.0, "yaw": 3.14159},
      "speed": 0.35,
      "duration": 4.0
    },
    {
      "id": 2,
      "mode": "scripted",
      "behavior": "crossing",
      "start": {"x": -12.2, "y": -1.2, "yaw": 1.5708},
      "speed": 0.40,
      "duration": 4.0
    },
    {
      "id": 3,
      "mode": "autonomous",
      "speed": 0.30,
      "loop": true,
      "waypoints": [
        [-10.0,  2.0],
        [ -7.0,  2.0],
        [ -7.0, -2.0],
        [-10.0, -2.0]
      ]
    },
    {
      "id": 4,
      "mode": "stationary",
      "start": {"x": -9.5, "y": 1.2, "yaw": 0.0}
    }
  ]
}

Commands:
    # Validate + reset humans only
    python3 human_scenario_manager.py \
        --scenario scenario_001.json \
        --reset-only

    # Run one scenario for 4 seconds
    python3 human_scenario_manager.py \
        --scenario scenario_001.json \
        --duration 4.0

    # Hide all humans below world
    python3 human_scenario_manager.py --hide-all
"""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple


# ============================================================
# Constants
# ============================================================

DEFAULT_WORLD = "arena_dataset"
DEFAULT_HZ = 10.0

HUMAN_IDS = (1, 2, 3, 4)

VISUAL_NAME = "human_{:02d}_visual"
PROXY_NAME = "human_{:02d}_proxy"

VISUAL_Z = 1.0
PROXY_Z = 0.85

HIDDEN_Z = -50.0

EPS = 1e-9


# ============================================================
# Utility
# ============================================================

def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def heading_to(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.atan2(
        b[1] - a[1],
        b[0] - a[0],
    )


def run_cmd(
    cmd: List[str],
    timeout: float = 6.0,
    check: bool = True,
) -> str:
    p = subprocess.run(
        cmd,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        env=os.environ.copy(),
    )

    if check and p.returncode != 0:
        raise RuntimeError(
            f"Command failed ({p.returncode}):\n"
            f"{' '.join(cmd)}\n"
            f"{p.stdout}"
        )

    return p.stdout.strip()


def load_scenario(path: Path) -> Dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(path)

    suffix = path.suffix.lower()

    if suffix == ".json":
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)

    if suffix in (".yaml", ".yml"):
        try:
            import yaml
        except ImportError as exc:
            raise RuntimeError(
                "YAML scenario requested but PyYAML is not installed.\n"
                "Install with: sudo apt install python3-yaml\n"
                "or use JSON."
            ) from exc

        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f)

    raise ValueError(
        f"Unsupported scenario extension: {suffix}. "
        "Use .json, .yaml or .yml."
    )


def parse_start(h: Dict[str, Any]) -> Tuple[float, float, float]:
    start = h.get("start", {})

    return (
        float(start.get("x", 0.0)),
        float(start.get("y", 0.0)),
        float(start.get("yaw", 0.0)),
    )


# ============================================================
# Gazebo entity controller
# ============================================================

class GazeboPoseController:

    def __init__(self, world: str):
        self.world = world

    def service_name(self) -> str:
        return f"/world/{self.world}/set_pose"

    def check_service(self) -> None:
        out = run_cmd(
            ["gz", "service", "-l"],
            timeout=8,
            check=True,
        )

        if self.service_name() not in out.splitlines():
            raise RuntimeError(
                f"Gazebo service not found: {self.service_name()}\n"
                f"Check that world '{self.world}' is running."
            )

    def set_pose(
        self,
        entity: str,
        x: float,
        y: float,
        z: float,
        yaw: float,
        quiet: bool = True,
    ) -> None:

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        request = (
            f'name: "{entity}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: 0.0, y: 0.0, z: {qz}, w: {qw}}}'
        )

        out = run_cmd(
            [
                "gz", "service",
                "-s", self.service_name(),
                "--reqtype", "gz.msgs.Pose",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000",
                "--req", request,
            ],
            timeout=6,
            check=True,
        )

        low = out.lower()

        if "false" in low and "true" not in low:
            raise RuntimeError(
                f"set_pose failed for entity '{entity}':\n{out}"
            )

        if not quiet:
            print(
                f"[POSE] {entity:<18} "
                f"x={x:7.3f} y={y:7.3f} "
                f"z={z:6.3f} "
                f"yaw={math.degrees(yaw):7.2f} deg"
            )

    def set_human_pair(
        self,
        human_id: int,
        x: float,
        y: float,
        yaw: float,
        quiet: bool = True,
    ) -> None:

        visual = VISUAL_NAME.format(human_id)
        proxy = PROXY_NAME.format(human_id)

        # Same x, y, yaw. Only z differs because actor and proxy origins differ.
        self.set_pose(
            visual,
            x,
            y,
            VISUAL_Z,
            yaw,
            quiet=quiet,
        )

        self.set_pose(
            proxy,
            x,
            y,
            PROXY_Z,
            yaw,
            quiet=quiet,
        )

    def hide_human(
        self,
        human_id: int,
        quiet: bool = True,
    ) -> None:
        visual = VISUAL_NAME.format(human_id)
        proxy = PROXY_NAME.format(human_id)

        self.set_pose(
            visual,
            0.0,
            0.0,
            HIDDEN_Z,
            0.0,
            quiet=quiet,
        )

        self.set_pose(
            proxy,
            0.0,
            0.0,
            HIDDEN_Z,
            0.0,
            quiet=quiet,
        )

    def hide_all(self) -> None:
        for human_id in HUMAN_IDS:
            self.hide_human(
                human_id,
                quiet=True,
            )


# ============================================================
# Human state
# ============================================================

@dataclass
class HumanRuntime:
    human_id: int
    mode: str

    x: float
    y: float
    yaw: float

    speed: float = 0.0

    behavior: str = ""

    active: bool = True

    # Autonomous
    waypoints: List[Tuple[float, float]] = field(
        default_factory=list
    )
    waypoint_index: int = 0
    loop: bool = True
    waypoint_tolerance: float = 0.10

    # Scripted linear
    duration: Optional[float] = None

    # Scripted segments
    segments: List[Dict[str, Any]] = field(
        default_factory=list
    )
    segment_index: int = 0
    segment_elapsed: float = 0.0

    # Audit
    initial_x: float = 0.0
    initial_y: float = 0.0
    initial_yaw: float = 0.0

    def reset(self) -> None:
        self.x = self.initial_x
        self.y = self.initial_y
        self.yaw = self.initial_yaw

        self.waypoint_index = 0
        self.segment_index = 0
        self.segment_elapsed = 0.0


# ============================================================
# Human scenario manager
# ============================================================

class HumanScenarioManager:

    def __init__(
        self,
        world: str,
        hz: float = DEFAULT_HZ,
    ):
        self.world = world
        self.hz = float(hz)

        if self.hz <= 0:
            raise ValueError("hz must be > 0")

        self.dt = 1.0 / self.hz

        self.gz = GazeboPoseController(world)

        self.humans: Dict[int, HumanRuntime] = {}

        self.episode_id = None

    # --------------------------------------------------------
    # Scenario parsing
    # --------------------------------------------------------

    def load(
        self,
        scenario: Dict[str, Any],
    ) -> None:

        self.episode_id = scenario.get(
            "episode_id",
            None,
        )

        humans_cfg = scenario.get(
            "humans",
            [],
        )

        if not isinstance(humans_cfg, list):
            raise ValueError(
                "'humans' must be a list."
            )

        self.humans.clear()

        seen = set()

        for h in humans_cfg:
            human = self._parse_human(h)

            if human.human_id in seen:
                raise ValueError(
                    f"Duplicate human id: {human.human_id}"
                )

            seen.add(human.human_id)

            self.humans[
                human.human_id
            ] = human

        # Humans not configured in this episode are hidden.
        for hid in HUMAN_IDS:
            if hid not in self.humans:
                self.humans[hid] = HumanRuntime(
                    human_id=hid,
                    mode="hidden",
                    x=0.0,
                    y=0.0,
                    yaw=0.0,
                    active=False,
                )

    def _parse_human(
        self,
        h: Dict[str, Any],
    ) -> HumanRuntime:

        hid = int(h["id"])

        if hid not in HUMAN_IDS:
            raise ValueError(
                f"human id must be one of "
                f"{HUMAN_IDS}, got {hid}"
            )

        mode = str(
            h.get("mode", "stationary")
        ).strip().lower()

        if mode not in (
            "stationary",
            "autonomous",
            "scripted",
            "hidden",
        ):
            raise ValueError(
                f"Human {hid}: unsupported mode '{mode}'"
            )

        x, y, yaw = parse_start(h)

        speed = float(
            h.get("speed", 0.0)
        )

        if speed < 0:
            raise ValueError(
                f"Human {hid}: speed must be >= 0"
            )

        runtime = HumanRuntime(
            human_id=hid,
            mode=mode,
            x=x,
            y=y,
            yaw=yaw,
            speed=speed,
            behavior=str(
                h.get("behavior", "")
            ),
            active=(mode != "hidden"),
            initial_x=x,
            initial_y=y,
            initial_yaw=yaw,
        )

        # ---------------- autonomous ----------------
        if mode == "autonomous":

            raw_waypoints = h.get(
                "waypoints",
                [],
            )

            if len(raw_waypoints) < 2:
                raise ValueError(
                    f"Human {hid}: autonomous mode "
                    "requires at least 2 waypoints."
                )

            runtime.waypoints = [
                (
                    float(p[0]),
                    float(p[1]),
                )
                for p in raw_waypoints
            ]

            runtime.loop = bool(
                h.get("loop", True)
            )

            runtime.waypoint_tolerance = float(
                h.get(
                    "waypoint_tolerance",
                    0.10,
                )
            )

            # If no explicit start was supplied, begin at first waypoint.
            if "start" not in h:
                runtime.x = runtime.waypoints[0][0]
                runtime.y = runtime.waypoints[0][1]
                runtime.initial_x = runtime.x
                runtime.initial_y = runtime.y

                runtime.waypoint_index = 1

        # ---------------- scripted ----------------
        if mode == "scripted":

            runtime.duration = (
                float(h["duration"])
                if "duration" in h
                else None
            )

            raw_segments = h.get(
                "segments",
                [],
            )

            if raw_segments:
                runtime.segments = []

                for i, seg in enumerate(raw_segments):
                    duration = float(
                        seg.get("duration", 0.0)
                    )

                    if duration <= 0:
                        raise ValueError(
                            f"Human {hid}: segment {i} "
                            "duration must be > 0"
                        )

                    runtime.segments.append(
                        {
                            "duration": duration,
                            "speed": float(
                                seg.get(
                                    "speed",
                                    runtime.speed,
                                )
                            ),
                            "heading": float(
                                seg.get(
                                    "heading",
                                    runtime.yaw,
                                )
                            ),
                        }
                    )

        return runtime

    # --------------------------------------------------------
    # Lifecycle
    # --------------------------------------------------------

    def validate_world(self) -> None:
        self.gz.check_service()

    def reset_all(self) -> None:
        """
        Reset all configured humans to the exact episode initial state.
        Human visual and proxy are synchronized immediately.
        """
        for hid in HUMAN_IDS:
            human = self.humans[hid]

            if not human.active:
                self.gz.hide_human(
                    hid,
                    quiet=True,
                )
                continue

            human.reset()

            self.gz.set_human_pair(
                hid,
                human.x,
                human.y,
                human.yaw,
                quiet=True,
            )

        print(
            f"[RESET] episode={self.episode_id} "
            f"humans synchronized."
        )

    def hide_all(self) -> None:
        self.gz.hide_all()
        print("[OK] All humans hidden.")

    # --------------------------------------------------------
    # Motion updates
    # --------------------------------------------------------

    def update(self, dt: float) -> None:

        for hid in HUMAN_IDS:
            h = self.humans[hid]

            if not h.active:
                continue

            if h.mode == "stationary":
                self._update_stationary(
                    h,
                    dt,
                )

            elif h.mode == "autonomous":
                self._update_autonomous(
                    h,
                    dt,
                )

            elif h.mode == "scripted":
                self._update_scripted(
                    h,
                    dt,
                )

            self.gz.set_human_pair(
                hid,
                h.x,
                h.y,
                h.yaw,
                quiet=True,
            )

    def _update_stationary(
        self,
        h: HumanRuntime,
        dt: float,
    ) -> None:
        # Explicitly no movement.
        return

    def _update_autonomous(
        self,
        h: HumanRuntime,
        dt: float,
    ) -> None:

        if not h.waypoints:
            return

        target = h.waypoints[
            h.waypoint_index
        ]

        current = (
            h.x,
            h.y,
        )

        d = distance(
            current,
            target,
        )

        if d <= h.waypoint_tolerance:
            self._advance_waypoint(h)

            if not h.active:
                return

            target = h.waypoints[
                h.waypoint_index
            ]

            d = distance(
                (h.x, h.y),
                target,
            )

        if d <= EPS:
            return

        h.yaw = heading_to(
            (h.x, h.y),
            target,
        )

        step = min(
            h.speed * dt,
            d,
        )

        h.x += step * math.cos(h.yaw)
        h.y += step * math.sin(h.yaw)

    def _advance_waypoint(
        self,
        h: HumanRuntime,
    ) -> None:

        next_idx = h.waypoint_index + 1

        if next_idx < len(h.waypoints):
            h.waypoint_index = next_idx
            return

        if h.loop:
            h.waypoint_index = 0
        else:
            # Stop at final waypoint.
            h.waypoint_index = (
                len(h.waypoints) - 1
            )
            h.speed = 0.0

    def _update_scripted(
        self,
        h: HumanRuntime,
        dt: float,
    ) -> None:

        # Segment-based scripted trajectory has priority.
        if h.segments:
            self._update_scripted_segments(
                h,
                dt,
            )
            return

        # Simple linear scripted motion.
        h.x += (
            h.speed
            * dt
            * math.cos(h.yaw)
        )

        h.y += (
            h.speed
            * dt
            * math.sin(h.yaw)
        )

    def _update_scripted_segments(
        self,
        h: HumanRuntime,
        dt: float,
    ) -> None:

        if h.segment_index >= len(
            h.segments
        ):
            return

        seg = h.segments[
            h.segment_index
        ]

        h.yaw = normalize_angle(
            float(seg["heading"])
        )

        speed = float(
            seg["speed"]
        )

        h.x += (
            speed
            * dt
            * math.cos(h.yaw)
        )

        h.y += (
            speed
            * dt
            * math.sin(h.yaw)
        )

        h.segment_elapsed += dt

        if (
            h.segment_elapsed
            >= float(seg["duration"])
        ):
            h.segment_index += 1
            h.segment_elapsed = 0.0

    # --------------------------------------------------------
    # Execution
    # --------------------------------------------------------

    def run(
        self,
        duration: float,
        verbose_sec: float = 1.0,
    ) -> None:

        self.reset_all()

        start = time.monotonic()
        last = start
        last_print = start

        while True:
            now = time.monotonic()
            elapsed = now - start

            if elapsed >= duration:
                break

            dt = now - last

            if dt <= 0:
                dt = self.dt

            # Avoid giant jumps if the process was briefly delayed.
            dt = min(
                dt,
                2.0 * self.dt,
            )

            self.update(dt)

            if (
                verbose_sec > 0
                and now - last_print
                >= verbose_sec
            ):
                self.print_state(
                    elapsed
                )
                last_print = now

            last = now

            sleep_time = self.dt - (
                time.monotonic() - now
            )

            if sleep_time > 0:
                time.sleep(
                    sleep_time
                )

        self.print_state(
            duration
        )

    # --------------------------------------------------------
    # Audit / export
    # --------------------------------------------------------

    def state_dict(self) -> Dict[str, Any]:

        humans = []

        for hid in HUMAN_IDS:
            h = self.humans[hid]

            humans.append(
                {
                    "id": hid,
                    "mode": h.mode,
                    "active": h.active,
                    "behavior": h.behavior,
                    "x": h.x,
                    "y": h.y,
                    "yaw": h.yaw,
                    "speed": h.speed,
                }
            )

        return {
            "episode_id": self.episode_id,
            "world": self.world,
            "humans": humans,
        }

    def print_state(
        self,
        elapsed: float,
    ) -> None:

        print(
            f"\n[t={elapsed:6.2f}s]"
        )

        for hid in HUMAN_IDS:
            h = self.humans[hid]

            if not h.active:
                print(
                    f"H{hid}: hidden"
                )
                continue

            print(
                f"H{hid}: "
                f"{h.mode:<10} "
                f"x={h.x:7.3f} "
                f"y={h.y:7.3f} "
                f"yaw={math.degrees(h.yaw):7.2f}° "
                f"v={h.speed:5.2f}"
            )


# ============================================================
# Example scenario generator
# ============================================================

def example_scenario() -> Dict[str, Any]:
    return {
        "episode_id": 1,
        "humans": [
            {
                "id": 1,
                "mode": "scripted",
                "behavior": "head_on",
                "start": {
                    "x": -11.4,
                    "y": 0.0,
                    "yaw": math.pi,
                },
                "speed": 0.35,
                "duration": 4.0,
            },
            {
                "id": 2,
                "mode": "scripted",
                "behavior": "crossing",
                "start": {
                    "x": -12.2,
                    "y": -1.2,
                    "yaw": math.pi / 2.0,
                },
                "speed": 0.40,
                "duration": 4.0,
            },
            {
                "id": 3,
                "mode": "autonomous",
                "speed": 0.30,
                "loop": True,
                "waypoints": [
                    [-10.0,  2.0],
                    [ -7.0,  2.0],
                    [ -7.0, -2.0],
                    [-10.0, -2.0],
                ],
            },
            {
                "id": 4,
                "mode": "stationary",
                "start": {
                    "x": -9.5,
                    "y": 1.2,
                    "yaw": 0.0,
                },
            },
        ],
    }


# ============================================================
# CLI
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--world",
        default=DEFAULT_WORLD,
    )

    parser.add_argument(
        "--scenario",
        type=Path,
        default=None,
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=4.0,
    )

    parser.add_argument(
        "--hz",
        type=float,
        default=DEFAULT_HZ,
    )

    parser.add_argument(
        "--reset-only",
        action="store_true",
    )

    parser.add_argument(
        "--hide-all",
        action="store_true",
    )

    parser.add_argument(
        "--write-example",
        type=Path,
        default=None,
        help=(
            "Write an example JSON scenario "
            "and exit."
        ),
    )

    parser.add_argument(
        "--verbose-sec",
        type=float,
        default=1.0,
    )

    args = parser.parse_args()

    if args.write_example is not None:
        args.write_example.parent.mkdir(
            parents=True,
            exist_ok=True,
        )

        with args.write_example.open(
            "w",
            encoding="utf-8",
        ) as f:
            json.dump(
                example_scenario(),
                f,
                indent=2,
            )

        print(
            f"Example scenario written: "
            f"{args.write_example}"
        )
        return

    manager = HumanScenarioManager(
        world=args.world,
        hz=args.hz,
    )

    manager.validate_world()

    if args.hide_all:
        manager.hide_all()
        return

    if args.scenario is None:
        scenario = example_scenario()
        print(
            "[INFO] No --scenario supplied; "
            "using built-in example."
        )
    else:
        scenario = load_scenario(
            args.scenario
        )

    manager.load(
        scenario
    )

    manager.reset_all()

    if args.reset_only:
        manager.print_state(0.0)
        return

    manager.run(
        duration=args.duration,
        verbose_sec=args.verbose_sec,
    )


if __name__ == "__main__":
    main()
