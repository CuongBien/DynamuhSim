# PBL6 Step 2 — Scenario Generator V1

Static acceptance: **PASS** (2026-09-26)

| Family | Seeds 0–9, low | Seeds 0–9, high | Deterministic |
| --- | --- | --- | --- |
| S00 EMPTY | PASS | PASS (0 humans by design) | PASS |
| S01 HEAD_ON | PASS | PASS | PASS |
| S02 SAME_DIRECTION | PASS | PASS | PASS |
| S03 CROSSING | PASS | PASS | PASS |
| S04 EXIT_ROOM | PASS | PASS | PASS |

- `python3 -m unittest discover -s tests -v`: 8/8 PASS.
- `python3 ../validate_demo.py`: original Step 1 topology/map checks PASS.
- `gz sdf -p`: all five generated sample worlds parse successfully.
- HuNavSim loader in the existing Docker container read sample S03's
  `humans.yaml` and reported three agents, their goals and behavior without
  a parameter error. The loader was stopped after eight seconds; this was a
  schema smoke test, not a motion test.
- `ros2 launch ../school_hunav_demo.launch.py --show-args`: existing launch
  loads and retains its original defaults.

Validation checks semantic zone containment, full spawn footprint clearance,
all selected graph edges against occupancy, distinct starts/goals, human to
robot and human to human spawn separation, event route semantics, and the
space/time conflict for door crossing and room exit. Failed samples retry up
to `max_attempts`.

Live Gazebo/Nav2/HuNav motion, collision and encounter outcomes have not been
measured here. They require launching an episode and driving the robot to its
sampled goal; that runtime acceptance remains to be checked before claiming
behavioral performance in simulation.
