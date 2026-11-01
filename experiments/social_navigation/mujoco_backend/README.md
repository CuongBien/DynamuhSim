# MuJoCo backend — receiver workspace

**Scaffold only. No MJCF, simulator runner, renderer, sensors or bridge has been implemented yet.**

Start with [MUJOCO_HANDOVER.md](../../../docs/MUJOCO_HANDOVER.md). Task status, reference commit and recorded topic types: [mujoco_port.json](../../../configs/handoff/mujoco_port.json). Existing contract: [BACKEND_BOUNDARY.md](../../../docs/BACKEND_BOUNDARY.md).

| Directory | Intended responsibility |
|---|---|
| assets/ | World/robot/collision model |
| runtime/ | Step, commands, clock/pause/reset and owned shutdown |
| sensors/ | Depth, scan, IMU, odometry, calibration and availability |
| human_adapter/ | Existing human worker ↔ new engine; actual applied state labels |
| ros_bridge/ | Opt-in ROS2 topics/TF/clock/Nav2 integration |
| checks/ | New backend-specific tests and acceptance receipts |
| generated/ | Local outputs ignored by Git; each run uses a new directory |

Keep new code here initially to avoid expanding the frozen Gazebo package source closure. These are responsibility folders, not a completed backend API. Shared code changes later need explicit source versions and regression checks; preserve the baseline checkout/branch for reproducing old runs.

Receiver preflight from repository root:

```bash
python3 tools/mujoco_handoff.py check
```

It checks code/data contracts only and prints mujoco_backend=NOT_IMPLEMENTED. No MuJoCo start command exists yet. First implementation milestone: robot/world physical motion, then sensors/time, then bridge/humans and two new episodes. Do not start WM/RL or a data batch before backend acceptance.
