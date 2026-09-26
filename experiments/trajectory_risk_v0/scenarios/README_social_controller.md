# Multi-human Social Force + Reaction FSM patch

## Files
- `multi_human_scenario_system_social.cpp`: backward-compatible plugin with new `mode: "social"`.
- `arena_dataset_social.sdf`: keeps `human_XX_proxy` as authoritative collision body and adds cosmetic `human_XX_visual` actors.
- `scenario_social_demo.json`: deterministic example episode.

## Critical configuration
In `arena_dataset_social.sdf`, set:

```xml
<robot_model_name>robot</robot_model_name>
```

to the exact Gazebo model name of the robot spawned by your launch file. If it does not match, humans will still follow waypoints and interact with each other, but robot-triggered FSM reactions will remain inactive.

## Controller semantics
`mode: "social"` uses:
1. waypoint-directed desired velocity;
2. exponential human-human repulsion;
3. exponential human-robot repulsion;
4. deterministic robot reaction FSM: `normal`, `sidestep_left`, `sidestep_right`, `yield`, `stop`;
5. acceleration and speed clipping;
6. integration into the authoritative `HumanState`, which is then copied to `human_XX_proxy` and only mirrored cosmetically by `human_XX_visual`.

No actor pose is read back into control or ground truth.

## New social-mode JSON fields
- `preferred_speed`
- `max_speed`
- `max_accel`
- `radius`
- `personal_space`
- `relaxation_time`
- `social_A`, `social_B`
- `robot_A`, `robot_B`, `robot_radius`
- `reaction_distance`, `release_distance`, `stop_distance`
- `ttc_threshold`, `reaction_time`
- `sidestep_speed`, `yield_scale`
- `reaction_profile`: `sidestep_right | sidestep_left | yield | stop`

The old `stationary`, `autonomous`, and `scripted` modes are retained.

## Dataset / ground truth topic
The plugin advertises:

```text
/multi_human/state    gz.msgs.StringMsg
```

at 20 Hz by default. The JSON message contains robot state and, for each active human:

```text
id, mode, behavior,
x, y, yaw,
vx, vy, speed,
desired_vx, desired_vy,
reaction_state, reaction_profile,
waypoint_index,
robot_rel_x, robot_rel_y,
robot_distance, robot_ttc
```

Use this topic as behavior/controller ground truth. RGB can contain the Actor; LiDAR/physics/collision come from the proxy.

## Actor skins
The supplied SDF uses the official Gazebo `walk.dae` / `moonwalk.dae` compatible skins so the visual layer can be tested immediately. They only provide two distinct clothing appearances. For a true multi-outfit dataset, replace each actor's `<skin><filename>` with your own compatible `.dae` skin while keeping the `walk` animation file unchanged.

Do not encode behavior from `appearance_id`. Appearance randomization and behavior parameters should be sampled independently.

## Build
No new build dependency is introduced by this patch. Replace your plugin source with `multi_human_scenario_system_social.cpp`, rebuild the same package, source the workspace, and launch the patched SDF.

## Scientific caution
This patch is a deterministic Social-Force-style baseline plus explicit reaction FSM. It is suitable as a reproducible simulator controller / dataset generator, but its hand-set reaction thresholds should not be described as measured human ground truth. For publication, calibrate or validate these parameters against an established implementation / trajectory data (for example HuNavSim / LightSFM or real pedestrian trajectories), and report that calibration separately from visual domain randomization.
