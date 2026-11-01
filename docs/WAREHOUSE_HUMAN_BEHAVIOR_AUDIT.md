# Warehouse Human Behavior Audit

Audit date: 2026-11-01. Audited source HEAD: `47fce50f402e65e76848509af91f5b65243d5eda`.
Branch: `feat/depth-detector-generalization`.
Worktree: `/home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/depth_identity_guard/generated/generalization_worktree`.

Scope is human movement in the Warehouse. This audit used remote MCP `execute_bash` with ROS auto-sourcing disabled. It made no production-code changes, installed no dependencies, and launched no simulation, training, or benchmark. Historical results are identified as historical; component probes below were run during this audit. A reachable source path is not proof that a process currently runs it.

Initial safeguards: the target was a registered worktree on the requested branch; its initial staged/unstaged/untracked status was empty, and local HEAD equaled the origin target-branch tip. The report had no existing user changes. Main-worktree changes were observed only in the required initial Git inventory and were not edited. All investigation and report work used the target worktree.

## 1. Executive Summary

Warehouse does not have one universal human controller. The maintained Nav2/capture paths use Python human backends and a Gazebo pose-writing adapter. ComplexityNav is a selectable hybrid backend, not the default group controller. SocNavGym is genuinely stepped in the individual/group paths, but its movement can be replaced by additional ORCA and safety-projection stages. The original standalone group demo and the older explicit `rvo2` backend follow different loops.

**Top three causes**, ranked by evidence, expected impact, and relevance to the retained default group-capture path:

1. **Unstable upper-level group avoidance intent (LIKELY causal contributor).** Group-circle envelopes and per-person motion can disagree, and raw group intent reverses even without a goal change. Historical measured-input replay and native captures show excessive turning and substantial reductions under isolated group treatments. This is stronger evidence than a visual impression, but it does not isolate radius, velocity initialization, smoothing, and downstream guard effects individually. Evidence: `python/dynamuhsim/simulation/human_groups.py:21`, `python/dynamuhsim/simulation/human_groups.py:72`, `docs/demos/2026-10-29/endpoint_humans/offline_reproduction.json:2`, `docs/demos/2026-10-29/sim_goal_clock/group_diagnostic.json:23`.
2. **Host-time human goal expiry in the default simulation path (CONFIRMED integration mismatch in the documented native-wall path).** Human lifecycle is gated and simulated at fixed dt, while the default leaves native wall-time expiry in place. Retargeting can therefore depend on initialization delay and real-time factor. The existing sim-clock treatment fixes the clock domain, but historical turning worsened under that treatment alone: it is not the sole cause of spinning. Evidence: `tools/capture_episode.py:26`, `experiments/social_navigation/navigation/socnavgym_group_nav2/worker.py:44`, `python/dynamuhsim/simulation/goal_clock.py:95`, `docs/demos/2026-10-29/sim_goal_clock/README.md:22`.
3. **Final movement lacks a coupled heading/velocity/acceleration contract (LIKELY contributor; structural limitation CONFIRMED).** Final ORCA/projection can change displacement after body-heading intent is collected; speed is bounded but the projection has no previous-velocity acceleration constraint. The adapter applies piecewise-linear XY and independent yaw directly. This permits sliding, rapid speed changes, or translating while still turning; frequency in an arbitrary current run remains unknown. Evidence: `experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:26`, `experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:43`, `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:26`, `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:83`.

Do not infer that people never avoid one another or the robot. The maintained per-person paths explicitly include both and independently validate predicted swept gaps. Conversely, positive physical clearance is not evidence of personal-space comfort, negotiation, or natural group motion. No new collision, oscillation frequency, or human-naturalness acceptance is claimed by this audit.

## 2. Actual Human-Control Pipeline

### Entrypoints and activation boundaries

| Entrypoint / selection | Actual human path | Activation evidence |
| --- | --- | --- |
| Bare `experiments/social_navigation/warehouse/warehouse.launch.py` | Loads the base Warehouse world and robot/Nav2; does not spawn a human worker itself | `experiments/social_navigation/warehouse/warehouse.launch.py:13`; base world system plugins at `experiments/social_navigation/warehouse/warehouse.sdf:4` |
| `tools/nav2_trial.py --experiment socnavgym_nav2_pilot --condition people` | Six individual SocNavGym humans, responsive per-person ORCA/projection | `tools/nav2_trial.py:20`; `python/dynamuhsim/runtime/profiles.py:66`; `experiments/social_navigation/navigation/common_gazebo/worker.py:30` |
| `--experiment socnavgym_group_nav2` | Explicit walking/standing triplets, native lifecycle, responsive or reactive-standing integration | `python/dynamuhsim/runtime/profiles.py:69`; `experiments/social_navigation/navigation/socnavgym_group_nav2/fixture.py:17` |
| `--experiment shared_nav2_pilot --model complex_geometry_mixed` | Containerized ComplexityNav native intentions plus roadmap/ORCA/swept geometry | `python/dynamuhsim/runtime/nav2_trial.py:170`; `python/dynamuhsim/runtime/workers.py:15`; `experiments/social_navigation/navigation/common_response/model_worker.py:35` |
| `tools/capture_episode.py prepare/run` | Uses the group profile; optional frozen motion/encounter/clock/lifecycle assets change selected hooks | `python/dynamuhsim/runtime/capture.py:78`; `python/dynamuhsim/runtime/capture.py:88`; `experiments/social_navigation/navigation/socnavgym_group_nav2/worker.py:13` |
| `experiments/social_navigation/gazebo_adapter/run.py` | Older three-person persistent-group demonstration with externally assigned patrol goals | `experiments/social_navigation/gazebo_adapter/run.py:144`; `experiments/social_navigation/gazebo_adapter/run.py:159`; `experiments/social_navigation/gazebo_adapter/kernel.py:36` |
| Older `experiments/social_navigation/navigation/trial.py --backend rvo2` | Explicitly loads the in-engine C++ RVO2 plugin; separate from maintained shared/group workers | `experiments/social_navigation/navigation/trial.py:199`; `experiments/social_navigation/navigation/trial.py:217` |

The bare world has no human movement plugin/actors. The generated-world helper adds an adapter, cylinder collision proxies, and separate skinned actors: `experiments/social_navigation/gazebo_adapter/run.py:26`, `experiments/social_navigation/gazebo_adapter/run.py:58`, `experiments/social_navigation/gazebo_adapter/run.py:78`. Proxies are gravity-free, kinematic cylinders with backend-provided radii and 1.7 m height (`experiments/social_navigation/gazebo_adapter/run.py:64`). Actors use inactive dummy script waypoints; `auto_start=false` and `loop=false` mean those waypoints are not the maintained runtime route generator (`experiments/social_navigation/gazebo_adapter/run.py:87`).

### Maintained loop, from launch to final pose

1. The runner selects profile/native dt, starts a worker, and reads six initial humans from `ready.json`: `python/dynamuhsim/runtime/nav2_trial.py:127`, `python/dynamuhsim/runtime/nav2_trial.py:167`, `python/dynamuhsim/runtime/nav2_trial.py:184`.
2. It generates a Warehouse SDF, replaces the initial plugin with `response_experiment::Adapter`, and uses physics dt 0.0025 s: `python/dynamuhsim/runtime/nav2_trial.py:188`, `python/dynamuhsim/simulation/warehouse.py:11`. This is **not** `librvo2_human_system.so` in these profiles.
3. The normal launch passes the generated world through `custom_world`; the capture launch directly starts Gazebo with `NAV2_PILOT_WORLD`: `python/dynamuhsim/simulation/warehouse.py:52`, `src/custom_corridor/launch/corridor_tb3.launch.py:100`, `python/dynamuhsim/simulation/capture_launch.py:30`. The helper runs `gz sim -r` and optionally `-s`: `src/custom_corridor/launch/_dynamuhsim_simulation.py:29`.
4. Humans remain frozen until the accepted robot navigation goal writes `start_gate`: `python/dynamuhsim/runtime/navigation.py:30`, `python/dynamuhsim/runtime/navigation.py:37`; workers check it before advancing (`experiments/social_navigation/navigation/common_gazebo/worker.py:69`, `experiments/social_navigation/navigation/socnavgym_group_nav2/worker.py:44`). This intended warmup stop is not a planner deadlock.
5. At each planning tick the adapter measures robot world pose and finite-difference velocity, performs a synchronous Unix-socket exchange, and validates time/population/identity: `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:69` through line 80. SocNavGym uses 0.1 s; ComplexityNav uses 0.25 s (`python/dynamuhsim/runtime/profiles.py:37`, `experiments/social_navigation/navigation/shared_nav2_pilot/protocol.json:18`).
6. The worker advances native intention/lifecycle, then its selected safety integration returns future state at `t+dt`. The adapter interpolates from `before` to `humans` throughout that same interval, independently interpolating yaw by shortest angle: `experiments/social_navigation/navigation/common_gazebo/worker.py:69`, `experiments/social_navigation/navigation/common_gazebo/worker.py:74`, `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:77`, `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:83`.
7. **Final pose authority is the C++ response adapter**, using `Model::SetWorldPoseCmd` for the cylinder and `Actor::SetTrajectoryPose` for the visual, every physics tick. Animation time increases with actual interval travel speed times 4.1534: `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:53`. Human locomotion is kinematic pose execution of backend-computed motion, not a ROS human `cmd_vel` controller or a dynamically integrated articulated body.

The interpolation smooths position between planning samples; it does not guarantee continuous velocity or acceleration across interval boundaries. The published executed speed is `hypot(to-before)/planningDt`, not necessarily native requested speed (`experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:86`). Worker future states, current adapter states, and rendered gait must be compared at aligned timestamps.

The old standalone adapter instead applies each new reply immediately at 0.1 s boundaries and holds pose between them (`experiments/social_navigation/gazebo_adapter/adapter.cc:100`, `experiments/social_navigation/gazebo_adapter/adapter.cc:123`). Its visible stepping must not be attributed to the interpolating response adapter.

## 3. Complexity Nav Runtime Integration

**Status: reachable and historically executed only when explicitly selected; not the group-capture backend.** The shared profile accepts `complex_geometry_mixed` (`python/dynamuhsim/runtime/profiles.py:24`), starts the pinned ComplexityNav container, and mounts the live geometry stage (`python/dynamuhsim/runtime/workers.py:15`, `python/dynamuhsim/runtime/workers.py:24`). Its worker wrapper selects `WORKER_VARIANT=shared`, which deliberately does not enable SocNavGym ResponsiveMotion (`experiments/social_navigation/navigation/shared_nav2_pilot/worker.py:7`, `experiments/social_navigation/navigation/common_gazebo/worker.py:27`). Historical execution is supported by `experiments/archive/social_navigation/reports/shared_nav2_pilot/complex01/result.json:2` and `experiments/archive/social_navigation/reports/shared_nav2_pilot/SHARED_NAV2_REPORT.md:21`; these are not runs made for this audit.

Pipeline:

- Native `CrowdSim`, time step 0.25 s, visible robot, three ORCA and three social-force humans; repeated start/goal legs are supplied to native reset: `experiments/social_navigation/navigation/complexitynav_contract/kernel.py:13`, `experiments/social_navigation/navigation/complexitynav_contract/kernel.py:22`, `experiments/social_navigation/navigation/complexitynav_contract/kernel.py:41`, `experiments/social_navigation/navigation/common_response/model_worker.py:40`.
- `GeometryKernel` installs `ObstaclePlanner` as the native centralized planner: `experiments/social_navigation/navigation/complexitynav_geometry/kernel.py:49`.
- Each tick, a visibility roadmap chooses a currently visible corner/goal, and the stock native multi-policy predicts intention toward that temporary waypoint: `experiments/social_navigation/navigation/complexitynav_geometry/kernel.py:21`, `experiments/social_navigation/navigation/complexitynav_geometry/roadmap.py:50`. This recomputes from current position; it is not blind playback of recorded XY waypoints.
- A fresh obstacle-aware RVO simulator includes all Warehouse boxes, current human velocities, and measured robot velocity. It caps motion at the runner's 0.8 m/s ceiling, then a swept guard applies **one shared scalar fraction to every human displacement**: `experiments/social_navigation/navigation/common_response/model_worker.py:44`, `experiments/social_navigation/navigation/complexitynav_geometry/kernel.py:28`, `experiments/social_navigation/navigation/complexitynav_geometry/kernel.py:43`.
- The guard checks 33 fractions in steps of 1/32 with a 2 mm floor and fixed predicted robot displacement: `experiments/social_navigation/navigation/socnavgym_warehouse_lifecycle/geometry.py:35`. The preparation script copies this exact source to generated `swept.py` and the contract kernel to generated `native_contract.py`: `experiments/social_navigation/navigation/common_response/preflight.py:17`.
- Native environment stepping uses measured robot action and preserves human identities (`experiments/social_navigation/navigation/complexitynav_contract/kernel.py:78`). Output heading is then rate-limited toward **executed** velocity by the common worker at 1.57 rad/s (`experiments/social_navigation/navigation/common_gazebo/worker.py:35`).

Thus planned native intentions can differ from executed trajectories through roadmap goal substitution, the second ORCA solve, velocity clipping, shared scaling, and adapter interpolation. This is a hybrid geometry variant, not trained RGL or an unmodified upstream planner (`experiments/social_navigation/navigation/common_response/model_worker.py:45`). No trained robot social policy is invoked by this selector.

**Important negative result:** read-only inspection of archived `complex01` and `complex02` compressed worker traces found 90 and 107 active guard samples respectively, all `safe_fraction=1.0`. The shared-scaling limitation is demonstrable in a synthetic component probe, but is **not an observed cause of stopping in those two runs**. Historical run source revisions differ from this audit HEAD; no current live ComplexityNav execution was attempted.

## 4. Social Nav Runtime Integration

Here “Social Nav” is separated into SocNavGym human lifecycle, C++ RVO2 social behavior, and ROS Nav2 robot navigation; the names alone do not establish which one controls humans.

### Maintained SocNavGym path

`Kernel` creates `SocNavGym-v2`, installs all factory-wall objects, and actually calls `wrapper.step(zeros(3))` with robot position/velocity supplied from Gazebo (`experiments/social_navigation/navigation/socnavgym_warehouse_lifecycle/kernel.py:26`, `experiments/social_navigation/navigation/socnavgym_warehouse_lifecycle/kernel.py:57`). Zero action prevents the Python robot from moving independently; it does not freeze human lifecycle. `ContinuousGroups` hooks `reset`, `step`, group-formation finish, and collision integration (`experiments/social_navigation/navigation/socnavgym_lifecycle_fix/continuous_groups.py:47`). It preserves membership/RNG selection while removing instantaneous native formation position snaps (`experiments/social_navigation/navigation/socnavgym_lifecycle_fix/continuous_groups.py:62`).

For the group fixture, two triplets are placed explicitly and assigned moving or stationary interaction objects; subsequent formation/dispersal probabilities are 0.03/0.01 (`experiments/social_navigation/navigation/socnavgym_group_nav2/fixture.py:20`, `experiments/social_navigation/navigation/socnavgym_group_nav2/config.yaml:52`). Native initial group goal radius is 1.44 m, not the individual 0.25 m radius (`experiments/social_navigation/navigation/socnavgym_group_nav2/fixture.py:26`, `experiments/social_navigation/navigation/socnavgym_group_nav2/config.yaml:45`).

Walking integration calls native collision/update first as a tentative proposal, then performs another ORCA solve and convex per-person safety projection; the final coordinates replace the native result (`experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:20`, `experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:33`, `experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:43`). Native navigation outputs are therefore used but are not final authority.

Explicit capture variants:

| Motion profile | Hook and behavior | Current activation |
| --- | --- | --- |
| `legacy` | Retains native upper group-velocity method; responsive per-person guard remains | Capture default: `tools/capture_episode.py:24`; omitted asset leaves hooks alone: `python/dynamuhsim/simulation/human_motion_profiles.py:35` |
| `group_orca_v1` | Uses measured mean velocity and actual enclosing member radius for group agents | Opt-in dispatcher: `python/dynamuhsim/simulation/human_motion_profiles.py:27`; algorithm: `python/dynamuhsim/simulation/human_groups.py:72` |
| `group_goal_v1` | Replaces upper group ORCA with shared normalized goal velocity; per-person guard remains | `python/dynamuhsim/simulation/group_intent.py:31` |
| `group_cohesion_v1` | Adds pre-guard soft regrouping; failed turning screen historically | `python/dynamuhsim/simulation/group_cohesion.py:30`; `docs/demos/2026-10-29/group_cohesion_heading/README.md:33` |
| `group_cohesion_v2` | Adds pre-guard bounded heading and cosine braking during active regrouping | `python/dynamuhsim/simulation/group_cohesion_heading.py:16`; opt-in, not default: `docs/demos/2026-10-29/group_cohesion_heading/README.md:28` |

The v2 heading treatment is conditional on active regrouping; compact/no-adjustment proposals remain native, and the downstream guard can still change final displacement (`python/dynamuhsim/simulation/group_cohesion_heading.py:24`, `python/dynamuhsim/simulation/group_cohesion_heading.py:43`). Do not claim a complete final-motion guarantee from that profile.

### Other reachable Warehouse paths

The standalone `GroupKernel` has a fixed two-point patrol, manually calls native group ORCA/update_speed/contact integration, and explicitly documents that native group ORCA omits the robot (`experiments/social_navigation/gazebo_adapter/kernel.py:38`, `experiments/social_navigation/gazebo_adapter/kernel.py:64`). It does not run the full formation/reward loop. The older navigation worker can optionally install robot-aware group ORCA only with explicit `--robot-avoidance` and circle-contact geometry (`experiments/social_navigation/navigation/worker.py:29`, `experiments/social_navigation/navigation/worker.py:42`); that optional solver discards its solved robot velocity (`experiments/social_navigation/navigation/robot_orca.py:35`).

The C++ RVO2 plugin **is** reachable through older `navigation/trial.py --backend rvo2`, but is not loaded by the maintained group/shared generated worlds. In that Warehouse selection `corridor_mode=false`, `wait_for_robot=false`, autonomous H1 or stationary H4 humans follow the provided scenario route (`experiments/social_navigation/navigation/trial.py:206`). Its loop measures robot motion, computes preferred velocity, solves RVO, updates human state/yaw, and writes poses (`src/custom_corridor_plugins/src/rvo2_human_system.cpp:182`, `src/custom_corridor_plugins/src/rvo2_human_system.cpp:265`, `src/custom_corridor_plugins/src/rvo2_human_system.cpp:825`). Corridor recess yielding is inactive in that Warehouse configuration. README corridor features must not be assumed to apply to SocNavGym Warehouse captures.

## 5. Human-Human Constraints

- **Maintained per-person avoidance exists.** Every human is added to the final RVO solve with previous measured human velocity and configured radius; final projection constrains pair separation and checks continuous relative-segment clearance (`experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:35`, `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:19`, `experiments/social_navigation/navigation/socnavgym_warehouse_lifecycle/geometry.py:32`). Missing all human-human reaction is not supported.
- **Physical clearance is small and isotropic.** Human nominal diameter is 0.72 m; ORCA adds 5 mm radius padding. Projection tries 4 cm, 2 cm, then 2 mm surface margins, with a 2 mm independent validation floor (`experiments/social_navigation/navigation/socnavgym_group_nav2/config.yaml:20`, `experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:33`, `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:43`, `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:48`). These constraints are not anisotropic personal-space or comfort guarantees.
- **Group envelope overlap differs from contact.** In preserved sim-clock group runs, virtual overlap fractions are 0.6172/0.6937 while minimum sampled actual member surface gaps are positive, 0.4275/0.0421 m (`docs/demos/2026-10-29/sim_goal_clock/group_diagnostic.json:23`, `docs/demos/2026-10-29/sim_goal_clock/group_diagnostic.json:60`). This supports an abstraction conflict, not humans passing through one another.
- **Cohesion is not guaranteed by collision avoidance.** Removing upper group avoidance reduced turning but historical maximum member-centroid distances reached 6.746/2.681 m (`docs/demos/2026-10-29/group_goal_ablation/README.md:15`). The later heading/cohesion candidate improved the two tested scenes; one full audit still failed during teardown (`docs/demos/2026-10-29/group_cohesion_heading/README.md:13`).
- **Priority/deadlock scope:** the examined final ORCA/projection code has no explicit persistent passing-side commitment, negotiated priority queue, or deadlock-recovery state. It minimizes deviation from a proposal subject to geometric constraints, not social negotiation (`experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:29`). This is a scoped code observation; a particular live deadlock is UNVERIFIED.

## 6. Human-Robot Constraints

The maintained response layer explicitly inserts a robot agent with measured velocity and radius, then constrains humans against a fixed predicted robot displacement (`experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:39`, `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:16`). SocNavGym configuration requests avoidance probability 1.0 (`experiments/social_navigation/navigation/socnavgym_group_nav2/config.yaml:25`). The upper `GroupORCA` solver itself has no robot-agent insertion; robot reaction is supplied downstream at the per-person layer (`python/dynamuhsim/simulation/human_groups.py:54` through line 83). Thus “no robot in group intent” must not be generalized to “no robot avoidance anywhere.”

Final RVO models a reciprocal robot agent, but its solved robot trajectory is not executed by this human controller; Nav2 drives the real robot. Projection uses current measured velocity as a constant prediction instead. The next robot action can differ. This is a prediction/authority mismatch to test under braking/turning, not proof of collision (`experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:40`, `experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:42`, `python/dynamuhsim/runtime/nav2_trial.py:293`). The human guard uses a nominal robot disc of 0.25 m (`experiments/social_navigation/navigation/socnavgym_group_nav2/worker.py:23`); it does not model the robot's full rotating body in this projection.

Standing triplet members become eligible for bounded sidesteps only when robot-center distance is strictly below 1.4 m. Their allowed speed becomes 0.2 m/s; native static individuals remain fixed. There is no activation hysteresis in this test (`experiments/social_navigation/navigation/socnavgym_group_nav2/reactive_motion.py:33`, `experiments/social_navigation/navigation/socnavgym_group_nav2/standing_projection.py:26`, `experiments/social_navigation/navigation/socnavgym_group_nav2/standing_projection.py:47`). Threshold chatter is possible in principle, but was not demonstrated here. A stationary conversation group is not automatically an avoidance failure.

No broad Hall-style front/side personal-space field appears in the examined response projection: it uses circular center/radius support planes, small margins and AABBs (`experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:16`). The separate legacy C++ backend does have H1/H2 lateral nudging, proximity slowing and a low-pass preferred-velocity filter (`src/custom_corridor_plugins/src/rvo2_human_system.cpp:764`, `src/custom_corridor_plugins/src/rvo2_human_system.cpp:796`). Those are not active in maintained SocNavGym captures.

## 7. Algorithm and Runtime Findings

### F1 — Group intent instability: observed excessive turning, plausible causal mechanism

**Active scope:** default legacy walking-group capture; modified envelope behavior also exists in opt-in `group_orca_v1`.

The retained native solver limitation is documented at `python/dynamuhsim/simulation/human_groups.py:3`: fresh group agents start with zero velocity and use interaction radius plus full human diameter. The correction uses actual member envelope and measured mean velocity. With initial member offset 0.72 m and body radius 0.36 m, its initial envelope is 1.08 m rather than the documented native 1.44 m (`experiments/social_navigation/navigation/socnavgym_group_nav2/fixture.py:26`, `python/dynamuhsim/simulation/human_groups.py:24`). The native vendor is absent in this worktree; its exact present binary implementation was not independently re-read.

Historical fixed-input 120-step replay records 78.784942 rad aggregate absolute yaw for legacy versus 21.240376 rad with `group_orca_v1`, a 73.0401% reduction, with legacy pose reproduction RMSE zero (`docs/demos/2026-10-29/endpoint_humans/offline_reproduction.json:2`). This audit recomputed that arithmetic from the committed JSON; it did not rerun the native replay. Radius and velocity initialization changed together, so individual causality is not isolated. Same-goal raw ORCA intent flips over 90 degrees occur 11 times head-on and 7 times crossing in later sim-clock runs (`docs/demos/2026-10-29/sim_goal_clock/group_diagnostic.json:32`, `docs/demos/2026-10-29/sim_goal_clock/group_diagnostic.json:70`). This links the symptom to the planner layer rather than only gait rendering.

Minimal proposal: keep per-person safety unchanged; test an explicit upper group-intent treatment with soft cohesion and bounded heading. Reuse the existing opt-in candidate rather than remove all avoidance or promote it by default. Reproduce against identical measured robot inputs before closed-loop testing. Full naturalness and unique causality remain unproven.

### F2 — Goal clocks and acceptance thresholds can change intent independently of simulated progress

**Active scope:** default `native_wall` capture and group runtime without a clock asset.

The worker freezes lifecycle during warmup, but absent `human_goal_clock.json` the clock loader leaves native methods unchanged (`experiments/social_navigation/navigation/socnavgym_group_nav2/worker.py:44`, `python/dynamuhsim/simulation/goal_clock.py:132`). Preserved diagnostics show group retargeting near 50 wall seconds while goals remain far away (`docs/demos/2026-10-29/encounter_regression/goal_clock.json:82`). The documented sim-clock correction retains strict `age>50` and resets initial age at navigation acceptance (`python/dynamuhsim/simulation/goal_clock.py:95`, `python/dynamuhsim/simulation/goal_clock.py:107`). This makes default timing sensitive to wall/sim speed and potentially startup waiting; startup-specific expiry was not reproduced here.

Group geometric acceptance is also intentionally broad: human half-width 0.36 m plus interaction goal radius 1.44 m gives a 1.80 m default-offset reach threshold, as recorded in `docs/demos/2026-10-29/encounter_regression/goal_clock.json:4` and implemented in the corrected predicate at `python/dynamuhsim/simulation/goal_clock.py:96`. Early route changes at that radius need not be a bug. Do not shrink it solely to hide turns.

Minimal proposal: explicitly freeze/select `sim_v1` for new simulation-time evaluations and test goal assignment timestamps; retain native geometric semantics. A later default change requires behavioral regression evidence. Historical sim-clock-only runs still spin, so clock repair must not be presented as a complete motion fix (`docs/demos/2026-10-29/sim_goal_clock/README.md:22`).

### F3 — Safety-corrected translation and independently bounded yaw can disagree

**Active scope:** response integration and its group variants; common worker decorates individual/shared output with a separate velocity-following yaw cap.

`ResponsiveMotion` collects native target yaw before the final RVO/projection, then writes projected XY while turning toward that earlier target (`experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:26`, `experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:45`). Group worker returns kernel yaw directly (`experiments/social_navigation/navigation/socnavgym_group_nav2/worker.py:47`); individual/shared common worker instead rate-limits yaw toward executed velocity (`experiments/social_navigation/navigation/common_gazebo/worker.py:39`). Neither final stage establishes that translation follows the current body heading. The projection limits speed but has no previous-velocity/acceleration argument (`experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:6`, `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:26`).

The adapter interpolates XY/yaw independently and derives a constant velocity for each interval (`experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:83`). A collision-free orthogonal proposal change from (0.8,0) to (0,0.8) is accepted by the current projection in the component probe below: finite-difference velocity change is 1.13137 m/s over 0.1 s, or 11.3137 m/s². This is a synthetic capability demonstration, not an observed human acceleration. In ideal piecewise-linear execution, velocity jumps at the interval boundary; position continuity alone is insufficient.

Minimal proposal: add a prior-velocity acceleration bound and heading-aware preferred motion before the guard, include reachability constraints within safety projection, and reconstruct final heading/displacement diagnostics after all corrections. Never rotate or move poses after safety validation without revalidating the entire swept interval. If smoothness and safety constraints are infeasible together, expose the event and stop safely. Existing `group_cohesion_v2` is a useful bounded proposal but is not a complete downstream guarantee (`python/dynamuhsim/simulation/group_cohesion_heading.py:24`).

### Additional scoped findings and non-findings

- **F4, Complexity shared scaling:** one blocked human can scale all humans, even remote ones, in discrete 1/32 increments. The component probe reproduced this. However both archived ComplexityNav traces examined here have zero scaled active samples; attribution to those runs is rejected. Evidence: `experiments/social_navigation/navigation/complexitynav_geometry/kernel.py:43`, `experiments/social_navigation/navigation/socnavgym_warehouse_lifecycle/geometry.py:38`.
- **F5, conditional legacy Warehouse static-geometry omission:** the older `rvo2` selection sets `corridor_mode=false`, while the C++ plugin's only obstacle insertion is inside `if(corridorMode_)`. Its Warehouse configuration therefore does not add the 42 factory boxes to its RVO planner (`experiments/social_navigation/navigation/trial.py:219`, `src/custom_corridor_plugins/src/rvo2_human_system.cpp:450`). Waypoint steering alone does not route around unseen shelves. Missing planner geometry is CONFIRMED statically for this selector; shelf penetration in a specific run is UNVERIFIED. This is not a finding against the maintained response/Complexity geometry paths.
- **F6, standalone stepping/fixed patrol:** the old three-person demo repeatedly switches between two assigned patrol goals, and applies 0.1 s pose steps (`experiments/social_navigation/gazebo_adapter/kernel.py:38`, `experiments/social_navigation/gazebo_adapter/kernel.py:69`, `experiments/social_navigation/gazebo_adapter/adapter.cc:100`). Fixed high-level routes do not mean all local avoidance is missing. Add interpolation only in this separate adapter if this is the affected entrypoint.
- **F7, hard constraints/failure behavior:** response projection can reduce target margins, run up to 3000 cycles, or fail terminally. Worker errors trigger adapter zero robot command plus world pause (`experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:32`, `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:48`, `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:64`). Such a freeze is an explicit runtime interlock, not evidence the social planner is choosing to stand still. Synchronous IPC can block wall time up to its socket timeout, but alone does not prove simulated-motion jerk (`experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:45`). No current fault log was observed.
- **F8, constraint scope:** millimeter/centimeter physical margins, reciprocal prediction assumptions, and the standing 1.4 m activation rule are weaker than a social-comfort/negotiation model. Neither global absence of reaction nor guaranteed yielding/deadlock avoidance follows from these implementations.
- **F10, stopped gait appearance:** the maintained runner calls the world builder with `walking` even for the standing profile, selecting the walking mesh. The response adapter always selects animation `walk` and advances it only with translation speed. A stopped person therefore holds the current walk frame rather than switching to an idle/turn animation (`python/dynamuhsim/runtime/nav2_trial.py:188`, `experiments/social_navigation/gazebo_adapter/run.py:80`, `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:58`). This is a CONFIRMED animation-policy limitation, not proof of foot sliding in a particular recording. A minimal follow-up is an explicit idle/walk/turn animation selector driven by final executed motion without changing navigation poses.
- **F9, runtime prerequisites:** this worktree currently lacks `experiments/social_navigation/.venv-socnavgym`, `experiments/social_navigation/vendor`, `experiments/social_navigation/navigation/complexitynav_geometry/generated`, and `experiments/social_navigation/navigation/socnavgym_response_fix/build`. These were checked read-only. Their absence prevents fresh native execution here; it does not establish that an external running installation has failed. Preparation/gates and installed-package provenance must be checked before a future run (`docs/SHARED_NAV2_RUNTIME.md:35`, `python/dynamuhsim/runtime/nav2_trial.py:159`).

## 8. Prioritized Root-Cause Table

Confidence applies to the claim as qualified below: CONFIRMED means directly established mechanism/mismatch, LIKELY means supported causal interpretation, and UNVERIFIED means a symptom or causal link still requires measurement. Confirmed permissive code is not proof the corresponding symptom occurs in every run. Priorities weight maintained group-capture relevance; legacy-only findings are conditional.

| Priority | Cause | Category | Evidence (file:line) | Confidence | Verification |
| --- | --- | --- | --- | --- | --- |
| P1 / TOP 1 | Upper group-envelope/intent instability contributes to spinning | ALGORITHM | `python/dynamuhsim/simulation/human_groups.py:72`; `docs/demos/2026-10-29/endpoint_humans/offline_reproduction.json:2`; `docs/demos/2026-10-29/sim_goal_clock/group_diagnostic.json:32` | LIKELY; excessive turning and same-goal flips documented | Matched measured-ego replay; separate radius, initial velocity and group-intent ablations; preserve per-person guard |
| P2 / TOP 2 | Native-wall goal expiry remains default while lifecycle advances in simulated ticks | INTEGRATION | `tools/capture_episode.py:26`; `experiments/social_navigation/navigation/socnavgym_group_nav2/worker.py:44`; `docs/demos/2026-10-29/encounter_regression/goal_clock.json:82`; `python/dynamuhsim/simulation/goal_clock.py:95` | CONFIRMED for documented default path; current vendor absent | Same ego inputs under different host schedules; compare goal-event sim timestamps with sim_v1 |
| P3 / TOP 3 | Final translation, yaw and acceleration are not jointly constrained | CONSTRAINT | `experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:43`; `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:26`; `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:83` | LIKELY contributor; missing final contract CONFIRMED | Orthogonal/reversal proposals; log executed speed/acceleration, yaw error and backward travel after projection |
| P4, conditional | Legacy C++ Warehouse selection omits factory obstacles from RVO | INTEGRATION | `experiments/social_navigation/navigation/trial.py:219`; `src/custom_corridor_plugins/src/rvo2_human_system.cpp:450` | CONFIRMED planner omission; physical penetration UNVERIFIED | Inspect selected generated SDF/plugin; count inserted AABBs; one shelf-route component case |
| P5 | Complexity guard scales every human by one discrete fraction | ALGORITHM | `experiments/social_navigation/navigation/complexitynav_geometry/kernel.py:43`; `experiments/social_navigation/navigation/socnavgym_warehouse_lifecycle/geometry.py:38` | CONFIRMED mechanism; not activated in two examined historical runs | Two-human blocked/free probe below; report per-ID changes and safe_fraction frequency |
| P6 | Older standalone adapter holds then directly steps pose at 10 Hz | RUNTIME | `experiments/social_navigation/gazebo_adapter/adapter.cc:100`; `experiments/social_navigation/gazebo_adapter/adapter.cc:123` | CONFIRMED selector-specific behavior | Sample native pose at physics rate; distinguish zero-order hold from response interpolation |
| P7 | Small isotropic surface gaps do not enforce personal-space comfort | CONSTRAINT | `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:16`; `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:48` | CONFIRMED limited constraint scope; perceived discomfort UNVERIFIED | Separate physical gaps from directional proxemics/time-to-collision/side commitment |
| P8 | Standing sidestep eligibility switches at 1.4 m without hysteresis | CONFIGURATION | `experiments/social_navigation/navigation/socnavgym_group_nav2/reactive_motion.py:34`; `experiments/social_navigation/navigation/socnavgym_group_nav2/standing_projection.py:47` | CONFIRMED threshold; chatter UNVERIFIED | Replay robot distances just above/below threshold and count eligibility/speed changes |
| P9 | Worker failure can intentionally freeze and pause the world | RUNTIME | `experiments/social_navigation/navigation/common_gazebo/worker.py:80`; `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:64` | CONFIRMED interlock; occurrence in current run UNVERIFIED | Inspect adapter_ok, fatal/restart_required and pause events before interpreting a stop |

## 9. Minimal Proposed Code Fixes

No fixes were implemented. These are reviewable follow-up proposals, not authorization to change production behavior or historical datasets.

1. **F1, group intention:** preserve the existing guard and use an explicit, frozen upper-motion profile. Start from existing `group_cohesion_v2`; measure native group proposal, envelope overlap, per-person projection correction, and final group spread. Do not globally remove all ORCA. Isolate envelope radius and measured velocity initialization separately if exact upstream-cause attribution is needed. Touch points: `python/dynamuhsim/simulation/human_motion_profiles.py:25`, `python/dynamuhsim/simulation/human_groups.py:72`, `python/dynamuhsim/simulation/group_cohesion_heading.py:36`.
2. **F2, clocks:** select/freeze the existing `sim_v1` clock for new runs and ensure all initial goal ages begin at accepted navigation time. Only after regressions should a narrowly scoped default/configuration change be considered. Preserve strict 50 s and geometric acceptance; do not edit vendor/global time. Touch points: `tools/capture_episode.py:26`, `python/dynamuhsim/runtime/capture.py:146`, `python/dynamuhsim/simulation/goal_clock.py:103`.
3. **F3, final motion:** add previous executed velocity and heading-aware reachability to the safety-stage contract; cap acceleration/deceleration, brake translation for large turning error, preserve static inward-facing behavior, and expose infeasibility. Update gait from actual final travel. Do not use post-guard smoothing or spline interpolation that can invalidate swept-clearance checks. Touch points: `experiments/social_navigation/navigation/socnavgym_response_fix/responsive_motion.py:43`, `experiments/social_navigation/navigation/socnavgym_response_fix/projection.py:6`, `experiments/social_navigation/navigation/socnavgym_response_fix/adapter.cc:83`.
4. **Conditional alternatives:** for F4 import Warehouse obstacles/waypoint routing into the selected legacy C++ backend; for F5 replace the shared scalar with independently validated per-person corrections if activation is measured; for F6 reuse the response adapter's interval contract in the old standalone adapter. Do not apply these changes to an unrelated selected runtime.
5. **Constraints/configuration:** add explicit optional social margins/priority policies and standing activation hysteresis only after separately measuring discomfort/chatter; keep these distinct from physical collision acceptance. Do not silently enlarge margins or accept a previously failed run.

## 10. Reproducible Verification Plan

### Checks actually performed in this audit

All ran via MCP in this worktree with `auto_source_ros=false`, without creating probe artifacts or bytecode:

- Static runtime tracing and numbered-source inspection; relevant JSON/configuration and historical reports read in place. `rg` was unavailable, so searches used `git grep`, `git ls-files` and bounded `find`.
- In-memory execution of current `geometry.py`: blocked/free two-human case returned global fraction **0.46875**, reducing even the unobstructed person's 0.08 m proposal to **0.0375 m**. Independent minimum static gap was **0.0025 m**.
- In-memory execution of current `projection.py`: separate collision-free orthogonal 0.8 m/s proposals were both accepted unchanged. This confirms no temporal acceleration constraint in that function; it is not a native trajectory measurement.
- In-memory GoalClock/mock-human check: age 50.0 s does not expire, age 50.1 s does. This tests the opt-in implementation, not the unavailable vendor clock.
- Recomputed committed endpoint replay summary: legacy aggregate yaw **78.7849420696 rad**, candidate **21.2403759245 rad**, reduction **0.7304005643**, recorded legacy reproduction RMSE **0**. This verifies summary arithmetic only.
- Read-only decompression of archived ComplexityNav worker logs: **complex01 127 rows / 90 active**, **complex02 140 rows / 107 active**, every active guard fraction **1.0**. No simulated rerun or log rewrite occurred.

Reproduce the two geometry/projection probes from the target-worktree root with system Python/Numpy; all source loading is in memory:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
import json, math, sys, types
from pathlib import Path
base = Path('experiments/social_navigation/navigation')
g = types.ModuleType('geometry')
p = base / 'socnavgym_warehouse_lifecycle/geometry.py'
exec(compile(p.read_text(), str(p), 'exec'), g.__dict__)
f, gaps = g.admissible_fraction(
    [(0., 0.), (0., 5.)], [(.08, 0.), (.08, 0.)], [.36, .36],
    (-10., -10.), (0., 0.), .25,
    [dict(x=.45, y=0., sx=.1, sy=2.)])
assert f == .46875
print('global fraction', f, 'free-person delta', .08*f, 'gaps', gaps)
sys.modules['geometry'] = g
p = base / 'socnavgym_response_fix/projection.py'
ns = {}; exec(compile(p.read_text(), str(p), 'exec'), ns)
kw = dict(starts=[(0., 0.), (0., 5.)], radii=[.36, .36],
          robot=(-10., -10.), robot_delta=(0., 0.), robot_radius=.25,
          boxes=[dict(x=20., y=20., sx=1., sy=1.)],
          dt=.1, max_speed=.8, fixed=set())
a, _ = ns['project'](deltas=[(.08, 0.), (.08, 0.)], **kw)
b, _ = ns['project'](deltas=[(0., .08), (0., .08)], **kw)
assert math.isclose(a[0][0], .08) and math.isclose(b[0][1], .08)
print('accepted separate proposals', a, b, 'delta-v/dt', math.sqrt(2)*.8/.1)
PY
```

Reproduce the historical fraction check without starting a backend:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 - <<'PY'
import gzip, json
from pathlib import Path
base = Path('experiments/archive/social_navigation/reports/shared_nav2_pilot')
for case in ('complex01', 'complex02'):
    with gzip.open(base/case/'worker_trajectory.jsonl.gz', 'rt') as stream:
        rows = [json.loads(line) for line in stream]
    active = [r for r in rows if r.get('active_phase')]
    fractions = [r['motion_guard']['safe_fraction'] for r in active]
    print(case, len(rows), len(active), min(fractions),
          sum(f < 1 for f in fractions))
PY
```

### Follow-up verification, not executed here

1. **Identify the affected run first.** Capture exact CLI, generated-world plugin, frozen profile assets, source/binary hashes, native dt, `ready.json`, gate timestamp and worker logs. Do not infer backend from the word “Warehouse,” from a plugin search-path entry, or from README feature lists. Verify installed ROS launch/plugin provenance matches the chosen checkout before execution.
2. **F1 matched-input test:** after dependencies are provisioned in the selected worktree, replay 60–120 recorded robot inputs through the baseline and one predetermined group treatment. First require exact baseline reproduction. Count same-goal intent flips, cumulative yaw/rate, path length, member-centroid radius, heading/velocity error, guard corrections and independent swept gaps. Split radius/initial-velocity ablations if claiming a unique cause. Retain negative outcomes.
3. **F2 clock test:** use identical simulated inputs and deliberately different synthetic host schedules, including pre-gate waiting. Sim-clock goal assignments/expiry must be identical; check strict 50.0 versus 50.1 boundary and native geometric arrival separately. Existing focused test entrypoint: `PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -p test_goal_clock.py -v`. These tests were inspected, not executed during this audit.
4. **F3 component test:** include 90/180-degree reversals, near-zero motion, start/stop, late robot braking, standing-facing and an avoidance-induced lateral correction. Evaluate final executed XY/yaw at physics rate and after projection; require proposed acceleration/turn limits and independently checked swept safety. Synthetic smoothness success is not native visual acceptance.
5. **Only if component checks justify it, run a short, separately authorized native smoke capture** with fresh output directory, predetermined head-on or crossing fixture, fixed human seed, and explicit clock/motion profile. Compare worker future state with adapter interpolation at the same time; compare actor and proxy world positions; inspect original simulation-time RGB, not only gait replay. Keep near-encounter coverage so reduced turning cannot pass by avoiding all interactions. No full benchmark/training is needed.
6. Preserve both motion and integration outcomes. The existing heading/cohesion candidate has historical motion PASS in two scenes but integrated PASS in only one because of a crossing bridge teardown abort (`docs/demos/2026-10-29/group_cohesion_heading/README.md:26`). Do not misclassify that shutdown failure as a human planner failure or claim a fully accepted default.

Report acceptance for this documentation task is separate: all eleven sections and source references must exist, only this Markdown path may change, the report-only commit must contain exactly one path, and local/remote branch hashes must match after a normal push.

## 11. Remaining Unknowns

- No CLI/log for the user's particular unnatural-motion observation was supplied. Which reachable Warehouse entrypoint/profile produced it remains unknown. No running process was inspected or restarted.
- The target worktree lacks native vendor/venv/generated/binary prerequisites. Exact current upstream SocNavGym and container internals, native goal-switching implementation, binary/source correspondence, and fresh Gazebo behavior were not independently executed here. Historical receipts and committed summaries provide bounded evidence, not a new runtime acceptance.
- Original ignored raw bags for the 2026-10-29 group experiments were not replayed. Published scalar summaries/diagnostics were checked; no new native visual QA or force-contact analysis is claimed. Native camera sampling at 1 Hz cannot resolve every physics-step discontinuity (`docs/demos/2026-10-29/endpoint_humans/README.md:44`).
- The exact share of group spinning due to envelope size, solver velocity initialization, native smoothing/noise, goal transitions, heading mismatch, or downstream projection remains unseparated. Existing treatments change more than one semantic property; closed-loop robot trajectories diverge and MPPI RNG is unpinned (`docs/demos/2026-10-29/endpoint_humans/plan.json:49`).
- Actual acceleration/jerk distributions, personal-space comfort, negotiated passing/yielding, deadlock frequency, and human penetration in arbitrary current runs remain UNVERIFIED. Physical clearance screens are not social-naturalness judgments.
- Two archived ComplexityNav runs do not exercise global scaling. Whether dense/adversarial Warehouse layouts activate it frequently requires a targeted component case or new bounded recording.
- No production fix, default promotion, dataset relabeling, Task 2 implementation, branch switch, worktree change, or simulation restart is included in this audit.
