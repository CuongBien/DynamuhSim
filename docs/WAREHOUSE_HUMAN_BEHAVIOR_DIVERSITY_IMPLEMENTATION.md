# Warehouse Human Behavior Diversity Implementation

Date: 2026-11-01. **STATUS: PARTIAL.** No staging, commit or push. All repository operations used remote MCP execute_bash on DESKTOP-NVIVQ2U, with auto_source_ros=false.

## 1. Executive Summary

Implemented and executed the opt-in `social_diversity_v1` preference layer in the active six-human SocNavGym Warehouse pipeline. Saved a fresh Task 2 baseline BEFORE behavioral edits, two candidate revisions, native ROS/Gazebo telemetry and MCAP, deterministic re-executions, pose playback, decoded GIFs, quantitative comparisons and 20 component fixtures. Nav2/MPPI source and configuration were not tuned.

The final candidate reduces same-input yaw, yaw switching and group spread and increases goal progress in one 30s/seed45 comparison. It also reduces human-human physical surface clearance from 0.355261 to 0.029478m. The first candidate had a robot-clearance regression and a 21.1s stop; its evidence remains untouched. Final opt-in turn-at-rest and robot-buffer changes remove that prolonged-stop symptom in the new live window (longest stop 0.7s) and improve matched robot clearance. Those live runs have different robot trajectories and durations, so their differences are descriptive only.

**Not overall behavioral acceptance:** all three Gazebo episodes timed out with true exit124 and runner ERROR; group social comfort/HH clearance remains adverse, overtaking completion is unproven, and no real-human dataset validation was performed. Keep the profile opt-in. Saved-pose replay is evidence inspection, not controller validation.

Evidence root **E** (remote absolute path):

`/home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/depth_identity_guard/generated/generalization_worktree/experiments/social_navigation/benchmark/generated/human_behavior_diversity_20261101T073539_924830Z`

E is ignored by Git. `manifest.json` is the finalized inventory; `manifest.initial.json` preserves initialization provenance. `SCHEMA.md` documents field mappings, unavailable signals and timestamps. Authoritative paired comparison is `E/comparison_final/`; live-pose media is `E/comparison_v2/`. Earlier comparison outputs remain with explicit correction notes.

## 2. Source Changes

| Path | Task 3 contribution |
| --- | --- |
| `python/dynamuhsim/simulation/social_diversity.py` (new) | Seeded traits, persistent contextual interaction states, shared-goal/soft-cohesion preferences; reuses GroupGoal. |
| `python/dynamuhsim/simulation/human_motion_profiles.py` | Strict opt-in dispatch/configuration for social_diversity_v1. |
| `experiments/social_navigation/navigation/socnavgym_group_nav2/fixture.py` | Retain seed on kernel for trait identity; Task 2 motion contract retained. |
| `.../socnavgym_group_nav2/worker.py` | Save controller state/traits/partner/events/group preference in worker response and raw JSONL. |
| `.../socnavgym_response_fix/responsive_motion.py` | Pre-ORCA preference hook; per-agent neighbor distance/speed/turn/acceleration; turn at rest for reversed goals; opt-in 0.12m robot buffer inside projection. No post-projection XY smoothing. |
| `python/dynamuhsim/runtime/capture.py`, `tools/capture_episode.py` | Accept/freeze explicit profile; require simulation-clock contract. Default legacy profile unchanged. |
| `tools/replay_warehouse_humans.py` (new) | Replay real native kernel using saved robot requests and initial fixture/seed/clock; exclusive JSONL and receipt. |
| `tools/warehouse_human_evidence.py` (new) | Normalize actual Gazebo poses, record schema, recompute metrics, verify/render GIFs and figures. |
| `tools/warehouse_behavior_components.py` (new) | Bounded one/two/three-human fixtures of real motion stack, separate from native lifecycle/Gazebo. |
| `tests/test_social_diversity.py` (new) | Seed isolation, encounter persistence, bounded wait/exit, overtaking trigger, stopped robot, regroup hysteresis, conversation and turn-at-rest regression. |
| `tests/test_warehouse_behavior_evidence.py` (new) | Seven tests against retained actual data, matching, replay, decoded GIFs, schema, interpolation and metric recomputation. |

Initial HEAD/branch: `51fc28d2f8dc46890c3dec806cfec1398c7f8018`, `feat/depth-detector-generalization`. Initial status/diff/source copies: E/logs/initial_status.txt, initial.diff and initial_source/. Task 2 projection.py, motion_contract.py, motion_constraints.json, goal-clock tests, final-motion tests, diagnostics, nav2_trial.py and prior report are byte-identical to their initial hashes. Five initially dirty files were intentionally extended; E/logs/task2_preservation.json separates them. No reset/stash/clean/checkout/add/commit/push was used.

Revision1 source is preserved in E/candidate/source_snapshot and its capture_freeze.json; revision2 in E/candidate_v2/source_snapshot and capture_freeze.json. Final native freeze verification PASS before removing session-owned dependency links. Final behavior bytes were not modified after revision2 recording. Analysis-tool corrections are separate from controller revisions.

## 3. Active Human Behavior Architecture

Supported capture CLI -> group worker -> fixture/real SocNavGym-v2 kernel -> native lifecycle and goal clock -> shared GroupGoal upper intention -> SocialDiversity preference -> heading-aware Task 2 preference -> individual human/robot ORCA -> coupled swept physical projection and interval delta-v constraints -> final bounded yaw -> existing C++ adapter -> Gazebo proxy and actor.

Native step/lifecycle/RNG operations still run. This profile replaces the noisy tentative velocity as the final preference; it does not inject saved human poses into controller replay. Shared upper intention does not represent a rigid group collision disc. Individual avoidance, all 42 AABBs and independent projection validation remain active. ComplexityNav and older RVO2 plugin paths are not selected. No robot social policy/training was added.

## 4. Individual Behavioral Diversity

Independent SHA256(seed,human-ID)-derived RNG gives stable traits without consuming global native RNG: preferred speed 0.46–0.70m/s, interval acceleration 0.60–0.80m/s², deceleration 1.2–1.6m/s², reaction neighbor distance 1.6–2.4m, personal-space encounter margin 0.18–0.36m, turn rate 1.05–1.55rad/s and varying yield tendency. Values are recorded in worker behavior.traits and component result.json. These are engineering ranges, not a fitted population distribution.

The field named goal_persistence_s currently controls encounter/intention commitment (0.8–1.4s), **not individual native goal expiry**. Per-agent native goal-lifetime diversity is NOT IMPLEMENTED. Native sim-clock expiry/arrival and shared group goal lifecycle remain unchanged. No heading or velocity jitter was introduced.

## 5. Human-Human Interaction

A bounded closest-approach test uses actual previous executed velocities, desired goal motion, distance and encounter margin. Head-on traffic receives a persistent right-side preference; crossing can yield according to identity priority, motion context and trait. A slower same-direction partner can trigger overtaking state. Partner retention and minimum commitment plus 0.6s clear hysteresis prevent repeated side reselection. ORCA can override the preferred side for safety.

Yielding becomes waiting below 0.08m/s; waiting is bounded at 0.8s before passing, with cooldown. Resuming has a valid timed exit. Events and state occupancy are saved. Actual final Warehouse data contains walking, passing, overtaking, yielding, waiting, resuming and regrouping labels with nontrivial displacement; labels alone do not certify maneuver completion. Component head-on x-order passing completes at both seeds; the overtaking fixture does **not** reverse x order at either seed. Successful overtaking is therefore NOT ACCEPTED.

## 6. Human-Robot Interaction

Measured Gazebo robot pose/velocity is an environmental agent input. Moving crossing encounters may yield/wait; stopped robots receive passing preferences rather than unconditional yielding. The same persistence/resume rules apply. Final opt-in projection includes an additional 0.12m robot-disc buffer, validated within the same solver and swept check. Physical metrics convert the guard's 0.37m robot envelope back to the common physical 0.25m disc.

Final live worker data has 49 behavior transitions (30 with a human partner), 23 passing-side selections, four yielding starts and three waiting starts; **zero transitions select the robot as partner**. Human-robot behavioral-state acceptance is therefore limited to component fixtures and the robot-aware ORCA/projection, not demonstrated native robot yielding. The 29 proximity entries include physical proximity, not an inferred reaction. Full counts: E/comparison_final/interaction_analysis.json.

Moving/stopped/constrained robot component fixtures save real motion-stack results at both seeds. Live crossing contains observed human-robot proximity episodes, and native controller re-execution reproduces actual candidate states exactly. Sensor-perception reaction latency and robot mesh clearance are NOT MEASURED. Nav2/MPPI policy is unchanged; live robot trajectories differ as the closed loop responds to humans.

## 7. Group Behavior

Reused GroupGoal instead of unstable upper group-envelope ORCA. Actual member envelope and measured centroid are recorded. Flexible centroid attraction begins outside 0.9m, with gain0.65/s and cap0.24m/s; forward pace is reduced as spread grows, without prescribing member slots or rigid formation. Regrouping labels have 1.15m enter/0.95m exit hysteresis.

Raw baseline solver telemetry and prior source establish native enclosing-radius/zero-initial-velocity limitations, but this experiment is not radius-only or velocity-initialization-only causal ablation. `group_orca_v1` and `group_cohesion_v2` remain available; their regressions ran. Final profile bypasses the upper disc solver while preserving per-human avoidance. Reduced spread is measured; universal resolution of P3 is not claimed.

Initial lifecycle/group objects are retained and native formation/dispersal hooks remain. New formation/dispersal cycles were not demonstrated in these short native windows. Standing conversation is exercised only in saved component fixtures, not in a new native standing run. Component regrouping ends below 1m spread at both seeds. Group-obstacle fixtures progress around declared boxes; this is not acceptance of arbitrary Warehouse routing.

## 8. Motion Execution and Safety

Task 2 interval contract remains: acceleration/deceleration discs are solved together with physical speed/HH/HR/static constraints, with explicit safety-only fallback/infeasibility reporting. No unchecked XY smoothing follows projection. New per-agent turn/acceleration limits apply only to the opt-in profile; legacy replays remain exactly equal. At almost-zero executed speed with a nonzero reversed intention, bounded turn-at-rest permits progress rather than freezing heading. This changes yaw only, retaining projected XY.

Adapter source/binary were unchanged and reused, not rebuilt. Source SHA256 `ffbcda748dc1dba4e06beb79c6e8fa93c77da25fbe2348b10f92062c36279abb`; binary SHA256 `524fe8e945daad94319d7ac1a5f6377a5f36cbf6c505bb580e50e2e73102e984`. No C++ modification/build was required.

The adapter remains piecewise-linear. Final native sampled acceleration max10.625528m/s² versus baseline11.636364m/s² is cadence-dependent and exposes interval-boundary velocity jumps. A continuous 1.6m/s² physical guarantee is NOT CLAIMED. Final matched interval max1.597348m/s² and zero constraint-relaxation steps do not imply continuous acceleration acceptance.

No nominal-disc linear-sample clearance violations were recorded in the three live runs or 20 component windows. Matched final HH gap falls to0.029478m (baseline0.355261); live HH minima0.040000m (baseline0.367837). **This is a material physical-clearance regression despite remaining positive.** Robot matched physical gap improves to0.128264m against recorded robot segments. Full mesh, contact forces and unsampled robot dynamics remain unverified. Safety/social-comfort acceptance stays PARTIAL.

## 9. Experiment Configuration

All native runs: maintained `walking` scenario, supported `crossing` encounter, seed45, six humans/two initial triplets, Warehouse/social_warehouse geometry, 42 boxes, human radius0.36m, robot physical radius0.25m, planner/worker dt0.1s, physics dt0.0025s, sim_v1 clock, accepted Nav2 start gate, native lifecycle, headless Gazebo Harmonic8.15.0 and ROS2 Jazzy. moderate_v3 config copied changing only sim_budget_s to35.0; camera, Nav2 and MPPI settings retained. The external100s timeout includes startup and is not the measured active sim duration. Runtime package versions/commands: E/logs/runtime_versions.json.

| Run | Scenario | Seed | Executed active sim seconds | Humans / human samples | Absolute pose window s | Proximity entries | Runtime status |
| --- | --- | --- | ---: | --- | --- | ---: | --- |
| baseline | walking / crossing | 45 | 29.90 | 6 / 10926 | [15.3075, 45.2075] | 14 | ERROR / exit124 / INCOMPLETE |
| candidate | walking / crossing | 45 | 36.10 | 6 / 13002 | [15.1075, 51.2075] | 18 | ERROR / exit124 / INCOMPLETE |
| candidate_v2 | walking / crossing | 45 | 36.71 | 6 / 13224 | [11.4075, 48.1175] | 29 | ERROR / exit124 / INCOMPLETE |

Each run directory is E/<run>/native; raw poses/MCAP in episode/, worker/ready/start_gate/geometry/runtime config in runtime/, configuration in E/<run>/config.json, source/binary freeze in native/capture_freeze.json. Proximity entries count new HH or HR surface-gap<0.8m episodes, not intentional avoidance successes. All recordings include warmup in raw streams; pose movement metrics use the full post-gate complete-sample window.

Matched re-execution: baseline/controller.jsonl and candidate_v2/controller.jsonl, exactly300 shared robot requests, [15.3075,45.3075]s, dt0.1, six IDs, same initial state/geometry/seed/clock. Each has1800 future human endpoints plus initial state in measurement (1806 timeline samples). The final worker future endpoint is a replay result, not evidence it was executed in Gazebo. Native baseline executed299 complete intervals; final candidate executed367 of368 proposed intervals.

Component experiments (all16s windows; native lifecycle not stepped):

| Scenario | Seed | Humans / endpoint samples | Sim s | Goal progress m | Absolute yaw rad | Clearance violation intervals | Raw file relative to E |
| --- | --- | --- | ---: | ---: | ---: | ---: | --- |
| independent | 45 | 1 / 160 | 16.0 | 5.968 | 0.000 | 0 | components/independent_seed45/trajectory.jsonl |
| crossing | 45 | 2 / 320 | 16.0 | 11.929 | 2.162 | 0 | components/crossing_seed45/trajectory.jsonl |
| head_on | 45 | 2 / 320 | 16.0 | 11.329 | 4.624 | 0 | components/head_on_seed45/trajectory.jsonl |
| overtaking | 45 | 2 / 320 | 16.0 | 11.571 | 6.103 | 0 | components/overtaking_seed45/trajectory.jsonl |
| group_obstacle | 45 | 3 / 480 | 16.0 | 18.855 | 5.390 | 0 | components/group_obstacle_seed45/trajectory.jsonl |
| regrouping | 45 | 3 / 480 | 16.0 | 21.516 | 2.828 | 0 | components/regrouping_seed45/trajectory.jsonl |
| constrained_robot | 45 | 1 / 160 | 16.0 | 5.126 | 6.524 | 0 | components/constrained_robot_seed45/trajectory.jsonl |
| moving_robot | 45 | 1 / 160 | 16.0 | 5.855 | 3.993 | 0 | components/moving_robot_seed45/trajectory.jsonl |
| stopped_robot | 45 | 1 / 160 | 16.0 | 5.944 | 1.467 | 0 | components/stopped_robot_seed45/trajectory.jsonl |
| conversation | 45 | 3 / 480 | 16.0 | 0.000 | 4.423 | 0 | components/conversation_seed45/trajectory.jsonl |
| independent | 46 | 1 / 160 | 16.0 | 5.995 | 0.000 | 0 | components/independent_seed46/trajectory.jsonl |
| crossing | 46 | 2 / 320 | 16.0 | 11.982 | 1.675 | 0 | components/crossing_seed46/trajectory.jsonl |
| head_on | 46 | 2 / 320 | 16.0 | 11.982 | 2.210 | 0 | components/head_on_seed46/trajectory.jsonl |
| overtaking | 46 | 2 / 320 | 16.0 | 11.732 | 9.935 | 0 | components/overtaking_seed46/trajectory.jsonl |
| group_obstacle | 46 | 3 / 480 | 16.0 | 18.795 | 5.480 | 0 | components/group_obstacle_seed46/trajectory.jsonl |
| regrouping | 46 | 3 / 480 | 16.0 | 21.557 | 2.821 | 0 | components/regrouping_seed46/trajectory.jsonl |
| constrained_robot | 46 | 1 / 160 | 16.0 | 5.994 | 1.065 | 0 | components/constrained_robot_seed46/trajectory.jsonl |
| moving_robot | 46 | 1 / 160 | 16.0 | 5.978 | 3.590 | 0 | components/moving_robot_seed46/trajectory.jsonl |
| stopped_robot | 46 | 1 / 160 | 16.0 | 5.991 | 1.827 | 0 | components/stopped_robot_seed46/trajectory.jsonl |
| conversation | 46 | 3 / 480 | 16.0 | 0.000 | 4.423 | 0 | components/conversation_seed46/trajectory.jsonl |

Every component has its own config/result/traits, and inherits final revision2 source/binary provenance. Its geometry is explicitly saved and differs from full Warehouse. Endpoints are actual component outputs, not Gazebo poses. Per-case state occupancy, interaction events, clearance and group summaries: E/components/metrics.json. No component baseline A/B was run.

## 10. Saved Raw Data Inventory

For baseline, candidate (revision1) and candidate_v2, E/<run>/ retains:

- config.json, source_provenance.json; native/capture_freeze.json and plan.json freeze source/assets/configuration.
- native/runtime/worker_trajectory.jsonl: every request, initial/before/future state, robot input, group membership, native/group intent, projection stages, clock and events actually available.
- native/episode/gazebo_poses.jsonl: authoritative observed proxy/robot poses; ground_truth.jsonl: adapter-applied/interpolated telemetry; raw_bag/raw_bag_0.mcap and ROS metadata retained.
- human_states.jsonl, human_intentions.jsonl, group_states.jsonl, robot_inputs.jsonl, behavior_events.jsonl, safety_events.jsonl: normalized views without replacing original streams.
- pose_timeline.jsonl, metrics.json, result.json, process.json and runtime.log; runtime result remains native/episode/result.json.
- controller.jsonl: same-input native-controller re-execution, separately identified from live execution.

Original bags are22,848,280 /20,935,428 /33,655,163 bytes for baseline/revision1/revision2. No raw bag/trajectory was deleted. SHA256 and byte counts of retained artifacts are in finalized manifest. Timestamp/field mappings and NOT MEASURED signals: E/SCHEMA.md. No missing baseline state labels or active constraint multipliers are synthesized.

## 11. Replay Implementation

**Mode A, controller re-execution:** tools/replay_warehouse_humans.py rebuilds actual native kernel from saved plan/seed/encounter, loads shared IDs/goals, restores sim clock and feeds every recorded robot request at its original timestamp. It calls actual kernel.advance, behavior, ORCA/projection and records new outputs. No final human pose injection occurs. Baseline legacy replay was performed before edits and again with final source. Final native candidate re-execution uses its own recorded robot inputs and reproduces all368 steps.

**Mode B, saved-pose playback:** tools/warehouse_human_evidence.py loads observed Gazebo poses, copies them into a timeline, interpolates shortest yaw/XY at5Hz, and renders IDs/robot/obstacle context with scenario/seed/active and absolute sim time. It saves GIFs and checks every keyframe against the saved source. This is labelled 2D recorded-pose replay, not fabricated Gazebo camera footage and not a controller test.

No compatible real actor trajectory playback facility was adopted; bounded GIF playback is the fallback. MP4 was not produced because ffmpeg was absent; no packages were installed. Original runtime onboard-camera previews remain separate and accelerated4x.

## 12. Replay Verification Results

| Verification | Result | Evidence |
| --- | --- | --- |
| Pre-edit baseline300-step reproduction | max XY0m, yaw0rad, clock snapshot/events identical | baseline/controller.receipt.json |
| Final-source baseline300-step reproduction | max XY0m, yaw0rad, clock snapshot/events identical | replay/baseline_final.receipt.json |
| Revision1 native362-step reproduction | max XY0m, yaw0rad, clock/events identical | replay/candidate_native.receipt.json |
| Final native368-step reproduction | max XY0m, yaw0rad, clock/events identical | replay/candidate_v2_native.receipt.json |
| Baseline1821 / final2204 saved-pose keyframes | reconstruction XY0m and yaw0rad | replay/playback_verification_v2.json |
| Gazebo vs adapter interpolation | max XY<=1.114e-14m; max yaw<=1.529e-10rad | replay/*_adapter_execution_alignment.json |
| GIF decode/hash/frame/timeline verification | PASS | playback_verification_v2.json and saved_evidence_tests.log |
| Focused source/runtime regression suite |117 tests PASS | logs/final_focused_tests.log |
| Tests on retained real evidence |7 tests PASS | logs/saved_evidence_tests.log |
| Final Python compilation |19 modified/new Python files PASS, in memory | logs/final_compilation.json |
| C++ rebuild |NOT RUN; no C++ changed | source/binary hashes above |

Nonzero original replay-file hash differences can arise from recorded diagnostic wall compute time, while human states/clock events match exactly. No numerical deviation in reproduced XY/yaw was hidden.

## 13. Baseline vs Candidate Metrics

Authoritative data: E/comparison_final/metrics_before_after.csv/json and baseline_controller_metrics.json / candidate_v2_controller_metrics.json. Both full300-step windows are identical external inputs. No favorable sub-window selection. Different live inputs/durations are NOT substituted for matched A/B.

| Metric (units in key) | Baseline | Final candidate | Candidate minus baseline |
| --- | ---: | ---: | ---: |
| goal_progress_same_target_m | 14.839897 | 56.421342 | +41.581445 |
| path_length_m | 28.333717 | 64.673468 | +36.339751 |
| total_absolute_yaw_rad | 169.773224 | 50.379722 | -119.393502 |
| heading_error_p95_deg | 5.062313 | 4.201795 | -0.860518 |
| yaw_sign_switches | 514.000000 | 112.000000 | -402.000000 |
| interval_acceleration_max_mps2 | 1.600000 | 1.597348 | -0.002652 |
| interval_acceleration_p95_mps2 | 0.839811 | 0.743181 | -0.096630 |
| group_spread_max_m | 2.688263 | 1.640362 | -1.047901 |
| group_spread_p95_m | 2.611166 | 1.503615 | -1.107551 |
| predicted_HH_gap_m | 0.355261 | 0.029478 | -0.325783 |
| predicted_HR_gap_m | 0.077559 | 0.128284 | +0.050725 |
| replayed_robot_segment_HR_gap_m | 0.077559 | 0.128264 | +0.050705 |

Position/yaw metrics come from saved endpoints; speed=deltaXY/dt; acceleration=vector delta-v/dt (projection interval audit includes initial rest). Heading error uses midpoint adapter yaw for segments with speed>0.1m/s. Yaw total sums absolute wrapped increments; net yaw is separately saved per agent. Sign-switch count discards angular rates<=0.1rad/s. Stop speed<0.01m/s; episodes/durations are retained. Progress sums reductions in distance only while the same recorded goal is active; it is not full episode completion. Spread is max member-centroid distance per recorded group, with p95/max over all group samples. Same-target native completion checks are not an independent success score; baseline has0 geometric checks, candidate103 and6 assignment events during replay, reflecting retained native group threshold semantics.

Clearance is nominal circle/AABB swept linear geometry. `predicted_clearance_m` uses worker robot constant-velocity prediction, converted to common physical radii. `linear_sample_swept_clearance_m` uses successive saved robot inputs aligned to human interval; only the final future endpoint is extrapolated. Raw guard residuals use an inflated robot envelope and are separately retained. Earlier E/comparison/analysis misaligned robot time and E/comparison_v2/CSV used unconverted guard residual; correction notes preserve and explicitly invalidate those comparisons. Use comparison_final as authoritative. No negative observed clearance is inferred from the superseded misaligned calculation.

Native full-window observations (descriptive, not paired): baseline/revision1/final HH surface minima0.367837/0.040000/0.040000m; HR0.078065/0.477992/0.486323m; zero linear-sample violation segments. Longest individual stop0.2/21.1/0.7s. Final actual state occupancy includes seven categories; baseline categories are NOT MEASURED. Occupancy/traits/event variation is saved, not an empirically validated diversity score.

## 14. Visual Evidence Inventory

| Render | Relative path under E | Frames | Duration s | Bytes | Verification |
| --- | --- | ---: | ---: | ---: | --- |
| baseline | `baseline/media/recorded_pose_replay.gif` | 181 | 36.2 | 1204378 | decoded every frame / PASS |
| candidate_v2 | `candidate_v2/media/recorded_pose_replay.gif` | 184 | 36.8 | 1287388 | decoded every frame / PASS |
| comparison | `comparison_v2/replay_comparison.gif` | 184 | 36.8 | 2748351 | decoded every frame / PASS |

Other actual generated media: comparison_v2/trajectories_before_after.png, behavior_timeline.png, group_cohesion.png, speed_acceleration.png (speed plot), acceleration.png; comparison_final/controller_trajectories_events.png (matched inputs, yield/wait entry crosses); components/trajectories_seed45.png and trajectories_seed46.png. Axes use world/fixture metres and simulation seconds, speed m/s, acceleration m/s². Common limits are used for paired trajectory and same-measure panels. Raw native camera images/4x preview GIFs remain in native/episode. The final native trajectory figure was reopened visually through remote MCP, and every GIF frame decoded programmatically.

The synchronized live GIF aligns first active observed poses, preserves5Hz timing and visibly holds the final baseline after its shorter trace. It is a full-window qualitative comparison with different live inputs; the matched controller event plot supplies the causal-input comparison. Recorded-pose reconstruction is validated at original saved keyframes, not an unsupported claim that5Hz GIF pixels preserve every native tick.

| Scenario | Baseline Data | Candidate Data | Replay Artifact | Comparison | Status |
| --- | --- | --- | --- | --- | --- |
| Native walking/crossing45, revision1 | baseline/native | candidate/native | comparison/replay_comparison.gif | comparison/ plus explicit correction notes | INCOMPLETE runtime; superseded candidate retained |
| Native walking/crossing45, final | baseline/native | candidate_v2/native | comparison_v2/replay_comparison.gif | independent live windows; comparison_final matched controller data | PARTIAL |
| Matched native controller crossing45 | baseline/controller.jsonl | candidate_v2/controller.jsonl | comparison_final/controller_trajectories_events.png | comparison_final/metrics_before_after.csv | reproduction PASS; HH clearance adverse |
| Component A–J seed45/46 | NOT RUN | components/*/trajectory.jsonl | components/trajectories_seed45.png / seed46.png | no baseline A/B |20 bounded component windows; no Gazebo claim |

## 15. Gazebo Runtime Results

Three fresh actual headless `gz sim -r -s` runs started, accepted Nav2 gate, advanced /clock and recorded all six proxy IDs, robot inputs and worker responses. Each used a fresh output path and100s external timeout with INT and20s kill allowance. True process exit124, runner ERROR/KeyboardInterrupt, and existing rcl_shutdown-already-called teardown message are retained. Final source/binary freeze verified PASS. These are INCOMPLETE episodes even though raw trajectories/replays are usable.

All three: no worker terminal error or adapter fault, Gazebo clean exit true, ROS launch shutdown return0, owned cleanup PASS with no remaining members. Final process inventory has no owned native/replay process. Existing installed vendor/venv were linked only into previously absent paths; both session-owned links were removed after exact-target checks. External targets, main checkout sources and server configuration were not written. No system install or full rebuild occurred.

## 16. Acceptance Gates

| Gate | Status | Evidence / limit |
| --- | --- | --- |
|1 Actual data |PASS |3 native runs, six humans, saved observed poses/MCAP |
|2 Replay |PASS |controller reproduction and saved-pose reconstruction verified |
|3 Visual evidence |PASS |decoded GIFs and reopened trajectory PNG |
|4 Matched comparison |PASS scoped |one300-step crossing45 shared-input pair; live A/B unmatched |
|5 Human naturalness / motion |PARTIAL |yaw/progress/spread improve; HH clearance worsens, continuous acceleration not guaranteed |
|6 Interaction |PASS scoped |real live proximity, contextual states/events; not all maneuver outcomes accepted |
|7 Safety |PARTIAL |no nominal-disc sampled collisions; HH gap regression fully reported; no mesh/contact certification |
|8 Diversity |PASS scoped |seven live states and seeded traits; conversation only component; overtaking completion unproven |
|9 Reproducibility |PASS scoped |raw hashes, source freezes, commands and124 executed tests |

Overall PARTIAL; not SUCCESS. No claim of statistical human realism, general naturalness, a complete navigation episode or completed human goal program.

## 17. Remaining Problems

1. Final HH clearance is materially worse and personal space is a preference, not a validated comfort constraint. Do not promote profile to default without resolving this regression.
2. Continuous velocity/acceleration discontinuities remain in the unchanged straight-segment adapter. No unchecked spline was introduced.
3. Only one native encounter/seed has a matched controller pair; live walking/head_on and multiple native seeds were not extended. Component head_on/crossing support does not replace native coverage.
4. Overtaking states occur, including live data, but isolated overtaking completion did not occur in either fixture. Maneuver acceptance remains incomplete.
5. Per-agent native goal-expiry persistence, new formation/dispersal cycles, native standing conversation and full geometry detour robustness are not validated.
6. All episodes timed out before capture acceptance. Successful cleanup is not episode PASS. Robot policy remained unchanged; its closed-loop outcome was not optimized.
7. No dedicated compatible Gazebo actor-pose playback integration/3D human visual audit or full mesh safety certification.2D pose playback is provided and checked.
8. No real-human reference dataset, statistically representative seeds, reaction-latency classifier, universal deadlock metric or learned social policy. The historical Task2 metrics were not reused as a fresh baseline.
9. Analysis corrections were necessary for robot endpoint timing and inflated-radius units; raw evidence and original outputs remain, authoritative corrected comparison is explicit.

## 18. Exact Reproduction Commands

Run remotely through MCP execute_bash, auto_source_ros=false, from the exact worktree. Pick NEW output names; tools refuse overwrite. Dependency mappings removed at cleanup must be restored only when absent. Do not reset working changes or run a historical baseline launch against changed source bytes.

```bash
cd /home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/depth_identity_guard/generated/generalization_worktree
pwd
git branch --show-current
git status --short
E="$PWD/experiments/social_navigation/benchmark/generated/human_behavior_diversity_20261101T073539_924830Z"
source /opt/ros/jazzy/setup.bash
source /home/cuongbien/DynamuhSim/install/setup.bash
# Only create absent paths; preserve anything already present.
test ! -e experiments/social_navigation/.venv-socnavgym && test ! -L experiments/social_navigation/.venv-socnavgym && ln -s /home/cuongbien/DynamuhSim/experiments/social_navigation/.venv-socnavgym experiments/social_navigation/.venv-socnavgym
test ! -e experiments/social_navigation/vendor && test ! -L experiments/social_navigation/vendor && ln -s /home/cuongbien/DynamuhSim/experiments/social_navigation/vendor experiments/social_navigation/vendor

# Mode A: native baseline replay using final legacy-compatible source.
PYTHONDONTWRITEBYTECODE=1 experiments/social_navigation/.venv-socnavgym/bin/python tools/replay_warehouse_humans.py --experiment "$E" --output "$E/replay/baseline_RECHECK_NEW.jsonl"
# Native final candidate reproduction with its own recorded inputs.
PYTHONDONTWRITEBYTECODE=1 experiments/social_navigation/.venv-socnavgym/bin/python tools/replay_warehouse_humans.py --experiment "$E" --input-run "$E/candidate_v2/native" --output "$E/replay/candidate_native_RECHECK_NEW.jsonl"
# Same baseline external inputs, candidate algorithm; outputs are controller replay.
PYTHONDONTWRITEBYTECODE=1 experiments/social_navigation/.venv-socnavgym/bin/python tools/replay_warehouse_humans.py --experiment "$E" --profile social_diversity_v1 --output "$E/replay/candidate_matched_RECHECK_NEW.jsonl"

# Fresh native smoke with final candidate; native/NEW must not exist.
PYTHONDONTWRITEBYTECODE=1 python3 tools/capture_episode.py prepare --run-dir "$E/candidate_v2/NATIVE_NEW" --config "$E/candidate_v2/config.json" --scenario walking --seed 45 --encounter-profile crossing --human-motion-profile social_diversity_v1
PYTHONDONTWRITEBYTECODE=1 python3 tools/capture_episode.py verify --run-dir "$E/candidate_v2/NATIVE_NEW"
timeout --signal=INT --kill-after=20s 100s env PYTHONDONTWRITEBYTECODE=1 python3 tools/capture_episode.py run --run-dir "$E/candidate_v2/NATIVE_NEW"
# Preserve actual shell return code immediately and save it; timeout124 is INCOMPLETE.

# Recompute from saved raw inputs into NEW derived output, no simulator.
python3 -c 'import sys;from pathlib import Path;Path(sys.argv[1]).mkdir(exist_ok=False)' "$E/comparison_RECHECK_NEW"
MPLBACKEND=Agg PYTHONDONTWRITEBYTECODE=1 python3 tools/warehouse_human_evidence.py --experiment "$E" --candidate-case candidate_v2 --comparison-dir comparison_RECHECK_NEW --skip-render
# Mode B re-render: use another NEW comparison directory and unique receipt name.
python3 -c 'import sys;from pathlib import Path;Path(sys.argv[1]).mkdir(exist_ok=False)' "$E/render_RECHECK_NEW"
MPLBACKEND=Agg PYTHONDONTWRITEBYTECODE=1 python3 tools/warehouse_human_evidence.py --experiment "$E" --candidate-case candidate_v2 --comparison-dir render_RECHECK_NEW --verification-name playback_RECHECK_NEW.json
# Existing baseline/candidate GIFs are reused and decoded; new synchronized GIF is rendered.

PYTHONDONTWRITEBYTECODE=1 experiments/social_navigation/.venv-socnavgym/bin/python tools/warehouse_behavior_components.py --output "$E/components_RECHECK_NEW"
DYNAMUHSIM_BEHAVIOR_EVIDENCE="$E" PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=python:tests python3 -m unittest test_social_diversity test_warehouse_behavior_evidence test_goal_clock test_final_motion test_group_diagnostics test_group_projection_regressions test_group_orca test_group_intent test_group_cohesion test_group_cohesion_heading test_capture_runtime test_nav2_runner_integration test_capture_audit test_nav2_runtime test_encounters test_persistent_conversation -v

git diff --check
git status --short
git diff --stat
git diff --name-only
git ls-files --others --exclude-standard
```

The prepare command above uses the supported `--seed 45` argument. No population-size flag exists or was used. Exact actually executed process argv/exit/start/end records are in E/<run>/process.json and logs/*_exit.json. The original pre-edit controller replay command/script is preserved at E/replay/controller_replay.py. Analysis/render implementation is tracked; retained extra figure commands are E/replay/plot_matched.py and analyze_components.py (choose new destination names before another exclusive export). Source/asset hashes are in each native/capture_freeze.json; cryptographic raw inventory and commands are in manifest.json.

Git safety: exact target worktree/branch remained, Task2 preserved, only listed human behavior/evidence/test/report files extended. All task writes were within target worktree; this is not a certification that other processes never changed the main checkout concurrently. New recordings stay ignored. No git add, commit or push.

Final tracked diff --stat (includes preserved Task2 changes; untracked files listed separately in E/logs/final_git_inventory.json):

```text
 .../navigation/socnavgym_group_nav2/fixture.py     |  3 ++
 .../navigation/socnavgym_group_nav2/worker.py      |  7 +++
 .../socnavgym_response_fix/projection.py           | 53 ++++++++++++++++++----
 .../socnavgym_response_fix/responsive_motion.py    | 41 ++++++++++++++---
 python/dynamuhsim/runtime/capture.py               |  6 +--
 python/dynamuhsim/runtime/nav2_trial.py            |  4 +-
 .../dynamuhsim/simulation/human_motion_profiles.py |  7 ++-
 tests/test_goal_clock.py                           | 33 ++++++++++++++
 tools/capture_episode.py                           |  4 +-
 9 files changed, 134 insertions(+), 24 deletions(-)
```

Staged diff empty; diff --check PASS. The report and six new Task3 source/test/tool files remain untracked, alongside the six pre-existing Task2 untracked files.
