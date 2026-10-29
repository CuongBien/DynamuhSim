# Human goal expiry on simulation time

This isolated profile replaces only the human elapsed-goal-time predicate with Gazebo simulation time. Initial goals start at the accepted navigation goal timestamp written into runtime/start_gate. Later native set_goal/set assignments reset the deadline at their native step. Native geometric arrival and strict age>50s are retained, as are all native setter side effects/RNG. The existing native_wall path remains the default; vendor files and ROS robot/default launch are unchanged.

The adapter binds the existing six Human instances. Native membership/type changes may move those objects between lists/groups. Object replacement, untracked direct goal mutation, gate changes or clock discontinuities are terminal rather than silently reverting to wall time. A run-local human_goal_clock.json is frozen and audited. Logs retain native wall-age telemetry for diagnosis only; sim_v1 behavior uses the separate goal_clock trace.

Before capture:266 shared unit tests OK (2 conditional skips). Tests exercise exact deadline boundary, geometric/default/explicit offsets, static behavior, original setter side effects and same-target resets, pre-start waiting, freeze ownership and altered-clock-trace detection. The native preflight runs identical measured ego inputs under two deliberately different synthetic wall clocks; this is component determinism verification only.

Plan.json declares four fresh GPU captures with the existing head-on/crossing fixtures and legacy/group_orca_v1 human motion, all using the unchanged recipe7 controller and seed45. Historical native_wall captures from encounter_regression_20261029 remain the reference and are never rewritten. Subsequent live trajectories differ; MPPI RNG is not pinned. No WM/RL training or default promotion is part of this step.

## Native capture results and acceptance decision

Runtime commit732bd47. Four fresh native RTX4060 GPU captures reached the actual world goal, native capture audit PASS, clock-trace reconstruction PASS, exact input/treatment verification PASS. Every run had zero controller progress failures and zero action recoveries; all exercised near encounters with positive conservative robot/human, human/human and human/static clearance.

| Case | Active sim seconds | Min robot/human surface gap | Mean absolute turns/person | Turn-rate change vs wall-clock reference | Goal assignments | Expired checks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| head_on_legacy_sim | 36.0150 | 12.51cm | 6.314 | +213.2% | 3 | 0 |
| head_on_group_sim | 33.7075 | 12.54cm | 5.050 | +140.8% | 3 | 0 |
| crossing_legacy_sim | 36.7225 | 24.85cm | 6.123 | +69.5% | 0 | 0 |
| crossing_group_sim | 33.2575 | 12.23cm | 4.827 | +67.2% | 0 | 0 |

The goal-clock defect is repaired in the opt-in profile, but **human motion is not accepted**: turning worsens in all four runs. Mean rates increase213.2%/140.8% head-on (legacy/group), and69.5%/67.2% crossing. These are recorded comparisons with unpinned MPPI and different subsequent trajectories, not a sole-cause estimate. No defaults are promoted and no new WM/RL dataset is registered. Both negative and positive outcomes are retained.

All runs end before50 active simulation seconds. Clock trace reconstruction verifies that initial ages start at the accepted goal and later ages reset at native setter calls. Native wall-age>50 while sim-age<=50 occurs1005/789/1344/1152 human-snapshot observations; these do not trigger expiry. Zero expired checks in this bounded cohort is expected, not evidence of disabling the timeout: the strict50.0 versus50.1 boundary is explicitly unit-tested. Goal assignments are individual-human calls (three calls for one three-person group retarget), not a group target count. The clock audit also covers the recorded label tail.

The paired native-kernel preflight passes all four cells: fast/slow synthetic host schedules produce exactly identical human XY/yaw over60 measured-ego input steps (6s). This establishes the tested component's host-time independence within that bounded window. It does not establish long-horizon determinism, multi-seed generalization, closed-loop motion quality or physics acceptance.

## Additional group-motion diagnostic

The new group_orca_v1 head-on/crossing runs have virtual group-circle overlap in61.7%/69.4% of paired group samples, versus28.2%/38.6% in their wall-clock references. Virtual group radii grow from1.08m initially to1.95m/1.86m; actual sampled member disc gaps remain positive (about.428m/.042m). The native continuous world-pose safety screen is separate and also PASS. These virtual circles describe group-level planning envelopes, not body contacts.

With an unchanged group target, raw ORCA group intent flips by more than90 degrees11 times total in the new head-on run and7 times in crossing. See group_diagnostic.json for eligible samples and reference counts. This additional diagnostic motivates testing the group-level avoidance/envelope abstraction while retaining per-person ORCA and safety projection. It is not a new frozen acceptance gate or proof of one unique cause of all rotations.

Decision: retain sim_v1 as an audited clock experiment; do not call the walking behavior natural or training-ready. Next test one isolated group-motion change that addresses the virtual group avoidance layer, with the same simulation clock, native robot/Nav2, fixtures and per-person safety guard. Do not shorten the timeout or reintroduce wall-clock retargeting to hide persistent motion problems.

## Evidence and reproduction

[Review page, four native clips and figures](index.html), [results](results.json), [clock/native capture audits](audits.json), [exact input verification](verification.json), [component preflight](preflight_results.json), [group diagnostic](group_diagnostic.json), [native media hashes](media_evidence.json). Source/native videos are recorded Gazebo observer RGB (1Hz), not AI-generated. Encoded clips preserve GIF hold durations; viewer defaults to simulation-time1x via playbackRate.25. Pose-based replay skeletal animation is a separate reconstructed display mode.

The clock figure plots the maximum age across six humans; a reset may not change that maximum if another group keeps its old target. Brown and green ages use different declared clock domains. Costmap display uses the original uint8 grids and nearest map/world pose/TF with offsets labeled; physical acceptance uses native world poses independently. Raw captures and component traces remain in raw_root from plan.json, untouched and ignored in Git; small reports/plots/native exports are committed. Historical reference captures remain untouched.

```bash
# Prepare a NEW run, then run with ROS/install environment sourced.
python3 tools/capture_episode.py prepare --run-dir experiments/social_navigation/benchmark/generated/NEW_CLOCK_RUN --config configs/capture/geometry_v7_mppi_resample.json --scenario walking --seed 45 --planner-diagnostics --encounter-profile head_on --human-motion-profile group_orca_v1 --human-goal-clock sim_v1
python3 tools/capture_episode.py run --run-dir experiments/social_navigation/benchmark/generated/NEW_CLOCK_RUN

# Read-only analysis; always choose fresh output paths.
python3 docs/demos/2026-10-29/sim_goal_clock/analysis.py --output experiments/social_navigation/benchmark/generated/NEW_CLOCK_ANALYSIS
python3 docs/demos/2026-10-29/sim_goal_clock/verify.py --output experiments/social_navigation/benchmark/generated/NEW_CLOCK_VERIFY.json
python3 docs/demos/2026-10-29/sim_goal_clock/group_diagnostic.py --output experiments/social_navigation/benchmark/generated/NEW_GROUP_DIAGNOSTIC.json
```

Full unit suite266 OK (2 conditional skips), final seven focused clock tests OK. Every native run has independent numeric TF/raw sensor/costmap and clock-trace audits; all frozen input sets are verified unchanged at the end. No new Windows GUI visibility or browser visual QA is claimed for headless captures.

Final report checks: HTTP assets200 and JS syntax/DOM-stub smoke PASS (four clock/result cards and clips, default simulation-time playback, failed-behavior warning). This does not verify browser visual rendering or Windows GUI. See validation.json.
