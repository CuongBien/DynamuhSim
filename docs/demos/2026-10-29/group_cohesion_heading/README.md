# Heading-consistent soft cohesion

Phase1 group_goal_v1 reduced spinning but lost group cohesion. Soft-cohesion v1 kept member radius near1.15m in component tests but increased turning, so its declared native cells were not run; that source and every result remain preserved. This new version retains the same five cohesion constants and repairs how the desired velocity is mapped to bounded body motion.

When regrouping is active, the desired vector uses native smoothed shared-goal direction and individual native step-speed magnitude plus soft attraction. Body yaw advances at the existing bound; translation is braked by the cosine of heading error, allowing turn-before-walk for large reversals. Below.05m/s desired speed, hold the proposal. Native/no-force compact proposals are unchanged. Native integration/RNG is called once; all adjusted proposals still pass through the original per-person ORCA and swept projection. No post-guard position/yaw correction or teleport is performed.

Two fresh native cells, constants, controller semantics and original ORCA-reference motion criteria are declared before testing in plan.json. Fixed-ego component control uses the captured phase1 group_goal_v1 traces. This is custom SocNavGym behavior; no default or WM/RL dataset promotion. Preserve every outcome and do not tune the frozen constants during this series.

276 shared tests OK (2 conditional skips). Component launch gate PASS: exact phase1 control reproduction,120 candidate steps per scene, positive guard clearances, valid simulation clock and heading/cohesion proposal reconstruction. Candidate max member-centroid radii1.170m/1.135m over this bounded counterfactual; not a native or long-horizon acceptance.

## Measured native result

Runtime18adcee, two fresh native RTX4060 captures. Both actual robot goals SUCCEEDED with no progress failure/action recovery, positive robot/human,human/human and human/static continuous geometric clearance, and exercised near encounters. Declared yaw/cohesion/backwards motion screens PASS in both scenes. Full integrated acceptance is **1/2 PASS**, because crossing has a bridge abort during teardown; do not merge the two status levels.

| Scene | Active sim time | Yaw-rate reduction vs original group ORCA | Max native member-centroid radius | Min robot/human surface gap | Motion screen | Full audit |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| head_on | 33.150s | 78.0% | 1.225m | 12.11cm | PASS | PASS |
| crossing | 36.235s | 87.8% | 1.142m | 38.23cm | PASS | FAIL |

Original group_orca_v1/sim_v1 max radii1.594m/1.502m and yaw rates.1498/.1451 turns per second are the fixed motion references. Goal-only phase1 is shown separately: radius grew6.75m/2.68m despite reduced yaw. The new model keeps radii1.225m/1.142m with mean cumulative yaw about1.094/.640 turns per person. Actual native human travel is roughly14–17m head-on and11–14m crossing, so the reduction is not achieved by freezing people or cosmetic video/yaw edits. Per-person ORCA/swept projection and native camera/robot/Nav2/clock sources remain matched.

Near duration increases by8.21s in head-on and decreases1.87s in crossing relative to the original sim-clock/group-ORCA captures. Thus lower turning and tighter groups are not proof of overall social comfort or universally better avoidance. Human poses, per-person motion and safety remain descriptive on seed45; MPPI randomness is not pinned.

## Capture failure retained

Crossing full audit FAILS only no_child_exit_errors. After the goal and3.3s label tail were complete and recorded, ROS launch sent SIGINT to its children. nav2_pilot_ground_truth_bridge (parameter_bridge-9) emitted a fatal glibc pthread_mutex_lock owner assertion and aborted with exit-6. Other raw depth/TF/native costmap/source/GPU/goal-clock/direct-goal/heading-cohesion reconstruction checks PASS, recorder closed and scoped cleanup is empty, but the original audit and verification remain FAIL. No policy exception or retry was used. See teardown_failure.json, audits.json and verification_summary.json. This evidence does not establish the root cause of the bridge's shutdown race or that the behavior model caused it.

Decision: retain group_cohesion_v2 as the current behavior candidate; do not promote defaults or register new WM/RL training data. Next isolate and fix/verify bridge shutdown, then run a small predetermined seed cohort and standing-group regression before freezing a new data-collection profile. Do not loosen audits or overwrite this failed capture to make the series appear accepted.

## Preserved development outcomes

1. group_goal_v1: two native runs reached the physical goal, but both failed motion quality (cohesion loss; crossing also yaw screen). Code/report/videos remain under group_goal_ablation.
2. group_cohesion_v1: four component executions (control/candidate per scene) retained; radius improved but turning worsened. The two declared native cells are explicitly NOT_RUN_AFTER_COMPONENT_REVIEW in group_cohesion/native_disposition.json. They are not silently counted as passes or failures of native physics.
3. group_cohesion_v2:276 shared unit tests OK (2 conditional skips), exact phase1 component reproduction and positive guard/clock/proposal checks; two native runs with the outcomes above. All sources/constants were committed before capture. No retry, source tuning or historical raw edit occurred during native execution.

## Run current candidate

This profile is for walking-group experiments; standing conversation keeps the existing pipeline. Use a new run directory each time:

```bash
cd /home/cuongbien/DynamuhSim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/capture_episode.py prepare --run-dir experiments/social_navigation/benchmark/generated/NEW_GROUP_RUN --config configs/capture/geometry_v7_mppi_resample.json --scenario walking --seed 45 --planner-diagnostics --encounter-profile head_on --human-motion-profile group_cohesion_v2 --human-goal-clock sim_v1
python3 tools/capture_episode.py run --run-dir experiments/social_navigation/benchmark/generated/NEW_GROUP_RUN
```

[Review page/two recorded clips](index.html), [results](results.json), [audits](audits.json), [input verification](verification.json), [teardown failure](teardown_failure.json), [component preflight](preflight_results.json), [media hashes](media_evidence.json). RGB clips are recorded native Gazebo1Hz, exported with original GIF hold duration and decoded frame count verified; default playback restores simulation-time1x. No AI image generation. Gait reconstructed in the separate pose-replay tool is not logged skeleton playback.

Native scientific yaw/cohesion plots and RGB contact sheet were visually inspected. Costmap display uses original native uint8 grids with nearest pose/TF alignment, labeled offsets; physical geometry is calculated independently from world poses. Raw bags/paired depth/worker traces remain untouched under raw_root in plan.json and are ignored by Git; small source/report/video/plot artifacts are committed. Analysis/media entrypoints refuse to overwrite outputs. No new Windows GUI or browser-visual QA confirmation is claimed.

Final verification: both frozen input sets unchanged. Report JS syntax/DOM-stub smoke and HTTP asset checks PASS, preserving crossing integrated FAIL distinctly from motion PASS. This is not browser visual QA. See validation.json.
