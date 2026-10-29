# Seed46–48 walking and standing regression

This frozen nine-cell cohort checks the existing group_cohesion_v2/sim_v1 walking candidate and unchanged standing sidestep pipeline with the validated bridge shutdown ordering. plan.json declares all cells and thresholds before execution. No runtime/human/Nav2 source changes. Historical failures remain preserved.

Six walking cells use two initial encounter layouts ×three seeds. Three standing cells use the factory conversation groups with legacy standing motion and sim_v1 clock. Standing is not a test of moving cohesion and does not require direct-goal/cohesion traces. Native observations are not speech/audio or articulated social conversation ground truth.

Walking yaw/cohesion/backwards thresholds are fixed numerical regression limits derived from historical seed45 ORCA/sim_v1, with the same original .5×yaw, radius+.25m, backwards+.05 rules. New seeds do not have matched fresh ORCA baselines; never call passing these limits a measured percentage improvement on seeds46–48. MPPI randomness remains unpinned.

All cells require full capture/source/GPU/sensor/TF/costmap/clock/shutdown audit, successful world goal, no recovery/progress failure, near encounters and positive continuous geometric clearances. Standing additionally independently checks unchanged membership, per-step .2m/s sidestep cap, no displacement outside yielding set and existing yaw bound. Maximum total sidestep/group spread is descriptive; no unpublished limit is added after observing results. Geometry is not contact-force truth.

Run once from ROS Jazzy environment with CPU thread caps1:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 docs/demos/2026-10-29/seed_regression/run_cohort.py
```

No raw run directory or audit is overwritten. Reports/video analysis are read-only and use a fresh output directory. No dataset/default promotion or WM/RL trainer is introduced in this cohort.

## Final measured outcome — runtime382804e

**Cohort FAIL:5 PASS,3 FAIL,1 startup ERROR.** Nine attempts were executed once. Eight native episodes all reached the actual goal without progress failure/recovery, had positive geometric clearances and clean bridge/server shutdown with empty cleanup. Crossing seed48 failed before Gazebo launch. Full capture audits:7 PASS,1 FAIL,1 ERROR. No collection profile is accepted or dataset registered.

| Cell | Full capture audit | Integrated result | Reason |
| --- | --- | --- | --- |
| head_on_s46 |PASS |PASS |All frozen limits pass |
| crossing_s46 |PASS |PASS |All frozen limits pass |
| standing_s46 |PASS |FAIL |Original stationary triplets dissolve/reform |
| head_on_s47 |PASS |PASS |Limits pass; native lifecycle later dissolves groups |
| crossing_s47 |PASS |PASS |Limits pass; native lifecycle later dissolves groups |
| standing_s47 |PASS |FAIL |Original stationary triplets change membership |
| head_on_s48 |PASS |PASS |Limits pass; native lifecycle later dissolves groups |
| crossing_s48 |ERROR |ERROR |ready.json read during non-atomic publication; no native episode |
| standing_s48 |FAIL |FAIL |goal_clock_trace requires at least one goal query; this stationary episode produces zero queries |

A walking PASS is a pass of the published yaw/spread/coverage/physical limits, not a promise that all groups persist throughout. Native member-count plots and transitions are in lifecycle_observations.json/native_group_membership.png. Native fixture constructor defaults can_disperse=True; config retains crowd dispersal.01 and formation.03 per native step. These are intended native lifecycle features, but violate the declared continuous-standing scenario contract. They are not caused by a bridge fault. Designated persistent conversation should be an explicit separate fixture, not global lifecycle disablement.

Original standing_trace failures retain their strict before/after checks for all original IDs. After dissolution, former members may legitimately become dynamic individuals at the existing.8m/s cap. Conditional guard diagnosis confirms actual stationary members keep their.2m/s bound; do not present former-member motion as a safety overspeed. See standing_s46_diagnosis.json and lifecycle_observations.json. Frozen FAIL gates are unchanged.

Standing seed48 keeps both groups and has zero goal evaluations/assignments. The clock audit reconstructs snapshot ages but its final generic requirement evaluations>0 fails. This is a query-coverage/contract limitation on a fully stationary scene, not evidence that simulated age drifted. Original audit remains FAIL, no exemption. A separate standing-aware clock contract is future work.

Crossing seed48 has a JSONDecodeError reading ready.json at nav2_trial.py:184. The worker initially creates it, then rewrites it for motion/encounter/clock metadata. Existence can therefore be visible before complete JSON/metadata. The later complete file does not cure the original runtime ERROR. Native server never launched, worker was cleaned, and no trajectory/events/MCAP exist for this cell. It has no video and is never counted as a native run or navigation failure. Next fix will publish one complete ready payload atomically in a distinct validation cohort.

[Review all available native videos](index.html), [summary](summary.json), [all9 audit receipts](audits.json), [full results](results.json), [execution](execution.json), [lifecycle transitions](lifecycle_observations.json), [source/evidence preservation](validation.json). Raw sensor bags remain ignored in raw_root; code/small reports/recorded media are committed. No AI-generated visual or new Windows GUI observation is claimed.

Subsequent isolated fix: [atomic readiness publication](../readiness_atomic/README.md), runtime2553efe.282 tests OK (2 conditional skips), one fresh crossing_s48_atomic_ready capture full audit and motion/physical screen PASS. Original cohort and crossing_s48 ERROR are unchanged. Standing contract issues remain open.
