# Close-encounter and fixed-gap regression

This bounded stage tests the existing opt-in recipe7/group_orca_v1 without further tuning. Four social cells compare legacy and corrected group motion at seed45 in explicit head-on and crossing initial fixtures. Two static gap cells exercise the same native recipe7 controller in the unchanged80cm and52.7077cm fixtures.

Only human initialization is new: fixed group centers, goals and facing direction. Initial body/wall, body/body and body/virtual-robot clearances are checked before any capture. The native3-step preflight passed both fixtures. Subsequent native goals/lifecycle, ORCA/projection and robot Nav2 control remain active. A per-run encounter.json is hashed and validated; omitted flags retain existing captures. ROS/default robot sources are unchanged.

Plan.json is declared before capture. Near-encounter coverage requires native robot/human center distance<=1.5m for at least0.5s; a fixture title or distant safe pass is not evidence that close avoidance was tested. Report actual initial-intent execution, turning, geometry, native retargeting and all progress recoveries independently. No default promotion or WM/RL dataset registration is implied. MPPI RNG is not pinned.

The native human50s goal timeout still uses host wall time. The worker logs goal ages/targets for these opt-in runs to expose this limitation; no clock or native behavior is silently rewritten. Comparison uses matching initial geometry, not fixed subsequent robot/human traces. After this bounded series, use the observed failures/coverage to decide the next change instead of adding seeds blindly.

Shared unit suite:259 tests OK,2 conditional skips. Meaningful new tests cover fixture safety/heading, initialization without replacing motion hooks, omitted-input behavior and required freeze ownership.

## Recorded results

All six native GPU runs at commit1ca8652 reached the independent world goal, audit PASS, no progress failure and zero action recoveries. Exact matched input/config/initialization verification PASS. All four social runs exercised actual close encounters; all robot/human, human/human and human/static conservative swept clearances are positive. This passes the frozen physical/coverage screen, not human-naturalness acceptance.

| Social case | Sim seconds | Near union duration (<=1.5m) | Min robot/human surface gap | Mean cumulative turns/person, first12s | Recoveries |
| --- | ---: | ---: | ---: | ---: | ---: |
| head_on_legacy | 38.910 | 21.150s | 12.08cm | 1.283 | 0 |
| head_on_group | 36.877 | 35.069s | 12.24cm | 0.957 | 0 |
| crossing_legacy | 36.635 | 29.704s | 12.23cm | 1.300 | 0 |
| crossing_group | 40.205 | 25.413s | 12.34cm | 1.181 | 0 |

The adapter reduces aggregate yaw during first12s25.4% head-on and9.1% crossing. Across full episodes, normalized by elapsed simulation time, **head-on turning rate increases11.1%**, while crossing decreases11.8%. Head-on near duration rises21.15->35.07s; crossing falls29.70->25.41s. These are mixed behavior outcomes. The earlier73% fixed-ego improvement does not generalize to all encounter fixtures. Subsequent ego/human trajectories and goal histories differ; MPPI RNG is unpinned. Do not infer an isolated causal effect or social comfort from these counts.

| Static gap | Sim seconds | Minimum continuous static clearance lower bound | Recovery |
| --- | ---: | ---: | ---: |
| 80.000cm | 29.9525 | 22.009cm | 0 |
| 52.708cm | 30.1525 | 8.286cm | 0 |

Both recipe7 runs crossed the real aperture with the native collision bounding circle and reached the actual goal. Camera/URDF/map/fixture assets are identical to the earlier successful recipe6 gap captures; only native MPPI noise regeneration differs in the controller config. The robot/navigation portion passes this bounded regression. It is still one native attempt per width, not a stochastic robustness guarantee or default promotion.

## Native goal clock diagnostic and next decision

Native human.py accepts geometric goal arrival OR50 seconds of host wall time. New logs show2–4 goal switches per social run near50 wall seconds with at least one member still far beyond the1.8m geometric goal threshold; see goal_clock.json. This is consistent with expiry changing intended direction before geometric arrival and makes simulator-speed comparisons unreliable. The existing goal age also includes pre-start waiting time. This diagnostic does not establish that all turning is caused by timeout; group ORCA/local goal routing remains a separate limitation.

Decision: no promotion of group_orca_v1, no new WM/RL training registration. Next isolate a simulation-time goal clock (starting at the active gate) while retaining current ORCA, native robot/Nav2 and the same encounter fixtures. Then rerun the matched encounter cells and a small predetermined seed cohort. Do not expand simulator scope or tune inflation to address a human target-clock problem.

## Evidence

[Interactive report and six recorded native clips](index.html), [results](results.json), [audit receipts](audits.json), [exact input verification](verification.json), [goal-clock diagnostic](goal_clock.json), [native media metadata/hashes](media_evidence.json). Camera clips are lossy VP8 from recorded native Gazebo observer RGB at1Hz, with every GIF hold duration preserved; encoded4× and viewer defaults to simulation-time1×. No AI image generation or synthetic trajectory was used.

Costmap illustrations use original uint8 global costmaps and nearest recorded map/world pose alignment; actual pose versus costmap offsets are labeled (up to about0.49s). They are asynchronous visual diagnostics; safety/proximity calculations use native world poses independently. Native RGB contact sheet and scientific plots were inspected. No new desktop GUI visibility is claimed for these headless tests.

The physical near-exposure metric solves the relative linear-motion circle interval at1.5m for each person, unions intervals to avoid double-counting, and integrates across native pose samples. Analytic stationary/transit/tangent/overlap test cases pass in validate_analysis.py. No force-contact stream exists. Cumulative yaw measures all changes in either direction; it is not a count of whole pirouettes.

All raw attempts are preserved under raw_root in plan.json. Large raw bags and paired depth remain ignored and require separate backup/restore; Git contains the small reports, source, plots and native video exports. analysis_v1, gap_analysis_v1, media_v1, verification_v1.json and both initial/corrected costmap illustration files are retained. No overwrite or retries occurred.

Reanalyze only into new output paths:

```bash
python3 docs/demos/2026-10-29/encounter_regression/validate_analysis.py
python3 docs/demos/2026-10-29/encounter_regression/analysis.py --output experiments/social_navigation/benchmark/generated/NEW_ENCOUNTER_ANALYSIS
python3 docs/demos/2026-10-29/encounter_regression/verify.py --output experiments/social_navigation/benchmark/generated/NEW_ENCOUNTER_VERIFY.json
python3 docs/demos/2026-10-29/encounter_regression/goal_clock.py --output experiments/social_navigation/benchmark/generated/NEW_CLOCK_DIAGNOSTIC.json
python3 docs/demos/2026-10-29/encounter_regression/media.py --output experiments/social_navigation/benchmark/generated/NEW_ENCOUNTER_MEDIA
```

Use tools/capture_episode.py prepare with --encounter-profile head_on|crossing and --human-motion-profile legacy|group_orca_v1, walking scenario, seed45 and configs/capture/geometry_v7_mppi_resample.json. Fixed gaps use tools/static_gap_episode.py prepare --case wide|narrow --treatment resample. All inputs are frozen before run; no default ROS launch changes.

Final verification: all six frozen input sets remain unchanged. Report JavaScript syntax/DOM-stub smoke checks passed; linked HTTP media/images return200. This is not browser visual QA. The report service was restarted on the same localhost35315 URL. See validation.json.
