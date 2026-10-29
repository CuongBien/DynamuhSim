# Endpoint convergence and group spinning experiments

The recorded walking_h5 native poses contain about 9.8-10.1 cumulative full turns per person in 51s, with only 8-11m of travel. This is actual simulated motion, not just reconstructed gait or an invented video. A bounded replay of the first 120 measured robot inputs reproduces the behavior.

The opt-in `group_orca_v1` human adapter uses the actual enclosing radius of group members (initially 1.08m instead of the vendored 1.44m) and initializes each fresh RVO group agent with measured mean member velocity. It preserves native goals/lifecycle, turn/speed limits, individual ORCA and the existing collision projection. This is a customized SocNavGym experiment, not an upstream-native result. `legacy` remains the capture default. The human planner's virtual robot radius .25m is retained.

Separately, recipe7 changes only MPPI `regenerate_noises` from false to true relative to recipe6. The earlier five-second controller completed after an optimizer reset, which motivates resampling as a hypothesis; it does not prove noise reuse caused the endpoint failure. No goal/progress tolerances are loosened. The Jazzy implementation generates noise once/reset when this parameter is false, and regenerates through a worker thread when true: https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_mppi_controller/src/noise_generator.cpp

`plan.json` is declared before capture. Robot and human treatments are evaluated in separate cells. MPPI RNG is not pinned; seed45 and one run per cell cannot establish robustness. No default promotion or WM/RL dataset acceptance is implied. Every attempt is retained.

Shared unit verification before captures: 255 tests, OK, 2 conditional skips. The targeted tests check the native controller diff, live contract, measured group envelope/velocity, native solo/obstacle branches, default legacy behavior and profile freeze ownership.

Run bounded offline probe using the existing SocNavGym venv:

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 experiments/social_navigation/.venv-socnavgym/bin/python docs/demos/2026-10-29/endpoint_humans/offline.py --output experiments/social_navigation/benchmark/generated/NEW_HUMAN_OFFLINE
```

Native human goal expiry also uses host wall time. Keep the matched input test bounded to12 simulated seconds; longer independent replays may have different goal-expiry histories. Closed-loop captures remain necessary for clearance and navigation outcomes.

## Measured outcome

Four fresh native RTX4060 GPU captures at runtime commit `8c158f7`; all four audit PASS, exact initialization/treatment verification PASS, actual goal reached and no progress failure/action recovery. Goal tolerances and progress checker were retained. The prior recipe6 standing/walking runs required one recovery each and47.63s/51.07s; recipe7 legacy runs finish in32.05s/34.09s without recovery. This supports the tested treatment on seed45; it is not proof of a unique optimizer cause or multi-seed acceptance.

| Case | Active sim seconds | Min swept robot/human gap | Min robot/static lower bound | Min human/static lower bound | Recoveries |
| --- | ---: | ---: | ---: | ---: | ---: |
| standing_resample | 32.0550 | 0.3296m | 0.9055m | 1.6165m | 0 |
| walking_resample | 34.0900 | 0.1281m | 1.0563m | 0.6090m | 0 |
| walking_group | 30.6975 | 1.7009m | 1.6817m | 0.2089m | 0 |
| standing_group | 31.5325 | 0.3314m | 0.9053m | 1.6165m | 0 |

Both standing cases: no actual human displacement or rotation; zero robot-center entries into standing-trio triangles (3204/3154 tests). Walking has no standing-group diagnostic. Triangle-center checks do not establish full social-space compliance. Clearances use actual native poses and conservative disc/segment calculations; no force-contact stream is present.

The fixed-ego12s probe reproduces every old human pose exactly (RMSE0); group adapter reduces aggregate absolute yaw73.04%. Paths increase from2.60–3.40m to5.84–6.87m; maximum group-centroid/member distance decreases1.271m->1.115m. This is a small, bounded counterfactual with identical ego inputs, not a closed-loop interaction acceptance.

Closed-loop native Gazebo pose cross-check: during first12s the legacy humans accumulate2.03–2.15 turns each; the new humans0.47–0.87 turns. Aggregate yaw is68.3% lower. Across complete runs, legacy5.30–5.52 turns/34.08s versus adapter1.73–2.08 turns/30.68s; normalizing by elapsed simulation time gives60.9% lower aggregate turning rate. The adapter still has large turns, including a direction change near22–27s: behavior realism is not fully resolved. Native camera clips must be reviewed alongside these scalar measures.

The new walking groups move ahead and clear the robot route. Their1.70m minimum robot/human gap is not evidence of better close-range avoidance; the two controllers see different subsequent human trajectories. Reciprocal encounters, crossing, tight passages, group goal progression and multiple seeds remain unaccepted. Native host-wall-time goal expiry is another reproducibility limitation. Do not label this outcome as human-naturalness acceptance or register it as a new WM/RL training dataset.

Decision: retain both opt-in treatments for further tests; do not promote defaults. Next bounded series should test head-on/crossing group encounters and recipe7 on both existing static gap fixtures, then multiple seeds. Dataset01 remains unchanged and reflects its original geometry/behavior; no retrospective relabeling.

## Evidence and replay

[Open report](index.html), [metrics](metrics.json), [capture audits](audits.json), [input verification](verification.json), [native media hashes](media_evidence.json). Four WebM clips in this folder are lossy encodes of recorded native observer RGB, with merged-frame hold duration preserved. The report defaults to simulation-time1× playback; camera samples are1Hz. The2D comparison uses logged human XY/yaw from the bounded kernel execution. No AI image generation was used.

Raw captures and all attempts are under the `raw_root` in plan.json; `offline_v1`, preliminary probes, `analysis_v1` and the extended `analysis_v2` are all preserved. The ignored raw bags are not included by `git clone`. The small reports, plots and native video exports are committed. `media.py --output NEW` and `analyze.py --output NEW` refuse to overwrite; `verify.py --output NEW.json` verifies exact initial conditions and expected controller differences. Audit invocation uses the canonical `tools/audit_episode.py` with `configs/audit/capture_native_geometry.json` and a new JSON output path.

Final targeted group/resampling checks:5 tests OK after preserving the native delayed-prediction arithmetic. The broader255-test suite passed before captures (2 conditional skips). Browser-script verification is reported separately from Windows GUI visibility; no new GUI interaction confirmation is claimed for these headless captures.

Report JavaScript syntax and DOM-stub smoke checks passed (slider12s, four results/video cards, simulation-time playback rate). HTTP assets return200. Browser visual QA could not run because the computer-use helper failed at startup; no screenshot of the report or new Windows GUI operation is claimed. See validation.json. The offline `reference_pose_rmse` mixes XY/yaw coordinates solely to check exact legacy reproduction; its adapter value is not a physical quality score.
