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
