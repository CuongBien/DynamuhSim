# Isolated static bottleneck test

## Current accepted isolated demo

Use `--treatment long_horizon` for the separately versioned recipe6: CostCritic weight3.81, near-collision threshold253, time_steps100, model_dt .05s, all existing collision/map/speed settings retained. Default capture and ROS/Nav2 configurations are unchanged.

```bash
python3 tools/static_gap_episode.py prepare --run-dir experiments/social_navigation/benchmark/generated/NEW_HORIZON_DEMO --case narrow --treatment long_horizon
python3 tools/static_gap_episode.py run --run-dir experiments/social_navigation/benchmark/generated/NEW_HORIZON_DEMO
```

Results: wide80 succeeded43.6875s, path8.06858m, XYerror.05050m, minimum conservative continuous gap .18414m. Narrow52 succeeded46.48s, path8.07399m, XYerror.05653m, minimum gap .06548m. Both crossed gate x=-24 with their entire native enclosing circle fitting, and both passed integrity audit. Full acceptance: `horizon5/results.json`.

`comparison.json` retains all six results: original and CostCritic-only treatments failed both widths, horizon5s treatment passed both. `verification.json` confirms exact same maps, fixture/camera/URDF/world bytes per width; only the two critic values changed in treatment2 and only time_steps changed from treatment2 to3. Collision_cost, costmap inflation/scale, radius/padding and speed caps remain unchanged. Native parameters time_steps100 and model_dt .05 are verified live.

Each width/treatment has one attempt; MPPI RNG was not fixed (the static plan seed0 is only a placeholder and does not seed MPPI). The result is not a multi-seed benchmark, a single-parameter causal proof against the original joint treatment, contact-force truth or social-navigation acceptance. Next: regression in the factory with people before promoting any defaults.

Media are recorded native RGB frames; render_camera.py re-encodes the original GIF sequence lossily to VP8 with frame-count verification and refuses overwrite. Viewer GUI, when opened, replays recorded XY/yaw without physics/Nav2.


The two predeclared cases in plan.json are an .80m wide control and a .52707658m opening matching the original H5-H6 surface gap. Two fixed barrier slabs meet the perimeter walls, so there is no outer detour. Native robot collision geometry, aligned URDF/TF, NavFn/Dijkstra, MPPI Omni, radius .17m + padding .01m, inflation .45m and speed limits are unchanged.

This is a separate no-human scene with a matching generated static map. Ordinary warehouse/social launch defaults remain unchanged. The launch accepts a run-local map only through the isolated capture context. The recorder's six-person readiness contract remains the default; this runner explicitly requires the zero-human topic subset. A separate audit policy selects the static-gap schema and zero scene humans, retaining the sensor/TF/native-costmap integrity checks.

The original robot start and goal stay (-28,0,0) and (-20,0,0). No waypoint or velocity command forces passage. Success requires actual arrival, recorded left-to-right gate crossing and a positive conservative swept-circle clearance lower bound against every fixture box. This checks all robot yaw angles using a .168062484748657m native collision enclosing radius. The positive margin is a geometric certificate for the recorded linearly interpolated pose segments, not native contact-force evidence.

Each attempt is preserved, with no automatic retry or parameter adjustment. Test failures remain results. This does not establish safe passage through moving conversation groups or qualify WM/RL data.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
python3 tools/static_gap_episode.py prepare --run-dir experiments/social_navigation/benchmark/generated/NEW_STATIC_GAP --case narrow
python3 tools/static_gap_episode.py run --run-dir experiments/social_navigation/benchmark/generated/NEW_STATIC_GAP
python3 tools/audit_episode.py --run-dir experiments/social_navigation/benchmark/generated/NEW_STATIC_GAP --source-root /home/cuongbien/DynamuhSim --policy configs/audit/static_gap.json --output experiments/social_navigation/benchmark/generated/NEW_STATIC_GAP_AUDIT.json
```

Use NEW paths; existing runs and audit outputs refuse overwrite.

## Original treatment: measured failure retained

Both baseline runs TIMEOUT at90s despite global plans passing through the opening. Final actual x is -24.572m (wide) and -24.702m (narrow), before gate x=-24. All sensor/TF/native-costmap capture audits PASS. They are navigation failures, not missing-data failures.

In the wide local costmap, samples .4m ahead of the stopped robot have native cost149. The configured CostCritic near_collision_cost140 treats those traversable cells as near-collision and adds critical_cost300 per evaluated point before averaging/weighting. CostCritic weight16 is also much higher than the Jazzy source default3.81; source default near_collision_cost is253. The rollout critic breakdown itself was not recorded, so this is an inspected-configuration/source diagnosis, not a claim to have logged each critic's score.

Primary source: [Jazzy CostCritic](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_mppi_controller/src/critics/cost_critic.cpp). Subsequent treatments, if run, will be separately declared and captured, never replacing these failures.

## Native-cost-only treatment: measured failure retained

Both `wide80_cost` and `narrow52_cost` also TIMEOUT at90s with the declared3.81/253 live parameters verified. Changing CostCritic alone was insufficient; this falsifies a simple two-field fix claim. The prediction horizon is still30 steps x .05s=1.5s, at most .45m at the velocity cap, shorter than the .4m slab plus inflation approach/exit regions (.45m on each side).

A separate horizon treatment may therefore be compared with the native-cost treatment by changing only time_steps to100 (.05s model_dt unchanged). Five seconds permits a kinematic upper bound1.5m, covering that1.3m cost region; acceleration limits reduce the achievable distance somewhat. This is a testable hypothesis, not a logged native critic-score explanation. No velocities or waypoint commands are injected.

Camera re-encoding preserves the original GIF hold durations: the wide camera has44 original timestamps but42 stored GIF frames (a750ms merged hold). The VP8 file expands the hold to44 frames/11s at4fps; narrow has47 frames/11.75s. Original first encodings are retained under the ignored media_before_duration_fix directory. See media_evidence.json for timing and hashes.
