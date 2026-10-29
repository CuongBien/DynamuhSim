# Global planner cost diagnosis

Predeclared plan: `plan.json`. One new standing/seed45 capture uses the identical v4/.45 m control configuration with opt-in native costmap observation only. No goal, planner, MPPI, speed, footprint, inflation, or human behavior parameter changes are introduced.

`--planner-diagnostics` records exact uint8 `/global_costmap/costmap_raw` and `/local_costmap/costmap_raw` both in MCAP and a compressed JSONL sidecar. It verifies planner plugin parameters live. Audit compares the sidecar snapshots, including metadata/origin and raw bytes, against the MCAP. Ordinary capture commands retain their existing topics and behavior.

The isolated C++ probe links the installed `nav2_navfn_planner` library. It follows NavfnPlanner's source/target reversal, starting-cell clearing and Dijkstra propagation, with exact endpoints (no tolerance fallback). It does not run MPPI or execute a robot action. Snapshots are asynchronous to live planning, so its path is a diagnostic recomputation rather than an exact replay of a specific planner call.

For each predeclared time offset, compare free planning, two legs through the H5-H6 midpoint, and an offline copy with soft costs 0..252 flattened while preserving 253/254 and unknown255. Never apply flattened costs or midpoint waypoints to the live robot. Geometry ground truth locates the diagnostic waypoint only; the controller does not receive it.

NavFn uses neutral cost50 plus .8 times native cost, and treats native253/254 as forbidden. This explains why available width alone does not determine its choice. Primary references: [NavFn cost conversion](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_navfn_planner/src/navfn.cpp), [planner wrapper and coordinate conversion](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_navfn_planner/src/navfn_planner.cpp).

Build/smoke check:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S experiments/social_navigation/navigation/planner_diagnostics -B experiments/social_navigation/navigation/planner_diagnostics/generated/build
cmake --build experiments/social_navigation/navigation/planner_diagnostics/generated/build --parallel 1
python3 experiments/social_navigation/navigation/planner_diagnostics/validate_probe.py
```

Capture uses a NEW run directory with `tools/capture_episode.py prepare --config configs/capture/geometry_v4_inflation045.json --scenario standing --seed 45 --planner-diagnostics`. Existing runs refuse overwrite. Policy: `configs/audit/capture_native_geometry.json`.

## Measured outcome

Run D completed on native RTX4060 rendering: goal SUCCEEDED, actual world XY error .05823m, active31.355s, path9.15186m. Conservative actual swept human gap .34361m; static box clearance lower bound .92874m. No force-contact stream exists. Native YAML and camera SDF match C byte for byte. Audit PASS, including all33 global and152 local native sidecar snapshots matched against MCAP.

The unconstrained native paths closely match recorded /plan (descriptive maximum nearest-point distances: 0.000000353m, .05648m, .01275m; asynchronous snapshots). At the three predeclared offsets the gate-center native costs are0,117,173. None is a hard253/254 barrier. Yet the returned route via the midpoint has potential sums1.011x,1.083x,1.549x the free route, and is longer from the robot's current position. Flattening soft costs still selects an upper path, so inflation is not the only explanation; the robot has already moved above the groups.

**Offline reachability is not physical acceptance.** Every via-midpoint path overlaps H4's snapshot collision proxy. At early times H4 is beyond the3m obstacle-marking range; at the last snapshot its cylinder is fully angularly occluded by H5 while its costmap-center cell stays0. These are geometric/observability diagnostics from recorded poses, not a detector output. The free global path at the earliest snapshot also overlaps still-unobserved H5; live navigation replans as sensing progresses. Do not interpret a one-time global path as an executed trajectory or a safe future rollout.

The result supports investigating global route choice rather than attributing the missed gap solely to MPPI. No MPPI rollout/critic score breakdown was captured, and the experiment does not prove MPPI can execute the narrow passage. Suggested next isolated test: two fixed obstacles, start facing the gap, and separately measure planner path and actual controller clearance. Do not make a conversation group the pass/fail target for geometry tuning.

Native cost conversion and wrapper behavior are documented in the primary Nav2 links above. Runtime commit `9a7005c`. Tests241, OK,2 conditional skips; the first validation caught an old-v1 fixture without plan.json, which was corrected before capture. Earlier analysis artifacts were preserved under the ignored run parent, not substituted into raw evidence.

Recompute into NEW directories after restoring the raw run:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 docs/demos/2026-10-29/planner_review/analyze.py --output-root experiments/social_navigation/benchmark/generated/NEW_PROBE --report-dir experiments/social_navigation/benchmark/generated/NEW_PROBE_REPORT
```

`verify.py` saves immutable verification.json and refuses overwrite. Reports record the installed native package version, library/probe hashes, source raw paths and byte-level capture provenance. `physical_run.json` is generated with the existing geometry-ab read-only pose comparison helper.

Camera encoding is reproducible with `python3 docs/demos/2026-10-29/planner_review/render_camera.py --output NEW.webm`; it keeps the recorded source-frame sequence and uses lossy VP8, refusing to replace existing media.
