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
