# Close-encounter and fixed-gap regression

This bounded stage tests the existing opt-in recipe7/group_orca_v1 without further tuning. Four social cells compare legacy and corrected group motion at seed45 in explicit head-on and crossing initial fixtures. Two static gap cells exercise the same native recipe7 controller in the unchanged80cm and52.7077cm fixtures.

Only human initialization is new: fixed group centers, goals and facing direction. Initial body/wall, body/body and body/virtual-robot clearances are checked before any capture. The native3-step preflight passed both fixtures. Subsequent native goals/lifecycle, ORCA/projection and robot Nav2 control remain active. A per-run encounter.json is hashed and validated; omitted flags retain existing captures. ROS/default robot sources are unchanged.

Plan.json is declared before capture. Near-encounter coverage requires native robot/human center distance<=1.5m for at least0.5s; a fixture title or distant safe pass is not evidence that close avoidance was tested. Report actual initial-intent execution, turning, geometry, native retargeting and all progress recoveries independently. No default promotion or WM/RL dataset registration is implied. MPPI RNG is not pinned.

The native human50s goal timeout still uses host wall time. The worker logs goal ages/targets for these opt-in runs to expose this limitation; no clock or native behavior is silently rewritten. Comparison uses matching initial geometry, not fixed subsequent robot/human traces. After this bounded series, use the observed failures/coverage to decide the next change instead of adding seeds blindly.

Shared unit suite:259 tests OK,2 conditional skips. Meaningful new tests cover fixture safety/heading, initialization without replacing motion hooks, omitted-input behavior and required freeze ownership.
