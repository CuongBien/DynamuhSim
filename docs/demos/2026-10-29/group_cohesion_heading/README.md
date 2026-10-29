# Heading-consistent soft cohesion

Phase1 group_goal_v1 reduced spinning but lost group cohesion. Soft-cohesion v1 kept member radius near1.15m in component tests but increased turning, so its declared native cells were not run; that source and every result remain preserved. This new version retains the same five cohesion constants and repairs how the desired velocity is mapped to bounded body motion.

When regrouping is active, the desired vector uses native smoothed shared-goal direction and individual native step-speed magnitude plus soft attraction. Body yaw advances at the existing bound; translation is braked by the cosine of heading error, allowing turn-before-walk for large reversals. Below.05m/s desired speed, hold the proposal. Native/no-force compact proposals are unchanged. Native integration/RNG is called once; all adjusted proposals still pass through the original per-person ORCA and swept projection. No post-guard position/yaw correction or teleport is performed.

Two fresh native cells, constants, controller semantics and original ORCA-reference motion criteria are declared before testing in plan.json. Fixed-ego component control uses the captured phase1 group_goal_v1 traces. This is custom SocNavGym behavior; no default or WM/RL dataset promotion. Preserve every outcome and do not tune the frozen constants during this series.

276 shared tests OK (2 conditional skips). Component launch gate PASS: exact phase1 control reproduction,120 candidate steps per scene, positive guard clearances, valid simulation clock and heading/cohesion proposal reconstruction. Candidate max member-centroid radii1.170m/1.135m over this bounded counterfactual; not a native or long-horizon acceptance.
