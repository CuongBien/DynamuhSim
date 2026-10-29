# Group-level avoidance ablation

The previous sim_v1 runs fixed goal expiry but did not fix spinning. Virtual group circles frequently overlapped while actual human discs stayed separated. This bounded experiment changes only moving-group velocity proposal: native normalized shared goal preference replaces the upper rigid-group-circle ORCA solve. The existing individual-human ORCA and swept collision projection stay intact; native group noise, smoothing, turn/speed limits, goal sampling/lifecycle and simulation clock remain unchanged.

`group_goal_v1` is a custom experiment, not an upstream SocNavGym-native result or a social-planner acceptance. Old group_orca_v1 module bytes remain unchanged. A separate dispatcher validates both old/new frozen assets; omitted flags retain legacy behavior. The new capture profile requires explicit sim_v1 to avoid silently going back to host goal expiry. Default ROS/Nav2/robot sources stay unchanged.

Plan.json is declared before the matched-input preflight and native capture. Two candidate native cells (head-on/crossing, seed45) compare to the previous sim-clock/group-ORCA captures. Baseline reproduction and per-person safety are launch gates. Turning reduction, group cohesion and facing/motion alignment are separate measured screens. MPPI RNG is unpinned; no naturalness acceptance, default promotion or WM/RL training registration is implied.

Shared unit suite270 OK (2 conditional skips). Fixed-input preflight launch gate PASS: baseline worker XY/yaw reproduced exactly; both candidates complete120 steps with positive predicted guard clearance and valid sim-clock trace. Head-on/crossing yaw reduction is42.8%/73.2% approximately; head-on misses the declared50% diagnostic screen and crossing exceeds the declared cohesion-radius increase limit. These motion flags do not change the frozen safety launch gate, and are preserved before the two closed-loop captures.

## Phase1 measured outcome

Both native GPU runs reach the actual world goal in33.02s/32.88s, native capture/clock/direct-goal trace audits PASS, no recoveries, positive swept clearances and near coverage. Exact input and protected per-person guard/native/clock source verification PASS. The upper group proposal change reduces whole-episode yaw rate83.4% head-on and36.6% crossing versus group_orca_v1/sim_v1.

The declared motion screen FAILS both cells: actual max member-centroid radius grows from1.594m to6.746m head-on and1.502m to2.681m crossing (allowed increase .25m). Crossing also misses the50% yaw-rate screen. This is not accepted group walking despite physical safety. All failed behavior outcomes and raw attempts remain untouched; no defaults or WM/RL data registration.

Follow-up is a separate cohesion profile, not a rewrite of these results. Keep the same per-person safety guard and sim clock; add soft regrouping before that guard in a new frozen stage. Artifacts and source here describe only phase1. Unit suite270 OK (2 skips), baseline fixed-ego reproduction exact; native GIF holds/frame counts checked during export.
