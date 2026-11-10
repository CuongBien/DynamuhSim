# Human navigation scenario matrix

The six catalogs in `configs/benchmarks/scenarios/` record the source [Scenario Cards](../research/HUMAN_MODEL_FINAL.md). The catalogs and source hash are retained for traceability. [The TASK 08B bindings](../../configs/benchmarks/bindings.yaml) resolve their execution choices. The [deviation ledger](../research/HUMAN_MODEL_DEVIATIONS.md) records the decisions against the source.

## TASK 08B research lock

All positions, goals and probes are in `world` XY in metres. Spawn yaw is in radians. The YAML defines each spawn and destination explicitly. Human reference paths are straight segments from spawn to goal. The robot follows an open-loop schedule of body-frame `[vx, vy, omega]` in m/s and rad/s at integer nanosecond offsets from reset. Each command holds until the next entry. Collision or goal terminates the episode, with a 30 s limit. Advance has a finite drive interval followed by a zero command.

| Card | Effective width Easy / Critical / Hard (m) | Geometry and routes | Density and policy |
| --- | --- | --- | --- |
| FA · Frontal Approach | 2.1 / 1.2 / 0.9 | 12 m straight corridor. Robot travels +X on the south lane, opposite people travelling −X. | 1 / 1 / 2 neutral. Hard adds a staggered second person. |
| ND · Narrow Doorway | 1.2 / 0.9 / 0.6 | Two rooms span X = −5 to +5 m and have interior widths of 4.8 / 4.5 / 4.2 m. A 0.25 m thick divider at X = 0 has one opening centred at (0, 0). Opposed centreline routes cross the threshold. | 1 / 1 / 2 neutral. Hard adds a second person in file. At 0.6 m the largest disc has 0.10 m of individual slack and the robot–human pair has −0.26 m side-by-side slack. |
| BC · Blind Corner | 2.1 / 1.2 / 0.9 | 10 m horizontal leg turning north, with a 4 m extension beyond the inner wall. The existing recess is upstream of the encounter. Robot moves +X for 7 s, then +Y at fixed chassis yaw. The person approaches southbound on the inside lane and ends within the bend. The encounter centre is the intersection of the two approach lines. | 1 neutral at every level. Width checked independently on each leg. |
| IN · Intersection | 2.1 / 1.2 / 0.9 | Plus-shaped cross at (0, 0), with arm ends 4.75 / 4.30 / 4.15 m from the centre. Robot traverses west to east and human south to north. Width checked independently on both axes. Nominal arrival is distance to the centre divided by 0.50 m/s for the robot and 0.80 m/s for the person. | 1 neutral at every level. |
| OT · Overtaking | 2.1 / 1.6 / 0.9 | 16 m straight corridor. Both start facing and travelling +X. Robot uses the south lane at 0.55 m/s. The north-lane pedestrian starts behind at preferred speed 1.10 m/s, or ahead at 0.40 m/s. | 1 neutral. Even seed indices select pedestrian overtaking and odd indices select robot overtaking. Each backend has 1/1 at Easy in Vòng 0, 3/2 per level in Vòng 1 and 10/10 at Critical in Vòng 2. |
| MG · Small Group | 2.1 / 1.8 / 1.6 | 14 m enclosed aisle. A side-by-side pair moves +X ahead of the robot, with staggered counterflow along −X. At least three start within 4 m of the robot. | 3 / 5 / 7 independent. The paired initial layout represents spacing only. Cohesion requires a separate policy condition. |

The robot and human footprint radii are 0.18 and 0.25 m. A side-by-side margin measures disc geometry. At 0.90 m it is +0.04 m, so OT Hard is a near-fit condition with little passing clearance. For ND Critical the study measures competition and yielding at the opening with the same +0.04 m geometric slack. Clearance for a single actor and swept reference routes are checked separately. Contact, guard events and failures to pass remain experimental outcomes.

All four native backends receive the same scene and nominal preferred speed of 0.80 m/s, except OT's two speed profiles. Each backend retains its own interaction parameters. Neutral and independent designate the native behavior condition. Cooperative, non-cooperative, cohesive, sensor-triggered and feedback-based action variants are outside the locked allocation. BC line of sight is an offline geometric label from the compiled polygon. The approach thresholds remain 2 m for the Card label and 1.5 m as the source's descriptive target for Critical. Sensors are disabled and the native models still use their existing actor context.

Each seed offsets all human spawns together by up to 0.15 m along their approach axis, or 0.12 m in BC. It uses the episode's `spawn/global` stream and keeps headings, goals, pair spacing and robot spawn fixed. Pre-resize offsets, stream seed and RNG type are recorded with the attempt. Phase and backend selections share these draws. Subtype selection uses the seed index, so the original OT slots keep their condition when later phases are added.

### Branch B

Critical is locked at FA 1.2 m, ND 0.9 m and IN 1.2 m. Each of the five seeds uses the same scene, reset and command prefix as its same-backend Advance reference. Only requested commands within the listed window differ. Body yaw stays fixed.

| Card | Stop | Yield | Reverse |
| --- | --- | --- | --- |
| FA | Zero from 4 to 6 s | 4–7 s at 0.25 m/s forward. Lateral −0.20 m/s for 1 s, zero for 1 s, +0.20 m/s for 1 s. | 4–5 s at −0.20 m/s, then wait to 6 s. |
| ND | Zero from 2.5 to 4.5 s | 2.5–5.5 s in the approach room at 0.25 m/s forward. Lateral −0.20 m/s for 1 s, zero for 1 s, +0.20 m/s for 1 s. | 2.5–3.5 s at −0.20 m/s, then wait to 4.5 s. |
| IN | Zero from 2 to 4 s | 2–4 s at 0.15 m/s forward on the approach. | 2–3 s at −0.20 m/s, then wait to 4 s. The phase formula supplies this IN variant. |

Each Stop and Reverse hold is flagged as intentional waiting for the freeze metric. All branches resume the Advance schedule at the same absolute tick. The robot-only preflight integrates the requested commands for a swept-footprint check. The actual action, displacement and lane recovery are measured through requested, limited and executed motion. See the [metrics and label definitions](METRICS_AND_THRESHOLDS.md).

## Plan and execution

Pass `--bindings configs/benchmarks/bindings.yaml` to the existing CLI when planning each phase. Plan totals are 48/540/900, with 48/492/360 new slots. Round 0 uses Card-specific Easy, including ND 1.2 m. Round 1 adds the remaining Advance slots and Branch B. Round 2 adds only Critical Advance seeds 1005–1019. A shared plan fingerprint, slot identity and matching runtime provenance are required to reuse attempts. The unbound plan remains available for auditing the original specification gaps.

Dry-run verifies backend availability, compiles the scenes, checks apertures and route clearance, and resolves the schedules. No SPL denominator is supplied by these bindings. A valid SPL would require a separate shortest-path reference for the defined footprint and goal. Failed labels, timeouts, overlap and guard events stay in the QA report. See the [runbooks and provenance requirements](BENCHMARK_REPRODUCIBILITY.md).
