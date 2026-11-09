# Human navigation scenario matrix

The six versioned catalogs in `configs/benchmarks/scenarios/` transcribe the [research Cards](../research/HUMAN_MODEL_FINAL.md) and pin their source hash. A Card catalog describes a research condition. A separate approved binding instantiates a scene, routes and action schedule.

| Card | Source geometry and widths in metres (Easy / Critical / Hard) | Actors and actions | Label evidence |
| --- | --- | --- | --- |
| FA · Frontal Approach | Straight corridor at least 8 m long. 2.1 / 1.2 / 0.9 | One human or two abreast/staggered. Advance, Stop, Yield, Reverse | Initial separation ≥5 m and headings within 15° of opposition |
| ND · Narrow Doorway | Two rooms and one door. 1.2 / 0.9 / 0.6 | One human or two in file. Advance, Stop, Yield, Reverse | Both actors enter the 3 m doorway neighborhood and their paths cross the aperture |
| BC · Blind Corner | L corridor. 2.1 / 1.2 / 0.9 | One human. Advance, Stop, Yield | Perpendicular approaches and geometry occluding the actors until both are within 2 m |
| IN · Intersection | X or T intersection. 2.1 / 1.2 / 0.9 | One or two crossing humans. Advance, Stop, Yield | Heading difference 90° ±20° and predicted arrival difference ≤2 s |
| OT · Overtaking | Straight corridor at least 10 m long. 2.1 / 1.6 or 1.2 / 0.9 | One human. Advance, Follow, Yield. Separate overtaker subtypes | Heading difference ≤20° and initial separation ≤4 m |
| MG · Small Group | Area between 1.6 and 2.1 m wide. Three levels are unspecified | Moderate 3–4 and dense 5–9. Advance, Yield, Follow | At least three humans simultaneously within 4 m |

Policy labels and behavioral descriptions are recorded in the YAML with their source meanings. Only an approved neutral or independent profile can currently bind to the native backend roster. Cooperative, non-cooperative and cohesive conditions need an approved representation. The runner does not retrofit behavior into model equations or infer policy from a trajectory.

The Round 0 generic Easy width conflicts with ND. The ND example YAML omits Reverse and the IN Card does not specify it. OT needs a Critical width and a seed/subtype allocation. MG needs width, layout, density and policy allocation by level. Branch B still needs a same-level Advance reference for each condition. All six Cards need surveyed scene IDs, routes, policies and robot command schedules before admission. Existing 04U and 06/07 technical fixtures are separate evidence. [Deviation ledger](../research/HUMAN_MODEL_DEVIATIONS.md)

## Binding review

Supply a schema version 1 YAML with `conditions` keyed by `<Card ID>/<level>/<action>`. See `compile_plan` and the small synthetic binding test for the exact schema. Each binding supplies an approval reference, measured `width_m` and `width_probe_xy_m`, integer density, human policy, scene-only Hydra overrides and a robot command schedule. OT also supplies the subtype. Branch B supplies a Card level in the top-level `branch_b_levels` mapping and `branch_window_ns` in the binding. It shares the same scene, label context and other controls with A. Approved deviation references are required for each enumerated `hm_spec_*` conflict and for actions absent from a Card. Optional SPL references require a positive path length and a source. The engine verifies the actual compiled scene ID, aperture and individual footprint routes. A negative side-by-side margin is retained as feasibility evidence because sequential passage may still be possible.

OT binds the native preferred speed to the source subtype (1.1 m/s for the pedestrian overtaker or 0.4 m/s for the pedestrian being overtaken). Actual speed remains a measured outcome.

A missing label is unavailable. A failed label, timeout, geometric overlap or guard event stays in the artifact and QA report. Reported label coverage is independent of backend availability and task completion. [Data QA](DATA_QA_PROTOCOL.md)
