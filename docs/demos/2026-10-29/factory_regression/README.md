# Factory regression: existing treatment versus five-second MPPI candidate

Four fresh headless native GPU runs, declared before execution in plan.json: standing and walking, baseline v4 versus candidate v6, seed45, six people. Initial/backend geometry is matched within each scenario. Human reactions differ with robot motion; this is not fixed trajectory replay. MPPI noise RNG is not fixed.

Only CostCritic weight16->3.81, near_collision_cost140->253 and time_steps30->100 differ. Native camera/SDF, aligned URDF/TF, radius .17 + padding .01, inflation .45, cost scale3 and speed caps remain unchanged. The previous fixed-gap acceptance is not used as evidence of social behavior here.

Capture integrity, physical goal arrival, descriptive swept human clearance and static clearance are separate from social review. Robot-center entry into the triangle of a standing three-person group is reported, including the subset whose native speeds are all<=.05m/s. This is a narrow diagnostic, not an O-space/persona label. A human-gap drop greater than .10m versus baseline is flagged for review.

No defaults are promoted, no WM/RL model or classifier is trained/modified, and failed or uncomfortable outcomes are preserved. The last shared unit suite was250 tests OK (2 conditional skips); this stage changes no runtime code and uses the frozen capture and audit entrypoints.

## Measured results and adoption decision

All four runs reached the physical goal and capture integrity PASS. Candidate physical screen PASS, but post-run recovery-free diagnostic FAIL: each h5 run required one controller progress failure, optimizer reset, costmap clear and retry; both baselines had0 action recoveries. This diagnostic is explicitly additional to the frozen physical-screen criteria, not a retroactive threshold change.

| Scenario/treatment | Active sim time | Path | Min swept human gap | Goal world XY error | Recoveries |
| --- | ---: | ---: | ---: | ---: | ---: |
| standing base |30.5325s|9.0311m|.34428m|.09763m|0|
| standing h5 |47.6275s|9.0883m|.32026m|.05478m|1|
| walking base |31.6225s|9.0132m|.12659m|.10719m|0|
| walking h5 |51.0675s|9.2515m|.14408m|.05301m|1|

After first actual XY arrival within .20m, baselines finish in1.2475s/1.12s; h5 takes16.8825s/17.1825s. Native progress failures occur at46.6575s/50.1675s after goal; completion follows .97s/.90s later. Recorded yaw curves plateau near .3rad until the recovery. This is direct event/timing evidence, not a captured critic-score explanation or proof of one inner optimizer cause.

Standing triangle-center intrusion:0 entries over3052/4177 nondegenerate checks. No >.10m human-gap reduction flags. Walking has no standing groups in these clips; the triangle diagnostic is not applicable there. Actual native standing-group members remain at their initial positions within floating-point noise while classified standing. Later native lifecycle changes are retained. Human IDs P1-P6 are not behavior-persona classes.

The existing SocNavGym backend configuration still declares a virtual robot_radius .25m, while Nav2 is .17m and the native collision enclosing radius is .16806m. This shared human-response buffer was not changed in the comparison; it is not evidence that all human-perception radii are calibrated to physical mesh dimensions. No force-contact stream exists.

Decision: no promotion of defaults or new training registration. Candidate is useful in the fixed-gap fixture, but the near-goal recovery dependency must be resolved before wider adoption. Next work should target near-goal position/yaw convergence, retain the arrival/clearance checks, and rerun the matched scenarios.

Raw runtime commit9b91b42. Runtime code is unchanged in this stage. Existing full suite250 OK (2 skips), plus the recorded analysis oracle check. Every attempted raw capture and report is preserved.

## Reproduce analysis into new locations

```bash
python3 docs/demos/2026-10-29/factory_regression/validate_analysis.py
python3 docs/demos/2026-10-29/factory_regression/analyze.py --report-dir experiments/social_navigation/benchmark/generated/NEW_FACTORY_ANALYSIS
python3 docs/demos/2026-10-29/factory_regression/endpoint.py --report-dir experiments/social_navigation/benchmark/generated/NEW_ENDPOINT_ANALYSIS
python3 docs/demos/2026-10-29/factory_regression/recovery.py --report-dir experiments/social_navigation/benchmark/generated/NEW_RECOVERY_ANALYSIS
python3 docs/demos/2026-10-29/factory_regression/verify.py --output experiments/social_navigation/benchmark/generated/NEW_FACTORY_VERIFICATION.json
```

Restore the raw paths recorded in plan.json first if archived. Each output refuses overwrite. `render_camera.py --case walking_h5 --output NEW.webm` preserves native GIF hold durations during lossy VP8 encoding; media_evidence.json records input/output hashes and source timestamps.
