# Ground-truth bridge shutdown validation

The preserved crossing group_cohesion_v2 capture aborted in parameter_bridge after Gazebo server exit, during ROS launch SIGINT teardown (pthread_mutex_lock owner assertion, exit-6). Temporal evidence does not establish the upstream root cause.

New capture-only mitigation: after acknowledged stable pause and recorder closure, send exactly one SIGINT to the session-owned /common/ground_truth parameter_bridge while the Gazebo publisher is alive. Exact executable/topic/node arguments and session token are checked; a Linux pidfd pins process identity across the final check and signal. Wait for ROS launch child-watcher terminal record without reaping its child, then close Gazebo and remaining ROS launch normally. Failed receipt remains failed; capture audit adds an ordered bridge exit check and retains no_child_exit_errors. Defaults and installed ROS libraries are untouched.

plan.json declares two fresh seed45 cells before execution. No tuning, retry, overwrite, policy waiver, dataset promotion or claim of universal repair. Original crossing FAIL remains visible. New outcomes will be appended after execution.

## Measured outcome

Runtime474ea8c. Both new seed45 native RTX4060 captures PASS the full source-resolved capture audit and the predeclared integrated shutdown/physical/coverage checks. The exact ground-truth bridge receives one pidfd-pinned SIGINT after stable pause, ROS launch records its clean exit before Gazebo clean exit, and scoped cleanup has no remaining processes or fallback signals.

| Scene | Actual goal | Active sim time | Recovery / progress failures | Min swept robot/person surface gap | Max native group member-centroid radius | Full audit |
| --- | --- | ---: | --- | ---: | ---: | --- |
| head_on | SUCCEEDED |39.4825s |0 /0 |11.77cm |1.181m |PASS |
| crossing | SUCCEEDED |37.8375s |0 /0 |12.06cm |1.134m |PASS |

Human-human/static swept geometric lower bounds remain positive. Protected human, guard, goal-clock, vendor and simulation sources match the preceding capture; frozen robot/camera/Nav2/initial encounter inputs match byte-for-byte, with only the run-local socket normalized in world comparison. No deterministic MPPI equivalence or speed improvement is claimed; its randomness is unpinned. This series assesses shutdown, not a new frozen yaw-quality or social-comfort acceptance.

The first two audit CLI invocations omitted --source-root, producing FAIL source_hashes because ignored generated/vendor/build assets cannot be resolved from Git. Those original JSON receipts are retained, including their hashes and missing-source list. Read-only audits were then run with --source-root /home/cuongbien/DynamuhSim into distinct *_source_verified_audit.json outputs; every source hash actually matches and both full audits PASS. No simulator rerun, raw edit, source exemption or policy change. The report records both receipt levels explicitly; see audit_invocation_note.json and results.json.

The original group_cohesion_heading crossing audit remains FAIL no_child_exit_errors. Two clean fresh runs support this shutdown-order mitigation on this machine, not a proven upstream root cause or a universal race fix. No default promotion, dataset registration or WM/RL training. Next is the separately declared seed46–48 walking cohort plus standing-group regression before freezing a collection profile.

[Review videos](index.html), [results and shutdown receipts](results.json), [full source-resolved audits](audits.json), [media provenance](media_evidence.json), [unit tests](unit_tests.json). Camera clips are recorded native Gazebo RGB; original GIF holds are preserved, exported frame count verified, and default .25 playback of encoded4x restores simulation-time1x. The native RGB contact sheet was visually inspected. HTTP checks do not assert browser visual QA or a new Windows Gazebo GUI confirmation.

Reproduce analysis in a NEW output folder:

```bash
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 python3 docs/demos/2026-10-29/bridge_shutdown/build_report.py --output /tmp/NEW_bridge_report
```

For source-resolved raw audits, use a distinct output filename and --source-root /home/cuongbien/DynamuhSim. Preserve the original source freeze, raw run directories and every audit receipt.
