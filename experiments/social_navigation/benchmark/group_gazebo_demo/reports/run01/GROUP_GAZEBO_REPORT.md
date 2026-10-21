# SocNavGym live Gazebo demonstration — partial run01

Six trials were planned: standing/counterflow × native/circle_contact/robot_orca, seed42, baseline AMCL, 75sim seconds, sequential. The recorded [protocol](protocol.json) and [progress](progress.json) retain every attempted outcome. This is closed-loop Nav2, not identical robot-input replay.

| Scene / variant | Nav2 result | Actual final XY error | Minimum reported circle clearance |
|---|---|---:|---:|
| standing / native | SUCCEEDED | 0.2210m | −0.00151m |
| standing / circle_contact | SUCCEEDED | 0.1652m | −0.01100m |
| standing / robot_orca | SUCCEEDED | 0.1668m | −0.00732m |
| counterflow / native | startup ERROR, no goal | unavailable | unavailable |
| counterflow / circle_contact | not run | unavailable | unavailable |
| counterflow / robot_orca | not run | unavailable | unavailable |

Three Nav2 actions reported success; only two meet a separate 0.20m actual-position threshold. Success is not a collision-free claim. Signed clearances above are the trial's circle-proxy metrics, not newly recomputed rectangular-footprint metrics. Negative values are retained. Standing humans stayed fixed (maximum displacement approximately 3.7e−15m), including robot_orca: that patch affects walking-group ORCA only. These are not three reactive conversation policies.

The fourth trial stopped before navigation: scan476, IMU9515, odom2642 and clock23784 messages, but AMCL0. There were zero trajectory samples and zero camera frames. The underlying cause is not established; a missed initial publication is only a hypothesis. The cohort stopped on this infrastructure error and the final two trials were not attempted. Do not substitute a successful rerun for this failure.

All four attempts verified NVIDIA rendering and clean Gazebo server shutdown/owned-process cleanup. The first trial opened a GUI but user visibility was initially uncertain. A separate native walking viewer used QT_QPA_PLATFORM=xcb, domain89 and an isolated partition, with no Nav2 and a stationary robot. The user subsequently confirmed seeing the window: “thấy cửa sổ rồi tôi bấm nhầm”. This confirms visibility only; drag/zoom/pause remain unconfirmed. [GUI evidence](gui_confirmation.json).

The viewer ended before its ten-minute timer: its log reports clean Gazebo exit followed by a worker-exited exception. The cause is not established. Follow-up process inspection found no simulation processes remaining. Do not describe this as a successful ten-minute stability test.

The three PNG/GIF pairs in this directory come from the existing live Gazebo observer camera capture pipeline, distinct from the labelled offline replay diagrams. No camera media were produced for the startup failure. This partial report does not add a new media decode/hash validation or a completed six-trial acceptance claim.

Next: diagnose the counterflow AMCL readiness failure in the experiment harness, preserve the failed attempt, then record any retry under a new protocol/output directory. Verify GUI interaction during a subsequent visible session. ComplexityNav remains standalone.

Follow-up: [recovery01](../recovery01/RECOVERY_REPORT.md) preserves this failed attempt and adds three successful counterflow trials with a read-only subscriber-QoS amendment. The user also confirmed GUI drag/zoom and Pause/Play during that later session. Historical status above describes run01 at the time it stopped.
