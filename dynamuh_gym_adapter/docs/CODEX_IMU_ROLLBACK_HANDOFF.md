# IMU Rollback Debug Handoff

## Current task
TASK 2D — Diagnosis only

## Status
IN_PROGRESS

## Confirmed findings
- Prior Task2C acceptance stopped at982 PPO transitions with a snapshot-freshness SimulatorFault. Recorded clock35.2, IMU header timestamp.02, scan35.2 and odom35.172; WorldControl acknowledged and executed the action. IMU age35.18s exceeded.26s freshness limit.
- Prior evidence did not capture native IMU messages or ROS IMU publisher identity. The origin of the rollback is unconfirmed.
- An older run archive changed during that acceptance; the writer was not identified. No attribution to simulator interference is established yet.

## Hypotheses
- Native sensor timestamp reset, bridge/clock conversion, delayed/old-epoch messages, or cross-process ROS topic interference remain possible.

## Files examined
- docs/CODEX_WORLD_CONTROL_HANDOFF.md; Task2C SUMMARY.md; current real_backend.py, core.py, env.py and mecanum_bridge.yaml (initial reads).

## Commands and results
- Bounded read-only source/handoff/status/run inventory started. No simulator launched or production source changed.

## Diagnostics collected
- Existing Task2C evidence remains at runs/worldcontrol_debug/20261031_193008_task2c_d99ef9/. New uniquely named evidence directory will be created under runs/imu_rollback_debug/.

## Root cause or unresolved blocker
- Missing provenance of the IMU message and comparison to native sensor time at rollback. Need current live process/publisher inventory before deciding whether another bounded capture is required.

## Minimal proposed fix
- None justified yet. Do not disable freshness checks, rewrite timestamps or implement Task2E.

## Next exact action
- Trace reset/clock/IMU lifecycle from current source and saved events; preserve current process/archive metadata read-only. If needed, capture real ROS IMU reception with publisher GID alongside native IMU and clock, stopping on first rollback under a bounded budget.

### Checkpoint 1 — source and existing-runtime review
- New evidence root: runs/imu_rollback_debug/20261101_124449_task2d_882b50/. manifest.json fingerprints current production files; existing_runs_inventory.json and historical_review.json preserve current manual-run metadata.
- Current backend uses one new rclpy Context and SingleThreadedExecutor per reset, subscribes to absolute /clock and /imu (sensor-data QoS), and stores message header stamp directly. Freshness compares ROS /clock (simulation time) against sensor header stamps; host wall/monotonic time is not part of that comparison. No timestamp conversion occurs in on_imu.
- reset() closes the previous owned session, assigns new epoch, sets clock=None and clears samples before creating the new context and new paused simulator. Each episode therefore starts a fresh native simulation timeline. Message contents are not epoch-tagged: snapshots assign current self.epoch to cached data, and callbacks do not validate publisher identity. Cache clearing does not isolate another live ROS publisher in the same domain.
- Native GZ_IP/GZ_PARTITION are isolated per launch, but ROS domain is fixed at constructor default61 (or supplied --domain), and all launches bridge the same ROS /clock, /imu, /odom, /scan and /cmd_vel names. The launch preserves ROS_DOMAIN_ID. Native isolation alone does not isolate those ROS topics.
- Simulator controls remain synchronous original gz service with2500ms deadline,50 steps*.004=.2; advance waits for fresh clock/sensors afterward. core.require_fresh correctly rejects IMU age35.18>.26. No evidence supports disabling that check.
- Read-only host inventory at2026-11-01T05:48:54.451262Z found no current training/Gazebo/bridge process or open FD to the old archive. This cannot identify a historical short-lived writer or prove absence during Task2C.
- Later manual runs/empty_2k/final.zip exists and its saved metadata records num_timesteps2048/_n_updates80. That is evidence a later manual workload completed, not proof the old intermittent issue was fixed. The older empty_smoke still has no final.zip; exact manual-run identity/outcome should not be conflated.
- Historical /tmp launch logs around the Oct31 failure are no longer available in this environment. Existing durable Task2C raw logs still show rollback but lack per-message IMU publisher GID/native IMU data. Writer attribution remains blocked by missing contemporaneous FD/process/audit evidence.

## Next exact action (updated)
- Build a bounded optional provenance harness using the actual production ROS callbacks (MessageInfo supported by installed rclpy subscription callback setter), native IMU/clock echo, publisher GIDs and epoch/context/reset timing.
- Use an unused diagnostic ROS domain so no unrelated workload can be disturbed. First exercise several ordinary production Gym resets/steps. If no rollback appears, a controlled second owned backend sharing ONLY that diagnostic ROS domain, with a separate GZ partition, can test the process-interference hypothesis using real sensor data. Stop on the FIRST rollback; do not learn PPO, rewrite timestamps or implement a fix.

### Checkpoint 2 — bounded actual-callback harness prepared
- Added only optional capture_imu_provenance.py in the unique evidence root; syntax compilation passed. Production sources still match Task2C fingerprints. No training, model save, package install or timestamp rewrite.
- Installed rclpy supports message+MessageInfo callbacks. Harness temporarily wraps Node.create_subscription only while original backend._ros_callbacks creates its real on_imu/on_clock subscriptions, calls each original callback unchanged, and records message stamp, prior cache value, publisher GID, DDS source/receive metadata, host monotonic reception time, active/callback epoch+context generation, reset phase, executor thread identity/liveness and independent native IMU/clock observations.
- Original _world_steps/_gz and freshness validation are used; an optional trace records actual predicates/results. Diagnostic cancellation prevents any additional control request after first rollback; an unresolved stepping request is never replayed. No callback data is filtered or rewritten.
- Procedure: private-domain171 preflight must show no other nodes/IMU publishers. Three original Gym resets with5 zero actions each establish ordinary lifecycle baseline. ONLY if clean, a second OWNED backend joins171 with another unique GZ partition; actual real native/ROS message provenance tests cross-run interference. No domain61 workload touched. Stop on first IMU decrease within the live cached state, preserve both native streams and endpoint snapshots, close only owned launches.
- Internal150s / external180s +15s termination grace; native observers and service queries separately bounded. Callback overhead/observer startup delay can perturb timing and is a stated diagnostic limitation. No PPO learning; PPO timestep is explicitly null, while actual environment action, epoch and episode index are recorded.

## Next exact action
- Execute this one bounded provenance capture outside sandbox for local Gazebo/DDS transport; then classify from the observed native versus ROS timestamps and actual publisher GID. Do not presume process interference or implement Task2E.

### Checkpoint 3 — initial diagnostic logging error, corrected separately
- First capture used unused domain171 and successfully reset the primary simulator, with actual IMU/clock/native and GID observations. Before the first environment action, diagnostic advance_start logging raised TypeError because both Base.advance and extended state supplied keyword epoch. No sensor rollback or production SimulatorFault occurred; secondary simulator was never launched.
- capture/ + capture.stdout/capture.stderr and original capture_imu_provenance.py are retained. Harness exited1 after owned cleanup, with0 successful primary actions. Do not count this as a simulator failure.
- Created separate capture_imu_provenance_v2.py: diagnostic state field renamed backend_epoch to remove the collision. Actual callback/step/reset/data behavior unchanged. Syntax check passed. Next bounded execution uses NEW capture_v2/ output and preserves all prior evidence.

### Checkpoint 4 — controlled rollback reproduced and source mechanism established
- Corrected capture_v2 completed all3 ordinary Gym resets +15 zero-action steps without rollback. Each original reset created fresh context/native timeline; no previous-cache retention was observed in this baseline.
- Then second owned backend joined only private diagnostic ROS domain171 with a DIFFERENT GZ partition. Primary epoch/context3 stayed unchanged, paused native clock+native IMU1.2. On secondary bootstrap, primary's ACTUAL on_imu callback replaced cached1.2 with.004 at2026-11-01T06:02:39.973603Z. Executor alive; reset_active=False, callback/active generation3 and epoch3. This was not a primary reset or native sensor rollback.
- Secondary's native /imu emitted.004; both primary and secondary real ROS callbacks received stamp.004 with EXACT same DDS source_timestamp1793512959971811872 and publication_sequence_number1. Primary native stream had NO messages after secondary startup and stayed1.2. This directly establishes cross-run DDS delivery of another native timeline into the primary's unqualified /imu subscription.
- ROS /clock also mixed (primary ROS clock temporarily0.0 while native1.2). Final primary sample after the already-issued secondary50-tick bootstrap completed: ROS/native clock1.2, cached scan.2/odom.18/IMU.2; derived ages1.0/1.02/1.0 violate unchanged freshness thresholds. No extra world request was issued after the first rollback; no invalid transition was accepted. Cancellation on first rollback is a distinct diagnostic stop, not a fabricated SimulatorFault.
- Raw per-message publisher_gid is unavailable in this installed MessageInfo (recorded None); endpoint GIDs are present in publisher snapshots. Do not claim a per-message GID was observed. Message source/sequence identity, native secondary header timestamp and controlled ownership/unchanged primary timeline establish the delivery mechanism independently.
- Proven mechanism: ROS process/topic interference due shared domain/global topic names, despite correct native GZ partition isolation. Primary reset itself, native timestamp conversion and executor death are not explanations for this captured event. Historical Task2C is consistent but lacks native IMU/message identity, so exact historical injector/writer is still not attributable conclusively.
- Diagnosis distilled to diagnosis.json; all checks and production fingerprint comparisons pass. No PPO training or fix implemented. Stop further simulation; finish cleanup audit, minimal-fix/regression plan and final handoff.

---

# Final Task2D handoff

## Current task / status
TASK 2D — Diagnosis COMPLETED; no Task2E implementation

Controlled reproduction establishes a ROS isolation defect that causes real IMU rollback. Exact historical Task2C sender/archive-writer attribution remains unavailable. Both conclusions are kept distinct.

## Confirmed cause and evidence
- Reproduction category: **process interference through shared ROS domain and global topic names**, not a native primary-simulator timestamp rollback, primary reset, timestamp-conversion bug or dead executor.
- Read evidence-root SUMMARY.md and diagnosis.json: runs/imu_rollback_debug/20261101_124449_task2d_882b50/.
- Host inventory initially had no trainer/simulator/bridge or open old-archive handle. Domain171 was checked empty. All tests used171; no unrelated domain61 workload touched.
- Three ordinary production Gym resets and15 steps passed. Each reset closed the previous owned launch/context, cleared samples and restarted native time. No rollback/cache carryover was observed in this baseline.
- Primary epoch/context3 paused at native clock+IMU1.2. Its current IMU publisher endpoint before secondary:010ff049505f20e20000000000001903. Primary GZ partition dynamuh_gym_c2bc332fb17c, launch155455/Gazebo155470.
- Second owned backend used the SAME ROS domain171, another GZ partition dynamuh_gym_14250185f1b0, launch155776/Gazebo155787. It started a new valid native timeline.
- At2026-11-01T06:02:39.973603Z, primary's original on_imu callback stored.004 after cached1.2. Primary reset_active=False; callback/current epoch3 and generation3; executor thread alive. Its own native IMU remained1.2 and emitted NO messages after secondary startup. It did not reset or advance.
- Secondary native IMU emitted.004. Both actual primary and secondary ROS callbacks received stamp.004 with identical DDS source_timestamp1793512959971811872 and publication_sequence_number1. These are the same newly delivered DDS sample, not retained primary cache. The controlled source/native mapping establishes cross-run delivery even though per-message publisher_gid was unavailable in this installed MessageInfo (recorded None). Endpoint GIDs are logged, not fabricated.
- ROS /clock also crossed: primary observed0.0 despite native1.2. After the one already-issued secondary50-tick bootstrap finished, primary ROS/native clock1.2 coexisted with cached scan.2/odom.18/IMU.2. Ages1.0/1.02/1.0 exceed source freshness thresholds. This is the same class of incompatible timeline mixing as Task2C; no timestamp rewriting can make the sensor data belong to the right simulation.
- Stop was requested at the FIRST rollback;0 new WorldControl requests after that event. The pending native multistep completed; diagnostic cancellation prevented subsequent requests/snapshot waiting. No fake success/observation or accepted invalid transition.

## Relevant files and branches
- real_backend.py:25 default domain61; `_ros_callbacks`41-91 creates per-reset context/executor. on_clock63-65 stores any matching ROS /clock; on_imu82-84 unconditionally stores header stamp; subscriptions86-89 are absolute global topic names.
- reset178-192 clears memory and starts a fresh epoch/native GZ partition but retains ROS domain. Snapshot136-160 tags cached values with RECEIVER epoch; neither messages nor publishers are bound to that epoch. A live foreign publisher can enter the new/current cache after reset.
- advance220-234 uses current ROS clock and actual WorldControl then waits for fresh samples. WorldControl native services are scoped by each GZ partition and did not cause the captured primary rollback.
- core.py:95-110 computes sim_time-sensor_stamp and rejects stale/future ages. Correct guard, not a demonstrated defect; keep it.
- env.py:42-67 reset increments epoch and validates; step69-105 delegates to backend/validates .2s/reward/observation. No policy/reward change needed.
- phase2_common.py:113-121 / train_ppo.py:51 pass default ROS domain61 to each worker. `train_ppo.main` allows reused output directories and writes config/model files in --run-dir; it has no ownership lock.
- src/custom_corridor/config/mecanum_bridge.yaml bridges identical ROS clock/imu/scan/odom/cmd_vel names for each launch. corridor_tb3.launch.py:538-539 preserves ROS_DOMAIN_ID. Its generated world filename173-176 is reused by UID/width/obstacle; assets/output resource paths are shared, but there is no evidence that file reuse caused the observed timestamp event. The two native partitions/data streams stayed separate.

## Historical attribution / manual run outcome
- Task2C's IMU rollback35.0->.02 while native time35.2 is consistent with this demonstrated interference mechanism. However, that historical trace lacks native IMU/message provenance, so its exact injector is NOT proven. Do not relabel correlation as certainty.
- Old runs/empty_smoke/untrained.zip rewrite~3s before Task2C anomaly remains unattributed to a specific PID. Current process/FD inventory cannot identify a closed historical writer; historical /tmp launch logs were gone. Source output reuse permits overwrites, but exact historical writer is a precise unresolved blocker. No restoration or overwrite performed by Task2D.
- Later runs/empty_2k/final.zip saved metadata records2048 timesteps and80 optimization epochs; a later completed artifact exists. This is distinct from older empty_smoke (no final.zip), and not a diagnosis trace or permanent-fix proof. Inventory preserved read-only in existing_runs_inventory.json / historical_review.json.

## Commands / diagnostics / results
- COMMANDS.md contains exact commands and limits. Both captures: ai12/workspace sourced, optional CODEX_IMU_TRACE=1,150s internal/180s external+15s grace; service/native observer limits explicit. No PPO learning, full training, package install, source repair or timestamp rewriting.
- First capture failed on diagnostic duplicate epoch logging before its first action; valid initial reset passed. Original script/capture/stdout/stderr retained. v2 renames only that diagnostic state field; new capture_v2 output preserves earlier evidence.
- Corrected capture exit0 / ROLLBACK_CAPTURED after3 clean resets/15 primary steps and the controlled secondary startup. Full real ROS receptions, monotonic arrival time, epoch/context/reset flags, DDS source/receive/sequence metadata, thread liveness, native IMU+clock raw streams, provider/endpoints, source line stepping traces and simulator logs retained.
- Actual PPO timestep fields are null because no PPO model.learn runs here; actual environment action/episode indices and real backend epochs are recorded. No invented PPO counter.
- diagnosis.json assertions PASS: empty initial domain,3 clean baseline resets,15 valid steps, unchanged primary context at rollback, no new primary native data, secondary native .004 match, identical DDS sample at both callbacks,0 further control requests. All five production fingerprints unchanged from Task2C.
- Post-grace cleanup_check_060840_792079.json: no live owned children from either capture; no audit signals sent. Normal backend close signalled only its owned launch groups. No unrelated process touched.
- Files changed: this new handoff plus unique runs/imu_rollback_debug evidence/scripts only. Existing world-control handoff and previous evidence preserved. No production behavior fix implemented.

## Smallest safe proposed fix
1. Enforce exclusive ROS-domain ownership for each active worker, held for its ENTIRE environment lifetime across episode resets. Use the existing --domain option to assign distinct domains to concurrent workers; native GZ partition isolation is insufficient.
2. Add a fail-fast domain ownership lease/check at environment creation/startup, released only on final environment close (for example via phase2_common.make_env/lifetime wrapper). Do NOT tie lease release to current backend.close() called internally by every reset, which would create a reset race. Full ROS namespace/topic remapping is a larger alternative and not required for the minimal repair.
3. Keep original sensor stamps, strict freshness checks and50-tick stepping. Do not clamp timestamps, insert offsets, disable freshness or silently discard mismatched data as proof of a repair.
4. Keep unique output directories; a separate output ownership guard in train_ppo.main can address concurrent checkpoint overwrite risk if authorized. This is distinct from proving the unknown historical writer.

No proposed change has been implemented; Task2E requires a new user request.

## Regression tests required
- Sequential resets: sample cache starts empty, expected native timeline resets, current-generation ROS/native timestamps align, no old-context data accepted.
- Two workers request one domain: second refuses before simulator launch; first continues unchanged.
- Two workers use distinct domains but same topic names: no cross-sensor/clock delivery, including during resets.
- Domain lease remains held across reset and released on final close/process exit; no race permits temporary second owner.
- Bad/rolled/foreign timestamps remain fail-closed with age/source/epoch diagnostics; no rewrites or freshness weakening.
- Output ownership prevents concurrent writers to one run directory, if included in later scope.
- After implementation, bounded real backend/Gym/PPO reset validation before any separately authorized2048 acceptance. No tests or training beyond this diagnosis were started.

## Next exact action
Stop Task2D. Read evidence-root SUMMARY.md + diagnosis.json before any continuation. Implement only after Task2E is explicitly requested. Historical PID/writer attribution would require a newly authorized provenance/file-audit capture; do not rerun full PPO or repeat successful baseline diagnostics automatically.
