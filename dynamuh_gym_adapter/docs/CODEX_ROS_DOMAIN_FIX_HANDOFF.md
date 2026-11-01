# ROS Domain Fix Handoff

## Current task
TASK 2E.1 — Implementation and focused tests; TASK 2E.2 — staged validation. Both completed.

## Status
COMPLETED

## Root cause
- Task2D reproduced live cross-delivery of clock/IMU data between distinct Gazebo partitions sharing a ROS domain and global ROS topic names. A new simulator's IMU .004 overwrote another worker's1.2 cache. Native GZ partition isolation does not isolate ROS.
- Freshness checks correctly rejected mixed timelines; they and sensor/reward/PPO semantics must remain unchanged.

## Files changed
- New ros_domain.py and tests/test_ros_domain.py.
- real_backend.py: domain reservation, ROS initialization guard and worker/episode cleanup lifecycle.
- phase2_common.py, train_ppo.py, eval_ppo.py, smoke_gazebo.py, motion_probe.py: automatic domain defaults; train manifest records selected domain.
- README_PHASE2.md, docs/ROS_DOMAIN_OWNERSHIP.md and this handoff.
- Raw diagnostic scripts/logs retained under runs/ros_domain_fix_debug/. Existing untracked IMU handoff and prior evidence preserved. No core.py/env.py changes, commit or push.

## Implementation completed
- OS-backed atomic exclusive domain lease acquired before ROS initialization; automatic distinct domains or explicit fail-fast request, with Linux DDS/UDP port policy.
- Explicit ROS context domain and inherited child environment agree. Lease survives episode resets and owned launch lifetime; final close/initialization failure releases after owned resources stop. Failed cleanup retains ownership.
- Initial best-effort graph check rejects pre-existing critical-topic publishers. Local advisory limitations documented. Sensor and WorldControl/PPO semantics unchanged.

## Tests executed
- Focused regressions: PASS,21 tests in5.81s.
- Task2E.2 Phase1 isolation: PASS, actual distinct-domain simulators, contention/reset/cleanup and invalid timestamp rejection evidence.
- Phase2 backend/Gym: PASS,18 bounded real actions across resets/workers, valid finite observations/rewards and .2s advancement.
- Phase3 one2048 PPO acceptance: PASS,exit0,2048 transitions,80 optimization epochs,all models load. Final detailed evidence below.
- Final git diff --check, production source integrity, previous model hashes, owned-process cleanup and lease release: PASS.

## Remaining risks
- An OS-backed local lock only coordinates cooperating local workers; it cannot exclude unrelated/remote ROS participants.
- Processes must share the host lock filesystem; separate container /tmp namespaces require shared lock storage or stronger DDS/network isolation. Discovery guard is bounded/best effort against non-cooperators joining late.
- Native WorldControl timeouts still occur and recover through existing fallback. No claim of eliminating every intermittent/cross-host failure. Per-poll transient freshness rejections are not counted; recorded state samples and terminal validation paths are checked.

## Next exact action
- Stop after reporting. No additional training, speculative fixes, commit or push. Use a fresh run directory for future authorized runs and consult the isolation limitations in docs/ROS_DOMAIN_OWNERSHIP.md.

## Historical milestones
The chronological checkpoints below preserve work completed before the final result; their former IN_PROGRESS/next-action statements are superseded by the final status and acceptance section.

### Implementation milestone — cheap tests PASS
- Initial Git status had only the unrelated/untracked docs/CODEX_IMU_ROLLBACK_HANDOFF.md; preserved it. No applicable AGENTS.md found. Existing tests were6 core/mock tests; no production changes preceded inspection.
- Added ros_domain.py: Linux kernel flock, global persistent lock inode, atomic nonblocking acquisition, auto allocation or explicit fail-fast request; PID metadata informational only. Real Linux ephemeral range32768-60999 read and full120-participant/250-port blocks enforced, conservative allowed0-100/215-231 (auto skips0). Unsupported/unreadable policy fails closed.
- Backend reserves before ROS context initialization, fixes domain for worker lifetime, passes it explicitly to rclpy and all child environments. Owned ros2 launch inherits lease FD for safe crash behavior. Reset now uses _stop_episode rather than final close; final close releases only after owned process group and executor/context stop. Cleanup failure retains lease. Forked backend reuse is rejected.
- Added bounded initial critical-topic publisher check for non-cooperating pre-existing ROS publishers. It does not claim to prevent late/remote participants. Documented local advisory scope, port budgets, persistent files, child inheritance and final-close semantics in docs/ROS_DOMAIN_OWNERSHIP.md.
- CLI/factory defaults now automatic (None) instead of fixed61 in train/eval/smoke/motion and phase2_common.make_env; explicit --domain retained. Training config.json records actual selected domain. PPO hyperparameters/training algorithm unchanged.
- Focused suite command: `timeout 90s .../ai12/bin/python3 -B -m pytest tests -q -p no:cacheprovider`; final result21 PASS (~5.8s). Includes real process contention, SIGKILL release, inherited-FD hold, stable lease across3 mocked-resource resets, failed initialization cleanup, failed teardown retaining lease, live owned child termination before lease release, fork safety, range/symlink rejection, stale/future IMU rejection, and original WorldControl branch tests. Pure unit mocked WorldControl rejection executes no actual simulation requests.
- Semantic comparison PASS: actual ASTs of _world_steps, advance and _snapshot unchanged; core.py and env.py byte hashes unchanged. git diff --check PASS. No full training run during Task2E.1.
- Files changed: ros_domain.py, real_backend.py, phase2_common.py; domain-argument defaults in train_ppo.py/eval_ppo.py/smoke_gazebo.py/motion_probe.py and actual domain manifest; tests/test_ros_domain.py; README_PHASE2.md; new domain documentation/handoff. No commit/push.
- Task2E.1 implementation + cheap regressions PASS; real DDS isolation regressionB remains to be exercised in explicitly requested Task2E.2 Phase1. Phase2 and single full acceptance remain NOT_RUN/gated.

## Next exact action
- Read this checkpoint before Task2E.2. Prepare bounded live validation of same-domain rejection, distinct-domain native/ROS separation, repeated reset retention, shutdown/initialization failure release and freshness rejection. Adapt optional tracing to new _stop_episode lifecycle/source line offsets; no production fallback change. Stop if Phase1/2 fail. Only if both pass execute one2048 original training command in fresh runs/empty_ros_isolation_acceptance.

### Task2E.2 — live validation prepared
- Re-read this handoff before validation. Evidence root runs/ros_domain_fix_debug/20261101_133051_validation_aa3f31/; manifest records implemented source fingerprints. validate_live_domains.py compiled successfully.
- Optional actual-callback/native-IMU tracing adapted to _stop_episode lifecycle and current _world_steps source-line offset; no change to production stepping. Planned180s internal/210s external+15s grace.
- Phase1: real same-domain constructor rejection in a separate process; two automatic distinct-domain simulators with distinct publisher endpoints/native timelines; lease FD actually present in owned ros2 launch; repeated3 primary resets/domain+FD retention; final close/reuse; controlled initialization failure after owned Popen with domain/resource release; initial foreign-publisher rejection before Gazebo. Invalid timestamp input is checked on a DETACHED copy of a valid snapshot, never published/written into live cache.
- Phase2: valid/finitely observed original Gym steps across those resets and domains with .2s advancement; any actual rollback/fault fails phases and blocks PPO. No actual fake sensor messages or accepted invalid transition.
- Phase3 remains NOT_RUN; requested runs/empty_ros_isolation_acceptance does not yet exist. No full training until Phase1/2 PASS.

### Task2E.1 completed / Task2E.2 Phases1-2 PASS
- Task2E.1: COMPLETED.21 focused tests PASS, unchanged core/env and WorldControl/snapshot/advance semantics verified, ownership lifecycle documented. Real DDS regressionB now also PASS in staged validation.
- Task2E.2 live command: timeout210s(+15s grace), internal180s, ai12/workspace sourced, CODEX_IMU_TRACE=1. Actual result live/result.json PASS in44.6s. Phases1=PASS,2=PASS,3=NOT_RUN.
- Evidence: same_domain_worker_attempt in a separate process rejected configured active domain1 immediately. Actual workers on domains1/2 had separate IMU/clock endpoints and independent native/ROS times; no cross-delivery or rollback. First ros2 launch actually retained the inherited lease FD (/proc FD link verified).
- Repeated primary3 resets kept domain1 and same parent lease FD, with same-domain rejection checked again after resets.18 verified real Gym steps total across primary/secondary/replacement: valid/finitely observed rewards, boolean end flags, .2s increments. No unexpected SimulatorFault.
- Final close released domain1; replacement worker successfully reset/stepped using1. Injected startup failure after real Popen cleaned ROS/Gazebo and released1. A non-cooperating standalone ROS node advertised /imu (no fake messages published): initial guard refused startup and released lease before any Gazebo launch.
- Stale/future validation checks used DETACHED copies of an actual valid snapshot only; core rejected both and live cache stamp was unchanged. No sensor message was rewritten/published or invalid observation accepted.
- Live result checks all7 PASS; no IMU rollback, no ambiguity guard. ROS logging warned about duplicate node name in the one-process multi-context diagnostic; actual domain context IDs/publisher separation were verified.
- No production code changed after focused tests. No commit/push. All prior handoffs/runs preserved.

## Next exact action
- Phase3 now authorized by successful gates: one original `train_ppo.py --scenario empty --timesteps 2048 --run-dir runs/empty_ros_isolation_acceptance`. Use fresh paths and optional trace adapted to _stop_episode and current source offsets,2100s external bound. No extra conflicting workers during training; no automatic rerun after failure. Record actual training/model/fallback metrics and stop after report.

### Phase3 prepared — one-run guard
- Created NEW runs/empty_ros_isolation_acceptance exclusively and acceptance/ under validation evidence root. Previous model hashes/source hashes captured in acceptance/manifest.json; no previous run overwritten.
- Acceptance uses literal requested train_ppo.py command. Optional sitecustomize activates only for that main script, not ROS children. Actual original _world_steps/_gz tracing adapted to shifted source and episode _stop_episode lifecycle so native observers stop at each reset without releasing the domain lease prematurely.
- No PPO/hyperparameter alteration. Full original callbacks/checkpoint behavior retained. Every real Gym result is recorded for finite/space/delta verification; PPO counters/losses and optimization epochs recorded. Unknown repeated WorldControl requests are stopped before resend by diagnostic guard; original source fallback remains unchanged.
- Internal2050s/external2100s(+20s grace) based on prior~.6s/action. Scripts and exclusive invocation_guard prevent automatic reruns. Model verifier prepared for untrained, checkpoint2048 and final; final policy finite/changed weights and2048 timesteps/80 epochs required.
- NEXT: invoke acceptance/run_once.sh ONCE outside sandbox, monitor acceptance/progress.json and record milestones here. If failure, stop, preserve traceback and no speculative fix/rerun. No further conflicting workers to be launched during acceptance.

### Phase3 RUNNING — do not restart
- Single training invocation PID170158 began2026-11-01T06:39:00.408704Z with exact requested argv. exec session74013 tracks it. invocation_guard and training_invoked.json persist; DO NOT call acceptance/run_once.sh again.
- Actual automatically selected domain1 is stored in runs/empty_ros_isolation_acceptance/config.json; seed11/original2048 budget and PPO settings unchanged. Initial reset/service readiness passed.
- As of06:40:33.339880Z:160 validated steps/~93s elapsed,1 reset,0 native timeouts,0 fallbacks rejected,0 observation/reward/delta validation errors, stderr empty. Phase3=IN_PROGRESS; phases1/2 stayPASS.
- On continuation read acceptance/progress.json/exit_status.json first. Continue waiting on the ONE run; no extra workers during acceptance. At exit verify actual counters/losses, all artifacts load, no IMU rollback, ownership release and source integrity. If failure preserve trace and do not rerun/fix speculatively.

### Phase3 progress checkpoint
- At2026-11-01T06:46:40Z:768/2048 validated steps,3 resets/rollout logs,~460s elapsed;10 native timeouts accepted by original clock fallback,0 rejected/guard/validation errors,stderr empty. Still same single invocation/session74013; do not restart.
- Next: monitor current acceptance/progress.json until exit, then run prepared artifact/analyzer/owned-cleanup checks and update final phase results.

### Phase3 halfway checkpoint
- At2026-11-01T06:49:31Z:1056/2048 validated steps,3 resets,4 rollout logs,~631s elapsed.12 native timeouts/12 accepted clock fallbacks,0 rejected/guard/validation errors,stderr empty. Past historical982-step failure point; this does not establish permanent elimination.
- Next: continue same single invocation; after exit verify final optimization/artifacts/owned cleanup. No extra ROS workers or training retry.

### Phase3 progress —1376 steps
- At2026-11-01T06:52:57Z:1376/2048 steps,4 resets,5 rollout logs,~837s elapsed;20 native timeouts/20 accepted fallbacks,0 rejected/guard/validation errors,stderr empty. Completed episodes so far:450-step time_limit,243-step goal_reached,450-step time_limit.
- Next: finish same single invocation; retain exact final results and artifact/counter evidence.

### Phase3 progress —1696 steps
- At2026-11-01T06:56:18Z:1696/2048 steps,5 resets,6 rollout logs,~1038s elapsed;26 native timeouts/26 accepted fallbacks,0 rejected/guard/validation errors,stderr empty. Same invocation still running.
- Next: wait for original training exit, then prepared model verifier, acceptance analyzer, cleanup/lease-release/source-integrity checks. Never rerun training.

### Phase3 training exited0 —verification underway
- Exact single2048 command completed at2026-11-01T06:59:56Z,exit0,1256.85s wall time.2048 validated env transitions,6 resets,8 rollout logs;learn_completed=true.33 native timeouts/33 accepted clock fallbacks,0 rejected/ambiguous guards/validation errors,stderr0bytes.
- No automatic rerun. Post-run model verifier session81457 and owned-process inspection session54869 underway; final PASS requires their results and acceptance analyzer counters/losses.
- Next: collect verifier results, analyze acceptance, confirm domain lease release and final diff/semantic integrity, then final handoff/report.

## Final acceptance — TASK 2E.1 and TASK 2E.2 COMPLETED

### Phase results
- Implementation/focused regressions: PASS. `timeout 90s /home/thaonhi/miniconda3/envs/ai12/bin/python3 -B -m pytest tests -q -p no:cacheprovider` =>21 passed in5.81s. Log: runs/ros_domain_fix_debug/focused_tests.txt.
- Phase1 isolation: PASS. Same-domain separate worker rejected before ROS communication; domains1/2 demonstrated independent real IMU/clock timelines/endpoints. Three primary resets retained the same domain/lease FD. Actual shutdown/replacement and startup-failure cleanup released ownership; initial foreign-publisher guard rejected an occupied domain. Invalid stale/future timestamps remained rejected on detached snapshots; live data was not changed.
- Phase2 backend/Gym: PASS.18 actual actions across workers/resets; finite observations/rewards, observation_space membership, bool flags, .2s advancement and no unexpected fault/rollback. Both phases finished in44.6s under180s internal/210s external bounds. Evidence: runs/ros_domain_fix_debug/20261101_133051_validation_aa3f31/live/result.json and events/native simulator logs.
- Phase3 PPO: PASS. Exactly ONE command `python3 train_ppo.py --scenario empty --timesteps 2048 --run-dir runs/empty_ros_isolation_acceptance`, externally bounded2100s(+20s kill grace), internal2050s. No rerun. Exit0 at2026-11-01T06:59:56Z,wall1256.852s. Selected ROS domain1 throughout all six resets.

### Actual PPO and simulator evidence
-2048 recorded/validated environment transitions and PPO num_timesteps2048. Eight rollout/optimization phases,10 epochs each; final `_n_updates=80`. Checkpoint at2048 stores70 updates because its callback executes before the final optimization; final.zip confirms80.
- All transition observations belong to the space and are finite; rewards finite; terminated/truncated bool. Per-action sim_dt min0.19999999999998863/max0.20000000000000284,total409.6s. No invalid transition silently skipped.
-2055 actual WorldControl requests;33 native service timeouts,33 actual production clock-fallback acceptances,0 rejected fallbacks,0 ambiguous resend guards. Every timed-out action was reconciled with the original first-attempt clock predicate and .2s advancement; no diagnostic resend occurred. Existing source fallback unchanged.
- No unexpected SimulatorFault, terminal timestamp validation failure, sampled within-session IMU rollback, uncaught/capture error or stderr output. Scope: sampled sensor state plus actual terminal validation path; diagnostic does not count every transient per-poll freshness reject.
- Five completed episodes: goal_reached1 (243 steps),time_limit4 (450 steps each);5 transitions in unfinished sixth episode. Collision truth remains unavailable (`collision_truth_valid=false`), unchanged from baseline.
- Finite final metrics:loss=-0.006316103041172028,value_loss=0.3474885620176792,policy_gradient_loss=-0.021713236160576344,approx_kl=0.011966433376073837,learning_rate=.0003,explained_variance=.18503618240356445.
- untrained.zip,checkpoints/ppo_2048_steps.zip,final.zip all CRC-verified and loaded via PPO.load(env=None); policy/optimizer tensors finite and spaces match.13 final parameter tensors differ from untrained. final.zip SHA256 db186bb3630aee1425a481ebbe5130b24267862f2d34537dfa1284a50592e589.
- Owned-process audit found no live owned launch/children after exit and sent no signals. Domain1 was immediately reacquired/released without ROS initialization, proving lease release. No unrelated process terminated.
- Acceptance production source hashes unchanged; core.py/env.py and original _world_steps/advance/_snapshot semantics preserved. All previous model hashes recorded before acceptance still match. Final git diff --check PASS. Existing unrelated untracked IMU handoff preserved. No commit/push.

### Post-run commands and diagnostic files
All paths below are under runs/ros_domain_fix_debug/20261101_133051_validation_aa3f31/acceptance/ unless stated otherwise.
- `run_once.sh`: exclusive invocation_guard/training_invoked.json, train.stdout/train.stderr, exit_status.json, manifest.json, capture/events.jsonl, per-session simulator/native clock logs and capture/result.json. Optional tracing applied only to actual training entrypoint, with no production timestamp/reward/PPO change.
- `timeout 45s /home/thaonhi/miniconda3/envs/ai12/bin/python3 -B runs/ros_domain_fix_debug/20261101_133051_validation_aa3f31/acceptance/verify_models.py` =>PASS; model_verification.json.
- `timeout 15s /usr/bin/python3 -B runs/ros_domain_fix_debug/20261101_133051_validation_aa3f31/acceptance/analyze_acceptance.py` =>PASS,all14 acceptance criteria true; acceptance_report.json and SUMMARY.md.
- `timeout 15s /usr/bin/python3 -B runs/ros_domain_fix_debug/20261101_133051_validation_aa3f31/acceptance/check_owned_cleanup.py runs/ros_domain_fix_debug/20261101_133051_validation_aa3f31/acceptance` =>no live owned processes,no signals; cleanup_check_070032_874458.json.
- Bounded5s DomainLease.acquire(selected_domain1)/close probe =>PASS; lease_release_verification.json (no ROS context/worker started).
- `timeout 5s git diff --check` =>PASS.

### Remaining risks and next exact action
- Tested configuration passes this2048-step workload. Local flock cannot exclude unrelated/remote participants or workers in separate lock namespaces; graph guard is best effort. This does not establish elimination of every intermittent failure.
- Native timeouts remain observable but all33 recovered correctly in this run. No timeout increase or speculative WorldControl repair implemented.
- Stop and report phase1 PASS,phase2 PASS,phase3 PASS. Do not run more training, commit or push without new authorization.
