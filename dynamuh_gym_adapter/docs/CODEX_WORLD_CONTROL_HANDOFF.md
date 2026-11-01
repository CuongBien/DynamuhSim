# WorldControl Debug Handoff

## Current task
TASK 1 — Diagnosis

## Status
COMPLETED

Diagnosis scope is complete with an explicitly unresolved causal blocker: the reported timeout did not reproduce in bounded diagnostics. The failure layer and unsafe retry behavior are established; the underlying reason for missing replies in the original run is not established. No fix has been implemented. Stop after Task 1.

## Confirmed findings
- Investigated on 2026-10-31, Ubuntu/ROS2 Jazzy/Gazebo Harmonic transport13; ai12 Python 3.12.14. Workspace: /home/thaonhi/nav_ws/dynamuh_gym_adapter.
- Reported exception is raised by real_backend.py:_world_steps(), using native Gazebo Transport CLI, before advance() calls the sensor snapshot wait. It is not a ROS service timeout or an SB3 rollout timeout.
- Request is exactly:
  ```bash
  gz service -s /world/corridor_0.90/control \
    --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
    --timeout 2500 --req 'pause: true multi_step: 50'
  ```
- `_gz()` uses subprocess.run(capture_output=True, text=True, timeout=4.5) for this request. Three attempts are made. Each requires rc=0 and stdout containing `data: true`, or a clock fallback reaching original clock + steps*.004 - .002 after a 50ms delay.
- `Service call timed out` in the reported exception is CLI output, not Python subprocess.TimeoutExpired: that exception produces the different message `Gazebo transport unavailable/timeout`. The final message retains only the final attempt's last 300 stdout/stderr characters. It does not establish whether the server received, rejected, executed, or replied to a request. Wording `step rejected` overstates what a transport timeout proves.
- `startup_timeout=90` bounds service discovery; `step_timeout=20` bounds subsequent sensor waiting. Neither changes the hardcoded 2500ms native service timeout.
- Physics dt=.004, STEPS_PER_ACTION=50 and POLICY_DT=.2 agree with corridor_090.sdf. Do not change them for this failure.
- Reset sets the SAME child environment for CLI and simulator: GZ_IP=127.0.0.1, fresh GZ_PARTITION=dynamuh_gym_<uuid>, ROS_DOMAIN_ID=61 by default. A default-shell service query cannot assess that isolated simulator.
- `_wait_service()` establishes that the name is advertised and the launcher is alive; it does not verify a responsive request/reply route or the Gazebo child process. `_ensure_alive()` checks only ros2 launch, which can remain alive after a child exits.
- `paused:=true` omits `gz sim -r`; `gui:=false` adds `-s`. The backend never unpauses the world. Each action publishes velocity, sleeps .04s for transport delivery, and requests paused multistep. A successful Boolean response is acknowledgement, not completion: healthy traces show the clock still at the previous time on reply and advance() then waits for sensors/target time.
- No Python self-deadlock is evident: the callback mutex is released before subprocess.run and before snapshot waiting; ROS executor spins in a separate thread. Launch stdout/stderr go to a regular tempfile, not an undrained PIPE. These observations do not exclude a native simulator/plugin stall.
- Three bounded fresh-instance probes all passed, totaling 27 environment action steps plus three bootstrap calls. All 30 control requests returned rc=0 and `data: true`; no retry/fallback was needed. Sensor freshness, observation and clock checks passed.
- Installed launch is a symlink to inspected source, SHA256 697a5dd1571e6741483e5019db88e74877b4786ad7a5384427a92576fd05f0ef. Empty launch removes RVO2; world includes Physics, UserCommands, SceneBroadcaster, Sensors(ogre2), Imu. Robot uses MecanumDrive; lidar=5Hz, IMU=100Hz, odom=30Hz.
- Existing latest failure-associated launch log /tmp/dynamuh_gym_vlv_dl77.log matches runs/empty_smoke timestamps (~17:40 local). Robot spawn and bridges succeeded; TwistStamped reached Gazebo. It contains no explicit simulator exit or failure-time service/clock trace. monitor.csv has only header; failed transition number is unknown.
- Earlier /tmp/dynamuh_gym_6ly3t97m.log ends `NodeShared::RecvSrvRequest() error sending response: Host unreachable`. This is relevant transport evidence but may be caused by the short-lived client already exiting; it cannot alone establish cause for the 17:40 failure.
- Launch forces GALLIUM_DRIVER=d3d12 and MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA. Both historical and successful probe logs show `libEGL warning: egl: failed to create dri2 screen`. Those warnings alone do not prove a render deadlock.
- Pre-existing Gazebo PID 30238, PGID 30199, partition dynamuh_gym_c29c109002ad, domain61, Conda ai12 remained untouched. Read-only service listing in that partition was empty; info returned no provider. Its process being alive is insufficient to prove usability. It has no GZ_IP in its environment and is not a substitute for a failing fresh backend session.
- No applicable AGENTS.md found in inspected ancestry/project paths. Initial git status already had unrelated changes, and the adapter directory was untracked as a whole. No production files were changed; final SHA256 verification passes for all five requested Python files.

## Hypotheses not yet verified
- Leading hypothesis family: intermittent native Gazebo Transport request/reply failure or an unavailable/unresponsive server while ros2 launch remains alive. This is supported by the timeout's layer and one historical `Host unreachable` log, but the specific trigger is unknown.
- Per-call short-lived `gz service` discovery/connection/reply timing may be involved. There is no evidence yet that a persistent transport client would fix it.
- Native simulator/render/physics stall, child crash, transient resource starvation, and delayed ROS /clock delivery remain alternatives. No failing-process stack, resource capture, native clock trace, or child exit status was available.
- A startup-only readiness race is less compelling for a failure in advance() after successful reset; fresh inference probes also passed without added readiness delay. It is not excluded for later service availability changes.
- Existing retries can execute a non-idempotent multi_step more than once if the acknowledgement or clock arrives late. This is a confirmed algorithmic hazard, NOT a demonstrated explanation for this timeout. Reusing the original clock and accepting >= target also allows overshoot to be treated as success until later checks.

## Commands executed and results
All runtime diagnostics were bounded. Filesystem reads/searches used rg, sed/nl, readlink, sha256sum, ls, cat and small Python readers; these did not run simulator training.

1. Created this file immediately, then read train_ppo.py, phase2_common.py, env.py, real_backend.py, core.py, direct launch/world/bridge/model configuration, README files and existing acceptance artifacts. Updated checkpoint after each meaningful step.
2. `git status --short`, ancestry AGENTS.md checks and scoped file discovery: unrelated pre-existing modifications recorded; no production edits made.
3. `timeout 5s gz service --help`: confirmed CLI options and timeout unit milliseconds.
4. `timeout 6s gz service -l` inside sandbox: emitted `error in getifaddrs: Unknown error -1`; this is a sandbox limitation, not simulator evidence. Repeated runtime diagnostics outside sandbox with explicit approval. Host default-partition list was empty.
5. `timeout 8s ...ai12/bin/python3 -B -c ...`: Python3.12.14 and rclpy/numpy/gymnasium/stable_baselines3 discoverable. `timeout 8s bash -c 'source .../install/setup.bash; ros2 pkg prefix ...'` resolved both custom_corridor and custom_corridor_plugins.
6. Bounded host ps and filtered /proc/30238/environ read identified the pre-existing isolated process. `timeout 8s env GZ_PARTITION=dynamuh_gym_c29c109002ad gz service -l` returned no services; `timeout 6s ... gz service -i -s /world/corridor_0.90/control` returned no providers. No control request sent to this process.
7. First fresh probe: /tmp/worldcontrol_diagnostic.py, sourced workspace, ai12 interpreter directly, startup_timeout15/snapshot_timeout4, alarm65s and outer `timeout -k 12s 75s`. Reset + 3 zero-action steps; no PPO model. Service-info added ~2.089s after discovery. Provider tcp://127.0.0.1:39705 had WorldControl->Boolean types. Calls .354/.345/.363/.307s; snapshots .2/.4/.6/.8. PID42163; partition dynamuh_gym_fb914ef190f6; log /tmp/dynamuh_gym_r5zx5byh.log; output /tmp/worldcontrol_diagnostic.out; normal CLOSED.
8. Second fresh probe: /tmp/worldcontrol_policy_probe.py, same ai12 interpreter, make_env(empty,domain62), loaded existing untrained.zip, model.predict only, 16 actions; original discovery without service-info delay. Torch 10 threads. Calls .284-.380s; reset .2 -> final3.4, every action dt=.2. PID43427; partition dynamuh_gym_a7e17d3489b8; log /tmp/dynamuh_gym_6m0055z0.log; output /tmp/worldcontrol_policy_probe.out; normal CLOSED. This probe did not activate Conda, so a final environment check followed.
9. Final fresh probe: /tmp/worldcontrol_activated_probe.py, explicit `source /home/thaonhi/miniconda3/etc/profile.d/conda.sh; conda activate ai12; source /home/thaonhi/nav_ws/install/setup.bash`, domain61, seed11, make_env wrappers, existing untrained checkpoint, 8 inference actions, no learning. `command -v` confirmed ai12 python and ROS-vendor gz; CONDA_DEFAULT_ENV=ai12. Bootstrap+8 calls .271-.330s, snapshots .2 ->1.8, dt=.2. PID44771; partition dynamuh_gym_de0e88f1777c; log /tmp/dynamuh_gym_z1n_3i49.log; output /tmp/worldcontrol_activated_probe.out; normal CLOSED. Both inference probes had a 65s alarm and 75s outer timeout, with ROS_LOG_DIR under /tmp and Python bytecode disabled.
10. Host /proc check confirmed earlier owned launch PIDs42163/43427 absent; pre-existing30238 still present. All three probes invoke backend.close() in finally. Never used pkill/killall or signalled the pre-existing process.
11. Read existing empty_acceptance/human_acceptance/motion_probe artifacts (100/100/10 rows). These were NOT generated by this investigation and do not demonstrate a current repair.
12. `sha256sum -c /tmp/worldcontrol_production_before.sha256`: all five production Python files OK. Scoped git diff for launch/world/bridge is empty. No packages installed, no environment settings persisted, no model.learn/full PPO training run.

## Relevant files and functions
- train_ppo.py:main (line62): make_env -> Monitor -> PPO -> learn; finally env.close(). Code before try includes env/model setup; reported failure occurs inside learn/step and reaches finally.
- phase2_common.py:make_env (line113), FixedTask.reset, LidarSafetyShield.reset/step: fixed scenario/start/goal; shield reads raw scan under short lock then calls env.step.
- env.py:DynamuhGazeboEnv.reset (line42), step (line69), close (line107): reset epoch/freshness; clamp; 50-step backend advance; clock/space checks; propagate infrastructure fault.
- real_backend.py:WORLD/CONTROL (20-21), _ros_callbacks (41), _gz (93), _ensure_alive (100), _wait_service (104), _world_steps (114), _snapshot (136), _publish (162), reset (178), advance (220), close (236).
- core.py:PHYSICS_DT/POLICY_DT/STEPS_PER_ACTION, SimulatorFault, require_fresh, clamp_action.
- /home/thaonhi/nav_ws/src/custom_corridor/launch/corridor_tb3.launch.py:launch_setup, particularly paused/server flags (178-185), forced render environment (202-211), spawn/bridge, return of child actions.
- /home/thaonhi/nav_ws/src/custom_corridor/worlds/corridor_090.sdf: world name, .004 physics step, system plugins.
- /home/thaonhi/nav_ws/src/custom_corridor/config/mecanum_bridge.yaml and models/mecanum_robot/model.sdf: clock/velocity/sensor bridges and sensor/drive rates.

## Root cause and supporting evidence
**Established failure mechanism:** native `gz service` did not provide a validated Boolean acknowledgement within its 2500ms request deadline; the backend ultimately exhausted three attempts without its ROS clock fallback reaching the required target. It then raised the reported SimulatorFault before sensor synchronization. This is a transport request/reply timeout condition, not proof of a server-side rejection or of a simulator execution deadlock.

**Underlying root cause remains unconfirmed.** All 30 diagnostic requests succeeded in fresh sessions, including fully activated ai12/domain61/nonzero PPO inference. Thus the request format, world name, paused stepping and 50*.004 timing are valid in this installation. The exact missing evidence is a contemporaneous trace of a FAILING request: all attempts' rc/stdout/stderr and elapsed times, simulator child status, correct-partition provider address/types, native versus ROS clock movement, and simulator log/stack before teardown. Historical logs lack those observations; full training was intentionally not run. This is the explicit blocker to causal attribution. Do not claim this task fixed or reproduced the original timeout.

## Proposed minimal fix
No implementation in Task1. The smallest justified repair area is **real_backend.py**, leaving PPO, reward, policy dt, STEPS_PER_ACTION and world configuration unchanged. Proposed implementation sequence for a later authorized task:

1. In `_gz()` / `_world_steps()`, preserve per-attempt command timing, rc, stdout/stderr, initial/final clock, partition and last_log on failure. Distinguish CLI transport timeout, Python process timeout, Boolean false and simulator exit. Capture health evidence before close() destroys it. This makes the failing condition actionable; it does not itself cure a missing reply.
2. In `_world_steps()` / `advance()`, replace blind replay of non-idempotent multi_step with one outstanding request and a bounded reconciliation against expected clock target. If a reply is lost and clock reaches exactly the target, collect fresh sensors without issuing another 50 steps. If execution status remains ambiguous, fail the owned session rather than adding duplicate steps. Detect overshoot explicitly; do not accept >= target as equivalent to exactly 50 ticks.
3. In `_wait_service()` / `_ensure_alive()`, distinguish name discovery from responsive control/server health. Use a bounded read-only typed request (first discover/confirm its type, e.g. `/gazebo/worlds`) and verify server child health; a paused but healthy server must be accepted without requiring clock advancement. This is health hardening, not an assumed cure for an after-reset timeout.
4. Select the actual transport/server repair ONLY after failure capture: if the server is responsive and only short-lived CLI replies fail, evaluate an installed native persistent transport client and prove exact-once stepping (do not assume bindings exist or install packages). If the server child exits, minimally update launch_setup in corridor_tb3.launch.py to propagate Gazebo exit to launch, and repair the demonstrated crash. If a native sensor/render stack is blocked, change only the demonstrated launch/render setting after a controlled comparison. EGL warnings alone do not justify a rendering change.
5. Do not simply increase 2500ms or step_timeout: healthy request replies were .271-.380s; there is no observed latency distribution supporting a larger timeout, and a longer timeout would not fix a dead/unreachable server.

## Tests still required
- Capture one original failing session with the evidence listed above. Do not repeat these already-passed short probes merely to claim a root cause; use an explicit bounded budget for any longer reproduction. Training remains outside Task1.
- For a later backend change, focused fault injection: lost reply with exactly 50 ticks; stalled clock; late completion; overshoot; Boolean false; CLI/Python timeout; simulator child exit while launcher remains alive. Verify no duplicate multistep and no invalid PPO transition accepted.
- Re-run bounded empty reset/zero/nonzero action tests after any authorized fix; require .2s increments and fresh sensors, then several sequential resets with service discovery in each new partition and cleanup verification.
- If failure is exclusively tied to PPO optimization/rollout length, capture thread/resource and simulator health around that boundary in a separately authorized bounded reproduction. The inference-only probes did not test optimization, 2048 transitions, or long episode behavior.

## Next exact action
Task1 is finished; stop. No Task2 or production edits are authorized by this task.

For a future session, read this file first. The failure mechanism is localized; the trigger remains unproven. Use the existing /tmp diagnostic scripts/output if present (temporary files may not survive reboot). The next useful investigation is failure-time tracing with an explicit bounded reproduction budget and simulator child/native-clock capture before cleanup, rather than another successful three-step smoke or a timeout increase. Apply the backend repair sequence only when separately authorized.

---

# Task 2 checkpoint

## Current task
TASK 2 — Repair

## Status
BLOCKED

Task 2 stopped at its root-cause confirmation gate, as explicitly required by the user: "If root cause remains unconfirmed, stop and document the blocker rather than making speculative modifications." The repair completion criteria have not been met. All Task 1 evidence above is preserved.

## Root cause
- Underlying cause remains UNCONFIRMED. The established failure layer is a native Gazebo Transport CLI request/reply timeout in `_world_steps()`, before sensor synchronization. A missing acknowledgement does not prove whether the simulator executed the request.
- Verified current source retains the exact Task 1 service/type/request, fixed 2500ms CLI timeout, 4.5s subprocess timeout, three-attempt replay, discovery-only readiness check and launcher-only liveness check.
- Existing replay of a non-idempotent multistep request is a confirmed hazard, but no evidence establishes it as the cause of the reported timeout. Editing it would not justify claiming the timeout fixed.
- Saved probe outputs were re-read in Task 2: 4 + 17 + 9 control requests, all rc=0 and `data: true`, all three sessions CLOSED, no traceback. These are historical Task 1 results, not new runtime tests.
- There is no new failing trace among the listed adapter launch logs. The missing evidence is still the failing call's elapsed time/rc/stdout/stderr, simulator child status, provider address/types in the owned partition, native versus ROS clock movement, and simulator log/stack captured before cleanup.

## Files modified
- `/home/thaonhi/nav_ws/dynamuh_gym_adapter/docs/CODEX_WORLD_CONTROL_HANDOFF.md` only: appended initial Task 2 checkpoint, then updated it with the source/evidence verification and blocker.
- No production code, simulator configuration, tests, package dependencies, models, or run artifacts modified.

## Changes implemented
- No repair implemented because the mandatory root-cause confirmation gate did not pass.
- Preserved STEPS_PER_ACTION=50, PHYSICS_DT=.004, POLICY_DT=.2, observations, rewards, shield, termination/truncation and PPO architecture.
- No timeout increase, retry-policy change, fake observations, skipped failed transition, or unrelated refactoring.

## Tests executed
- Command: `timeout 8s cat docs/CODEX_WORLD_CONTROL_HANDOFF.md`, `timeout 8s nl -ba real_backend.py`, bounded scoped git/file/log inventory.
- Result: PASS — handoff/current-backend consistency review; unrelated pre-existing git modifications remain untouched.
- Command: `timeout 5s sha256sum -c /tmp/worldcontrol_production_before.sha256`.
- Result: PASS — train_ppo.py, phase2_common.py, env.py, real_backend.py and core.py all match Task 1 production fingerprints.
- Command: bounded sed/rg review of env.py reset/step/close, phase2_common.py make_env/shield, train_ppo.py main and core.py timing constants; bounded Python SHA256 of launch/world/bridge and JSON parsing of the three saved probe outputs.
- Result: PASS — lifecycle and timing match Task 1; launch SHA256=697a5dd1571e6741483e5019db88e74877b4786ad7a5384427a92576fd05f0ef, world=6be1f2db6269b03395bdfdc7e2ec231b73159487115102e8b61f201636514d3c, bridge=d181b97cc88187d76b9dd3eba6c64b87123647e9784ced17554719edaa02c12c, unchanged from Task 1; all 30 historical service replies successful.
- Test A (new backend runtime smoke): NOT_RUN in Task 2 — stopped at root-cause gate; historical Task 1 reset + 3 zero-action steps passed.
- Test B (new Gymnasium runtime smoke): NOT_RUN in Task 2 — same gate; historical Task 1 wrapped-environment inference steps passed freshness/space/clock checks. Those results do not prove a repair.
- Test C (new PPO integration/learning smoke): NOT_RUN in Task 2 — same gate; historical Task 1 PPO inference passed (16 and 8 actions), but optimization was not tested.
- Original 2048-timestep command: NOT_RUN — no confirmed root cause or repair to validate, and the explicit stop condition takes precedence over downstream runtime validation.
- Regression test: NOT_ADDED — no root-cause-specific implementation to test. No new runtime PASS or FAIL is claimed.

## Remaining risks
- The original intermittent timeout may still occur. A service timeout does not establish rejection, server death, or a render deadlock.
- The existing duplicate-step retry risk remains; it is documented separately from the unconfirmed timeout trigger.
- Neither PPO optimization nor the original 2048-transition workload has been validated in this investigation.
- Production behavior is unchanged. Task 2 is blocked, not completed or fixed.

## Next exact action
Stop Task 2 here. To unblock a future continuation, obtain failure-time evidence using an explicitly authorized bounded reproduction with an external tracing harness, before simulator teardown. Capture every control call and expected/observed clock, simulator child status, correct-partition read-only service health and simulator logs; use a single outstanding multistep request and never replay a timed-out request whose execution is ambiguous. If a failing live session or prior detailed trace already exists, inspect it read-only first.

Do not infer the cause from successful short probes or EGL warnings. Once evidence distinguishes transport reply loss from a stalled/exited simulator or synchronization failure, make only the corresponding minimal repair described in Task 1 and then execute Tests A/B/C. No further task, optimization, training or production change was started in this Task 2 turn.

---

# Task 2A checkpoint

## Current task
TASK 2A — Runtime evidence capture (diagnostics only)

## Status
IN_PROGRESS

## Runtime architecture / confirmed starting point
- Re-read the complete Task 1/2 handoff and existing diagnostic script; reuse their lifecycle findings. Current backend owns a separate-session ros2 launch process; launch starts Gazebo and bridges. A separate short-lived gz service CLI sends each synchronous WorldControl request with a 2500ms native timeout and 4.5s subprocess timeout. ROS callbacks spin independently; advance waits for clock+fresh sensors after acknowledgement.
- Production sources still match Task 1 fingerprints. No production repair is authorized in this task.
- Original backend's three-attempt replay cannot be used unchanged during evidence capture: external diagnostic subclass/wrapper will stop immediately on any non-acknowledged stepping request before the production retry loop can replay it. This is diagnostic fail-stop instrumentation, not a repair or fabricated response.

## Commands executed / results
- Bounded handoff/script/source/config reads and source fingerprint check. Repeated only necessary consistency checks and actual CLI help discovery; no new simulator execution yet.
- Initial source read used workspace parent instead of adapter cwd and failed to locate real_backend.py; corrected read in adapter cwd. `timeout ... command -v` failed because command is a shell builtin; no runtime conclusion drawn.

## Files modified / diagnostic files
- Handoff appended; separate diagnostic harness and unique evidence directory under runs/worldcontrol_debug/ being prepared.

## Root cause / smallest evidence-supported repair
- Still unconfirmed. No repair proposed beyond the conditional Task 1 plan until a failing runtime trace is captured.

## Next exact action
- Build a bounded external harness with full command output, process/clock sampling, native clock echo, readiness/failure service introspection, and fail-stop stepping. Run reset, single advance, 8 advances, then a bounded longer sequence if prior stages pass. Preserve all raw logs; never overwrite prior evidence.

### Task 2A checkpoint — harness prepared
- Evidence directory (created uniquely; no previous evidence overwritten): runs/worldcontrol_debug/20261031_181959_058dea/.
- Added separate `capture_runtime.py` there; production files remain unchanged. Syntax compilation passed. manifest.json records production fingerprints and installed executable locations; copied previous /tmp probe outputs into historical/ for durability (explicitly historical, not fresh tests).
- Actual installed `gz topic --help` and `gz service --help` inspected: transport13 CLI supports service list/info/request, millisecond service deadline, native topic echo with JSON output. Harness will discover /clock and provider types from the live owned session, not assume another simulator API.
- Harness records timestamp+monotonic start/end, exact arguments, request subprocess PID/full stdout/stderr, ROS/native clock, launch/descendant /proc state+CPU/wait channel, read-only service-info readiness and every ~10s, and failure-time service list/info/worlds health. Native clock echo has its own external timeout. Failure capture happens before owned launch teardown and includes thread wait/stack availability.
- Diagnostic fail-stop wrapper calls the real CLI with original request/deadline. Any missing acknowledgement raises before production _world_steps can retry. No successful response is synthesized. It preserves logs before/after owned close and tracks descendant state; no unrelated process is signalled.
- Planned first execution: explicit ai12 activation + sourced workspace; script alarm180s and outer timeout210s (+15s forced-exit grace). Sequence: reset, single zero action, eight zero actions, then up to256 actual PPO inference actions through the existing wrappers. No model.learn in this stage. Added readiness probes and monitoring can perturb timing; record this limitation.

### Task 2A checkpoint — initial harness failure (not a simulator timeout)
- Executed staged harness under ai12 with 180s internal/210s external limits. Initial reset and single WorldControl advance both succeeded (two acknowledged requests); native and ROS clocks reached .4s. Owned launch51122 / simulator51235 remained alive at the observation.
- Harness then failed when serializing the existing shield info field min_sector_range_m=NaN for a zero action. This is a diagnostic logging bug, NOT the reported WorldControl timeout and NOT invalid sensor data. No production change warranted.
- Preserved original script capture_runtime.py and full staged/ + staged.console evidence, including failure-time read-only services and process/thread snapshots. Script exited1 after owned cleanup; no retry was performed.
- Created separate capture_runtime_v2.py (v1 retained). It records nonfinite telemetry as explicit tagged JSON values rather than rejecting the log; simulator state/observations/actions are unchanged. Syntax check passed. Rerun uses a NEW staged_v2/ directory; no previous logs overwritten.

### Task 2A checkpoint — progressive stages passing
- Corrected external harness started as staged_v2 with NEW evidence files; prior failed logger run preserved unchanged.
- Live readiness, reset, single zero-action advance, and eight sequential zero-action advances passed. Longer256 inference-action stage is running within its 180s internal budget. No learning/PPO optimization has started.
- The first harness's immediate post-close process sample showed some children still shutting down after ros2 launch had returned -15. This is an observation about teardown timing, not proof of a leak or the original timeout cause; verify those exact owned PIDs after a grace interval. Do not signal pre-existing unrelated simulator instances.

### Task 2A checkpoint — raw CLI timeout reproduced; simulator executed the step
- staged_v2 stopped on the FIRST missing acknowledgement, without replay. Exit1 is the intentional diagnostic fail-stop, not an outer execution timeout. Completed224 environment actions before the failing action. The pending rollout/optimization harness was prepared but will NOT be run: a raw native timeout is already captured without PPO learning.
- Failing request id227 / CLI PID64901 at 2026-10-31T11:27:46.889114Z (18:27:46.889 local); same service, request/type and2500ms native deadline. Command completed in2.666571827s, rc0, stdout empty, stderr exactly `Service call timed out`.
- At request start both clocks=45.0. Native clock reached45.2 by11:27:47.345992Z, only0.456887686s after request start; at CLI timeout both clocks and scan/IMU stamps=45.2 (odom45.18). Simulator53999 and launch53988 remained alive through timeout. This is NOT a failure to execute those50 world ticks.
- Correct-partition control provider tcp://127.0.0.1:38611 remained advertised during/after timeout. Post-failure `/gazebo/worlds` (types discovered live: gz.msgs.Empty -> gz.msgs.StringMsg_V) returned corridor_0.90 in.303s. Native clock publishing continued while world correctly remained paused at45.2. Launch log contains one `NodeShared::RecvSrvRequest() error sending response: Host unreachable`.
- Classification for THIS event: E / response delivery failure, manifested as D / no reply before native deadline. A missing/unadvertised service, B exited/deadlocked simulator, and C failed advancement are not supported by this trace. Underlying socket/route failure trigger remains unknown; server log lacks a precise timestamp/request id, so association with the sole failed call is strong but not an exact server/client correlation.
- Critical limitation: production `_world_steps()` already has a clock fallback. Its predicate would pass for this trace after45.0->45.2. Therefore this raw CLI timeout is reproduced, but the original terminal `WorldControl step rejected` after three failures/no clock progress is NOT yet reproduced or explained. Do not claim the whole original failure fixed or causally solved.
- Need finish focused evidence summary, verify own completed-run child cleanup after grace, preserve fingerprints, and stop diagnostics. No speculative repair or training.

---

# Task 2A final handoff

## Current task
TASK 2A — Runtime evidence capture (diagnostics only)

## Status
COMPLETED

Controlled diagnostics captured the raw native WorldControl timeout and classified that event. The original terminal backend exception remains a separately documented unresolved case. No repair implemented; Task 2 repair remains blocked on that distinction. Stop after diagnostics.

## Runtime evidence and classification
- New durable evidence root: `/home/thaonhi/nav_ws/dynamuh_gym_adapter/runs/worldcontrol_debug/20261031_181959_058dea/`.
- See SUMMARY.md for the readable report; summary.json for exact derived events/timing; COMMANDS.md for exact reproduction commands. Original Task 1 outputs are copied into historical/ and explicitly remain historical evidence.
- First staged run: reset + single request succeeded, then external logger failed on the shield's expected NaN telemetry. All raw evidence and capture_runtime.py retained. This was a logger failure, not a simulator fault. Corrected capture_runtime_v2.py is separate and logs nonfinite info as explicit tags without changing simulator state or responses.
- Corrected staged_v2: readiness/reset PASS; single advance PASS; eight advances PASS; longer inference sequence STOPPED at the first raw native timeout after224 completed actions (9 initial actions +215 inference actions). No retry occurred. There were226 actual WorldControl requests including bootstrap:225 acknowledged,1 timed out. Successful call durations .287315438-.465275745s. No Python subprocess timeout or harness wall-budget timeout triggered.
- Failing request id227, CLI PID64901, starts2026-10-31T11:27:46.889114Z /18:27:46.889114 local. Exact command: `gz service -s /world/corridor_0.90/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 2500 --req 'pause: true multi_step: 50'`.
- CLI completes after2.666571827s (Python limit remained4.5s). **Return code is0 despite failure**, stdout is empty, stderr is `Service call timed out`. Checking return code alone is therefore inadequate; current production also checks Boolean payload.
- Start native and ROS clocks45.0; expected target45.2. Timestamped native clock reaches45.2 after0.456887686s. At CLI timeout ROS/native clocks=45.2, scan/IMU=45.2, odom=45.18. World executed the intended50 ticks before the acknowledgement deadline; it correctly remained paused afterward. Raw native stream plus65170 timestamped decoded messages are saved.
- Launcher53988 and simulator53999 stayed alive during the timeout (launcher poll=None; simulator observed sleeping/running, not zombie). Live process snapshots and32 thread wait/syscall records saved. Simulator child is not a direct child of the diagnostic Python process; a waitpid exit code is unavailable. It was demonstrably alive at failure. Kernel stack reads were permission-denied and that limitation is recorded rather than inventing backtraces.
- Correct-partition provider tcp://127.0.0.1:38611 advertised WorldControl -> Boolean types before, DURING, and after timeout. During-timeout info query completed monotonic16727.824606383; timed-out request interval was16726.141797825-16728.800220387. Post-failure service list still included control. Read-only `/gazebo/worlds` (live-discovered Empty -> StringMsg_V types) returned corridor_0.90 in.303465143s.
- Simulator stdout/stderr is retained in launch_failure.log / launch_before_close.log / launch_final.log. It contains one `NodeShared::RecvSrvRequest() error sending response: Host unreachable`. That supports response-path trouble; log lacks a request id/precise timestamp, so exact association and socket-level trigger remain unconfirmed.
- **Classification E supported:** the world advance executed but the native client did not receive an acknowledgement. **D confirmed only as client deadline expiration**; an actual late response was not observed. Whole-service disappearance (A), simulator exit/global deadlock (B), and inability to execute this step (C) are not supported by this event. A per-client reply-route disconnect remains possible.

## Crucial remaining blocker / root cause
- This is a real native CLI timeout, but it is NOT a reproduction of the full original terminal `WorldControl step rejected` exception. The existing production fallback at real_backend.py:128-133 would see45.2 >=45.0+.2-.002 and return successfully for this trace.
- The diagnostic intentionally stops on the first missing acknowledgement, before any blind production replay; it does not fabricate a success or accept a failed transition. Thus its DiagnosticStop is an evidence-capture outcome, not proof that unmodified training would have failed at this exact point.
- The original terminal exception requires failed acknowledgements AND absent/insufficient ROS clock progress across the original three attempts. That condition has not been captured. Neither an arbitrary timeout increase nor adding a fallback already present in production explains or fixes it.
- The reply-route/socket trigger remains unresolved because there is no correlated server-send syscall/network trace; historical Host unreachable could follow a client timeout. No server exit or physics-latency explanation is established for the captured event.

## Files modified / generated
- Appended handoff checkpoints; preserved Task 1 and Task 2 sections.
- Added ONLY diagnostic scripts, manifests, raw/derived logs under the unique evidence root. No production Python, launch, world, bridge, PPO architecture, reward, observation, timing or safety-shield changes.
- Durable files: manifest.json; production_integrity.json; installed_service_help.txt / installed_topic_help.txt / installed_transport_version.txt; capture_runtime.py (v1), capture_runtime_v2.py, check_owned_cleanup.py, analyze_capture.py; SUMMARY.md / summary.json / COMMANDS.md.
- Raw evidence: staged/ and staged.console (logger error); staged_v2/ and staged_v2.console (timeout); numbered request stdout/stderr (notably request_0227.stderr); aux service health files; native_clock.raw / native_clock.stderr; events.jsonl with wall+monotonic timestamps, process IDs/states, full commands/output and clock traces; launch snapshots; failure_threads_53999.json.
- capture_rollout.py was prepared and syntax-checked but **NOT RUN**, since the native timeout reproduced without learning. No full PPO training or optimization occurred.

## Tests executed / cleanup
- Exact commands and limits are in evidence-root COMMANDS.md. Both staged executions: ai12 explicitly activated, workspace sourced, domain61, unique backend GZ partition/GZ_IP=127.0.0.1; internal alarm180s and external timeout210s with15s forced-exit grace. Service auxiliary calls have4-4.5s subprocess limits; native clock observer has180s external limit and owned group cleanup.
- Both runtime script versions compiled; corrected single/eight-step stages passed actual observation_space, finite reward, boolean termination/truncation, sensor freshness and .2s clock checks. Long stage reproduced native timeout without any retry.
- First attempted logging failure retained as FAIL; corrected longer stage intentionally stopped as FAIL on missing acknowledgement. No overall PASS or repaired-training claim is made.
- Owned cleanup audit used an external15s limit and checked original PID start ticks plus owned PGID before eligibility. After grace, both completed runs had NO live owned children; **no signals were sent** by the audit. Earlier immediate post-close children were transient shutdown, not established leaks. Audit: cleanup_check_113201_846578.json. Backend had signalled only its owned process groups at normal close. Pre-existing unrelated simulator instances were not controlled/killed.
- Fresh final fingerprints in production_integrity.json confirm real_backend.py/env.py/phase2_common.py/core.py/train_ppo.py match their Task 1 hashes.

## Smallest evidence-supported repair
- No production repair is implemented or yet justified for the original terminal fault. Retain actual-clock/sensor completion verification and Boolean payload validation; do not increase timeout based on this trace or blindly replay a non-idempotent request.
- Later hardening can remove unsafe replay and preserve full transport/clock diagnostics, but that is separate from claiming the original terminal defect fixed. A persistent transport client is only a candidate if reply-socket lifecycle evidence supports it, not a proven cure.

## Remaining risks / limits
- Service/process/native-clock observers and extra read-only readiness calls can perturb timing. Monitoring samples are roughly.2s apart except when that monitor thread performs its bounded service-info query; native clock capture is independent.
- Only one relevant native timeout captured. Native/ROS clocks agreed here; no trace yet demonstrates the original missing-clock condition. No optimization/2048-transition training validation performed. No syscall/packet-level proof of the reply-route trigger.
- Diagnostic scripts reside in the run directory and import production source; do not treat them as installed production behavior. Their output directories refuse reuse; future captures must use new names.

## Next exact action
Stop Task 2A diagnostics here. Read SUMMARY.md + summary.json first on continuation, then inspect staged_v2/events.jsonl around id227 and request_0227.stderr alongside native clock/launch logs.

If further reproduction is authorized, use a NEW bounded output directory. Preserve unmodified production behavior when an acknowledgement is lost but the measured target and fresh sensors already prove completion; explicitly log the existing fallback reconciliation. If the target is not reached, capture native-vs-ROS clock, server/reply syscall health before teardown and stop BEFORE a repeat multistep request. That can distinguish a harmless lost acknowledgement from the original terminal no-progress condition. Do not replay the ambiguous request, implement a speculative persistent-client fix, or launch full training as a default next step.

---

# Task 2B checkpoint

## Current task
TASK 2B — Trace actual production clock fallback (diagnostics only)

## Status
IN_PROGRESS

## Source conditions confirmed before execution
- Read attached Task 2B request, full prior handoff evidence and Task 2A SUMMARY.md, and current real_backend.py. Prior evidence is retained; current production source remains unmodified.
- Actual terminal branch real_backend.py:134 executes ONLY after all three iterations return neither rc0+Boolean true (126-127) nor accepted fallback (131-133). `_gz` Python timeout/missing executable and dead launcher produce different exceptions before that terminal branch.
- Fallback executes only after a non-acknowledged CLI result: sleep .05 at128, lock+single read `now_clock=self.clock` at129-130, then presence tests and `now_clock >= start_clock + steps*.004-.002` at131-132. `start_clock` is captured once at118 before all attempts. It is not refreshed for retries, and overshoot passes this lower-bound test.
- Clock is the last value set by on_clock (63-65) from ROS /clock, sensor-data QoS, in separate SingleThreadedExecutor. Reads are mutex-protected but have no receive-time freshness test; startup None/stale values or delivery delays are possible hypotheses, not confirmed causes. There is ONE clock sample per failed attempt after50ms, not an elapsed-time polling loop.
- advance() already rejects an absent old clock before calling _world_steps; reset bootstrap can enter _world_steps with start_clock=None. Current stack of interest is advance/step, not bootstrap; do not conflate them.

## Instrumentation plan / constraint boundary
- Separate external harness traces actual production `_world_steps` frame with sys.settrace and wraps ORIGINAL `_gz` only for full result/timing logging. It will let the actual fallback accept a completed action and continue. No synthetic CompletedProcess/success, no added fallback wait/poll, no production source change.
- Exact fallback locals/predicate/accepted-return source line and branch will be recorded. Independent native clock, ROS clock sampling, launch/server state and read-only service health run separately.
- User forbids ambiguous WorldControl retries. Therefore guard at the next production request after a rejected fallback must stop BEFORE resending. A diagnostic safety stop is not the terminal SimulatorFault. It is impossible to safely force three ambiguous repeats merely to reach134; this limitation must be explicit if encountered.

## Tests / next exact action
- Create uniquely named Task2B evidence directory and harness. Run up to400 empty-scenario PPO inference actions through the production environment with seed11 and bounded wall time. If no exact exception and an actual PPO reproduction is needed, use a separate fixed-size/bounded rollout budget, not unrestricted training. No repairs in this task.

### Task 2B checkpoint — actual-branch tracing prepared
- New evidence root: runs/worldcontrol_debug/20261031_184854_task2b_7e384d/; manifest.json and real_backend_source_snapshot.txt preserve starting source/fingerprints. Prior Task2A evidence untouched.
- Added external trace_production_fallback.py, syntax checked. CODEX_WORLD_CONTROL_TRACE=1 is required by the diagnostic script; production source itself is unchanged.
- `_gz` diagnostic wrapper calls original GazeboROSBackend._gz, including original subprocess.run, then returns the original result unchanged. It does not capture blocking read-only health evidence in the main thread before fallback.
- `_world_steps` diagnostic wrapper invokes the actual original method with sys.settrace limited to its frame; logs start_clock/now_clock/threshold/predicate, sleep/check elapsed times, actual line133 acceptance or fallthrough, source exceptions and line134 if reached. Observers are independent; no new fallback polls/sleeps.
- At a rejected first fallback, guard before the NEXT request prevents an ambiguous resend. It raises a separately named RuntimeError, not a fabricated SimulatorFault. Normal and accepted-fallback paths are unchanged apart from trace overhead.
- Planned execution:400 total environment actions, empty scenario, seed11, existing untrained checkpoint, real wrappers/backend. Internal300s alarm, outer330s timeout +15s termination grace. No PPO optimization in this first run. Actual terminal exception and raw CLI timeouts will be counted separately.

### Task 2B checkpoint — actual accepted fallback cases captured
- Inference run has passed300 completed actions; original _world_steps has now executed its fallback on TWO actual native CLI timeouts and accepted both. This is real caseB evidence, replacing Task2A's merely inferred fallback outcome.
- Zero-based action253 / request256 / world-call255: actual start_clock50.8, now_clock51.0, target threshold50.998; predicate True at line131 and actual accepting return at133. One clock observation after .050241165s wait; .052765860s after CLI wrapper return. Native clock at check51.0. No replay occurred.
- Zero-based action307 / request310 / world-call309: start61.6, now61.8, threshold61.798; predicate True; actual line133 return. One observation after .050358659s wait; .051618840s after wrapper return. Native61.8. No replay occurred.
- Current live clock publisher introspection reports a single ros_gz_bridge /clock publisher. This only describes this run, not the original failing session.
- No fallback rejection, retry guard or production terminal exception observed so far. Continue only to the existing400-action /300s budget; do not alter source or timeout settings.

### Task 2B checkpoint — inference400 completed, bounded actual PPO check next
- inference_400 finished exit0 / PASS_NO_ORIGINAL_EXCEPTION with400 completed actions. FIVE raw native timeouts reached the REAL fallback and returned via actual source line133. No fallback rejection, retry guard, original terminal SimulatorFault or ambiguous resend.
- Accepted cases include zero-based actions253,307,358,370,378; raw structured records retain every CLI result and exact comparison. Full derived analysis will be generated after the next bounded check to avoid overwriting an earlier analysis file.
- A separate actual PPO caller test is useful because inference does not run model.learn/optimization. Prepared trace_production_fallback_ppo.py as a NEW diagnostic file, preserving first harness. It adds Python thread stack capture only after an exception (to diagnose executor staleness if it appears); stepping/fallback behavior remains original.
- Next execution is ONE production-sized256-transition PPO rollout with original train_ppo.py hyperparameters and QualityGateCallback, one optimizer phase (n_epochs10), then8 post-update inference actions (264 total cap). Explicit internal240s/external270s +15s termination grace. It is not the original2048-transition/full training workload. No production source or architecture edits.

### Task 2B checkpoint — actual PPO reproduction completed
- ppo_264 finished exit0 / PASS_NO_ORIGINAL_EXCEPTION: actual model.learn collected256 transitions with original n_steps256 and train_ppo hyperparameters, ran optimizer phase, then8 post-update real environment actions. QualityGateCallback passed; no source/model artifact overwrite.
- Actual PPO rollout also captured one native timeout: zero-based action115/request119, CLI2.659884437s, rc0/empty stdout/`Service call timed out`. Actual fallback read23.4->23.6, threshold23.598, predicateTrue, native23.6; actual line133 returned. One check after.052740422s fallback wait. No resend.
- Across both completed runs:400 inference +264 separate PPO/post-update actions. Six actual native timeouts all accepted through production fallback; no original terminal exception, no rejected comparison or retry guard observed. Pending derived counts/integrity and final cleanup audit; do not run further training/reproduction beyond these budgets.

---

# Task 2B final handoff

## Current task
TASK 2B — Actual production clock-fallback tracing

## Status
COMPLETED — completion criterion B

The original terminal `SimulatorFault: WorldControl step rejected: Service call timed out` was NOT reproduced within the authorized bounded investigations. Actual successful timeout-fallback returns were captured. No original-fault repair is justified or implemented. Stop after this investigation.

## Evidence location / files changed
- Unique evidence root: `/home/thaonhi/nav_ws/dynamuh_gym_adapter/runs/worldcontrol_debug/20261031_184854_task2b_7e384d/`.
- Read `SUMMARY.md` for the A/B/C comparison, `fallback_analysis.json` for full derived request/check/return evidence, `INFERENCE_FALLBACK_CASES.md` for the inference case table, and `STATIC_CONDITIONS.md` for exact source conditions and regression requirements.
- `COMMANDS.md` records exact execution commands and wall limits. Raw evidence: inference_400/ and ppo_264/ events.jsonl, numbered CLI stdout/stderr, aux service health, process/ROS clock samples, per-session native clock raw streams and launch logs; separate console files retained.
- Optional external scripts: trace_production_fallback.py and separate trace_production_fallback_ppo.py (requires CODEX_WORLD_CONTROL_TRACE=1), analyze_fallback.py, copied identity-checked cleanup helper. They call the actual original methods and trace their frames; no copied/fixed fallback implementation.
- manifest.json + real_backend_source_snapshot.txt preserve source provenance; production_integrity.json confirms all five requested Python files unchanged from Task1 fingerprints.
- Only this handoff and new diagnostic evidence/scripts were modified/generated. No production source/configuration, PPO architecture, reward, observation, safety shield, timing or termination/truncation changes. Earlier Task1/2/2A evidence preserved.

## Exact exception condition and source lines
- real_backend.py:114-134 is the original _world_steps implementation. Line118 captures one ROS clock baseline before all attempts. Lines122-125 issue the native request with2500ms deadline /4.5s Python timeout;126-127 accepts rc0+Boolean true.
- After a failed acknowledgement,128 sleeps50ms,129-130 performs ONE locked read of `self.clock`,131-132 tests `start_clock is not None and now_clock is not None and now_clock >= start_clock + steps * PHYSICS_DT - .002`, and133 returns if true. There is no clock polling loop.
- Line134 is reached only after THREE iterations fail BOTH acknowledgement and fallback. `_gz`93-98 Python timeout or `_ensure_alive`100-102 launcher exit produces a different branch/message.
- Clock source is ROS /clock callback63-65 on a separate executor. Mutex protection does not assert delivery freshness. advance223-226 requires an existing clock before entering the step; reset bootstrap209 can begin with None. Stale/None/delayed values are possible source conditions, but none was observed at an actual failed-ack fallback in Task2B.
- Exact branch tracing used sys.settrace on ORIGINAL _world_steps; actual locals and real line133 returns recorded. ORIGINAL _gz/subprocess.run results returned unchanged. No new fallback waits or polling; health/native/process observers ran separately.

## Tests executed / results
- Inference400: seed11, empty scenario, existing untrained PPO checkpoint, real make_env-equivalent FixedTask/LidarSafetyShield/env/backend path; internal300s/external330s+15s grace. Exit0,400 completed actions,401 WorldControl calls:396 acknowledgement returns and5 actual fallback returns. No repeated step request.
- Actual PPO264: separate explicit240s internal/270s external+15s grace. Original train_ppo hyperparameters (n_steps256, batch64, n_epochs10, unchanged MultiInputPolicy network), original QualityGateCallback. model.learn(total_timesteps=256) completed one rollout and optimization phase (recorded _n_updates10), then8 post-update real steps. Exit0,264 completed actions,266 WorldControl calls (two bootstrap requests):265 acknowledgement returns and1 actual fallback return. No original2048-transition command, unrestricted learning or model save.
- Total:664 real environment actions across the two separately bounded runs;667 WorldControl calls;661 normal acknowledgement branches;6 genuine native timeouts accepted by actual production fallback. Zero fallback rejections, zero retry guards, zero line134 executions, zero original terminal exceptions, zero clock-mismatch/freshness/quality-gate failures. Source guard never activated and did not suppress an observed failing condition.
- Both runtime commands explicitly activated Conda ai12, sourced workspace, domain61 with backend-owned fresh partition and GZ_IP127.0.0.1. Source fingerprints unchanged after execution. No packages installed.
- Post-grace15s cleanup audit checked original PID start ticks/owned launch PGIDs: no live owned children remained in either completed run; NO audit signals sent. Audit file cleanup_check_121809_077228.json. No unrelated simulator instance controlled or killed.

## Comparison A — normal completion
- Actual return branch127, rc0 and stdout `data: true`; no fallback clock read/wait. The acknowledgement may precede completed physics; production advance then waits for actual fresh sensors/target clock.
-661 occurrences. CLI success ranges: inference .292759-.845375s, actual PPO .292798-.435260s. No conclusion about a timeout defect is drawn merely from durations.

## Comparison B — actual native timeout accepted by production fallback
All six cases returned CLI rc0 with empty stdout and `Service call timed out` stderr. Every case had attempt_index0, ONE post-timeout clock observation, predicateTrue, actual line133 return, valid downstream state and no resend. Action indices are zero-based.

| Run / action | Request | CLI seconds | Actual start -> fallback ROS clock | Threshold | Native at check | Wait to actual clock check, ms |
|---|---:|---:|---|---:|---:|---:|
| inference /253 |256 |2.639048 |50.8 ->51.0 |50.998 |51.0 |50.241 |
| inference /307 |310 |2.693421 |61.6 ->61.8 |61.798 |61.8 |50.359 |
| inference /358 |361 |2.643584 |71.8 ->72.0 |71.998 |72.0 |50.644 |
| inference /370 |373 |2.705648 |74.2 ->74.4 |74.398 |74.4 |50.204 |
| inference /378 |381 |2.702865 |75.8 ->76.0 |75.998 |76.0 |51.802 |
| actual PPO /115 |119 |2.659884 |23.4 ->23.6 |23.598 |23.6 |52.740 |

- Independent native clock reached each target about.436-.506s after request start; the clock at the actual fallback check already matched the full .2s advancement. This is direct actual-branch evidence, not a hypothetical evaluation as in Task2A.
- Example inference request256: check at2026-10-31T11:55:17.512984Z, actual line133 at11:55:17.514123Z. PPO request119: check12:03:01.896296Z, actual line133 at12:03:01.902506Z. Full wall/monotonic timings and exact floating-point deltas are retained.

## Comparison C — rejected fallback / original SimulatorFault
- NOT REPRODUCED. No observed case had None/stale/insufficient ROS clock at the actual fallback check; no predicateFalse, fallthrough/retry or original terminal raise134 was recorded.
- A clock synchronization race, threshold defect, incomplete advancement, dead simulator or unexpected CLI behavior causing the terminal rejection is therefore not established. Native acknowledgement loss is confirmed and production successfully handles the captured instances.
- A distinct external safety guard would stop before a second ambiguous WorldControl request if the actual first fallback rejects, per the user's constraint. It was NEVER activated here. The original three-iteration terminal failure cannot be deliberately forced by replaying ambiguous requests; no such reproduction or fabricated SimulatorFault was attempted.

## Smallest safe change proposed / regression requirement
- No original-fault code change is justified yet. Adding a fallback already present, increasing2500ms or loosening the threshold would not explain these observed successful fallback returns. No implementation in this task.
- The next essential evidence is an actual rejected first fallback, with clock values/receive timing and independent native advancement, rather than another successful native timeout. If native reaches target while ROS remains stale at the real check, then bounded reconciliation in _world_steps could be evaluated without resending. If native also fails to advance, investigate simulator/transport instead. These are conditional investigation branches, not confirmed repairs.
- Required focused regressions for a later evidence-backed fix are specified in STATIC_CONDITIONS.md: normal response; lost reply but completed target with one request; actual native-complete/ROS-stale case; unknown/no-progress without replay; overshoot; absent bootstrap clock; child exit/Python subprocess timeout. No fake runtime data or speculative production test change added.

## Remaining uncertainty
- Original failing run lacks correlated action/episode index and source fingerprint/clock trace. A longer-horizon or episode-reset issue has not been excluded by these budgets; none should be declared the root cause.
- Tracing and observer I/O can perturb scheduling. Measured fallback waits were50.2-52.7ms; no claim that all rare timing races are excluded. All clocks in observed timeout fallbacks already had full target advancement, not borderline partial progress.
- Current-run /clock publisher introspection showed a single ros_gz_bridge publisher; this cannot establish publisher state in the original failure. No native reply-socket trigger or server-send syscall correlation is proven here.
- The original2048-transition training workload remains unvalidated. These successful bounded runs do not declare the reported issue fixed.

## Next exact action
Stop Task2B. For continuation read this final section, evidence-root SUMMARY.md and fallback_analysis.json first. Do not repeat the completed400-action/264-action captures as proof of a repair.

To unblock a repair, obtain the FIRST real rejected fallback from the original failing session or a separately authorized new bounded reproduction, recording action/epoch/source fingerprint, actual start_clock/now_clock/predicate, native clock, simulator state and ROS executor stack before teardown. Stop before any unresolved resend. If no such session/trace exists, agree a new explicit horizon/reset reproduction budget rather than unrestricted2048-step training. Do not implement a timeout, polling or architecture change based solely on the six accepted cases.

---

# Task 2C checkpoint

## Current task
TASK 2C — One full 2048-timestep PPO acceptance run

## Status
IN_PROGRESS — prepared; training not yet invoked

## Preparation / configuration
- Read handoff and Task2B SUMMARY.md; rechecked train_ppo.py. Original parameters retained: PPO MultiInputPolicy, n_steps256, batch64, n_epochs10, learning_rate3e-4, gamma.99, gae_lambda.95, clip.2, ent.005, vf.5, grad.5, pi/vf[128,128], seed11, domain61, max90 simulated seconds, original shield/reward/observations and50*.004=.2 action timing.
- Requested run directory did NOT exist. Created NEW `/home/thaonhi/nav_ws/dynamuh_gym_adapter/runs/empty_acceptance_2048/` exclusively; earlier run/log/checkpoint directories remain untouched. manifest.json fingerprints all production sources/config and the one pre-existing model zip.
- New evidence root: `/home/thaonhi/nav_ws/dynamuh_gym_adapter/runs/worldcontrol_debug/20261031_193008_task2c_d99ef9/`. See manifest.json, train_ppo_source_snapshot.txt and run_acceptance_once.sh. Invocation-directory and trace-file exclusive guards prevent reruns. No full training has been launched yet.
- Previous runtime163s/256 steps projects~1304s for2048 plus restarts; reasonable bound set2100s external (35min) +20s force grace,2050s internal alarm for cleanup. No timeout/hyperparameter changes in production.

## Tracing / validation plan
- Reused actual-method Task2B tracing class in a separate optional codex_acceptance_trace.py. Guarded sitecustomize activates ONLY for main script argv0=train_ppo.py and CODEX_WORLD_CONTROL_TRACE=1; ROS child Python processes and artifact verification are excluded. Syntax checks passed.
- It calls ORIGINAL _gz/subprocess.run and _world_steps and traces actual fallback branches, retains raw CLI/native/process/service logs, and stops before an ambiguous resend if first fallback rejects. No changed successful/fallback-accepted behavior and no synthetic success data.
- Wraps original Gym reset/step to record observation validity/finiteness, reward finiteness and actual sim_dt. Wraps original PPO.learn/logger.dump to retain actual counters, optimization count, losses and runtime; no replacement training loop, callbacks or PPO parameters. Actual train_ppo.py main will generate its own standard checkpoints.
- Failure capture in diagnostic close runs BEFORE original train finally tears down simulator, retaining traceback, executor Python thread stacks, first failed actual predicate, native/ROS clocks and server state. A safety stop is explicitly distinct from original SimulatorFault; ambiguous second/third requests are not executed.
- Preflight will check ai12 imports, installed launch/plugin packages and CLI locations without another simulator run. Dynamic readiness will be confirmed by the actual acceptance run's first advertised service + valid reset snapshot, avoiding redundant earlier smoke tests.

## Exact single training command / next action
- `python3 train_ppo.py --scenario empty --timesteps 2048 --run-dir runs/empty_acceptance_2048`, invoked ONCE by run_acceptance_once.sh under external timeout2100s with ai12/workspace sourced. stdout/stderr separate in evidence root, exit/runtime written to exit_status.json.
- Execute that script once outside sandbox (Gazebo transport needs host interfaces); follow evidence-root progress.json. After success verify actual2048 counter, final optimization, all step validity/clock metrics, timeout/fallback counts, expected zip artifacts can load and finite/updated parameters. After failure stop, preserve evidence, DO NOT rerun. No speculative repair or further investigation unless actual failure occurs.

### Task 2C checkpoint — single acceptance command running
- run_acceptance_once.sh invoked ONCE. Preflight PASS: explicit ai12 Python, rclpy/numpy/gymnasium/SB3 imports, installed custom_corridor/custom_corridor_plugins and actual gz/ros2/timeout executables. No separate simulator smoke repeated.
- Actual training PID106399, trace_ready2026-10-31T12:42:08.689622Z; argv exactly train_ppo.py --scenario empty --timesteps2048 --run-dir runs/empty_acceptance_2048. Trace source fingerprints match baseline. Original main prints CPU/DummyVecEnv startup.
- First simulator service discovery/WorldControl bootstrap and valid/fresh Gym reset passed (env_reset recorded, all observation validity/finiteness counters zero errors). Dynamic readiness confirmed in the actual acceptance session.
- Continue following evidence-root progress.json and train.stdout/train.stderr. Tool exec session13034 is the RUNNING acceptance launcher. Do not invoke run_acceptance_once.sh or training again; invocation_guard and training_invoked.json mark the single run. External2100s/internal2050s limits apply.

### Task 2C checkpoint — first episode boundary passed
- As of2026-10-31T12:47:06.611746Z, acceptance remains RUNNING at480/2048 validated steps (~299s elapsed),2 valid resets,9 native CLI timeouts and9 actual accepted fallbacks;0 rejected fallbacks/validation errors. The first episode completed and automatic reset into a second owned simulator session succeeded.
- Every recorded observation/reward is finite/valid; sim_dt remains .2 within floating-point precision; all recovered requests used actual source fallback. No retry or speculative investigation started.
- Keep following the same single command/session13034 and evidence-root progress.json; do not launch another run. Expected artifacts must be verified only after this command exits.

### Task 2C checkpoint — acceptance FAILED; no rerun
- The one acceptance invocation exited1 at2026-10-31T12:52:33.960600Z after626.3708s command wall time (~10m26s), BEFORE2048.982 validated environment steps / PPO num_timesteps982; _n_updates30 (three completed256-step update phases,10 epochs each). Three valid resets; no observation/reward/sim_dt validation errors before failure.
- Actual exception is `core.SimulatorFault: Timed out awaiting fresh /clock, /scan, /odom and /imu`, from real_backend.py:160 (_snapshot) called at230 after WorldControl. This is DIFFERENT from original _world_steps:134 exception. Complete traceback in train.stderr and structured learn_exception/exception_before_close/uncaught events; source/failure-time states preserved before teardown.
- There were12 native acknowledgement timeouts and12 accepted real clock fallbacks,0 rejected fallbacks/0 retry guards/0 terminal line134 raises. The final failed environment action is separate from those recovered timeouts. Conditional analysis of its actual request, clock/sensor and process state is now authorized; no speculative change or additional training run.
- capture/result.json and exit_status.json are durable. Session13034 has ended exit1. DO NOT invoke the acceptance command again. Expected final/checkpoint2048 artifacts cannot be assumed; verify only what exists.

---

# Task 2C final handoff

## Current task
TASK 2C — One full PPO2048 acceptance attempt

## Status
FAIL — single attempt stopped; no rerun

## Command / runtime / completed workload
- Executed exactly ONCE: `python3 train_ppo.py --scenario empty --timesteps 2048 --run-dir runs/empty_acceptance_2048` with original training code/parameters. New run directory and unique evidence directory; no older run directory reused by this command.
- Evidence root: `/home/thaonhi/nav_ws/dynamuh_gym_adapter/runs/worldcontrol_debug/20261031_193008_task2c_d99ef9/`. Read SUMMARY.md, acceptance_report.json and failure_evidence.json first.
- Command exit1, runtime626.370802853s (~10m26s), exit recorded2026-10-31T12:52:33.960600Z. External2100s/internal2050s limits did NOT expire.
-982 successful Gym transitions / actual PPO num_timesteps982;983rd action failed before returning a transition. _n_updates30: three completed256-step rollout optimization phases at10 epochs each; fourth rollout incomplete. Full2048 acceptance criterion not achieved.
- Three valid resets, two completed episodes:450 steps/time_limit/reward-4.330997;358 steps/goal_reached/reward23.994872. Current epoch3 failed at zero-based global action982. No claims about physical safety or long-term learning quality.

## Validation results
- All982 returned observations were in observation_space and finite; all rewards finite. Actual delta range.19999999999998863-.20000000000000284s; validated policy time196.4s. Zero accepted incorrect deltas, zero silent skipped failed steps.
-986 WorldControl calls:974 Boolean acknowledgement returns and12 native CLI timeouts recovered by ACTUAL source fallback.12 accepted/0 rejected fallbacks,0 ambiguous retry guards,0 line134 terminal raises. No WorldControl resend occurred.
- All12 timeout comparisons had existing baseline/current ROS clocks, exact .2s advancement within floating-point precision and predicateTrue before actual return133. User's original terminal WorldControl-rejected exception still NOT reproduced.
- Available PPO losses are finite. Last dumped metrics (at768 transitions, before third update): loss.2521404028, policy_gradient_loss-.0221162069, value_loss.8265143186, approx_kl.0110105835, entropy_loss-4.2495162010, learning_rate.0003. This dump shows n_updates20; actual live failure counter30 records the completed third optimization afterward.

## Actual SimulatorFault and exact failure branch
- Full traceback: evidence-root train.stderr; structured learn_exception/exception_before_close/uncaught in capture/events.jsonl. Exception is `core.SimulatorFault: Timed out awaiting fresh /clock, /scan, /odom and /imu` at real_backend.py:160, via _snapshot at advance230 and env.step75. This is DIFFERENT from _world_steps134.
- Last request989, action982, epoch3: same pause:true multi_step:50 request, rc0 / `data: true`, .346091658s. Native and ROS clock35.0->35.2; actual50-tick world advancement completed. Thus attempt0 acknowledged; fallback not entered for this request, attempts1/2 not executed. There is no rejected fallback or raising WorldControl branch to identify in this failure.
- IMU timestamp rolled backwards during this action: previous35.0, then.004 at the WorldControl reply, finally.02. At snapshot failure clock35.2, scan35.2, odom35.172, IMU.02. Computed ages: scan0, odom.028, IMU35.18s. core.py:95-114 requires max_scan_age.26 / max_odom_age.16 / max_imu_age.26s. The recorded IMU age alone suffices to make the snapshot unacceptable.
- _snapshot136-160 catches require_fresh SimulatorFault internally and continues until20s step_timeout; invalid IMU stamp persisted throughout its wait and failure observation interval. The final generic error omits the per-sensor cause, but raw recorded stamps establish the freshness violation. No invalid observation was substituted or accepted.
- At failure launch143004 and Gazebo143015 remained alive. Control provider tcp://127.0.0.1:45909 still advertised; read-only /gazebo/worlds returned corridor_0.90 in.280s. Python executor stack was present in rclpy wait/spin, with no recorded global executor death. Native clock capture in session3 shows NO backwards jump. Scan/odom/current clock callbacks continued to update for the failed action.
- Session3 simulator logs also record robot_state_publisher `Moved backwards in time, re-publishing joint transforms!` warnings around12:52:06Z. These are supporting timestamp-anomaly observations; their exact message source/causal role is not proven.

## Artifact verification
- runs/empty_acceptance_2048/untrained.zip exists,316760 bytes; ZIP CRC/load PASS with ai12 SB3, expected Dict observation/action spaces, finite policy/optimizer state, num_timesteps0/n_updates0. See model_verification.json.
- final.zip and checkpoints/ppo_2048_steps.zip were NOT GENERATED because run ended at982. No final trained-model load verification is possible. Overall model-artifact acceptance FAIL due missing completion artifacts, not corruption of untrained.zip.
- config.json, monitor.csv and two episode_metrics.jsonl rows retained. PPO production sources and related launch/world/bridge hashes all unchanged. No production code changes, packages installed, timeout increases or hyperparameter changes.

## Preservation / concurrent-writer observation
- At preparation only one older model zip existed: runs/empty_smoke/untrained.zip. Its baseline SHA256 was a18ece9e64c7d7e9eacfdf8371ebe5cb9d644431dcc6e896a038c9a8044d2cb8. After failure it was66cc1a6dd0a9c336934a9953dfce3120b1a36dea009cc1a35f6c4a024e50c1bb, with mtime2026-10-31T12:52:03.347889Z, ~3s before the IMU rollback.
- Our literal argv/config and scripts write only the NEW acceptance/evidence paths and do not write runs/empty_smoke. This indicates an unattributed concurrent writer; no original contents were restored or overwritten by this task. Therefore byte-level preservation of that old artifact could not be certified; the hash discrepancy is retained in prior_model_change_observation.json / acceptance_report.json rather than hidden.
- A later read-only process inventory did not identify the writer and was not contemporaneous with the failure. This does NOT prove which process sent the IMU data, or that another simulator was active. Native IMU messages and ROS IMU publisher GIDs were not captured. Concurrent activity/shared-domain interference remains a hypothesis, not a confirmed root cause. Current /clock publisher introspection alone was insufficient to identify IMU provenance.
- All older Task1/2/2A/2B diagnostic directories remain retained and were not reused. No unrelated simulator process was killed or altered.

## Cleanup / files generated
- Post-grace bounded identity-checked cleanup audit: capture had NO live owned children; no audit signals sent. See cleanup_check_130223_375785.json. Normal backend close signalled only its own launch groups. No training command rerun.
- New files are confined to this handoff, runs/empty_acceptance_2048 and unique evidence root. Key files: manifest.json; run_acceptance_once.sh; invocation_guard/training_invoked.json (single-run evidence); sitecustomize.py/codex_acceptance_trace.py (optional external tracing); preflight stdout/stderr; train.stdout/train.stderr; exit_status.json; progress.json; capture/result.json/events.jsonl/per-session native+launch logs; python_threads_at_failure.json and native thread snapshots; failure_evidence.json; acceptance_report.json; model_verification.json; prior_model_change_observation.json; post_failure_process_inventory.json; SUMMARY.md.
- Verification scripts ran once against generated files only. Their nonzero results reflect failed acceptance/missing final artifacts, not another training invocation.

## Remaining risks / exact next action
- Stop Task2C here. Acceptance FAIL; no repair implemented and no automatic second run authorized.
- Confirmed blocker for THIS run is IMU timestamp rollback/freshness failure after successful world advancement. Underlying timestamp origin is unconfirmed and the original WorldControl-rejected exception remains unproven. Do not weaken require_fresh, fabricate IMU data, replay already executed world steps or increase timeout.
- If a separate continuation is authorized: capture native IMU header stamps and ROS /imu publisher identities/GIDs at arrival, correlate with clock and episode identity, and establish exclusive ROS-domain/run-directory ownership before a NEW acceptance attempt. Distinguish native sensor timestamp reset, bridge timestamp conversion and cross-publisher/old-episode messages using actual evidence. Do not assume the prior model rewrite caused the IMU anomaly without provenance.
