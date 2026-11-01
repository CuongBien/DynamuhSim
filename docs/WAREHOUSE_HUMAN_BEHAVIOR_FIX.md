# Warehouse Human Behavior Fix

Date: 2026-11-01. **Overall status: PARTIAL.** Production Python source was changed and regression tests executed. No commit, staging, push, branch switch, reset, stash, checkout or PR was performed.

## 1. Executive Summary

P1 is implemented: newly prepared captures default to the existing `sim_v1` clock, in both CLI and Python API. Explicit `native_wall` and absent-clock worker semantics remain available. Native arrival geometry and strict age `>50 s` are unchanged.

P2 implements a planning-interval motion contract for the maintained walking-group fixture: previous executed segment velocity, heading-aware preference, configurable vector acceleration/braking budgets inside safety projection, and final-velocity yaw. Physical speed, yaw-rate, human-human, robot and AABB constraints remain. Emergency reachability relaxation is explicit in telemetry and retains independent swept safety validation. This is **not a continuous physical acceleration guarantee**: the unchanged adapter still executes piecewise-linear segments.

P3 adds optional native solver telemetry without changing the production group solver. Group-intent instability remains unresolved. Matched replay improved heading/travel consistency and interval acceleration, but **increased total yaw, yaw sign switches and maximum group spread**, and reduced robot clearance while keeping it positive. These are material adverse findings, not a successful naturalness acceptance.

107 unit/component/integration tests PASS. Three fresh native Gazebo captures were attempted (including a smoke on final source); all timed out externally with exit 124 and runner `status=ERROR`. Runtime status is **INCOMPLETE**, never PASS for full episodes. No full benchmark or training was run.

## 2. Baseline Evidence and Scope

Both existing reports were read before editing: `docs/WAREHOUSE_HUMAN_BEHAVIOR_AUDIT.md` and `docs/WAREHOUSE_HUMAN_RUNTIME_VALIDATION.md`. Neither was modified. Historical runs in them are interrupted diagnostic runs, not accepted captures.

Target: `/home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/depth_identity_guard/generated/generalization_worktree`, registered branch `feat/depth-detector-generalization`; initial HEAD `51fc28d2f8dc46890c3dec806cfec1398c7f8018`, initial target status empty. Every repository read/write/test used remote `dynamuhsim-wsl.execute_bash` with `auto_source_ros=false`. No local checkout/shell or SSH was used.

Actual pipeline: capture runner → group worker/fixture → native SocNavGym lifecycle/group ORCA and smoothing/noise → ResponsiveMotion per-person ORCA → convex safety projection → response adapter → Gazebo proxies/actors. Planning dt is 0.1 s; physics dt 0.0025 s; population six, two initial triplets. ComplexityNav is not active. The legacy RVO2 backend, World Model, PPO/training, labels, perception and Nav2 tuning were not edited.

Prior evidence: native-wall head-on expired before geometric arrival, 9.42 simulated seconds after acceptance; crossing with sim_v1 still turned substantially without retargeting; native yaw was captured before downstream correction; adapter interpolation has velocity jumps. Prior maximum interval vector acceleration 7.463 m/s² and heading error over 60° were observations, not acceptance thresholds. They are not used as a matched baseline for new runtime runs.

Evidence directory, abbreviated **B** below:

`experiments/social_navigation/benchmark/generated/warehouse_behavior_fix_20261101`

It is a new ignored diagnostic directory. It retains the initial implementation plan, initial Git receipts, saved baseline motion/projection source, baseline and candidate JSONL, analyzer scripts, test logs, captures, raw pose/worker/adapter logs, MCAP and measurements/hashes. Existing ignored builds and previous diagnostics were preserved.

## 3. Root Causes Addressed

| Cause | Evidence and disposition |
| --- | --- |
| Host expiry while lifecycle is simulation-gated | Confirmed integration mismatch; corrected default selection for new captures using existing sim_v1 implementation. |
| Native yaw precedes final displacement correction | Confirmed integration defect; moving yaw now targets final projected travel, retaining yaw-rate limit. |
| No prior executed velocity in projection | Confirmed missing temporal constraint; optional convex delta-v discs now intersect existing safety constraints. |
| Group envelope/initialization versus individual intent | Native zero initial velocity and radius were freshly observed; causal attribution remains unresolved. Native smoothing, per-person noise, turn-dependent speed scaling and downstream feedback are competing mechanisms. |

## 4. Source Code Changes

| File / function | Final change and rationale |
| --- | --- |
| `tools/capture_episode.py:main` | Default `--human-goal-clock=sim_v1`; help documents explicit historical native_wall choice. |
| `python/dynamuhsim/runtime/capture.py:prepare_run` | API default sim_v1; existing asset creation, hashing and copying reused. |
| `python/dynamuhsim/runtime/nav2_trial.py:source_files` | Freeze new motion helper and group motion configuration; Python diagnostics module is included by existing package traversal. |
| `experiments/social_navigation/navigation/socnavgym_group_nav2/fixture.py:make_kernel` | Load/validate explicit motion budgets on the group motion instance. Reactive-standing override remains unchanged. |
| `.../socnavgym_group_nav2/motion_constraints.json` (new) | Acceleration 0.8 and braking 1.6 m/s², configurable engineering settings. |
| `.../socnavgym_response_fix/motion_contract.py` (new) | `validate_limits`, `preferred_velocity`, `velocity_change_limits`, `final_heading`: heading-aware preference, vector delta-v budget selection and final-velocity heading. |
| `.../socnavgym_response_fix/responsive_motion.py:integrate` | Use executed prior segment velocity, solve reachability with safety, derive yaw after projection, record per-person stage telemetry. Unconfigured callers retain legacy motion/yaw clamp. |
| `.../socnavgym_response_fix/projection.py:_project,project` | Optional prior-velocity discs in Dykstra; validate final residual and swept gaps; report safety-only fallback. Existing positional API preserved. |
| `python/dynamuhsim/simulation/group_diagnostics.py` (new) | `GroupDiagnostics`, `_RecordedSimulator`: instance-local forwarding instrumentation of native RVO input/output. |
| `.../socnavgym_group_nav2/worker.py` | Explicit environment opt-in for native solver telemetry. No native motion-profile default changed. |

No C++ source was modified. No adapter rebuild was needed. Reused adapter source SHA256 is `ffbcda748dc1dba4e06beb79c6e8fa93c77da25fbe2348b10f92062c36279abb`; binary SHA256 `524fe8e945daad94319d7ac1a5f6377a5f36cbf6c505bb580e50e2e73102e984`, matching the previous verified build. Fresh execution exercised that binary.

## 5. Clock Fix Implementation

Before: CLI and `prepare_run` defaulted to native_wall, omitting the clock asset. `load_goal_clock` returned None; native `initial_time` was host time from assignment, including warmup, while the worker only advanced humans after the navigation gate.

After: new CLI/API captures write/hash `assets/human_goal_clock.json` and declare sim_v1 in the plan by default. `CaptureContext.prepare` copies it into runtime. The group worker installs `GoalClock` after initial encounter/profile initialization and before publishing readiness. The first active request initializes initial assignment times to the accepted navigation gate. Later native `set`/`set_goal` calls retain native side effects and reset assignment time to the current simulation step. Expiry is evaluated at step start; snapshot state time is step end. Pauses do not advance simulation/lifecycle time. Pre-acceptance waiting leaves the clock inactive.

No clock algorithm/vendor/global time function was rewritten. Strict expiry `age > 50.0`, geometric `distance < offset + goal_radius`, static-human exclusion and native all-members group acceptance remain. The initial triplet geometric threshold remains 1.80 m (0.36 + 1.44), not a new threshold.

Compatibility: new capture defaults intentionally change timing. Reproduction clients must explicitly pass `--human-goal-clock native_wall` or `human_goal_clock="native_wall"`. A fresh native-wall compatibility prepare succeeded with no clock asset; both default prepares contained the sim clock asset. Absent asset still preserves native methods, including standalone group/Nav2 runs outside this capture default. Historical plans/configs/results were not rewritten. Existing source-freeze hashes appropriately reject changed source; old artifacts were not relabeled.

Clock regressions PASS: different synthetic host schedules on identical contiguous simulation timelines; pre-gate waiting and paused observations; 50.0 false/50.1 true; reached and unreached geometry; multiple humans in a triplet; native setters, native-wall absence and explicit selection; clock asset ownership; independent trace reconstruction. Native head-on additionally recorded geometric assignments and zero expired checks.

## 6. Motion Consistency Fix Implementation

Before: native tentative XY was capped and fed to downstream ORCA; projection enforced speed/safety only; final XY and stale native target yaw were applied separately. Preferred/native speed could not establish prior executed velocity.

After, for configured ResponsiveMotion:

1. `self.velocities` contains the previous final projected displacement divided by native dt. This is the velocity of the straight segment the adapter executes, not native preferred velocity. Initial/gated/rest state is zero. The adapter completes contiguous intervals; traces validate endpoint interpolation against Gazebo.
2. Preference turns toward native displacement within the existing yaw budget and applies cosine braking for remaining heading error. This is a soft preference; lateral/backward safety sidesteps remain possible. No nonholonomic constraint is imposed on all humans.
3. Post-ORCA desired velocity is projected jointly against existing speed/support planes and a per-person disc `||v - v_previous|| <= budget * dt`.
4. Settings are explicit in `motion_constraints.json`. 0.8 m/s² corresponds to one second from rest to the existing 0.8 m/s speed cap; 1.6 m/s² corresponds to half-second braking. They are **testable engineering choices, not empirically validated human limits**. The braking vector budget is selected when desired delta-v opposes prior velocity (negative dot product); otherwise the acceleration vector budget applies. These describe interval vector changes, not independently validated continuous positive/negative physical accelerations.
5. All existing margins (0.04, 0.02, 0.002 m) are tried with reachability. If rejected/nonconvergent, safety-only projection retries the same margins and exposes `motion_limit_relaxed`, `motion_constraints_satisfied`, actual per-person interval acceleration, budgets, attempts/reasons and `limiting_condition`. Numerical rejection is not proof that a mathematical intersection is empty. Max speed and independent swept clearance are never relaxed. If no safe bounded result is validated, the existing terminal worker error/adapter interlock remains.
6. Moving yaw targets final displacement with the existing 1.57 rad/s cap. At zero velocity it retains prior heading; native stationary/conversation facing remains. A safety correction can exceed attainable heading alignment, explicitly logged as start/end heading/travel error. No XY smoothing or rotation is applied after safety validation.

Telemetry in `motion_guard.motion_contract` records previous executed velocity, native proposal after existing speed/static caps, heading preference, post-ORCA velocity, final velocity, final target yaw, and heading errors. `motion_guard.projection` separately records the safety correction and feasibility. Worker endpoints are adapter-requested future state; ground_truth is adapter-applied state; gazebo_poses is observed proxy state. These are not interchangeable.

Scope/limits: fixture enables this for the maintained walking-group ResponsiveMotion path, including native lifecycle transitions to stationary groups. The dedicated ReactiveStandingMotion implementation is unchanged. Unconfigured ResponsiveMotion callers retain original preferences and yaw cap (verified by unit regression and zero-error native replay). The adapter still interpolates XY/yaw independently along the already validated straight segment. **Its velocity can jump at boundaries; continuous physical acceleration and zero heading error throughout a turn are not guaranteed.** Implementing curved/velocity-continuous execution needs a separately validated swept-trajectory contract with robot prediction; no unsafe post-projection spline was added.

## 7. Group-Intent Investigation and Changes

Native `compute_orca_interaction_velocities` creates a fresh simulator, inserts group radius `INTERACTION_RADIUS + HUMAN_DIAMETER` and does not initialize group velocity. Fresh baseline instrumentation observed 240 group proposals: every solver input speed zero, envelope radius approximately 1.440000057 m. For the initial equilateral triplet the actual body envelope is 1.08 m. This establishes an abstraction difference, not physical collision or unique causation.

Native `Human_Human_Interaction.update_speed` smooths prior group velocity, adds independent Gaussian noise and personal scaling, applies per-human heading limits, and reduces all member speeds when any member is still rotating. These mechanisms can conflict with subsequent per-person ORCA/projection and feed back into future group state. No unique mechanism was isolated in this task.

With `DYNAMUHSIM_GROUP_DIAGNOSTICS=1`, legacy worker records membership, current goal, center, actual solver radius, preferred velocity, solver input velocity and post-ORCA velocity. Instrumentation clones only the bound method's globals to substitute a forwarding simulator factory. It does not change vendor source, process-global RVO, random draws or solver operations. Native replay with/without telemetry reproduced human states exactly (maximum error zero). Other explicit profiles are rejected with a clear error when this legacy-only diagnostic opt-in is requested.

`group_orca_v1`, `group_goal_v1`, `group_cohesion_v2` were inspected; their existing regressions PASS. No candidate was promoted. In particular goal/cohesion candidates disable upper group avoidance and cannot be promoted from historical scalar summaries. Production native group solver and motion-profile default remain unchanged. Matched baseline had six raw same-goal direction flips over 90° (speed >0.05 m/s); candidate had zero, but candidate changes human state through the downstream contract. This is not an isolated group-solver treatment or proof P3 is fixed. Radius-only/initial-velocity-only causal ablations remain NOT RUN.

## 8. Tests Added or Updated

- Updated `tests/test_goal_clock.py`: host-schedule/gate/pause/group boundary regression and CLI/API defaults plus explicit native-wall selection. Existing geometry/static/setter/freeze/population tests retained. Nine tests PASS.
- New `tests/test_final_motion.py`: straight acceleration, braking, 90°/180° preference changes, safety-induced lateral/backward correction, robot braking/direction change, near-zero/static facing, moving-to-static emergency braking, narrow AABB clearance, head-on human swept pair safety, terminal infeasibility, legacy projection API/config validation, and unconfigured ResponsiveMotion yaw/speed compatibility. Nine tests PASS.
- New `tests/test_group_diagnostics.py`: exact forwarding of solver input/output and return, current membership/goal, no global/class-method mutation. One test PASS.

Early fixture errors (empty AABB list, incorrect gap dictionary key) were corrected; final tests use the existing geometry API. Final source preserves the legacy yaw clamp and has a dedicated regression. No safety failure was suppressed to make a test pass.

## 9. Unit and Component Test Results

Final command used a unittest loader with `PYTHONPATH` equivalent `python,tests`, `sys.dont_write_bytecode=True`; detailed log: `B/component_tests_verified.log`.

| Verification | Status | Result |
| --- | --- | --- |
| Relevant unit/component/integration suite | PASS | 107 tests, 0 failures, 0 errors in 3.026 s; detailed final log retained. |
| Clock regressions | PASS | 9 tests; default/explicit selection also exercised by fresh real prepare operations. |
| Motion regressions | PASS | 9 tests; physical continuous acceleration not asserted. |
| Group regressions | PASS | Existing group ORCA/goal/cohesion suites plus new telemetry test included. |
| Modified/new Python compilation | PASS | 12 Python files compiled in memory without bytecode output. |
| Baseline reproducibility | PASS | Two unmodified-source native replays; maximum human state error zero. |
| Telemetry and legacy compatibility replays | PASS | Each 120 steps; maximum human state error zero versus baseline. |
| Source/asset freeze of final native run | PASS | `capture_episode.py verify` after final execution. |
| C++ rebuild | NOT RUN | No C++ changed; existing verified binary executed. |
| Continuous physical acceleration guarantee | NOT RUN | Not implemented; observed sample accelerations exceed interval budget. |
| Group causal mechanism/default treatment acceptance | INCOMPLETE | Downstream feedback and group mechanisms remain unseparated. |

The suite includes goal_clock, final_motion, group_diagnostics, group_projection_regressions, group_orca, group_intent, group_cohesion, group_cohesion_heading, capture_runtime, nav2_runner_integration, capture_audit, nav2_runtime, encounters and persistent_conversation.

## 10. Gazebo Runtime Validation

Reused the already installed main-checkout vendor/venv only through two temporary symlinks at previously absent target paths. No existing path was overwritten. Existing target response adapter, depth filter and friction checker were reused; no package installation or full rebuild. Both session-owned symlinks were subsequently removed with exact target checks. External installations, existing builds and previous evidence were left intact.

No active Gazebo/ROS trial session was present before launches. The supported runner held `common_gazebo/generated/domain91.lock`, domain 91 with LOCALHOST discovery and a unique PID-based Gazebo partition. Runs were sequential. Each used `timeout --signal=INT --kill-after=20s 60s`, new capture directory, scenario walking, seed 45, legacy group profile, native lifecycle and default sim_v1; config copied from moderate_v3 changing only sim_budget_s to 30.0. No invented human-count selector or scripted robot commands were used.

| Run | Executed pose sim window s | Active worker steps / fully executed intervals | Runtime status | Outcome |
| --- | --- | --- | --- | --- |
| head_on_candidate | 14.1225–30.0700 (15.9475 s) | 159 / 158 | INCOMPLETE | exit124, result ERROR; 320 clock checks, 3 assignments, 0 expired checks. |
| crossing_candidate | 13.1950–30.1350 (16.9400 s) | 169 / 168 | INCOMPLETE | exit124, result ERROR; 338 clock checks, no assignments/expiry. |
| crossing_final | 13.3625–30.7075 (17.3450 s) | 174 / 173 | INCOMPLETE | final source, exit124, result ERROR; 348 clock checks, no assignments/expiry. |

The first two runs preceded the final compatibility-only yaw clamp restoration for unconfigured callers. Final configured replay state was identical to first candidate (max difference zero); crossing_final exercised the final source bytes. These separate closed-loop runs are not paired A/B trials.

All three: real native worker, six humans, Gazebo advancing clock/poses, accepted Nav2 gate, no worker error, no adapter fault, Gazebo clean exit and ownership cleanup PASS. Clock trace reconstruction PASS. Zero relaxed/failed motion-constraint steps among fully executed intervals. Timeout/KeyboardInterrupt and the existing `rcl_shutdown already called` teardown error are preserved in results/consoles; clean owned-process cleanup does not upgrade episode status. Full goal completion/capture acceptance: NOT RUN.

Final crossing observed 348 raw group proposals. Gazebo proxy XY matched same-time worker straight-segment interpolation to max 7.9441e-15 m. This ties requested, adapter-applied and Gazebo-observed pose streams together. It does not measure articulated-body dynamics, full robot mesh clearance or social comfort.

## 11. Before/After Measurements

**Controlled comparison:** baseline_diagnostic.jsonl versus combined_final.jsonl, fresh native component replays, seed45, crossing fixture, same fixed initial native state and same 120 recorded robot inputs from previous crossing_sim45, dt0.1 s, [0,12] active simulation seconds, six humans / 720 human intervals. Both use sim_v1; the unmodified baseline already had this explicit opt-in. Thus this comparison isolates the motion-contract treatment, not the clock default change. Baseline was first reproduced twice before edits, then exactly again with telemetry. Motion-only/native-wall replay produced the same measured summaries as combined in this short no-expiry window; host-schedule clock causality is established by focused regressions, not by claiming equivalence past timeout.

Velocity is endpoint displacement/dt, acceleration is vector delta-v/dt including known initial rest. Yaw totals sum absolute wrapped endpoint changes. Moving heading error uses speed >0.1 m/s and the midpoint of the adapter's shortest-angle yaw interpolation. Stop uses speed <0.01 m/s. Sign-switch count excludes yaw rates at or below 0.1 rad/s; travel switch excludes adjacent speeds at or below 0.1 m/s. These are measurement definitions, not new pass thresholds. No manually invented naturalness score.

| Metric | Baseline | After Fix | Unit | Scenario/Window | Evidence | Interpretation |
| --- | ---: | ---: | --- | --- | --- | --- |
| Maximum interval vector acceleration | 5.859196 | 1.600000 | m/s² | matched crossing, 12 s | B/replay_measurements.json | Temporal segment contract improved; not physical continuous acceleration. |
| p95 interval vector acceleration | 1.690442 | 1.143731 | m/s² | same, 720 samples each | same | Lower upper distribution. |
| Maximum vector velocity jump | 0.585920 | 0.160000 | m/s | same | same | Smaller boundary jumps, still discontinuous. |
| Maximum scalar speed jump | 0.508858 | 0.160000 | m/s | same | same | Lower speed discontinuity. |
| Moving midpoint heading/travel error p95 | 36.532230 | 4.966589 | degree | same, 569 / 653 moving samples | same | Better consistency. Moving populations differ by treatment. |
| Moving midpoint heading/travel error maximum | 62.088207 | 16.992477 | degree | same | same | Residual turning/avoidance disagreement remains. |
| Final endpoint heading/travel error p95 | 36.747953 | 0.468871 | degree | same | same | Final heading uses projected travel. |
| Maximum speed | 0.800000 | 0.486234 | m/s | same | same | Lower progress speed; not automatically better. |
| Maximum angular velocity | 1.570000 | 1.570000 | rad/s | same | same | Existing cap preserved. |
| Total absolute yaw, all humans | 50.559937 | 63.878092 | rad | same | same | **Worsened.** No overall smoother-motion acceptance. |
| Yaw sign switches | 135 | 283 | count | same | same | **Worsened.** Definition above. |
| Travel direction switches >90° | 0 | 0 | count | same | same | No eligible abrupt travel reversal. |
| Minimum predicted swept HH surface gap | 0.283075 | 0.372331 | m | same | independent geometry on segment proposals | Positive; model-based physical separation. |
| Minimum predicted swept HR surface gap | 0.557605 | 0.145577 | m | same | same | **Reduced**, still positive; no social comfort claim. |
| Minimum predicted swept static gap | 0.342950 | 0.096067 | m | same | same | Reduced clearance, still positive. |
| Predicted swept clearance violation intervals | 0 | 0 | count | same | same | Existing 2mm validation floor retained. |
| Stop duration summed over humans | 1.9 | 1.0 | person·s | same | endpoint segment speed | Lower stop exposure; no deadlock inference. |
| Longest individual stop | 0.2 | 0.1 | s | same | same | Short window only. |
| Stuck duration | NOT MEASURED | NOT MEASURED | s | same | no stuck classifier | No inferred deadlock result. |
| Goal retargeting | 0 | 0 | count | same | recorded group goals | Clock expiry not exercised by short replay. |
| Goal/episode completion | NOT MEASURED | NOT MEASURED | — | same | no navigation episode in replay | NOT RUN. |
| Maximum member-centroid distance | 1.073733 | 1.454090 | m | same, 720 samples each | recorded current memberships | **Worsened** maximum spread. |
| Projection intervention steps | 1 | 66 | count / 120 | same | projection delta-v norm >1e-9 | Includes intended reachability correction, not only obstacle emergencies. |
| Motion-limit relaxation steps | N/A | 0 | count / 120 | same | projection audit | No emergency relaxation in this replay. |
| Raw same-goal group intent flips >90° | 6 | 0 | count | same, 240 group proposals each | native solver telemetry | Downstream state feedback changed; not isolated P3 causality. |

Additional replay check against successive recorded robot positions (119 complete robot segments, linear interpolation assumption): minimum HH / HR / static surface gaps were baseline 0.283075 / 0.557605 / 0.351688 m and candidate 0.379905 / 0.164855 / 0.108072 m. This differs from constant-velocity prediction and excludes the last future interval.

**Fresh runtime observations, not matched A/B:**

| Metric | Baseline | After Fix | Unit | Scenario/Window | Evidence | Interpretation |
| --- | --- | ---: | --- | --- | --- | --- |
| HH / HR sampled surface gap minima | NOT MEASURED | 0.485154 / 0.277500 | m | head_on_candidate, 15.9475 s | B/runtime_measurements_final.json | Six proxy discs and 0.25m robot disc. |
| HH / HR sampled surface gap minima | NOT MEASURED | 0.340303 / 0.205697 | m | crossing_final, 17.3450 s | same | Positive measured samples, not robot-mesh clearance. |
| HH / HR / static linear-sample swept gap minima | NOT MEASURED | 0.340303 / 0.205697 / 0.040000 | m | crossing_final, 1042 pose segments | same | Assumes linear motion between recorded complete pose samples. |
| Linear-sample clearance violations | NOT MEASURED | 0 | count | crossing_final | same | No claim of unsampled full-mesh contact certification. |
| Maximum executed interval vector acceleration | NOT MEASURED | 1.600000 | m/s² | crossing_final, 173 full human planning intervals | same | Does not include unexecuted last endpoint. |
| Maximum sampled finite-difference acceleration | NOT MEASURED | 11.636364 | m/s² | crossing_final, variable pose cadence | same | **Continuous physical 1.6m/s² bound is not verified.** |
| Maximum moving heading/travel error | NOT MEASURED | 18.810740 | degree | crossing_final, speed >0.1m/s | same | Residual error within independently interpolated intervals. |
| Maximum executed speed / yaw rate | NOT MEASURED | 0.417742 / 1.570000 | m/s / rad/s | crossing_final | same | Existing limits preserved. |
| Total absolute yaw | NOT MEASURED | 86.212011 | rad | crossing_final, 17.3450 s | same | No historical-window causal comparison. |
| Expired clock checks / assignments | NOT MEASURED | 0 / 0 | count | crossing_final | independently verified clock events | Simulation-clock semantics observed. |

The earlier head-on/crossing candidate sample acceleration maxima were 16.000000 / 11.636364 m/s². These cadence-dependent values expose the unchanged adapter's discontinuity, not a verified physical acceleration cap. Full per-human distributions, sample cadence, duplicate timestamps, source/input hashes and all three runtime receipts remain in the measurement JSON.

## 12. Safety and Regression Assessment

Human-human and human-robot ORCA agents, exact Warehouse AABBs, physical speed/yaw bounds and independent continuous **linear-interval predicted** clearance validation remain. No avoidance layer was removed. Safety-only fallback is exercised by focused approaching-robot, constrained-pair and moving-to-static fixtures; requested motion limits can fail and are reported. An impossible robot approach remains terminal. This is physical circle/AABB safety, not personal-space comfort or future robot intent prediction.

Matched replay and three native windows recorded no clearance violations under stated models. Robot/obstacle clearance nevertheless decreased in matched replay. Turning and maximum group spread worsened. These tradeoffs block a claim that Warehouse behavior is fully fixed. Larger distributions, dense encounters, changing robot acceleration and full continuous execution need further validation.

Git safety: target branch/HEAD remained unchanged. Only target source/tests/new report were edited; task commands made no source writes to the main checkout. The main checkout already had unrelated changes. Final snapshots differ from initial snapshots in README.md and tools/build_report_hub.py hashes, and additionally show docs/verification/uniform_scene_2026-11-01/ and tools/render_scene_runtime.py. These paths were never written by this task; they were preserved. Therefore **whole-main-checkout immutability cannot be asserted** in this concurrently changing workspace. Initial/final status and initial hashes are retained in B. Main HEAD remained 34341911a77c230c7102642d2a7dcb514213d31e. No pre-existing change was reset or overwritten.

Final delivery inventory: 8 tracked modified files and 6 new untracked files, including this report. New files remain unstaged:

```text
docs/WAREHOUSE_HUMAN_BEHAVIOR_FIX.md
experiments/social_navigation/navigation/socnavgym_group_nav2/motion_constraints.json
experiments/social_navigation/navigation/socnavgym_response_fix/motion_contract.py
python/dynamuhsim/simulation/group_diagnostics.py
tests/test_final_motion.py
tests/test_group_diagnostics.py
```

Actual `git diff --stat` (Git excludes untracked files from this output):

```text
 .../navigation/socnavgym_group_nav2/fixture.py     |  2 +
 .../navigation/socnavgym_group_nav2/worker.py      |  6 +++
 .../socnavgym_response_fix/projection.py           | 53 ++++++++++++++++++----
 .../socnavgym_response_fix/responsive_motion.py    | 25 ++++++++--
 python/dynamuhsim/runtime/capture.py               |  2 +-
 python/dynamuhsim/runtime/nav2_trial.py            |  4 +-
 tests/test_goal_clock.py                           | 33 ++++++++++++++
 tools/capture_episode.py                           |  2 +-
 8 files changed, 108 insertions(+), 19 deletions(-)
```

`git diff --check` PASS; staged diff empty; branch and target HEAD equal their initial values. No remaining owned Gazebo/ROS trial process was observed at final process inventory.

## 13. Remaining Limitations

- P3 remains unresolved; no radius-only/velocity-initialization-only treatment or statistically repeated causal acceptance. The active native group solver remains unchanged.
- P2 improves an interval contract, not continuous velocity/acceleration execution. Adapter XY and yaw remain independently linear; observed physical finite differences exceed the interval budget. No curved interpolation was implemented without swept safety validation.
- Matched replay shows adverse yaw/oscillation/spread and reduced HR/static gaps. New limits are engineering settings, not validated human physiology. Overall behavioral acceptance: INCOMPLETE.
- ReactiveStandingMotion dedicated sidesteps were not changed. Existing related regressions passed; no new standing native capture was run.
- Native captures did not reach 50 active simulation seconds. Strict expiry at/after that boundary is verified by component regressions, not a long native episode.
- All new native episodes timed out; goal completion and accepted capture: NOT RUN. No quantitative improvement is inferred from non-equivalent historical/native runs.
- Robot clearance is a nominal disc, not full rotating mesh. Pose samples are not every physics tick; linear-sample swept interpolation does not certify every unsampled robot motion/contact. No contact-force or social-comfort validation.
- Main checkout changed concurrently outside task writes; its entire unchanged state could not be certified. No attempt was made to undo those changes.

## 14. Reproduction Commands

All commands below run remotely using MCP execute_bash, auto_source_ros=false, from the exact target worktree. Never replace an existing mapping/run directory. B contains retained evidence; choose a new run directory for another native run.

```bash
cd /home/cuongbien/DynamuhSim/experiments/social_navigation/benchmark/depth_identity_guard/generated/generalization_worktree
pwd
git branch --show-current
git status --short
B="$PWD/experiments/social_navigation/benchmark/generated/warehouse_behavior_fix_20261101"

# Focused suite actually executed (final result 107 tests).
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=python:tests python3 -m unittest \
  test_goal_clock test_final_motion test_group_diagnostics \
  test_group_projection_regressions test_group_orca test_group_intent \
  test_group_cohesion test_group_cohesion_heading test_capture_runtime \
  test_nav2_runner_integration test_capture_audit test_nav2_runtime \
  test_encounters test_persistent_conversation -v

# Read-only Python compilation without creating bytecode.
python3 - <<'PY'
import subprocess
from pathlib import Path
files=set(subprocess.check_output(['git','diff','--name-only'],text=True).splitlines())
files.update(subprocess.check_output(['git','ls-files','--others','--exclude-standard'],text=True).splitlines())
for name in sorted(files):
    if name.endswith('.py'): compile(Path(name).read_text(),name,'exec')
PY

# Recover absent dependencies using the prior report's installed packages.
source /opt/ros/jazzy/setup.bash
source /home/cuongbien/DynamuhSim/install/setup.bash
test ! -e experiments/social_navigation/.venv-socnavgym && test ! -L experiments/social_navigation/.venv-socnavgym && ln -s /home/cuongbien/DynamuhSim/experiments/social_navigation/.venv-socnavgym experiments/social_navigation/.venv-socnavgym
test ! -e experiments/social_navigation/vendor && test ! -L experiments/social_navigation/vendor && ln -s /home/cuongbien/DynamuhSim/experiments/social_navigation/vendor experiments/social_navigation/vendor

# Retained component replay script: treatment first, NEW output stem second.
# Exclusive file creation prevents overwriting an existing trace.
PYTHONDONTWRITEBYTECODE=1 experiments/social_navigation/.venv-socnavgym/bin/python "$B/replay.py" baseline_diagnostic baseline_recheck_NEW
PYTHONDONTWRITEBYTECODE=1 experiments/social_navigation/.venv-socnavgym/bin/python "$B/replay.py" combined_final candidate_recheck_NEW

# New native run: default clock is deliberately omitted to exercise sim_v1 default.
# NEW_RUN must not exist. Diagnostic config already exists in B (30s sim budget).
NEW_RUN="$B/NEW_CROSSING_RUN"
PYTHONDONTWRITEBYTECODE=1 python3 tools/capture_episode.py prepare \
  --run-dir "$NEW_RUN" --config "$B/diagnostic_config.json" \
  --scenario walking --seed 45 --encounter-profile crossing
PYTHONDONTWRITEBYTECODE=1 python3 tools/capture_episode.py verify --run-dir "$NEW_RUN"
timeout --signal=INT --kill-after=20s 60s env PYTHONDONTWRITEBYTECODE=1 \
  DYNAMUHSIM_GROUP_DIAGNOSTICS=1 python3 tools/capture_episode.py run --run-dir "$NEW_RUN"
# Actual head-on command used the same flags with --encounter-profile head_on.
# Native-wall compatibility prepare added --human-goal-clock native_wall.

# Recompute derived metrics only; raw inputs are unchanged.
PYTHONDONTWRITEBYTECODE=1 python3 "$B/analyze_replay_final.py"
PYTHONDONTWRITEBYTECODE=1 python3 "$B/analyze_runtime_final.py"

# Check every link target before removing ONLY links created by your session.
# Do not delete existing binaries, logs or the external dependency targets.
git diff --check
git status --short
git diff --stat
git diff --name-only
git ls-files --others --exclude-standard
```

No source-generation workaround, system installation, full benchmark, branch operation, git add, commit or push is part of this procedure. If the existing binary is absent on another machine, use the focused build/recovery commands in the prior runtime report, preserving existing paths.
