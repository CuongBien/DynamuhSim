# Capture-owned Gazebo bridge stop ordering

The preceding persistent-conversation cohort passes all three seeds' body/membership/snapshot-clock checks but full acceptance is3/4: obstacle_type_bridge aborts exit-11 after Gazebo has exited and ROS launch later escalates. Earlier mitigation stopped only ground_truth early. Original full FAIL stays preserved.

New capture-only mitigation stops four exact supported bridges found in this launch's startup log and current same-UID ownership-token inventory, while Gazebo is stably paused and alive. It validates executable/arguments and pins PID identity across final ownership recheck and SIGINT with a pidfd. Order: ground_truth, obstacle/status, camera image, robot topics. ROS launch owns child reaping; every child terminal ACK, including failures/timeouts, is recorded. No broad kill, dropped bridge functionality, altered sensors or timeout increase. The separate paired-depth ros2-run process is excluded from this launch-owned selector.

plan.json declares one conversation seed48 plus one natural-walking seed46 revised-runtime validation before execution. All old FAILs remain. New acceptance must include every original integrity/physics/motion gate plus all4 clean bridge ACKs before Gazebo, no launch escalation and empty cleanup. No universal upstream race/root-cause claim. Only if both PASS may a bounded factory pilot recipe be frozen using the existing three-seed conversation behavior evidence; no dataset/default/training promotion.

## Measured revised-runtime acceptance

Runtime **e0b5dd6**,295 tests OK (2 conditional skips). Both new native RTX4060 captures PASS full integrity, behavior and revised teardown gates. Four identity-pinned bridge SIGINTs each receive clean ROS launch terminal ACKs before Gazebo exits. No launch escalation or scoped-cleanup fallback signals; cleanup empty.

| New cell | Actual goal | Active simulation | Recovery | Min swept robot/person surface gap | Full acceptance |
| --- | --- | ---: | ---: | ---: | --- |
| conversation_s48_transport |SUCCEEDED |31.395s |0 |32.94cm |PASS |
| walking_s46_transport |SUCCEEDED |30.310s |0 |12.45cm |PASS |

Source comparison against matching preceding capture allows exactly shutdown.py, capture.py and capture auditor changes plus the new transport_shutdown.py helper. Complete initial ready state matches, protected human/robot/Nav2/clock inputs match and all new frozen sources are verified. The changed shutdown order is a bounded mitigation, not proof of upstream root cause or universal race elimination; future episodes still require strict audit and failure isolation.

**factory_social_pilot_v1 is ACCEPTED_FOR_BOUNDED_PILOT** under this new predeclared decision: combine two revised teardown passes with preceding exact native component reproduction and all three conversation seeds' membership/body/snapshot-clock contract passes. The original preceding cohort remains3/4 full PASS with seed48 FAIL; the still earlier nine-attempt cohort remains5 PASS/3 FAIL/1 ERROR. Those files have not been rewritten or counted as clean. No current dataset/default promotion or WM/RL training.

Two collection modes are frozen in collection_profile.json: persistent standing conversation and head-on natural group walking. Original standing captures have zero actual displacement; active sidestep correction NOT_EXERCISED. Standing clock verifies six-human simulated ages with arrival/expiry queries explicitly NOT_EXERCISED. Natural walking retains the original query verifier and may dissolve/reform groups; actual per-frame metadata must be retained. Same factory, avatars, initial layout and robot route do not establish broad naturalness or simulator generalization. Validation seeds are not a dataset split.

collection_profile.json is an acceptance/recipe manifest, not the --config capture schema. Use configs/capture/geometry_v7_mppi_resample.json and explicit mode options in the following commands, always choosing a new run directory.

```bash
cd /home/cuongbien/DynamuhSim
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1

# Persistent conversation
python3 tools/capture_episode.py prepare --run-dir experiments/social_navigation/benchmark/generated/NEW_CONVERSATION_RUN --config configs/capture/geometry_v7_mppi_resample.json --scenario standing --seed 46 --planner-diagnostics --human-goal-clock sim_v1 --human-lifecycle-profile persistent_conversation_v1
python3 tools/capture_episode.py run --run-dir experiments/social_navigation/benchmark/generated/NEW_CONVERSATION_RUN
python3 tools/audit_episode.py --run-dir experiments/social_navigation/benchmark/generated/NEW_CONVERSATION_RUN --source-root /home/cuongbien/DynamuhSim --output experiments/social_navigation/benchmark/generated/NEW_CONVERSATION_RUN_audit.json

# Natural walking, optional group dissolution/formation preserved
python3 tools/capture_episode.py prepare --run-dir experiments/social_navigation/benchmark/generated/NEW_WALKING_RUN --config configs/capture/geometry_v7_mppi_resample.json --scenario walking --seed 46 --planner-diagnostics --encounter-profile head_on --human-motion-profile group_cohesion_v2 --human-goal-clock sim_v1 --human-lifecycle-profile native
python3 tools/capture_episode.py run --run-dir experiments/social_navigation/benchmark/generated/NEW_WALKING_RUN
```

Next is a separately declared small data pilot with collection variation/splits fixed beforehand, then validate raw-to-observation/BEV export, masks and leakage boundaries. Do not train WM/RL or register unreviewed captures from this validation series. dataset01 remains untouched.

[New native RGB videos](index.html), [results and four-bridge receipts](results.json), [full audits](audits.json), [frozen collection recipe](collection_profile.json), [media hashes](media_evidence.json), [preceding three-seed group test and original failure](../persistent_conversation/index.html). Media comes from native Gazebo RGB GIFs, lossy VP8 export with decoded frame count and original holds verified, default playback.25 restores simulation-time1x. No AI-generated motion/image, browser visual QA or new Windows Gazebo GUI confirmation is claimed.
