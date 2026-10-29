# Persistent conversation and explicit standing snapshot clock

This new opt-in fixture keeps designated stationary triplets using SocNavGym native can_disperse=False. It changes neither global dispersal/formation probabilities nor motion/safety/Nav2/camera/defaults. Natural lifecycle mode remains the default and is separately regressed.

Clock runtime sim_v1 and its historical query verifier are unchanged. New declared standing_snapshot_v1 validates population, initial targets, every assigned time/age, accepted gate, one activation, contiguous.1s active steps and zero queries/assignments. It only applies with frozen persistent-conversation asset plus standing/legacy/sim_v1 plan. Arrival/expiry query coverage is explicitly NOT_EXERCISED; no artificial goal queries are injected. Old standing48 without the new contract still FAILS its original audit.

plan.json freezes120-step natural/persistent component gate and four fresh native cells before capture. All old FAIL/ERROR records remain untouched.291 tests OK,2 conditional skips. No dataset/default promotion or WM/RL training.

```bash
python3 tools/capture_episode.py prepare --run-dir experiments/social_navigation/benchmark/generated/NEW_CONVERSATION --config configs/capture/geometry_v7_mppi_resample.json --scenario standing --seed 46 --planner-diagnostics --human-goal-clock sim_v1 --human-lifecycle-profile persistent_conversation_v1
python3 tools/capture_episode.py run --run-dir experiments/social_navigation/benchmark/generated/NEW_CONVERSATION
```

Use ROS Jazzy environment and OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1. Always use a fresh path. Omit human-lifecycle-profile to retain native lifecycle.

RNG scope clarification: there are no extra RNG hooks/draws and no edited vendor RNG call sites. Selecting native non-dispersing groups naturally changes which later lifecycle branches execute, and therefore can change downstream RNG consumption relative to dispersing mode. Exact random-stream/trajectory reproduction is required only for the untouched natural control, not between natural and persistent modes.

## Preserved four-cell outcome

**Full acceptance3/4 PASS.** Conversation membership and standing snapshot-clock contracts PASS on all three seeds46–48. Each robot goal SUCCEEDED with zero progress failure/recovery, positive swept geometric clearances, six people retained in two triplets, zero actual standing displacement. Active sidestep correction was NOT_EXERCISED; near-robot yielding was eligible but no displacement needed. Walking native-mode control also passes original query verifier and motion limits, with no conversation asset/override.

conversation_s48 remains full audit FAIL clean_exit/no_child_exit_errors: after Gazebo clean exit and clean early ground_truth bridge exit, the separately launched obstacle_type_bridge parameter_bridge-8 aborts exit-11 at ROS shutdown; launch subsequently escalates. The new conversation/clock contracts are not the failing gates. Original raw result/audit/logs and derived report remain preserved. No collection profile is frozen yet. See teardown_failure.json.

Next isolate stop ordering for the other ROS-launch-owned Gazebo bridges while the paused server remains alive, then validate on a separately declared revised-runtime cohort. No source tuning or retry occurred within this four-cell cohort.
