# Persistent conversation and explicit standing snapshot clock

This new opt-in fixture keeps designated stationary triplets using SocNavGym native can_disperse=False. It changes neither global dispersal/formation probabilities nor motion/safety/Nav2/camera/defaults. Natural lifecycle mode remains the default and is separately regressed.

Clock runtime sim_v1 and its historical query verifier are unchanged. New declared standing_snapshot_v1 validates population, initial targets, every assigned time/age, accepted gate, one activation, contiguous.1s active steps and zero queries/assignments. It only applies with frozen persistent-conversation asset plus standing/legacy/sim_v1 plan. Arrival/expiry query coverage is explicitly NOT_EXERCISED; no artificial goal queries are injected. Old standing48 without the new contract still FAILS its original audit.

plan.json freezes120-step natural/persistent component gate and four fresh native cells before capture. All old FAIL/ERROR records remain untouched.291 tests OK,2 conditional skips. No dataset/default promotion or WM/RL training.

```bash
python3 tools/capture_episode.py prepare --run-dir experiments/social_navigation/benchmark/generated/NEW_CONVERSATION --config configs/capture/geometry_v7_mppi_resample.json --scenario standing --seed 46 --planner-diagnostics --human-goal-clock sim_v1 --human-lifecycle-profile persistent_conversation_v1
python3 tools/capture_episode.py run --run-dir experiments/social_navigation/benchmark/generated/NEW_CONVERSATION
```

Use ROS Jazzy environment and OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1. Always use a fresh path. Omit human-lifecycle-profile to retain native lifecycle.
