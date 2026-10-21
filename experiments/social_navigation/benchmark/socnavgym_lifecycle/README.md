# SocNavGym formation/dispersal contract check

[Measured report](reports/run01/LIFECYCLE_REPORT.md): three full native120s episodes (seeds42/43/44),4100 total updates including a500-step same-seed reset prefix. Human IDs are preserved within each epoch; group formation, shrinkage, moving/static groups and dissolution were observed.

**Formation is not promoted to Gazebo:** native circle-proxy penetrations reach56–70cm, and native final arrangement can instantaneously move a human by32.6cm. This is a measured limitation, not a harness startup failure. The existing fixed-group Gazebo adapter is unchanged.

From the repository root, run sequentially into a new output directory:

```bash
python3 experiments/social_navigation/benchmark/socnavgym_lifecycle/run.py \
  --report-dir experiments/social_navigation/benchmark/socnavgym_lifecycle/reports/my_run
python3 experiments/social_navigation/benchmark/socnavgym_lifecycle/analyze.py \
  --report-dir experiments/social_navigation/benchmark/socnavgym_lifecycle/reports/my_run
```

Uses the existing Python3.12 SocNavGym venv, existing formation config, two-core affinity and one BLAS thread. No ROS/Gazebo or system installs. The main process only coordinates sequential workers. Native code/config remains unchanged; instance wrappers record method calls without introducing group events, RNG draws, clipping or smoothing.

The analyzer uses a local audit checkpoint to preserve the user's original index/outside-worktree and11259 source/adapter/map files. A fresh checkout needs an equivalent local audit checkpoint before verification; do not rewrite historical source hashes or claims. Native2D GIF is labelled NOT GAZEBO and plays at4×. Its30s duration and event PNGs are decoded and hashed. It is not an AI image.

Reset reuses numeric human IDs, so a future lifecycle adapter needs a reset epoch. Observer group IDs here label group object identity, not native persistent IDs. Full historical seed42 trajectory reproduction is not established: first difference appears at completed step667; this experiment only validates exact repeat of a500-step reset prefix. Observation-space warnings are retained and RL observation validity remains unresolved.
