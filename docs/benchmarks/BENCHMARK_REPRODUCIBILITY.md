# Benchmark reproducibility

Run from the repository root. Use the isolated interpreter from the 04U build for a plan that requires all four backends. The gate checks audited integration evidence and the exact native JuPedSim model class and distribution. The version recorded on each attempt includes the native binary SHA. ORCA uses the native `rvo2` binding and its binary hash. The evidence register records behavior separately. [04U build recipe and pinned source](../reports/human_models/TASK_04U_REPORT.md) [External actor contract](../backends/JUPEDSIM_EXTERNAL_ACTOR_CONTRACT.md)

```bash
py=runs/task_04u/reproduction/venv/bin/python
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan --help
```

Use `plan` to write JSON and CSV. `validate` checks the deterministic selection and `dry-run` also verifies native availability and resolves bound scenes without stepping physics. Run commands require explicit `--execute`. Select one backend with `--backend`, one slot with `--episode`, or the selected phase with no filters. `--scenario` and `--seed` combine with these filters. Use a new empty output directory for an independent run. Resuming requires the same full plan, Git commit, dirty flag, code digest, interpreter and dependency provenance. Research runs require a clean worktree. One lock prevents concurrent writes to the same run. An exact execution key is required for reuse. New attempts never overwrite old evidence.

```bash
"$py" -B -m scripts.benchmarks.plan plan --phase smoke --output runs/benchmark_demo/smoke.json
"$py" -B -m scripts.benchmarks.plan dry-run --manifest runs/benchmark_demo/smoke.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan run \
  --manifest runs/benchmark_demo/smoke.json --episode benchmark_smoke \
  --output-dir runs/benchmark_demo/smoke --execute --max-episodes 1
"$py" -B -m scripts.benchmarks.plan report --run-dir runs/benchmark_demo/smoke \
  --output-dir runs/benchmark_demo/report
```

The diagnostic smoke uses one ORCA backend and a 0.5 s budget. A valid timeout is expected. Exit 0 indicates a valid attempt or successful reuse. Exit 2 indicates a preflight or runtime infrastructure problem. Inspect `qa.csv` and the raw attempt before adding `--resume --retry-invalid` to retry an invalid or interrupted attempt. Preserve the directory and diagnose provenance mismatches before creating a fresh run. Aggregation emits per-episode, per-condition, coverage, QA, outliers, summary, paired comparison and quality distribution CSVs plus `report.json`. All raw output stays under ignored `runs/`.

- [Round 0 runbook](ROUND_0_RUNBOOK.md)
- [Round 1 runbook](ROUND_1_RUNBOOK.md)
- [Round 2 runbook](ROUND_2_RUNBOOK.md)
