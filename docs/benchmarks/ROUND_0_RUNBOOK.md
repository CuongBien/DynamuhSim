# Round 0 · integration and data QA

From the repository root, enumerate 48 nominal slots. This is four backends × six Easy Cards × two shared evaluation seeds (1000 and 1001).

```bash
py=runs/task_04u/reproduction/venv/bin/python
"$py" -B -m scripts.benchmarks.plan plan --phase 0 --output runs/benchmark_plans/round_0.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan dry-run \
  --manifest runs/benchmark_plans/round_0.json
```

Expected now is JSON/CSV with 48 slots, 48 `BLOCKED_SPEC` entries and dry-run exit 2. The generic Easy width and ND Card width disagree. A later approved binding must fix the actual Easy conditions for both Round 0 and Round 1. Use the same binding file for all three plans from the start. A distinct 2.1 m ND diagnostic cannot count toward Round 1's 1.2 m ND.

The following is the execution template for a future approved and clean checkout. Generate all manifests with the same `approved_bindings.yaml`. Dry-run the new manifest first and require exit 0.

```bash
"$py" -B -m scripts.benchmarks.plan plan --phase 0 --bindings approved_bindings.yaml \
  --output runs/benchmark_plans/approved_0.json
"$py" -B -m scripts.benchmarks.plan dry-run --manifest runs/benchmark_plans/approved_0.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan run \
  --manifest runs/benchmark_plans/approved_0.json --output-dir runs/benchmark_study --execute
"$py" -B -m scripts.benchmarks.plan report --run-dir runs/benchmark_study \
  --output-dir runs/benchmark_reports/round_0
```

Review infrastructure completion, trace/GT alignment, missing values, stop/goal labels, and trajectory and encounter coverage before interpreting backend behavior. Do not substitute the technical smoke for a Card. Use the common [failure and retry procedure](BENCHMARK_REPRODUCIBILITY.md).
