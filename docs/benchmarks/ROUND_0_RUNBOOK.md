# Round 0 · integration and data QA

From the repository root, enumerate 48 nominal slots. This is four backends × six Easy Cards × two shared evaluation seeds (1000 and 1001).

Use the checked-in TASK 08B binding. Expect 48 bound slots. ND Easy uses 1.2 m in both this round and Round 1. Run from a clean checkout after the full dry-run exits 0.

```bash
py=runs/task_04u/reproduction/venv/bin/python
"$py" -B -m scripts.benchmarks.plan plan --phase 0 --bindings configs/benchmarks/bindings.yaml \
  --output runs/benchmark_plans/round_0.json
"$py" -B -m scripts.benchmarks.plan dry-run --manifest runs/benchmark_plans/round_0.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan run \
  --manifest runs/benchmark_plans/round_0.json --output-dir runs/benchmark_study --execute
"$py" -B -m scripts.benchmarks.plan report --run-dir runs/benchmark_study \
  --output-dir runs/benchmark_reports/round_0
```

Review infrastructure completion, trace/GT alignment, missing values, stop/goal labels, and trajectory and encounter coverage before interpreting backend behavior. Do not substitute the technical smoke for a Card. Use the common [failure and retry procedure](BENCHMARK_REPRODUCIBILITY.md).
