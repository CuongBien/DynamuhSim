# Round 2 · Critical seed extension

From the repository root, enumerate 900 nominal slots. The 360 additions extend only Critical Branch A to 20 shared evaluation seeds. They neither change the original five seeds nor add Branch B interventions. TASK 08B locks this allocation to the explicit Round 2 formula.

Use the same TASK 08B binding and runtime as the earlier rounds. Expect 900 bound slots. `--only-new` requires valid reusable evidence for the 540 prior slots.

```bash
py=runs/task_04u/reproduction/venv/bin/python
"$py" -B -m scripts.benchmarks.plan plan --phase 2 --bindings configs/benchmarks/bindings.yaml \
  --output runs/benchmark_plans/round_2.json
"$py" -B -m scripts.benchmarks.plan dry-run --manifest runs/benchmark_plans/round_2.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan run \
  --manifest runs/benchmark_plans/round_2.json --output-dir runs/benchmark_study \
  --execute --resume --only-new
"$py" -B -m scripts.benchmarks.plan report --run-dir runs/benchmark_study \
  --output-dir runs/benchmark_reports/round_2
```

Archive the manifest, code and dependency fingerprints and every attempt. Inspect failed and interrupted histories, exclusions, uncertainty and Critical coverage before interpreting paired results. [Reproducibility and retry](BENCHMARK_REPRODUCIBILITY.md) [Metrics](METRICS_AND_THRESHOLDS.md)
