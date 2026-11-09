# Round 2 · Critical seed extension

From the repository root, enumerate 900 nominal slots. The 360 additions extend only Critical Branch A to 20 shared evaluation seeds. They neither change the original five seeds nor add Branch B interventions. This is the allocation inferred from the Round 2 Card formula and recorded as a proposed deviation.

```bash
py=runs/task_04u/reproduction/venv/bin/python
"$py" -B -m scripts.benchmarks.plan plan --phase 2 --output runs/benchmark_plans/round_2.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan dry-run \
  --manifest runs/benchmark_plans/round_2.json
```

Expected now is 900 slots, zero approved bindings and dry-run exit 2. Obtain approval for the Critical-only interpretation. The following template requires a completed, compatible Round 1 under the same full binding and runtime.

```bash
"$py" -B -m scripts.benchmarks.plan plan --phase 2 --bindings approved_bindings.yaml \
  --output runs/benchmark_plans/approved_2.json
"$py" -B -m scripts.benchmarks.plan dry-run --manifest runs/benchmark_plans/approved_2.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan run \
  --manifest runs/benchmark_plans/approved_2.json --output-dir runs/benchmark_study \
  --execute --resume --only-new
"$py" -B -m scripts.benchmarks.plan report --run-dir runs/benchmark_study \
  --output-dir runs/benchmark_reports/round_2
```

Archive the manifest, code and dependency fingerprints and every attempt. Inspect failed and interrupted histories, exclusions, uncertainty and Critical coverage before interpreting paired results. [Reproducibility and retry](BENCHMARK_REPRODUCIBILITY.md) [Metrics](METRICS_AND_THRESHOLDS.md)
