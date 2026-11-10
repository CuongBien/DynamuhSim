# Round 1 · main conditions and action branches

From the repository root, enumerate 540 nominal slots. Branch A has 360 Advance conditions across three levels and five shared seeds. Branch B has 180 Stop, Yield/Turn and Reverse conditions over FA, ND and IN. There are 492 nominal additions if the 48 Easy slots are reusable.

Use the same TASK 08B binding and runtime as Round 0. Expect 540 bound slots. Branch B uses Critical with the same-level Advance references. `--only-new` requires valid reusable evidence for all 48 prior slots. A valid timeout is reusable as a recorded failure.

```bash
py=runs/task_04u/reproduction/venv/bin/python
"$py" -B -m scripts.benchmarks.plan plan --phase 1 --bindings configs/benchmarks/bindings.yaml \
  --output runs/benchmark_plans/round_1.json
"$py" -B -m scripts.benchmarks.plan dry-run --manifest runs/benchmark_plans/round_1.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan run \
  --manifest runs/benchmark_plans/round_1.json --output-dir runs/benchmark_study \
  --execute --resume --only-new
"$py" -B -m scripts.benchmarks.plan report --run-dir runs/benchmark_study \
  --output-dir runs/benchmark_reports/round_1
```

Check availability, one-sided pairs, labels and timeouts alongside aggregate rates. For action contrasts use the explicit `advance_slot_id` and inspect both attempts. [QA and failure handling](DATA_QA_PROTOCOL.md)
