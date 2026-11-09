# Round 1 · main conditions and action branches

From the repository root, enumerate 540 nominal slots. Branch A has 360 Advance conditions across three levels and five shared seeds. Branch B has 180 Stop, Yield/Turn and Reverse conditions over FA, ND and IN. There are 492 nominal additions if the 48 Easy slots are reusable.

```bash
py=runs/task_04u/reproduction/venv/bin/python
"$py" -B -m scripts.benchmarks.plan plan --phase 1 --output runs/benchmark_plans/round_1.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan dry-run \
  --manifest runs/benchmark_plans/round_1.json
```

Expected now is 540 slots, zero approved bindings and dry-run exit 2. Branch B requires a chosen Critical or Hard level and a same-level Advance reference. A branch changes only the approved intervention window. ND/IN Reverse remains unresolved. Geometry, policy and label QA remain prerequisites.

The following template requires an approved manifest and a completed, identical-provenance Round 0. `--only-new` checks that each prior slot in the selection is reusable before skipping it. A valid timeout is reusable as a failure. Without a compatible Round 0, record a new 540-slot run only after authorization and account for the separate pilot.

```bash
"$py" -B -m scripts.benchmarks.plan plan --phase 1 --bindings approved_bindings.yaml \
  --output runs/benchmark_plans/approved_1.json
"$py" -B -m scripts.benchmarks.plan dry-run --manifest runs/benchmark_plans/approved_1.json
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY "$py" -B -m scripts.benchmarks.plan run \
  --manifest runs/benchmark_plans/approved_1.json --output-dir runs/benchmark_study \
  --execute --resume --only-new
"$py" -B -m scripts.benchmarks.plan report --run-dir runs/benchmark_study \
  --output-dir runs/benchmark_reports/round_1
```

Check availability, one-sided pairs, labels and timeouts alongside aggregate rates. For action contrasts use the explicit `advance_slot_id` and inspect both attempts. [QA and failure handling](DATA_QA_PROTOCOL.md)
