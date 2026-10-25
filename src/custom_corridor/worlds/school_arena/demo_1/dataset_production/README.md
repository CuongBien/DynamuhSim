# Dataset V1 production

The production registry is `dataset_v1.0/production_plan.csv`. Each row has a stable episode ID, scenario family, seed, density and sampled configuration hash. `PASS` means the recorded bag, structured samples and QA passed; navigation `success`, `collision`, `timeout`, `stuck` and `nav_failure` remain valid outcomes. Runtime, recording, build and QA failures are `FAIL_INFRASTRUCTURE`.

From `demo_1`:

```bash
python3 produce_dataset_v1.py --config configs/dataset_v1.yaml --prepare-only
python3 produce_dataset_v1.py --config configs/dataset_v1.yaml --resume
python3 produce_dataset_v1.py --config configs/dataset_v1.yaml --audit-only
python3 produce_dataset_v1.py --config configs/dataset_v1.yaml --freeze
```

The normal `--resume` command records, builds, validates and audits. It skips `PASS`, retries infrastructure failures within policy, fills deficits with new seeds, and freezes splits plus fingerprint only after the full audit passes. For a single episode pilot, use `--episode-id ep_000021 --max-episodes 1` with `--resume`.

The long running local batch is managed by the persistent user unit `systemd/pbl6-dataset-v1-production.service`. Use `systemctl --user start pbl6-dataset-v1-production.service` and `systemctl --user status pbl6-dataset-v1-production.service`, or inspect `tail -f dataset_v1.0/production.log`. The unit is linked into the user's systemd directory. On another machine, run `systemctl --user link "$PWD/systemd/pbl6-dataset-v1-production.service"` and `systemctl --user daemon-reload` first. Do not regenerate split files after freeze.

A `PASS` episode stores `execution.yaml`, `raw_bag/`, `structured/samples.csv`, `structured/rgb/`, and `qa.yaml`. Raw bags are zstd compressed; readers use a temporary decompressed copy. `manifest.csv` lists accepted episodes. `dataset_audit.yaml`, `dataset_audit.csv`, `dataset_summary.yaml`, and `scenario_coverage.csv` report current state, including `INCOMPLETE` while production is running. After acceptance, `splits/` and `dataset_fingerprint.yaml` are written once and verified on repeat freeze calls.

## Historical payload hydration

Historical dataset/generated outputs are no longer tracked in current checkouts. Before reading or resuming them, run the appropriate command **from the repository root**:

```bash
python3 tools/hydrate_history.py --source git \
  --group world:school_arena/demo_1/dataset_v1.0 --report /tmp/hydrate-dataset.json
python3 tools/hydrate_history.py --source git \
  --group world:school_arena/demo_1/scenario_generator/generated --report /tmp/hydrate-generated.json
python3 tools/hydrate_history.py --source git \
  --group world:school_arena/demo_1/scenario_generator/generated_v1 --report /tmp/hydrate-generated-v1.json
```

Each report path must be new. Choose only the payloads needed. Git mode uses the exact historical revision pinned in `docs/artifacts/history_2026-10-25.json`; it does not need a local archive store. Full Git history must be available; shallow clones may need the pinned commit fetched first. Archive mode and verification are documented in `docs/HISTORICAL_ARTIFACTS.md` at the repository root.

Existing matching payloads are verified and retained; different, partial or locally changed data are never overwritten. Hydrated payloads are ignored by Git. Resume and audit preserve their original outcome/incomplete state; hydration does not certify a complete dataset or a new simulation run.
