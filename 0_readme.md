# DynamuhSim World Model Dataset — Inventory & AI-ready Profile

This toolkit **does not train a model or change the dataset**. It is for inspecting the
`wm_episode_batch/generated/dataset01` portable corpus from branch `test/gazebo-harmonic-benchmark`.

## Dependencies

- Ubuntu 24.04; `/usr/bin/python3` with `numpy` and `matplotlib` for charts.
- No Docker, ROS runtime, Gazebo runtime or CUDA needed.
- Install if required: `sudo apt install -y python3-numpy python3-matplotlib`

## Run (two commands)

Extract ZIP archive, put these scripts anywhere, then:

```bash
bash bash run_all.sh ~/nav_ws/wm_restore/wm_episode_batch/generated/dataset01 ~/nav_ws/wm_dataset_report
```

Optional third argument sets the complete filesystem inventory root: `bash run_all.sh PATH_TO_DATASET01 OUTPUT_DIR ~/nav_ws` (all workspace paths).

Alternatively, from the toolkit directory just `bash run_all.sh`:
It autodetects the restored path above or
`~/nav_ws/experiments/social_navigation/benchmark/wm_episode_batch/generated/dataset01`.

### Output files

- `ALL_PATHS.csv` — **ALL paths of restored WM backup** (or `wm_episode_batch` if using a dataset in the git workspace), including absolute path, kind, size, extension, mtime; can be opened in Excel/pandas or sent to AI in parts. Specify third argument to override the inventory root.
- `ALL_PATHS.jsonl` — same but line-oriented JSON.
- `DIRECTORY_TREE.txt` — entire folder hierarchy.
- `INVENTORY_SUMMARY.md` / `.json` — file counts, size, top-level subtrees, extensions.
- `DATASET_PROFILE.md` — **start here:** scientific semantics, episode splits, input vs label separation, field dictionary, sampled quality/distributions.
- `DATASET_CONTEXT.json` — full machine-readable profile, field dictionary and sampled numeric statistics.
- `FIELD_SCHEMA.csv` — **each `.npy`/`.npz` array key**, shape and dtype for train/validation (test sealed by default).
- `EPISODE_TABLE.csv` — navigation outcomes, splits and window counts.
- `PROMPT_FOR_AI.md` — ready-to-use prompt to attach with the profile.
- `01_windows_by_episode.png` and other PNG charts (if Matplotlib available).

For AI discussions, upload `DATASET_PROFILE.md`, `DATASET_CONTEXT.json` and `PROMPT_FOR_AI.md` first. Upload `FIELD_SCHEMA.csv` if you need code-level analysis. `ALL_PATHS.csv` can be large and usually need not be uploaded unless you want precise file navigation.

## Index an entire ROS repo, not just the dataset

```bash
/usr/bin/python3 01_inventory.py --root ~/nav_ws --out ~/dynamuhsim_repo_inventory
```

Place `--out` **outside** the scanned `--root`, or the script will refuse to run to prevent recursion.

## Performance and controls

- `01_inventory.py` is standard-library only. SHA256 is OFF by default, as hashing a large dataset is slow. Opt in to hash smaller files with `--hash-max-mb 5`, or all with `--hash-all`.
- `02_profile_dataset.py` reads NumPy file **headers** without decompressing full arrays, samples at most 120 evenly spaced windows per non-test episode, and uses `mmap_mode='r'` for raw Depth arrays (max 12 frames sampled per episode, spatially stride 8). Default distributions are *descriptive samples*, **not exact full-corpus moments**.
- Neither script modifies/rebuilds original data, and neither requires ROS, Gazebo or Docker.
- All test `*.npz`/`*.npy` files remain unopened by default, including future labels. Episode-level metadata and path inventory still list test episodes (not train leakage).
- To deliberately explore test labels only **after freezing model decisions**: `python3 02_profile_dataset.py --root PATH --out NEW_OUTPUT --include-test-labels`.
- If the restore was partial, the profiler records missing-episode warnings/errors and still creates a partial report.

## Important scientific limits

Input `*_SENSOR_ONLY` is distinct from privileged `*_LABEL_ONLY`. Future action conditioning is **recorded Nav2 commands**, not a causal intervention study of alternative robot choices. Future BEV labels are **observed** from future Depth; `-1` means unknown. The first cohort has no validated persona, sensor-tracker ID association or RL rewards. `Dataset.episode(test)` is intentionally sealed in the repo loader. Episode counts are the independent study unit, not overlapping windows.
