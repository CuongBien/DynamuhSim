# ORCA progress diagnostics — DynamuhSim mujoco-hummodel

These are standalone diagnostic scripts, not a modification to repository production code and not the final research benchmark.

## Requirements
- Run **from the root of the local project**, where `configs/config.yaml` exists.
- Use the local `.venv/bin/python` with the installed `narrow_nav`, MuJoCo 4.0.0 and the local `humans=orca` backend.
- The publicly visible GitHub branch may lag the user's local TASK 03 changes; if `humans=orca` does not exist locally these tools cannot run.

## All-world headless ORCA sweep

```bash
cd /home/thaonhi/nav_ws
env -u PYTHONPATH -u DISPLAY -u WAYLAND_DISPLAY \
  .venv/bin/python -B ~/Downloads/orca_sweep.py --ticks 400 --policy advance --seed 42
```

Run stop-policy comparison by replacing `advance` with `stop`. Filter for hospitals with `--case hospital`; pass `--seed 43` again for multiple seeds.

Covers 31 world/scenario/width configurations defined on GitHub; 7 contain no simulated moving humans and are marked `SKIP_NO_HUMANS` rather than count as ORCA tests. Saves `runs/orca_sweep_<timestamp>/summary.csv` and step-by-step JSON in `traces/`.

`EXECUTED` = a valid code path completed without Python exception; this is NOT a scientific success pass. Native contacts are counted, not concealed. Certain hard corridors may genuinely deadlock/collide.

## Dynamic viewer for individual scenes (temporary debug)

```bash
cd /home/thaonhi/nav_ws
.venv/bin/python -B ~/Downloads/view_orca_live_debug.py \
  --world corridor --scenario head_on --policy advance --ticks 400
```

Hospital example:

```bash
.venv/bin/python -B ~/Downloads/view_orca_live_debug.py \
  --world hospital_medium --scenario hospital_medium --policy stop --ticks 400
```

Width example:

```bash
.venv/bin/python -B ~/Downloads/view_orca_live_debug.py \
  --world blind_corner --scenario blind_corner --width w090 --policy advance --ticks 400
```

This script uses PRIVATE `engine._adapter._model` / `engine._adapter._data` **for desktop debugging only** because there is no public runtime-backed viewer in the previously inspected code. Do not rely on this script as a durable interface or a benchmark source. It should later be replaced by an official visualizer owned by the MuJoCo adapter, with replay and recording added via the episode protocol.
