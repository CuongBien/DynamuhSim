# Warehouse lifecycle components

This directory now contains the shared components still consumed by response/Nav2 and frozen WM capture:

- `kernel.py`, `motion.py`: six-person native lifecycle with measured robot input and explicit motion repair.
- `geometry.py`: swept geometry against the 42 warehouse AABBs.
- `config.yaml`: retained native configuration.
- `test_geometry.py`: tunneling, box and moving-robot regression fixtures.

```bash
python3 experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle/test_geometry.py
```

The completed pre-Nav2 experiment's build/adapter, launch, two live verifier/worker pairs, offline runner, preflight, analyzer and shell dispatcher have been removed. They published scripted robot velocities and are no longer entrypoints for new captures. Use the [shared Nav2 runtime](../../../../docs/SHARED_NAV2_RUNTIME.md) or [capture API](../../../../docs/CAPTURE_RUNTIME.md).

[Original integration](https://github.com/CuongBien/DynamuhSim/commit/b3f0022b26dc50483c645a5be734c7890ca6aaed) · [Code before retirement](https://github.com/CuongBien/DynamuhSim/tree/270330e2c66dbaf2cac3d793c45b07b4b0f18776/experiments/social_navigation/benchmark/socnavgym_warehouse_lifecycle) · [Report hydration](../../../../docs/REPORT_STORAGE.md).

The two 120s historical trials remain distinct: seed42 had partial lifecycle coverage; seed43 included releases and moving-group travel. Their measurements, source manifests and failures are unchanged. Historical reproduction requires the recorded source revision; the removed `offline02` checker belonged to that retired harness, not the current response/Nav2 preflight.
