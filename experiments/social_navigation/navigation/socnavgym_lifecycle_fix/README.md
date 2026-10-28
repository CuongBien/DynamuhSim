# Continuous lifecycle adapter

`continuous_groups.py` and `test_swept.py` remain in use. Warehouse/response motion imports the geometry/lifecycle helpers here; their bytes and the frozen WM references are unchanged.

The class is an explicit behavior modification for the tested square room and a stationary robot: continuous formation commits, bounded yaw, global individual-disc ORCA and an analytic swept-disc fallback. Warehouse and moving-robot behavior is implemented separately by the consumers; this class alone does not certify it.

```bash
experiments/social_navigation/.venv-socnavgym/bin/python experiments/social_navigation/navigation/socnavgym_lifecycle_fix/test_swept.py
```

The completed standalone cohort runner, worker, analyzer and two report-dependent probes have been removed from HEAD. [Original repair](https://github.com/CuongBien/DynamuhSim/commit/779c338892c69300d24aede9817a467dc3e04dda) · [Code before retirement](https://github.com/CuongBien/DynamuhSim/tree/270330e2c66dbaf2cac3d793c45b07b4b0f18776/experiments/social_navigation/benchmark/socnavgym_lifecycle_fix) · [Historical reports](../../../../docs/REPORT_STORAGE.md).

The recorded 5,476 updates, active-reset and moving-group checks retain their original scope and results. Reproduce historical measurements at their recorded source revision; use [shared Nav2](../../../../docs/SHARED_NAV2_RUNTIME.md) for fresh runs.
