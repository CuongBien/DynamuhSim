# Native lifecycle — historical experiment

The standalone `run.py`, `worker.py` and `analyze.py` have been retired from HEAD. This completed native square-room experiment is historical evidence, not a current runtime entrypoint.

[Original experiment](https://github.com/CuongBien/DynamuhSim/commit/248015807a4cd4c9fccc9d7a0f6febaf446c2caa) · [Code before retirement](https://github.com/CuongBien/DynamuhSim/tree/270330e2c66dbaf2cac3d793c45b07b4b0f18776/experiments/social_navigation/benchmark/socnavgym_lifecycle) · [Report hydration](../../../../docs/REPORT_STORAGE.md).

The three-seed study retained 56–70cm native disc penetrations and a 32.6cm formation jump; removing the harness does not change those outcomes. Use the source revision and manifest recorded with a historical result when reproducing it.

For new runs use the [shared Nav2 runtime](../../../../docs/SHARED_NAV2_RUNTIME.md). The continuous adapter implementation still used by current code is in `../socnavgym_lifecycle_fix/continuous_groups.py`.
