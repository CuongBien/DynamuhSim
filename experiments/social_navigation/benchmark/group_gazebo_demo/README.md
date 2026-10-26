# Group demonstration — historical cohort

The completed cohort runner, analyzer, QoS probe and bounded viewer have been removed from HEAD. [Original experiment](https://github.com/CuongBien/DynamuhSim/commit/e749fb51a8a35238544258c9bb01873c302e88ea) · [Code before retirement](https://github.com/CuongBien/DynamuhSim/tree/270330e2c66dbaf2cac3d793c45b07b4b0f18776/experiments/social_navigation/benchmark/group_gazebo_demo) · [Historical reports](../../../../docs/REPORT_STORAGE.md).

The original startup failure, recovery attempts and their measured goal outcomes remain preserved. The old viewer's worker-exit limitation is not converted into an accepted demo by removing its launcher. Reproduce those attempts at their recorded source revision.

For fresh group navigation use `tools/nav2_trial.py --experiment socnavgym_group_nav2 --scenario standing --seed 42 --report-dir /absolute/new/report`. See [shared Nav2](../../../../docs/SHARED_NAV2_RUNTIME.md) for setup and GUI options. For recorded dataset playback use [WM replay](../wm_dataset_replay/README.md).
