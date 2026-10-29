# Handoff data pilot — two accepted captures

A new export/QA cohort, separate from closed dataset01. Reuses one persistent conversation and one natural walking capture. **PASS:2episodes,517windows,910raw depth frames bitwise,2068BEV grids independently replayed.** No new simulation, training, model evaluation or train/validation/test split. Every sample has training_authorized=false; fit_allowed=false in dataset metadata.

[MuJoCo handoff](../../../../docs/HANDOFF_MUJOCO.md), [results/schema](../../../../docs/demos/2026-10-29/handoff_pilot/README.md), [audit](../../../../docs/demos/2026-10-29/handoff_pilot/audit.json).

Local portable dataset: generated/pilot01/dataset/. Archive: generated/package01/handoff_data_pilot01.zip (31.7MB compressed,about379MB extracted). Raw MCAP remains at source_run in protocol.json and is not included in portable ZIP. Full data/ZIP are ignored by Git; code/reports/one sample are tracked.

Read without ROS:

```bash
python experiments/social_navigation/datasets/handoff_pilot/inspect_pilot.py --dataset experiments/social_navigation/datasets/handoff_pilot/generated/pilot01/dataset
# After extracting the portable ZIP:
python tools/inspect_pilot.py --dataset .
```

Exporter:export.py,independent audit:audit.py,packaging:package.py. Export sources/inputs frozen before execution, audit sources frozen separately. Historical writer bytes were read only to confirm channel conventions; never executed. Shared loader/sensor/SE2/BEV helper bytes remain unchanged. Reproduction uses a NEW output directory; never overwrite pilot01/audit01/package01 or append dataset01.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
python experiments/social_navigation/datasets/handoff_pilot/export.py --output experiments/social_navigation/datasets/handoff_pilot/generated/NEW_EXPORT
python experiments/social_navigation/datasets/handoff_pilot/audit.py --dataset experiments/social_navigation/datasets/handoff_pilot/generated/NEW_EXPORT/dataset --output experiments/social_navigation/datasets/handoff_pilot/generated/NEW_AUDIT
```

The NumPy-only loader remains byte-identical to wm_episode_batch/loader.py. Raw arrays include full episode streams, while observation(index) returns only causal histories; action_condition and supervision are explicit separate accessors. H1–H5, sensor/GT person association and RL reward remain invalid. Rewards NaN,unknown BEV−1; inspect valid masks rather than treating unknown as free. See HANDOFF_MUJOCO.md for exact axes,channel order,timing and backend acceptance.
