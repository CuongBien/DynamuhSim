# Two-episode handoff data pilot

Purpose: finish a small raw-to-window/depth/BEV/loader check before handing off the simulator backend. Reuse two already accepted transport_order captures: one persistent conversation, one natural walking. No new Gazebo run, large batch, dataset01 edit, train/val/test split or WM/RL training.

The versioned exporter is separate from the retired dataset01 writers. Tracked NumPy-only loader and current sensor/target geometry are reused; source and raw hashes are frozen before export. All outcomes, including errors, are preserved. Protocol and exact raw identities are in experiments/social_navigation/datasets/handoff_pilot/protocol.json.
