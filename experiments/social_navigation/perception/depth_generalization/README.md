# Depth detector generalization — bounded ablation

Branch feat/depth-detector-generalization; separate worktree based on dd7a2b0. Main MuJoCo workspace is untouched.

Protocol is frozen before the run. Four variants isolate the multiplicative prior and include a score-matched 0.50 control. Existing two recordings are opened development/regression, never unseen test. No new Gazebo run or model fitting in this stage.

Outputs are new-only under generated/. Sensor inference is sealed before GT evaluation. Keep all failures. Cross-domain benchmark preparation must report missing data honestly; no hand-crafted synthetic point clouds count as unseen scenes/avatars.
