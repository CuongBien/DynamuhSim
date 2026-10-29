# Atomic worker readiness — isolated fix validation

The original crossing_s48 attempt in the preserved nine-cell cohort failed before Gazebo startup: the parent read ready.json while the worker created/rewrote it for optional metadata. The original ERROR and full cohort FAIL remain unchanged.

The group worker now assembles the complete base/motion/encounter/clock payload first, writes a closed staging file on the same filesystem, then atomically creates ready.json by hard link. Readers see either no final path or the complete payload; duplicate publication fails without overwriting the prior file. The staging path is cleaned on success/failure. This guarantees local IPC visibility, not crash durability. Runtime/robot/human behavior and audit gates are unchanged.

plan.json declares one fresh crossing seed48 probe on the changed runtime before capture. This is explicit fix validation, not a clean replacement for the original failed cell or full cohort acceptance. Standing lifecycle and zero-query clock audit coverage remain unresolved; no dataset/default promotion.

## Measured fix validation

Runtime **2553efe**, full shared suite282 OK (2 conditional skips). One new crossing seed48 probe completed actual navigation SUCCEEDED in30.2525s with zero recovery/progress failure, full source-resolved capture audit PASS, positive swept geometric clearances, clean bridge before clean Gazebo exit and empty scoped cleanup. Bounded original motion/physical regression screen also PASS; this does not retroactively replace the original failed cell.

Complete initial ready JSON matches the final complete payload from the original startup-error worker. Source comparison permits exactly the group worker publication change and the new readiness helper; human/clock/robot/Nav2/guard sources are unchanged. No pending staging files remain. The helper unit checks visibility before/after publication, full optional metadata, second-publisher rejection and invalid-JSON rejection.

Original nine-attempt cohort remains **5 PASS/3 FAIL/1 ERROR**, original crossing_s48 startup result remains ERROR. New probe is separate revised-runtime validation. Next work: declare persistent conversation fixtures separately from native dispersal/forming behavior, and define no-goal-query clock coverage for fully stationary scenes. No audit waiver, dataset registration/default promotion or WM/RL training.

[Native RGB video](index.html), [source/ready comparisons and full metrics](results.json), [full audit](audit.json), [media provenance](media_evidence.json), [unit suite](unit_tests.json), [original cohort](../seed_regression/index.html). Exported video frame count/holds are verified; encoded4x plays at.25 to restore simulation-time1x. No AI generation or new Windows GUI confirmation. One native probe does not establish a broad statistical startup reliability rate; atomic visibility is directly tested at the publication boundary.
