# Atomic worker readiness — isolated fix validation

The original crossing_s48 attempt in the preserved nine-cell cohort failed before Gazebo startup: the parent read ready.json while the worker created/rewrote it for optional metadata. The original ERROR and full cohort FAIL remain unchanged.

The group worker now assembles the complete base/motion/encounter/clock payload first, writes a closed staging file on the same filesystem, then atomically creates ready.json by hard link. Readers see either no final path or the complete payload; duplicate publication fails without overwriting the prior file. The staging path is cleaned on success/failure. This guarantees local IPC visibility, not crash durability. Runtime/robot/human behavior and audit gates are unchanged.

plan.json declares one fresh crossing seed48 probe on the changed runtime before capture. This is explicit fix validation, not a clean replacement for the original failed cell or full cohort acceptance. Standing lifecycle and zero-query clock audit coverage remain unresolved; no dataset/default promotion.
