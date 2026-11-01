# PROMPT — Help me analyze this actual DynamuhSim dataset

Read the ATTACHED `DATASET_PROFILE.md` (and optionally `DATASET_CONTEXT.json`, `FIELD_SCHEMA.csv`, `INVENTORY_SUMMARY.md`).

1. Restate exactly what is available: sensors, time rates/horizons, coordinate frames, labels, masks, units, split, episodes and navigation outcomes.
2. Distinguish what is directly VERIFIED from the dataset, what is SAMPLED and what is only a research hypothesis. Give precise numeric values only when supported by the attached tables.
3. Judge data quality: missingness, imbalance, train/validation/test integrity, action coverage, moving vs standing human evidence, timestamp consistency and BEV unknown/free/occupied balance.
4. Draw a rigorous boundary between SENSOR_ONLY (online allowed) and LABEL_ONLY (supervision only). DO NOT use simulator GT, test labels or future human trajectories in inference features.
5. Say which tasks can be trained now (observed future BEV) and which cannot be validated yet (M1 tracked trajectories, M3 persona H1-H5, valid RL reward and counterfactual action-response).
6. Recommend evidence-based next dataset experiments, evaluation metrics and ablations, including robot action diversity and paired interventions. Do not invent missing distributions, trained weights or success rates.
7. Provide code only if requested, and use actual `FIELD_SCHEMA.csv` keys and shapes. Treat unknown BEV cells as unknown, not free.

Only the analysis outputs are attached, NOT the full raw dataset; ask for a particular slice or raw episode if essential.
