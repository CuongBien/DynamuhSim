# Benchmark data QA

One human backend is injected at composition for each episode. Robot command schedules operate in `robot_base` at 50 ms. They are frozen inputs and receive no ground truth. MuJoCo supplies robot dynamics and authoritative pose and velocity. Human commands retain the requested, limited and executed stages. The committed trace uses the robot tick, the GT export uses exactly 100 ms, and the solver event channel preserves physics substep timestamps at 2 ms. Requested unavailable sensors carry a status. The current benchmark disables sensor requests and records no observation frames. [Runtime contract](../protocols/HUMAN_MODELS_RUNTIME_CONTRACT.md)

Every attempt has an intent, resolved config, trace, GT export, contact and three action channels, runtime manifest and final validity file. A missing manifest or validity file makes the attempt interrupted. The manifest records runtime failure and last valid tick where the engine supplies them. Configuration, compiled MJCF, source asset and backend fingerprints must agree with the preflight. Disagreement is `INVALID_INFRA`.

| Axis | Evidence |
| --- | --- |
| Infrastructure | Engine failure, exception or provenance disagreement, interruption, or valid termination. A failed prefix is retained with a prefix-only quality scope. |
| Human behavior | Geometric safety, motion guard count, progress and per-person goal times. Guard activations remain visible as interventions. A human goal can remain incomplete at robot termination. |
| Robot task | Observed goal, solver collision, timeout and progress. An episode may satisfy the engine goal condition and still fail benchmark safety. |
| Continuous quality | Timestamped clearance, TTC, PSC, speed, acceleration, jerk, path, SPL and available encounter statistics. Missing evidence remains null. |

Geometry overlap and solver contact have separate channels. Human–wall and human–human contacts marked unobservable by MuJoCo remain null even if geometric clearance is present. Unknown solver contact categories are retained and prevent the safety pass. Card labels are evaluated from committed GT and surveyed input metadata. An unobservable or ambiguous label remains null. Infrastructure-valid failures contribute to raw rates where the underlying measure exists. Label-conditioned and paired reports use separately identified denominators.

Retries append attempts. Only infrastructure-invalid or interrupted histories can receive a retry. A timeout or valid behavioral failure is final for reuse. Aggregation preserves every attempt and counts each slot once. Never ingest the diagnostic smoke into research comparisons. [Metrics](METRICS_AND_THRESHOLDS.md)
