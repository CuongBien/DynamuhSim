# Baseline inputs

Seven byte-preserved SDF inputs used by A3/reset/replay and A4/cold-start
consumers. World N=0/2/5/10/24 covers the pre-existing dynamic N input family;
Burger 640×480 at 10/30 Hz covers the fixed sensor consumers. Manifest records
their original Git blob, SHA256 and source revision.

Do not replace these with newly generated defaults: measured world N=5/10 uses
RTF=0.0, while the other baseline worlds use RTF=1.0. Original fields remain exact.

Per-trial scenario/world outputs still go to ignored `../generated/` and are
created by the existing generators. See [payload history](../../../../docs/history/closed_experiment_payloads.md)
for the archived 1,505 generated files and old experiment reports.
