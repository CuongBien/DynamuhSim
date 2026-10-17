#!/usr/bin/env python3
"""Verify preservation against the pre-round1 source/index snapshots."""
from pathlib import Path
import hashlib
import json
import subprocess

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
BEFORE=HERE.parent/'reports'
checked=0
changed=[]
for line in (BEFORE/'tracked_sha256_before.txt').read_text().splitlines():
    expected,sep,name=line.partition('  ')
    if not sep: continue
    path=ROOT/name
    checked+=1
    actual=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
    if actual!=expected: changed.append(name)
index=subprocess.check_output(['git','diff','--cached','--binary'],cwd=ROOT)
working=subprocess.check_output(['git','diff','--binary','--','.',':!experiments/social_navigation'],cwd=ROOT)
report=dict(original_tracked_files_checked=checked,original_files_changed=changed,
    preexisting_staged_diff_identical=index==(BEFORE/'index_before.patch').read_bytes(),
    preexisting_unstaged_diff_outside_experiment_identical=working==(BEFORE/'working_tree_before.patch').read_bytes(),
    prior_commits=['91f3620','1dbf832','86e125a'],round3_changes_scoped_to='experiments/social_navigation/round3, FRAMEWORK_COMPARISON.md and parent README link')
(HERE/'reports/source_preservation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
