#!/usr/bin/env python3
"""Preserve the current user's source/index state, relative to this turn's checkpoint."""
from pathlib import Path
import hashlib
import json
import subprocess

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[2]
checkpoint=HERE.parent/'gazebo_adapter/generated/commit_checkpoint'
expected=json.loads((checkpoint/'hashes.json').read_text())
changed=[]
for name,digest in expected.items():
    path=REPO/name
    actual=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
    if actual!=digest: changed.append(name)
index=subprocess.check_output(['git','diff','--cached','--binary'],cwd=REPO)
working=subprocess.check_output(['git','diff','--binary','--','.',':!experiments/social_navigation'],cwd=REPO)
manifest=json.loads((HERE/'reports/run_manifest.json').read_text())
binary=REPO/'install/custom_corridor_plugins/lib/librvo2_human_system.so'
binary_changed=hashlib.sha256(binary.read_bytes()).hexdigest() not in manifest['rvo2_binary_sha256']
report=dict(baseline='Checkpoint at the start of this commit-and-continue turn; includes existing user cleanup/deletions',
    tracked_paths_outside_social_experiment_checked=len(expected),changed_by_this_work=changed,
    preexisting_staged_diff_identical=index==(checkpoint/'index.patch').read_bytes(),
    preexisting_unstaged_diff_outside_experiment_identical=working==(checkpoint/'outside.patch').read_bytes(),
    existing_built_plugin_modified=binary_changed)
(HERE/'reports/source_preservation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
assert not binary_changed and not changed and report['preexisting_staged_diff_identical'] and report['preexisting_unstaged_diff_outside_experiment_identical']
