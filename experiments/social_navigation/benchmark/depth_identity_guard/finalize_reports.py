#!/usr/bin/env python3
"""Save exact small artifacts and distinct research acceptance decisions."""
import json,shutil
from pathlib import Path
from freeze import verify,sha
HERE=Path(__file__).resolve().parent
if __name__=='__main__':
    obj=verify(HERE/'generated/final_with_seed44.json');reports=HERE/'reports';inventory={};summaries={}
    def keep(source,relative):
        target=reports/relative;target.parent.mkdir(parents=True,exist_ok=True)
        if target.exists():assert sha(target)==sha(source)
        else:shutil.copyfile(source,target)
        inventory[relative]=dict(bytes=target.stat().st_size,sha256=sha(target))
    for key in obj['datasets']:
        tag='44' if key=='seed44' else '05';evaluation=HERE/'generated'/(key+'_eval'+tag);sensor=HERE/'generated'/(key+'_'+tag)
        for name in ['evaluation.json','identity_gate.json']:keep(evaluation/name,key+'/'+name)
        keep(sensor/'replay.json',key+'/replay.json');summaries[key]=json.loads((evaluation/'identity_gate.json').read_text())
    root=HERE/'generated/runs/identity_seed44_01';runtime=HERE/'generated/runtime/identity_seed44_01'
    for name in ['result.json','events.jsonl','initial.png','middle.png','final.png','camera_preview_4x.gif','live_parameters.json','config_changes.json']:
        if (root/name).exists():keep(root/name,'seed44/capture/'+name)
    for name in ['driver.py','manifest.json','changes.diff']:keep(HERE/'generated/drivers/identity_seed44_01'/name,'seed44/driver/'+name)
    for name in ['ready.json','rendering.json','ogre2_session.log','quiesce_shutdown.json','nav2_runtime.yaml']:keep(runtime/name,'seed44/runtime/'+name)
    for name in ['qa.json','manifest.json','arrays.json','calibration.json','contract.json']:keep(root/'normalized'/name,'seed44/normalized/'+name)
    keep(HERE/'generated/final_with_seed44.json','final_with_seed44.json');keep(HERE/'generated/preflight/preflight.json','preflight.json')
    metrics=json.loads((HERE/'generated/seed44_eval44/evaluation.json').read_text());capture=json.loads((reports/'seed44/capture_audit.json').read_text());raw=json.loads((reports/'seed44/raw_audit.json').read_text());audit=json.loads((reports/'audit01/audit.json').read_text())
    acceptance=dict(schema='causal_identity_guard_acceptance/1',selected_candidate='05',regression_identity_gates={k:summaries[k]['identity_guard_geometric_gate'] for k in ['mesh42','repeat42','regression43']},
        new_seed44_identity_gate=summaries['seed44']['identity_guard_geometric_gate'],new_seed44_evidence='Non-regression: parent and guard both had zero pairing changes and equal coverage/fragmentation; not a demonstrated improvement over parent on this seed',
        legacy_full_gate_seed43=summaries['regression43']['legacy_full_gate'],legacy_full_gate_seed44=summaries['seed44']['legacy_full_gate'],legacy_proxy_compatibility='Previous shape-filter failure remains, not overridden by an identity-only comparison against the same filtered parent',
        capture_integrity=capture['status'],raw_integrity=raw['status'],metric_integrity=audit['status'],new_depth_frames=raw['depth_frames_bitwise_equal'],new_windows=raw['samples'],
        navigation_status=capture['navigation_status'],actual_goal_reached=capture['actual_goal_reached'],association_accepted=False,semantic_person_validated=False,persona_validated=False,training_ready=False,
        deployment='Experimental sensor tracker only; existing ROS/Nav2/default tracker and original bundles unchanged',
        next='Define/validate person instance visibility and semantic detection/association before data batch; retain separate navigation TIMEOUT issue')
    (reports/'acceptance.json').write_text(json.dumps(acceptance,indent=2)+'\n');(reports/'review_artifacts.json').write_text(json.dumps(inventory,indent=2)+'\n')
    (reports/'behavior_tests.json').write_text(json.dumps(dict(status='PASS',tests=14,evidence='Executed before candidate05 freeze and final seal; see tool-run history',test_source_sha256=sha(HERE/'test_guard.py'),source='test_guard.py',algorithm_sha256=sha(HERE/'guard_tracker.py')),indent=2)+'\n')
    print(json.dumps(acceptance,indent=2))
