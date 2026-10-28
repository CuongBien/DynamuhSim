#!/usr/bin/env python3
"""Reuse byte-preserved numeric evaluator and add the predeclared identity guard gate."""
import argparse,json,sys,importlib.util
from pathlib import Path
from freeze import verify,sha
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'parent'))
spec=importlib.util.spec_from_file_location('parent_numeric_evaluation',HERE/'parent/evaluate.py');parent=importlib.util.module_from_spec(spec);spec.loader.exec_module(parent)


def run(frozen,dataset,sensor,output):
    obj=verify(frozen);parent.run(frozen,dataset,sensor,output)
    result=json.loads((output/'evaluation.json').read_text());new=result['variants']['combined'];old=result['variants']['parent'];gate=obj['config']['new_identity_gate']
    conditions=dict(paired_rows_retained=new['paired_rows']>=gate['min_paired_retention_vs_parent']*old['paired_rows'],zero_short_gap_pair_changes=new['paired_identity_changes_within_500ms']==0,
        fragmentation_not_increased=new['fragmentation_proxy']<=old['fragmentation_proxy'],no_known_wrong_reacquisition=new['dormant_reacquisitions_wrong_known_identity']==0)
    report=dict(dataset=dataset,role=obj['datasets'][dataset]['role'],conditions=conditions,identity_guard_geometric_gate=all(conditions.values()),legacy_full_gate=result['comparison']['geometric_improvement'],
        metric_scope='Geometric GT gates only, not semantic identity truth or person precision/recall',source_evaluation_sha256=sha(output/'evaluation.json'),association_accepted=False,training_ready=False)
    (output/'identity_gate.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--frozen',type=Path,required=True);p.add_argument('--dataset',required=True);p.add_argument('--sensor',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.frozen.resolve(),a.dataset,a.sensor.resolve(),a.output.resolve())
