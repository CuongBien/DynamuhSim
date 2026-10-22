#!/usr/bin/env python3
"""Seal the regression-accepted candidate before any seed44 capture."""
import argparse,datetime,json,shutil
from pathlib import Path
from freeze import verify,sha
HERE=Path(__file__).resolve().parent
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--tag',required=True);a=p.parse_args()
    obj=verify(a.candidate.resolve());out=HERE/'generated/final_engine.json';assert not out.exists() and not (HERE/'generated/runs/identity_seed44_01').exists()
    gates={}
    for key in ['mesh42','repeat42','regression43']:
        source=HERE/'generated'/(key+'_eval'+a.tag)/'identity_gate.json';gate=json.loads(source.read_text());assert gate['identity_guard_geometric_gate'];gates[key]=sha(source)
    obj['engine_sha256'].update({n:sha(HERE/n) for n in ['evaluate.py','freeze.py']})
    obj['seal']=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),selected_candidate_sha256=sha(a.candidate),selected_tag=a.tag,regression_gate_sha256=gates,
        seed44_raw_exists_at_seal=False,policy='No algorithm/config/gate changes after seed44 is captured; candidate failures retained')
    out.write_text(json.dumps(obj,indent=2)+'\n')
    snap=HERE/'reports/final_engine/source'
    for n,v in obj['engine_sha256'].items():
        q=snap/n;q.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(HERE/n,q);assert sha(q)==v
    shutil.copyfile(out,snap.parent/'frozen.json');print('Final engine sealed before seed44',sha(out))
