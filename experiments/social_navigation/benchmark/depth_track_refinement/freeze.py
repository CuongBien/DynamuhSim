#!/usr/bin/env python3
"""Freeze inference configuration/source and both data inputs before comparison."""
import argparse,datetime,hashlib,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
ENGINE=['config.json','refinement.py','replay.py','evaluate.py','snapshots/sensor.py','snapshots/association.py']


def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def verify(freeze):
    obj=json.loads(freeze.read_text())
    assert all(sha(HERE/n)==v for n,v in obj['engine_sha256'].items()),'Frozen engine changed'
    return obj


def create(output):
    assert not output.exists();output.parent.mkdir(parents=True,exist_ok=True)
    datasets=dict(development=HERE.parent/'portable_episode/generated/episode01/normalized',validation=HERE.parent/'dataset_pilot/episodes/pilot01/normalized')
    sources={}
    for name,norm in datasets.items():
        sources[name]=dict(normalized=str(norm.resolve()),input_sha256={n:sha(norm/n) for n in ['depth.npy','observations.npz','calibration.json','samples.npz']},
            scene_sha256=sha(norm/'human_states.jsonl'),manifest_sha256=sha(norm/'manifest.json'))
    obj=dict(schema='frozen_refinement/1',created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),inference_was_uncommitted=True,
        engine_sha256={n:sha(HERE/n) for n in ENGINE},config=json.loads((HERE/'config.json').read_text()),datasets=sources,
        validation_policy='Do not change engine/config after validation replay; same-family repeat only, not unseen-domain test')
    output.write_text(json.dumps(obj,indent=2)+'\n');print(json.dumps(dict(status='FROZEN',path=str(output),engine_files=len(ENGINE))))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();create(a.output.resolve())
