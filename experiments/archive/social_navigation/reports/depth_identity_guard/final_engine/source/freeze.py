#!/usr/bin/env python3
"""Freeze candidate identity engine and source-role map; no tuning on seed44."""
import argparse,datetime,hashlib,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
ENGINE=['config.json','guard_tracker.py','replay.py','parent/refinement.py','parent/config.json','parent/snapshots/sensor.py','parent/snapshots/association.py','parent/evaluate.py']


def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1<<20),b''):h.update(block)
    return h.hexdigest()


def verify(path):
    obj=json.loads(path.read_text());assert all(sha(HERE/n)==v for n,v in obj['engine_sha256'].items()),'Engine changed since freeze'
    return obj


def input_source(norm,role):return dict(normalized=str(norm.resolve()),role=role,input_sha256={n:sha(norm/n) for n in ['depth.npy','observations.npz','calibration.json','samples.npz']},scene_sha256=sha(norm/'human_states.jsonl'),manifest_sha256=sha(norm/'manifest.json'))


def freeze(output):
    assert not output.exists();output.parent.mkdir(parents=True,exist_ok=True)
    sources=dict(mesh42=HERE.parent/'portable_episode/generated/episode01/normalized',repeat42=HERE.parent/'depth_track_refinement/generated/fixed_source/normalized',regression43=HERE.parent/'depth_seed43_validation/generated/runs/depth_seed43_01/normalized',proxy_stress=HERE.parent/'dataset_pilot/episodes/pilot01/normalized')
    obj=dict(schema='identity_guard_frozen/1',created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),parent_commit=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),inference_uncommitted=True,
        engine_sha256={n:sha(HERE/n) for n in ENGINE},config=json.loads((HERE/'config.json').read_text()),datasets={k:input_source(p,'stress' if k=='proxy_stress' else 'development_regression') for k,p in sources.items()},
        seed44_policy='No seed44 raw/labels are available before final freeze; one later GPU capture, no tuning after inspecting it',training_ready=False)
    output.write_text(json.dumps(obj,indent=2)+'\n');print('Candidate frozen',sha(output))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();freeze(a.output.resolve())
