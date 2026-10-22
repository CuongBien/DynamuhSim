#!/usr/bin/env python3
"""Apply the parent frozen inference/metrics to seed43, with corrected current-scope metadata."""
import argparse,datetime,json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
sys.path.insert(0,str(BENCH/'depth_track_refinement'))
from freeze import verify,sha
from replay import run as replay
from evaluate import run as evaluate


def prepare(output):
    assert not output.exists();output.parent.mkdir(parents=True,exist_ok=True)
    parent=BENCH/'depth_track_refinement/reports/frozen01.json';fixed=verify(parent)
    capture=json.loads((HERE/'generated/drivers/depth_seed43_01/manifest.json').read_text())
    assert fixed['engine_sha256']==capture['inference_engine_sha256'] and fixed['config']==capture['inference_config']
    audit=json.loads((HERE/'reports/capture/capture_audit.json').read_text());assert audit['status']=='PASS'
    norm=HERE/'generated/runs/depth_seed43_01/normalized';fixed['datasets']['seed43']=dict(normalized=str(norm.resolve()),input_sha256={n:sha(norm/n) for n in ['depth.npy','observations.npz','calibration.json','samples.npz']},scene_sha256=sha(norm/'human_states.jsonl'),manifest_sha256=sha(norm/'manifest.json'))
    fixed['extension']=dict(parent_frozen_sha256=sha(parent),capture_manifest_sha256=sha(HERE/'generated/drivers/depth_seed43_01/manifest.json'),created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        source='New seed43 captured after inference was frozen in capture manifest',engine_and_config_unchanged=True,validation_scope=json.loads((HERE/'protocol.json').read_text())['independence'])
    output.write_text(json.dumps(fixed,indent=2)+'\n');print('Source extension prepared; inference unchanged')


def report(evaluation,output):
    value=json.loads((evaluation/'evaluation.json').read_text());protocol=json.loads((HERE/'protocol.json').read_text())
    result=dict(schema='new_seed_geometric_comparison/1',seed=43,frozen_sha256=value['frozen_sha256'],variants=value['variants'],comparison=value['comparison'],
        metric_scope=value['metric_scope'],validation_scope=protocol['independence'],
        source_evaluator_output_sha256=sha(evaluation/'evaluation.json'),
        metadata_note='Parent evaluator retains historical same-seed validation_scope text in its untouched raw output. This report supplies the current seed43 scope; numeric results are unchanged.',
        semantic_person_validated=False,association_accepted=False,training_ready=False)
    output.parent.mkdir(parents=True,exist_ok=True);assert not output.exists();output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['prepare','replay','evaluate','report']);p.add_argument('--frozen',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--sensor',type=Path);p.add_argument('--evaluation',type=Path);a=p.parse_args()
    if a.mode=='prepare':prepare(a.output.resolve())
    elif a.mode=='replay':replay(a.frozen.resolve(),'seed43',a.output.resolve())
    elif a.mode=='evaluate':evaluate(a.frozen.resolve(),'seed43',a.sensor.resolve(),a.output.resolve())
    else:report(a.evaluation.resolve(),a.output.resolve())
