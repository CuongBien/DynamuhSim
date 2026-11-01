#!/usr/bin/env python3
"""Fail closed on split leakage or missing cross-domain data. Reads metadata/hashes only."""
import argparse,hashlib,json
from pathlib import Path


def check(manifest,root):
    errors=[];pending=[];scenes={};avatars={};groups={};case_ids=set()
    required={'depth.npy','observations.npz','calibration.json','person_instance_LABEL_ONLY.npz'}
    splits=set()
    for case in manifest['cases']:
        ident=case['case_id'];split=case['split'];splits.add(split)
        if ident in case_ids:errors.append(ident+': duplicate ID')
        case_ids.add(ident)
        for key,value,seen in [('scene_family',case['scene_family'],scenes),('scene_hash',case.get('scene_geometry_sha256'),scenes),('episode_group',case['episode_group'],groups)]:
            if value:
                token=(key,value)
                if token in seen and seen[token]!=split:errors.append(ident+': cross-split '+key)
                seen[token]=split
        for value in case.get('avatar_families',[])+case.get('avatar_geometry_sha256',[]):
            if value in avatars and avatars[value]!=split:errors.append(ident+': cross-split avatar geometry/family')
            avatars[value]=split
        if not case.get('ground_truth_is_separate') or case.get('labels_kind')!='pixel_instance_owner':errors.append(ident+': missing trusted separate labels')
        if case['status']!='COLLECTED':pending.append(ident+': not collected')
        if not case.get('scene_geometry_sha256'):pending.append(ident+': scene hash missing')
        if case.get('has_people') and not case.get('avatar_geometry_sha256'):pending.append(ident+': human morphology hash missing')
        if not required.issubset(case.get('files',{})):pending.append(ident+': required data/labels missing')
        for name,entry in case.get('files',{}).items():
            path=(Path(root)/entry['path']).resolve()
            if not path.is_relative_to(Path(root).resolve()):errors.append(ident+': path escaped data root');continue
            if not path.is_file():pending.append(ident+': absent '+name);continue
            h=hashlib.sha256()
            with path.open('rb') as f:
                for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
            if h.hexdigest()!=entry['sha256']:errors.append(ident+': changed '+name)
    if splits!={'train','validation','test'}:errors.append('Must declare all three split roles')
    for split in splits:
        rows=[r for r in manifest['cases'] if r['split']==split]
        if not any(r['has_people'] for r in rows) or not any(not r['has_people'] for r in rows):errors.append(split+': missing person or empty/clutter control')
    ready=not errors and not pending
    return dict(status='READY' if ready else 'NOT_READY',errors=errors,pending=pending,data_ready=ready,
                test_evaluation_authorized=False,training_authorized=False,
                reason='Only data metadata/hashes checked. Separate train/validation curation and a model freeze are still required before test.',
                semantic_generalization_measured=False)


def main():
    p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,required=True);p.add_argument('--data-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    report=check(json.loads(a.manifest.read_text()),a.data_root)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as f:json.dump(report,f,indent=2)
    print(json.dumps(report,indent=2))
    if report['status']!='READY':raise SystemExit(2)


if __name__=='__main__':main()
