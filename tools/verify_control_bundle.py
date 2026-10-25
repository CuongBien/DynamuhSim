#!/usr/bin/env python3
"""Verify the registered control bundle and engine source bindings without reports."""
import argparse,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.controls.bundle import registered_bundle


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--registry',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists() or a.output.is_symlink():raise FileExistsError('Preserve existing verification')
    bundle=registered_bundle(ROOT,a.registry)
    result=dict(status='PASS',manifest_sha256=bundle.manifest_sha256,files=len(bundle.manifest['files']),
                engine_files={name:len(bundle.verify_engine(name,ROOT)['engine_sha256']) for name in bundle.manifest['engines']})
    a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open('x') as stream:stream.write(json.dumps(result,sort_keys=True,indent=2)+'\n')
    print('CONTROL BUNDLE PASS',result['engine_files'])


if __name__=='__main__':main()
