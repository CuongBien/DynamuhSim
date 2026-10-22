#!/usr/bin/env python3
"""Add the single fresh seed44 recording after the final engine was sealed."""
import argparse,datetime,json
from pathlib import Path
from freeze import verify,sha,input_source
HERE=Path(__file__).resolve().parent
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();assert not a.output.exists()
    final=HERE/'generated/final_engine.json';obj=verify(final);capture=json.loads((HERE/'generated/drivers/identity_seed44_01/manifest.json').read_text())
    assert capture['final_engine_sha256']==sha(final) and capture['engine_sha256']==obj['engine_sha256']
    quality=json.loads((HERE/'reports/seed44/capture_audit.json').read_text());assert quality['status']=='PASS'
    norm=HERE/'generated/runs/identity_seed44_01/normalized';obj['datasets']['seed44']=input_source(norm,'new_seed_validation_for_sealed_engine_only')
    obj['holdout_capture']=dict(added_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sealed_before_capture_sha256=sha(final),capture_manifest_sha256=sha(HERE/'generated/drivers/identity_seed44_01/manifest.json'),seed=44,tuned_on_seed44=False)
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(obj,indent=2)+'\n');print('Added untouched seed44 source; sealed inference/gates unchanged')
