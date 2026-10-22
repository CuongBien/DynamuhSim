#!/usr/bin/env python3
"""Run the byte-preserved legacy exporter bundled with this exact episode."""
import argparse,json,subprocess,sys
from pathlib import Path
from prepare import sha,dump
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);a=p.parse_args();root=a.bundle.resolve()
    assert not (root/'bundle.json').exists() and not (root/'normalized').exists()
    provenance=json.loads((root/'provenance.json').read_text())
    for name,v in provenance['copied_files'].items():assert sha(root/name)==v['sha256']
    script=root/'tools/legacy/build.py';subprocess.run([sys.executable,str(script),'--episode',str(root),'--output',str(root/'normalized')],check=True)
    dump(root/'export_receipt.json',dict(status='PASS',exporter='tools/legacy/build.py',exporter_sha256=sha(script),input_provenance_verified=True))
