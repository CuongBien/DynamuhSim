#!/usr/bin/env python3
import hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent;GEOM=HERE.parent/'complexitynav_geometry'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
(GEOM/'generated').mkdir(exist_ok=True)
for source,name in [(GEOM.parent/'complexitynav_contract/kernel.py','native_contract.py'),(GEOM.parent/'socnavgym_warehouse_lifecycle/geometry.py','swept.py'),(HERE/'../../warehouse/layout.json','layout.json')]:
    target=GEOM/'generated'/name
    if not target.exists():target.write_bytes(source.resolve().read_bytes())
    assert target.read_bytes()==source.resolve().read_bytes(),name
assert json.loads((GEOM/'reports/run01/independent_validation.json').read_text())['status']=='PASS_OFFLINE_ONLY'
for name,digest in json.loads((GEOM/'reports/run01/protocol.json').read_text())['source_sha256'].items():assert sha(GEOM/name)==digest,name
transport=json.loads((HERE/'reports/transport01/validation.json').read_text());assert transport['status']=='PASS' and transport['worker_sha256']==sha(HERE/'worker.py')
print('PASS: offline geometry, copied dependencies and measured IPC worker match')
