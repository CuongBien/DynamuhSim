#!/usr/bin/env python3
"""Reconstruct only ignored dependencies; retain exact locked-input hashes."""
import ast,hashlib,json
from pathlib import Path
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parent.parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
(HERE/'generated').mkdir(exist_ok=True)
base=(SOCIAL/'crowdsimplus_adapter/kernel.py').read_text()
text=base.replace('self.targets=[0,0,0]',"self.targets=[0]*len(scene['initial'])").replace('human_num=3',"human_num=len(scene['initial'])").replace('self.arrivals=[0,0,0]',"self.arrivals=[0]*len(scene['initial'])")
expected=json.loads((HERE/'inputs/crowd_bookkeeping_patch.json').read_text());assert hashlib.sha256(base.encode()).hexdigest()==expected['original_sha256'] and hashlib.sha256(text.encode()).hexdigest()==expected['generated_sha256']
target=HERE/'generated/crowd_kernel_n6.py'
if not target.exists():target.write_text(text)
assert target.read_text()==text
method=lambda source,name:ast.dump(next(node for cls in ast.parse(source).body if isinstance(cls,ast.ClassDef) for node in cls.body if isinstance(node,ast.FunctionDef) and node.name==name),include_attributes=False)
for name in ['advance','humans','metadata']:assert method(base,name)==method(text,name)
geom=HERE.parent/'complexitynav_geometry';(geom/'generated').mkdir(exist_ok=True)
for source,name in [(HERE.parent/'complexitynav_contract/kernel.py','native_contract.py'),(HERE.parent/'socnavgym_warehouse_lifecycle/geometry.py','swept.py'),(SOCIAL/'warehouse/layout.json','layout.json')]:
    target=geom/'generated'/name
    if not target.exists():target.write_bytes(source.read_bytes())
    assert target.read_bytes()==source.read_bytes(),name
for name,digest in json.loads((HERE/'inputs/protocol.json').read_text())['input_sha256'].items():assert sha(HERE/name)==digest,name
print('PASS: locked inputs and exact population-only bookkeeping change; motion methods unchanged')
