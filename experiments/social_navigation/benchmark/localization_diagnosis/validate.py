#!/usr/bin/env python3
"""Check ray geometry, result parity and source/input integrity without a simulator."""
import argparse,csv,hashlib,json,math,subprocess
from pathlib import Path
import numpy as np
from geometry import rays,MapField
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,default=HERE/'reports/run01');a=p.parse_args();checks=[]
box=dict(x=2.,y=0.,sx=1.,sy=2.);ro=dict(x=0.,y=0.,yaw=0.)
s,d=rays(ro,[0.,math.pi/2],[box],[dict(x=1.,y=0.)]);assert abs(s[0]-1.5)<1e-12 and math.isinf(s[1]);assert abs(d[0]-.64)<1e-12 and math.isinf(d[1])
s,d=rays(dict(x=1.5,y=0.,yaw=0.),[math.pi/2],[box],[]);assert abs(s[0]-1.)<1e-12
checks.append('analytic box/circle and parallel-boundary ray cases')
field=MapField(BENCH.parent/'warehouse/warehouse.yaml');assert not field.occupied_at(16.,0.) and field.occupied_at(16.,1.5)
checks.append('map orientation and occupied-machine/free-channel points')
summary=json.loads((a.report_dir/'summary.json').read_text());base=Path(json.loads((a.report_dir/'manifest.json').read_text())['inputs'][0]['report']).parent;old=json.loads((base/'analysis.json').read_text());expected={e['name']:e['amcl_position_error_max_m'] for e in old['results']};expected[old['empty_control']['name']]=old['empty_control']['amcl_position_error_max_m']
for row in summary['trials']:assert abs(row['amcl_error_max_m']-expected[row['name']])<1e-12,row['name']
with (a.report_dir/'results.csv').open() as f:assert len(list(csv.DictReader(f)))==5
checks.append('all five AMCL max errors identical to prior benchmark analysis; CSV has five rows')
native=json.loads((a.report_dir/'native_score_check.json').read_text());assert native['status']=='PASS';assert sum(c['poses'] for c in native['checks'])==168;assert max(c['maximum_absolute_score_error'] for c in native['checks'])<1e-5
for name,digest in native['libraries'].items():assert hashlib.sha256((Path('/opt/ros/jazzy/lib')/name).read_bytes()).hexdigest()==digest
checks.append('168 native sensor gains cross-checked; installed libraries match recorded hashes')
manifest=json.loads((a.report_dir/'manifest.json').read_text())
for item in manifest['inputs']:
 assert hashlib.sha256((Path(item['runtime'])/'diagnostics.jsonl').read_bytes()).hexdigest()==item['diagnostics_sha256']
checks.append('all five original diagnostic captures retain their recorded hashes')
checkpoint=BENCH/'generated/localization_checkpoint'
assert subprocess.check_output(['git','diff','--cached','--binary'])==(checkpoint/'index.patch').read_bytes()
assert subprocess.check_output(['git','diff','--binary','--','.',':!experiments/social_navigation'])==(checkpoint/'outside.patch').read_bytes()
modified=subprocess.check_output(['git','diff','--name-only','--','experiments/social_navigation'],text=True).splitlines()
allowed={'experiments/social_navigation/README.md','experiments/social_navigation/FRAMEWORK_COMPARISON.md','experiments/social_navigation/benchmark/README.md'};assert set(modified)<=allowed,modified
checks.append('existing staged/outside changes preserved; existing experiment runtime source unchanged')
for source in HERE.glob('*.py'):compile(source.read_text(),str(source),'exec');assert all(l==l.rstrip() for l in source.read_text().splitlines()),source
subprocess.run(['git','diff','--check','--','experiments/social_navigation'],check=True)
checks.append('new Python syntax and whitespace')
result=dict(status='PASS',checks=checks,code_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in sorted(HERE.iterdir()) if f.suffix in ('.py','.cpp')},native_max_absolute_score_error=max(c['maximum_absolute_score_error'] for c in native['checks']))
(a.report_dir/'checks.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
