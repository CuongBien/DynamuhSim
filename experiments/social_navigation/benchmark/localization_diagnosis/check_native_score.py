#!/usr/bin/env python3
"""Cross-check offline scores with the installed public AMCL sensorUpdate API."""
import argparse,hashlib,json,subprocess,shutil,os
from pathlib import Path
import numpy as np
from geometry import MapField
HERE=Path(__file__).resolve().parent;SOCIAL=HERE.parents[1]
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,default=HERE/'reports/run01');a=p.parse_args();reference=HERE/'generated/reference';inc=reference/'include';target=inc/'nav2_amcl/map/map.hpp';target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(reference/'map.hpp',target)
version=subprocess.check_output(['dpkg-query','-W','-f=${Version}','ros-jazzy-nav2-amcl'],text=True);assert version.startswith('1.3.13-'),version
binary=HERE/'generated/native_score'
cmd=['g++','-std=c++17','-O2','-I'+str(inc),str(HERE/'native_score.cpp'),'-L/opt/ros/jazzy/lib','-lsensors_lib','-lpf_lib','-lmap_lib','-Wl,-rpath,/opt/ros/jazzy/lib','-o',str(binary)]
subprocess.run(cmd,check=True,capture_output=True)
field=MapField(SOCIAL/'warehouse/warehouse.yaml');raw=HERE/'generated/map_cells.bin';np.where(field.occupied,1,-1).astype(np.int8).tofile(raw)
lines=[f'{field.width} {field.height} {field.res:.17g} {field.origin[0]+field.width//2*field.res:.17g} {field.origin[1]+field.height//2*field.res:.17g} {raw.resolve()}'];expected=[];labels=[]
snaps=json.loads((a.report_dir/'scan_snapshots.json').read_text());rng=np.random.default_rng(42)
for label,snap in snaps.items():
 scan=snap['scan'];scan=dict(scan,ranges=[v if v is not None else float('inf') for v in scan['ranges']]);poses=np.array([[snap[k]['x'],snap[k]['y'],snap[k]['yaw']] for k in ['actual','amcl']]+[[rng.uniform(13.,19.),rng.uniform(-2.5,3.5),rng.uniform(-3.14,3.14)] for _ in range(40)])
 for masked in [False,True]:
    mask=~np.array(snap['human_mask']) if masked else None;scores,used=field.score(poses,scan,mask);rs=np.array(scan['ranges']);rs=np.where(np.isfinite(rs)&(rs>scan['range_min'])&(rs<scan['range_max']),rs,scan['range_max'])
    if masked:rs[np.array(snap['human_mask'])]=scan['range_max']
    values=[len(rs),len(poses),scan['range_max'],scan['angle_min'],scan['angle_increment'],*rs.tolist(),*poses.ravel().tolist()];lines.append(' '.join(format(float(x),'.17g') for x in values));expected.append(scores);labels.append(label+('_human_masked' if masked else '_all'))
env=os.environ.copy();env['LD_LIBRARY_PATH']='/opt/ros/jazzy/lib:'+env.get('LD_LIBRARY_PATH','')
done=subprocess.run([str(binary)],env=env,input='\n'.join(lines)+'\n',text=True,capture_output=True);assert done.returncode==0,done.stderr
actual=[np.array([float(v) for v in line.split()]) for line in done.stdout.splitlines()];assert len(actual)==len(expected)
checks=[]
for label,x,y in zip(labels,actual,expected):
 error=float(np.max(np.abs(x-y)));assert error<1e-5,(label,error)
 checks.append(dict(case=label,poses=len(x),maximum_absolute_score_error=error,native_true_and_amcl_scores=x[:2].tolist(),reconstructed_true_and_amcl_scores=y[:2].tolist()))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
result=dict(status='PASS',installed_package=version,checks=checks,method='Public installed LikelihoodFieldModel::sensorUpdate on supplied poses with unit prior weights; recover raw sensor gain from w_slow and normalized weights. No motion/resampling/PF replay.',libraries={f:sha(Path('/opt/ros/jazzy/lib')/f) for f in ['libsensors_lib.so','libpf_lib.so','libmap_lib.so']},harness_sha256=sha(HERE/'native_score.cpp'),harness_binary_sha256=sha(binary))
(a.report_dir/'native_score_check.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
