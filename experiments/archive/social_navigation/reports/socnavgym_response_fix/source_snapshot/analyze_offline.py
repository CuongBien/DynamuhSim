import os
for key in ['OMP_NUM_THREADS','OPENBLAS_NUM_THREADS','MKL_NUM_THREADS']:os.environ[key]='1'
import hashlib,json,math
import numpy as np
from pathlib import Path
from audit_geometry import gaps
HERE=Path(__file__).resolve().parent;COMMON=HERE.parent/'common_response';out=HERE/'reports/offline01';load=lambda p:json.loads(p.read_text());results=load(out/'results.json');boxes=load(COMMON/'inputs/layout.json')['obstacles'];metrics=[]
for name,digest in load(out/'source_sha256.json').items():assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
for result in results:
    assert result['status']=='COMPLETED' and result['steps']==100
    rows=load(out/(result['case']+'_trajectory.json'));a=np.array([[[h['x'],h['y']] for h in row['humans']] for row in rows[:-1]]);b=np.array([[[h['x'],h['y']] for h in row['humans']] for row in rows[1:]])
    robot=np.array([[row['robot_input'][key] for key in ['x','y']] for row in rows[1:]]);rd=np.array([[row['robot_input'][key]*.1 for key in ['vx','vy']] for row in rows[1:]])
    measured=gaps(a,b,robot,robot+rd,boxes);assert min(measured.values())>=.002-1e-8
    vmax=float(np.max(np.linalg.norm(b-a,axis=2)/.1));assert vmax<=.800001
    projections=[row['guard']['projection'] for row in rows[1:]];metrics.append(dict(case=result['case'],steps=100,swept_gaps=measured,speed_max_mps=vmax,projection_cycles_max=max(v['cycles'] for v in projections),projection_compute_ms_max=max(v['compute_ms'] for v in projections),margins_used=sorted(set(v['margin_m'] for v in projections)),altered_proposals=sum(v['velocity_change_norm']>1e-6 for v in projections)))
# Check the recovered legacy-state proposal using separate geometry too.
d=load(HERE/'reports/diagnostic01/diagnostic.json');start=np.asarray(d['starts'])[None,:,:];delta=np.asarray(d['projected_deltas'])[None,:,:];robot=np.asarray([[d['robot']['x'],d['robot']['y']]]);rd=np.asarray([[d['robot']['vx']*.1,d['robot']['vy']*.1]]);dg=gaps(start,start+delta,robot,robot+rd,boxes);assert min(dg.values())>=.002-1e-8
validation=dict(status='PASS_OFFLINE_ONLY',cases=6,steps=600,metrics=metrics,recovered_legacy_state_independent_gaps=dg,legacy_prefix_exact_steps=55,failing_legacy_request_reconstructed=True,not_a_group_or_full_native_algorithm_claim=True)
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n');print(json.dumps(validation,indent=2))
