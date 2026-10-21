"""Reconstruct circular yaw consistently before native KD-tree clustering.

ParticleCloud quaternions do not retain the internal number of whole turns.
This independent circular-mean lift is compared with principal-yaw clustering;
it is a documented reconstruction, not recovery of the hidden PF state.
"""
import json,math,os,subprocess,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
sys.path.insert(0,str(BENCH/'localization_diagnosis'))
from geometry import stamp,yaw

def recluster(runtime,canonical_rows):
    raw={}
    for line in (runtime/'diagnostics.jsonl').open():
        r=json.loads(line)
        if r['kind']=='particle_cloud':raw[round(stamp(r['data']),6)]=r['data']['particles']
    lines=[];lifts=[]
    for r in canonical_rows:
        particles=raw[round(r['time'],6)];angles=np.array([yaw(p['pose']['orientation']) for p in particles]);weights=np.array([p['weight'] for p in particles]);resultant=complex(float(np.dot(weights,np.cos(angles))),float(np.dot(weights,np.sin(angles))))
        anchor=math.atan2(resultant.imag,resultant.real);lifted=anchor+np.arctan2(np.sin(angles-anchor),np.cos(angles-anchor))
        lines.append(str(len(particles)))
        for part,angle in zip(particles,lifted):
            po=part['pose']['position'];lines.append(' '.join(map(str,[po['x'],po['y'],angle,part['weight']])))
        lifts.append(dict(circular_anchor_yaw=anchor,circular_resultant_length=abs(resultant),particles_lifted=int(np.sum(np.abs(lifted-angles)>1.))))
    env=os.environ.copy();env['LD_LIBRARY_PATH']='/opt/ros/jazzy/lib'
    result=subprocess.run([str(BENCH/'navigation_diagnostics/generated/native_clusters')],input='\n'.join(lines)+'\n',capture_output=True,text=True,env=env,check=True)
    output=result.stdout.splitlines();assert len(output)==len(canonical_rows)
    rows=[]
    for original,line,lift in zip(canonical_rows,output,lifts):
        values=list(map(float,line.split()));n=int(values[0]);assert len(values)==1+n*4
        clusters=[dict(weight=values[1+4*i],x=values[2+4*i],y=values[3+4*i],yaw=values[4+4*i]) for i in range(n)];clusters.sort(key=lambda c:-c['weight'])
        r=dict(original);r['principal_yaw_clusters']=r['clusters'];r['clusters']=clusters;r.update(lift)
        r['principal_yaw_max_cluster_amcl_error_m']=original.get('max_cluster_amcl_error_m')
        if 'amcl' in r:r['max_cluster_amcl_error_m']=math.hypot(clusters[0]['x']-r['amcl']['x'],clusters[0]['y']-r['amcl']['y'])
        rows.append(r)
    return rows
