#!/usr/bin/env python3
"""Independent raw-evidence audit; no analyzer, plotting or fixture helpers imported."""
import argparse,hashlib,json,os,re,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def collision_rows(path):
    w=ET.parse(path).getroot().find('world');rows=[]
    for m in w.findall('model'):
        for l in m.findall('link'):
            for c in l.findall('collision'):
                for item in c.iter():
                    item.text=(item.text or '').strip();item.tail=None
                rows.append((m.get('name'),m.findtext('pose'),m.findtext('static'),l.get('name'),l.findtext('pose'),ET.tostring(c)))
    return rows

def audit(root,report):
    summary=json.loads((report/'summary.json').read_text());lineage=json.loads((root/'lineage.json').read_text())
    assert summary['status']=='PASS' and summary['accepted_scope']['planned_cohort_complete'] is False
    representative=np.load(report/'representative_depths.npz',allow_pickle=False);checked=0;sessions=set()
    for relative,item in summary['raw_files'].items():
        f=root/relative;assert f.stat().st_size==item['bytes'] and digest(f)==item['sha256'];checked+=1
    maximum=0.;human_evidence={};count=0
    for case in sorted(root.glob('snapshot*')):
        index=int(case.name.replace('snapshot',''));origin=lineage[str(index)];source=Path(origin['scene'])
        for relative,h in origin['files_sha256'].items():
            assert digest(case/relative)==h==digest(source/relative)
        conditions=['baseline','masked','null_visuals'];results={c:json.loads((case/c/'result.json').read_text()) for c in conditions}
        collisions=[collision_rows(case/c/'warehouse.sdf') for c in conditions]
        assert collisions[0]==collisions[1]==collisions[2] and len(collisions[0])==55
        scans=[np.load(case/c/'scan_geometry.npz')['ranges'] for c in ['baseline','masked']]
        assert scans[0].shape==scans[1].shape==(5,360) and scans[0].tobytes()==scans[1].tobytes()
        for c,r in results.items():
            assert r['status']=='MEASURED' and r['gazebo_returncode']==0 and r['cleanup']['status']=='PASS' and not r['cleanup']['remaining']
            assert any('NVIDIA' in line for line in r['cleanup']['ogre_renderer_lines'])
            sessions.add(r['cleanup']['session']);count+=1
            if c=='masked':
                hook='\n'.join(r['render_hook_log']);assert 'proxies=6' in hook
                match=re.search(r'teardown pre=(\d+) post=(\d+)',hook)
                assert match and match[1]==match[2] and int(match[1])>=20
        for level in ['low','high']:
            arrays={name:np.load(case/c/f'depth_{level}_{suffix}.npy',allow_pickle=False) for name,c,suffix in [('baseline','baseline','on'),('on','masked','on'),('off','masked','off'),('null','null_visuals','null')]}
            assert arrays['baseline'].tobytes()==arrays['on'].tobytes()
            for name,a in arrays.items():
                assert a.shape==(10,240,424) and a.dtype==np.float32
                assert a[5].tobytes()==representative[f'{case.name}_{level}_{name}'].tobytes()
                assert np.array_equal(np.isfinite(a),np.broadcast_to(np.isfinite(a[0]),a.shape))
                finite=np.isfinite(a)&np.isfinite(a[0]);d=np.zeros_like(a);np.subtract(a,a[0],out=d,where=finite)
                maximum=max(maximum,float(np.max(np.abs(d))));assert np.max(np.abs(d))<=.0001
            off,null=arrays['off'],arrays['null'];good=np.isfinite(off)&np.isfinite(null);d=np.zeros_like(off);np.subtract(off,null,out=d,where=good)
            nearer=np.isfinite(off)&((good&(d<-.02))|np.isposinf(null));assert nearer.any()
            human_evidence[f'{case.name}_{level}']=float(nearer.sum()/10)
    live=[]
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:data=(proc/'environ').read_bytes()
        except (OSError,PermissionError):continue
        if any(('DYNAMUHSIM_EXPERIMENT_SESSION='+s).encode() in data for s in sessions):live.append(int(proc.name))
    assert not live
    result=dict(status='PASS',scope='Two complete static scenes only; third scene and dynamic rollout NOT accepted',
                verified_raw_files=checked,clean_gpu_fixtures=count,representative_images=len(representative.files),
                max_static_depth_delta_m=maximum,original_on_depth_bitwise_equal=True,diagnostic_lidar_bitwise_equal=True,
                collision_definition_count=55,collision_definitions_equal=True,retained_human_pixels_per_frame=human_evidence,
                owned_processes_remaining=live,planned_three_scene_cohort_complete=False,
                auditor_sha256=digest(Path(__file__)))
    (report/'audit.json').write_text(json.dumps(result,indent=2)+'\n');return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--report',type=Path,required=True);a=p.parse_args()
    print(json.dumps(audit(a.capture.resolve(),a.report.resolve()),indent=2))
