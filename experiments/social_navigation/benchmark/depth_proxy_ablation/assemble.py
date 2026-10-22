#!/usr/bin/env python3
"""Assemble complete scenes explicitly; preserve failed runs and raw provenance."""
import argparse,hashlib,json,shutil
from pathlib import Path
from fixture import HERE,REPO

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def assemble(output,selections):
    assert not output.exists(), 'Never overwrite prior evidence'
    protocol=json.loads((HERE/'protocol.json').read_text())
    core=['schema','conditions','proxy_flag','all_mask','depth_no_proxy_mask','cameras','conditions_detail','validity','scope']
    expected=json.loads((HERE/'snapshots.json').read_text());sources={};validated=[]
    for index,root in selections:
        frozen=json.loads((root/'protocol.json').read_text())
        assert all(frozen[k]==protocol[k] for k in core), 'Physical or acceptance protocol differs'
        scene=root/f'snapshot{index:02d}'
        hashes={}
        for condition in protocol['conditions']:
            case=scene/condition;r=json.loads((case/'result.json').read_text())
            assert r['snapshot']==expected[index-1] and r['snapshot_index']==index
            assert r['status']=='MEASURED' and r['gazebo_returncode']==0
            assert r['cleanup']['status']=='PASS' and not r['cleanup']['remaining']
            for name in ['depth_proxy_filter.cc','fixture.py','generated/build_jazzy/libdepth_proxy_filter.so']:
                relative=str((HERE/name).relative_to(REPO))
                assert r['source_sha256'][relative]==sha(HERE/name), 'Render implementation differs'
            for f in case.rglob('*'):
                if f.is_file():hashes[str(f.relative_to(scene))]=sha(f)
        assert index not in sources
        sources[index]=dict(root=str(root.relative_to(REPO)),scene=str(scene.relative_to(REPO)),files_sha256=hashes)
        validated.append((index,scene))
    assert validated
    output.mkdir(parents=True)
    for index,scene in validated:
        destination=output/f'snapshot{index:02d}';shutil.copytree(scene,destination)
        for relative,digest in sources[index]['files_sha256'].items():assert sha(destination/relative)==digest
    (output/'protocol.json').write_text(json.dumps(protocol,indent=2)+'\n')
    (output/'lineage.json').write_text(json.dumps(sources,indent=2)+'\n')
    status=dict(status='PASS',scope='Complete accepted scenes only, not success of every attempted run',
                snapshot_indices=[i for i,_ in validated],planned_snapshot_indices=[1,2,3],
                planned_cohort_complete=len(validated)==3,fixtures=3*len(validated),
                source_runs_complete=False,excluded='Snapshot03 lacks clean shutdown; run06 startup and run07/run08 shutdown failures retained')
    (output/'capture.json').write_text(json.dumps(status,indent=2)+'\n')
    return status

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--scene',action='append',required=True,help='INDEX=RUN_ROOT');a=p.parse_args()
    selections=[(int(s.split('=',1)[0]),Path(s.split('=',1)[1]).resolve()) for s in a.scene]
    print(json.dumps(assemble(a.output.resolve(),selections),indent=2))
