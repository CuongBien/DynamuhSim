#!/usr/bin/env python3
"""Depth proxy ablation, retained-human negative control, LiDAR and collision parity."""
import argparse,hashlib,json,sys,importlib.util
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from fixture import HERE,BENCH,REPO,HEIGHT
sys.path.insert(0,str(REPO/'python'));from dynamuhsim.datasets.normalization import observed_bev
sys.path.insert(0,str(HEIGHT));from geometry import optical_transform,proxy_interior
plot_spec=importlib.util.spec_from_file_location('height_plot_helpers',HEIGHT/'analyze.py')
plot_helpers=importlib.util.module_from_spec(plot_spec);plot_spec.loader.exec_module(plot_helpers)
pictures=plot_helpers.pictures


def dump(p,v):Path(p).write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def diff(a,b,threshold=.02):
    fa,fb=np.isfinite(a),np.isfinite(b);both=fa&fb;d=np.zeros_like(a)
    np.subtract(b,a,out=d,where=both)
    return dict(max_abs_diff_m=float(np.max(np.abs(d))),finite_mask_changes=int(np.sum(fa!=fb)),changed_pixels=int(np.sum((both&(np.abs(d)>threshold))|(fa!=fb)))),d

def analyze(root,out):
    assert json.loads((root/'capture.json').read_text())['status']=='PASS'
    out.mkdir(parents=True,exist_ok=False);protocol=json.loads((root/'protocol.json').read_text())
    tol=protocol['validity']['reference_depth_tolerance_m'];checks=[];results=[];raw_files={};representative={}
    cases=sorted(root.glob('snapshot*'));assert cases, 'No completed snapshot to analyze'
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',14)
    titlefont=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    for case in cases:
        meta={c:json.loads((case/c/'result.json').read_text()) for c in protocol['conditions']}
        assert all(r['status']=='MEASURED' and r['gazebo_returncode']==0 and r['cleanup']['status']=='PASS' and not r['cleanup']['remaining'] for r in meta.values())
        assert all(any('NVIDIA' in l for l in r['cleanup']['ogre_renderer_lines']) for r in meta.values())
        collision_same=len({r['fixture']['collision']['sha256'] for r in meta.values()})==1
        checks.append(collision_same)
        q=out/case.name;q.mkdir()
        for condition,r in meta.items():dump(q/(condition+'_result.json'),r)
        scene=meta['masked']['snapshot'];inside=proxy_interior(scene['robot'],scene['humans'])
        row=dict(snapshot=case.name,source_stamp_ns=scene['stamp_ns'],collision_definitions_equal=collision_same,camera=[])
        for level,z in [('low',.08),('high',.36)]:
            baseline=np.load(case/'baseline'/f'depth_{level}_on.npy',allow_pickle=False)
            on=np.load(case/'masked'/f'depth_{level}_on.npy',allow_pickle=False)
            off=np.load(case/'masked'/f'depth_{level}_off.npy',allow_pickle=False)
            null=np.load(case/'null_visuals'/f'depth_{level}_null.npy',allow_pickle=False)
            assert baseline.shape==on.shape==off.shape==null.shape==(10,240,424)
            for name,stack in [('baseline',baseline),('on',on),('off',off),('null',null)]:
                representative[case.name+'_'+level+'_'+name]=stack[5]
            parity,_=diff(baseline,on,tol)
            effect,delta=diff(on,off)
            human,dh=diff(null,off)
            foff,fnull=np.isfinite(off),np.isfinite(null)
            nearer=foff&((fnull&(dh<-.02))|np.isposinf(null))
            parity_ok=parity['finite_mask_changes']==0 and parity['max_abs_diff_m']<=tol
            effects=effect['changed_pixels']>0
            humanoid_present=bool(effects and nearer.any())
            checks.append(parity_ok)
            # First two scenes contain front human proxies; last is an out-of-front-view control.
            if meta['masked']['snapshot_index']<=2:checks.extend([effects,humanoid_present])
            else:checks.append(not effects)  # Empty front view must remain unchanged.
            k=meta['masked']['calibration'][level+'_off']['k']
            grids={name:observed_bev(stack[5],k,optical_transform(z)) for name,stack in [('on',on),('off',off),('null',null)]}
            item=dict(level=level,world_camera_height_m=z+.04,
                original_vs_tagged_all_camera=parity,reference_parity_pass=parity_ok,
                paired_on_vs_off=effect,
                off_vs_null_visuals=human,
                retained_human_nearer_pixels_per_frame=float(nearer.sum()/10),
                human_visual_retained=humanoid_present,
                changed_pixels_per_frame=float(effect['changed_pixels']/10),
                bev={name:dict(unknown=int((g==-1).sum()),free=int((g==0).sum()),occupied=int((g==1).sum()),free_in_conservative_proxy_disc=int(np.sum(inside&(g==0)))) for name,g in grids.items()})
            row['camera'].append(item)
            board=Image.new('RGB',(800,850),'white');draw=ImageDraw.Draw(board)
            draw.text((12,8),f"{case.name}: same camera {100*(z+.04):.0f} cm above floor",font=titlefont,fill='black')
            draw.text((12,35),'Actual GPU depth at paired time; proxy collision and LiDAR retained',font=font,fill='black')
            for i,(name,stack) in enumerate([('on',on),('off',off)]):
                top=70+i*355;d,b=pictures(stack[5],grids[name])
                draw.text((12,top),'PROXY '+name.upper()+(' in depth' if name=='on' else ' excluded (experimental)'),font=titlefont,fill='black')
                board.paste(d,(12,top+30));board.paste(b,(460,top+30))
                valid=np.isfinite(stack[5])&(stack[5]>=.28)&(stack[5]<=5)
                g=item['bev'][name]
                draw.text((12,top+275),f"Valid depth {valid.mean()*100:.1f}%",font=font,fill='black')
                draw.text((12,top+299),f"BEV unknown/free/occupied: {g['unknown']}/{g['free']}/{g['occupied']}",font=font,fill='black')
            draw.text((12,790),'Depth near dark / far light / invalid purple. BEV unknown gray / free white / occupied red.',font=font,fill='black')
            draw.text((12,815),f"Human mesh evidence vs null: {nearer.sum()/10:.0f} nearer pixels/frame. No Nav2 or human policy step.",font=font,fill='black')
            board.save(out/(case.name+'_'+level+'_comparison.png'))
            np.savez(q/(level+'_bevs.npz'),**grids)
        lidar={}
        for name in ['native','geometry']:
            a=np.load(case/'baseline'/f'scan_{name}.npz')['ranges'];b=np.load(case/'masked'/f'scan_{name}.npz')['ranges'];c=np.load(case/'null_visuals'/f'scan_{name}.npz')['ranges']
            assert a.shape==b.shape==c.shape==(5,360)
            metrics,_=diff(a,b,.001);empty,_=diff(b,c,.05)
            both=np.isfinite(a)&np.isfinite(b);temp=np.zeros_like(a);np.subtract(b,a,out=temp,where=both);delta=temp[both]
            lidar[name]=dict(baseline_vs_masked=metrics,masked_vs_null=empty,
                            difference_std_m=float(np.std(delta)),difference_median_abs_m=float(np.median(np.abs(delta))))
            if name=='geometry':
                lidar['geometry']['parity_pass']=metrics['finite_mask_changes']==0 and metrics['max_abs_diff_m']<=protocol['validity']['lidar_geometry_tolerance_m']
                checks.append(lidar['geometry']['parity_pass'])
        row['lidar']=lidar;results.append(row)
        for f in case.rglob('*'):
            if f.is_file():raw_files[str(f.relative_to(root))]=dict(bytes=f.stat().st_size,sha256=sha(f))
    np.savez_compressed(out/'representative_depths.npz',**representative)
    summary=dict(status='PASS' if all(checks) else 'FAIL',accepted_scope=json.loads((root/'capture.json').read_text()),results=results,fixtures=3*len(results),
                 depth_frames=80*len(results),native_scan_frames=15*len(results),diagnostic_scan_frames=15*len(results),
                 runtime_capability='static sensor-selective mask only',
                 default_project_changed=False,training_ready=False,navigation_tested=False,
                 collision_dynamics_tested=False,raw_files=raw_files,
                 analysis_source_sha256={str(p.relative_to(REPO)):sha(p) for p in [Path(__file__),HERE/'protocol.json',HEIGHT/'geometry.py',HEIGHT/'analyze.py',REPO/'python/dynamuhsim/datasets/normalization.py']},
                 caveats=protocol['limits'])
    dump(out/'summary.json',summary)
    dump(out/'protocol.json',protocol)
    print(json.dumps(dict(status=summary['status'],results=results),indent=2))
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    result=analyze(a.capture.resolve(),a.output.resolve())
    raise SystemExit(0 if result['status']=='PASS' else 1)
