#!/usr/bin/env python3
"""Analyze paired raw GPU depth; show measurements, not generated illustrations."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from geometry import optical_transform,world_rays,floor_depth,cylinder_depth,proxy_interior
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent;REPO=HERE.parents[3]
sys.path.insert(0,str(REPO/'python'))
from dynamuhsim.datasets.normalization import observed_bev,depth_points


def dump(path,value):Path(path).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def pictures(depth,bev):
    valid=np.isfinite(depth)&(depth>=.28)&(depth<=5)
    rgb=np.zeros((*depth.shape,3),dtype=np.uint8);rgb[~valid]=[110,80,125]
    gray=(np.clip((depth[valid]-.28)/(5-.28),0,1)*255).astype(np.uint8)
    rgb[valid]=gray[:,None]
    b=np.zeros((64,64,3),dtype=np.uint8);b[bev==-1]=[80,80,80];b[bev==0]=[235,235,235];b[bev==1]=[220,70,40]
    d=Image.fromarray(rgb).resize((424,240),Image.Resampling.NEAREST)
    b=Image.fromarray(b[::-1,::-1]).resize((320,320),Image.Resampling.NEAREST)
    return d,b


def analyze(root,out):
    assert json.loads((root/'capture.json').read_text())['status']=='PASS'
    out.mkdir(parents=True,exist_ok=False)
    protocol=json.loads((root/'protocol.json').read_text());all_results=[]
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
    title_font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    for case in sorted(root.glob('snapshot*')):
        r=json.loads((case/'result.json').read_text());assert r['status']=='MEASURED'
        assert r['cleanup']['status']=='PASS' and r['gazebo_returncode']==0
        assert any('NVIDIA' in s for s in r['cleanup']['ogre_renderer_lines'])
        snapshot=r['source_snapshot'];robot=snapshot['robot'];humans=snapshot['humans']
        tree=ET.parse(case/'warehouse.sdf');world=tree.getroot().find('world')
        assert world.find("model[@name='mecanum_robot']/static").text=='true'
        assert not world.find("model[@name='mecanum_robot']").findall('plugin')
        for h in humans:
            m=world.find(f"model[@name='common_{h['id']}_proxy']")
            assert m.findtext('static')=='true'
        a=np.load(case/'depth_low.npy',allow_pickle=False);b=np.load(case/'depth_high.npy',allow_pickle=False)
        assert a.shape==b.shape==(10,240,424)
        board=Image.new('RGB',(800,850),'white');draw=ImageDraw.Draw(board)
        draw.text((12,8),f"Frozen snapshot {r['snapshot_index']} from episode t={snapshot['stamp_ns']/1e9:.4f}s",font=title_font,fill='black')
        draw.text((12,35),'Actual GPU depth at equal timestamps; no Nav2 / human stepping in this test',font=font,fill='black')
        panel=[];arrays={}
        inside=proxy_interior(robot,humans)
        for index,(label,z,frames) in enumerate([('low',.08,a),('high',.36,b)]):
            k=r['calibration'][label]['k'];depth=frames[len(frames)//2]
            u=ET.parse(case/f'robot_{label}.urdf')
            origin=np.array([float(x) for x in u.find("joint[@name='camera_joint']/origin").get('xyz').split()])
            np.testing.assert_allclose(origin,[.12,0,z],atol=1e-12)
            tf=next(t for t in r['tf_static'] if t['child']==f'camera_{label}_link')
            np.testing.assert_allclose(tf['xyz'],origin)
            sensor=world.find(f"model[@name='mecanum_robot']/link[@name='camera_link']/sensor[@name='height_{label}']")
            dz=float(sensor.findtext('pose').split()[2]);assert abs(.08+dz-z)<1e-12
            camera_origin,directions=world_rays(k,depth.shape,robot,z)
            expected_floor=floor_depth(camera_origin,directions)
            expected_proxy=cylinder_depth(camera_origin,directions,humans)
            finite=np.isfinite(depth)&(depth>=.28)&(depth<=5)
            with np.errstate(invalid='ignore'):
                floor=finite & np.isfinite(expected_floor) & (np.abs(depth-expected_floor)<.01)
                proxy=finite & np.isfinite(expected_proxy) & (np.abs(depth-expected_proxy)<.02)
            assert floor.sum()>=100, 'Too little floor to check height consistency'
            world_z=camera_origin[2]+depth[floor]*directions[...,2][floor]
            bev=observed_bev(depth,k,optical_transform(z))
            reference=frames[0];repeat_valid=np.isfinite(frames)&np.isfinite(reference)
            diff=np.zeros_like(frames);np.subtract(frames,reference,out=diff,where=repeat_valid)
            item=dict(label=label,world_camera_height_m=float(camera_origin[2]),
                finite_depth_fraction=float(finite.mean()),floor_match_fraction_of_valid=float(floor.sum()/finite.sum()),
                proxy_match_fraction_of_valid=float(proxy.sum()/finite.sum()),
                floor_world_z_abs_error_p99_m=float(np.quantile(np.abs(world_z),.99)),
                repeat_max_abs_diff_m=float(np.max(np.abs(diff))),
                repeat_finite_mask_changes=int(np.sum(np.isfinite(frames)!=np.isfinite(reference))),
                bev_unknown_cells=int((bev==-1).sum()),bev_free_cells=int((bev==0).sum()),bev_occupied_cells=int((bev==1).sum()),
                proxy_interior_cells=int(inside.sum()),proxy_interior_free_cells=int(np.sum(inside&(bev==0))),
                representative_stamp_ns=r['paired_stamp_ns'][len(frames)//2],
                camera_K_equal=True,urdf_sensor_tf_camera_extrinsics_agree=True,
                note='Proxy match is analytic depth agreement, not a complete semantic segmentation')
            assert item['floor_world_z_abs_error_p99_m']<.01
            assert item['repeat_max_abs_diff_m']<=1e-5 and item['repeat_finite_mask_changes']==0
            panel.append(item);arrays[label+'_bev']=bev
            depth_image,bev_image=pictures(depth,bev)
            top=70+index*355
            draw.text((12,top),f"{label.upper()} camera {camera_origin[2]*100:.0f} cm above floor",font=title_font,fill='black')
            board.paste(depth_image,(12,top+30));board.paste(bev_image,(460,top+30))
            draw.text((12,top+275),f"Valid depth {finite.mean()*100:.1f}% / cylinder agreement {item['proxy_match_fraction_of_valid']*100:.1f}%",font=font,fill='black')
            draw.text((12,top+299),f"BEV unknown/free/occupied: {item['bev_unknown_cells']}/{item['bev_free_cells']}/{item['bev_occupied_cells']}",font=font,fill='black')
        draw.text((12,790),'Depth: near dark, far light, invalid purple. BEV: unknown gray / free white / occupied red.',font=font,fill='black')
        draw.text((12,815),'BEV: x forward up; y left to left; identical 6.4m x 6.4m grid. Free area alone is not quality.',font=font,fill='black')
        board.save(out/(case.name+'_comparison.png'))
        np.savez(out/(case.name+'_bevs.npz'),**arrays)
        all_results.append(dict(snapshot=case.name,source_stamp_ns=snapshot['stamp_ns'],paired=panel))
    summary=dict(status='PASS',snapshots=len(all_results),camera_heights_m=[.12,.4],
                 paired_frames_per_camera=10,physical_robot_height_changed=False,
                 navigation_rerun=False,human_policy_rerun=False,
                 results=all_results,
                 algorithm_source_sha256=sha(REPO/'python/dynamuhsim/datasets/normalization.py'),
                 analysis_source_sha256={p.name:sha(p) for p in [Path(__file__),HERE/'geometry.py',HERE/'protocol.json']},
                 raw_sha256={str(p.relative_to(root)):sha(p) for p in sorted(root.rglob('*')) if p.is_file()},
                 caveats=protocol['limits'])
    dump(out/'summary.json',summary)
    print(json.dumps(all_results,indent=2))
    return summary

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();analyze(a.capture.resolve(),a.output.resolve())
