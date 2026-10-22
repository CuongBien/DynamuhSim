#!/usr/bin/env python3
"""Report dynamic prefix evidence separately from full navigation outcome."""
import argparse,hashlib,importlib.util,json,math,re,shutil,sys
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
HERE=Path(__file__).resolve().parent;BENCH=HERE.parent
sys.path.insert(0,str(BENCH/'dataset_pilot'));from dataset_core import observed_bev
sys.path.insert(0,str(BENCH/'camera_height_ab'));from geometry import optical_transform
spec=importlib.util.spec_from_file_location('height_plots',BENCH/'camera_height_ab/analyze.py');plots=importlib.util.module_from_spec(spec);spec.loader.exec_module(plots)

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def rows(p):return [json.loads(x) for x in p.read_text().splitlines()]
def nearest(poses,t):return min(poses,key=lambda r:abs(r['time']-t))
def planar(origin,directions,humans):
    best=np.full(directions.shape[0],np.inf);a=np.sum(directions**2,axis=1)
    for h in humans:
        delta=np.asarray(origin)-[h['x'],h['y']];b=2*(directions@delta);c=delta@delta-.36**2;disc=b*b-4*a*c
        t=(-b-np.sqrt(np.maximum(disc,0)))/(2*a)
        best=np.minimum(best,np.where((disc>=0)&(t>0),t,np.inf))
    return best

def analyze(root,out):
    out.mkdir(parents=True,exist_ok=False);result=json.loads((root/'result.json').read_text());index=json.loads((root/'paired_depth/index.json').read_text())
    stamps=sorted(set(index['on'])&set(index['off']),key=int);active=[s for s in stamps if index['off'][s]['phase']=='active' and index['on'][s]['phase']=='active']
    poses=rows(root/'gazebo_poses.jsonl');scans=rows(root/'scans.jsonl');gt=rows(root/'ground_truth.jsonl');k=result['recording']['observed']['/camera/camera_info']['k']
    protocol=json.loads((HERE/'protocol.json').read_text())['motion'];metrics=[];raw={};max_offset=0;on_errors=[];scan_errors=[]
    for stamp in stamps:
        arrays={n:np.load(root/'paired_depth'/n/(stamp+'.npy'),allow_pickle=False) for n in ['on','off']}
        for n,a in arrays.items():
            assert a.shape==(240,424) and a.dtype==np.float32
            f=root/'paired_depth'/n/(stamp+'.npy');raw[str(f.relative_to(root))]=sha(f)
        a,b=arrays['on'],arrays['off'];finite=np.isfinite(a)&np.isfinite(b);delta=np.zeros_like(a);np.subtract(b,a,out=delta,where=finite)
        changed=int(np.sum((finite&(np.abs(delta)>.02))|(np.isfinite(a)!=np.isfinite(b))))
        row=dict(stamp_ns=int(stamp),phase=index['off'][stamp]['phase'],changed_pixels=changed);metrics.append(row)
        if stamp not in active:continue
        pose=nearest(poses,int(stamp)/1e9);max_offset=max(max_offset,abs(pose['time']-int(stamp)/1e9));robot=pose['poses']['mecanum_robot'];humans=[v for n,v in pose['poses'].items() if n.endswith('_proxy')]
        c,s=math.cos(robot['yaw']),math.sin(robot['yaw']);rotation=np.array([[c,-s],[s,c]])
        rays=np.stack([np.ones(424),-(np.arange(424)-k[2])/k[0]],axis=1)@rotation.T
        predicted=planar(np.array([robot['x'],robot['y']])+rotation@np.array([.12,0]),rays,humans)
        actual=a[115];valid=np.isfinite(predicted)&(predicted>=.28)&(predicted<=5)&np.isfinite(actual)
        on_errors.extend(np.abs(predicted[valid]-actual[valid]).tolist())
    for scan in scans:
        t=scan['time'];pose=nearest(poses,t);robot=pose['poses']['mecanum_robot'];humans=[v for n,v in pose['poses'].items() if n.endswith('_proxy')]
        angles=robot['yaw']+scan['angle_min']+np.arange(360)*scan['angle_increment'];directions=np.stack([np.cos(angles),np.sin(angles)],axis=1)
        predicted=planar([robot['x'],robot['y']],directions,humans);actual=np.array([np.nan if x is None else x for x in scan['ranges']])
        valid=np.isfinite(predicted)&(predicted>=scan['range_min'])&(predicted<=scan['range_max'])&np.isfinite(actual)
        scan_errors.extend(np.abs(predicted[valid]-actual[valid]).tolist())
    effect=sum(m['changed_pixels']>=protocol['minimum_changed_pixels_in_effect_frame'] for m in metrics if m['phase']=='active')
    log=(root/'launch.log').read_text(errors='replace');match=re.search(r'teardown pre=(\d+) post=(\d+)',log)
    balanced=bool(match and match[1]==match[2] and int(match[1])>0)
    robot0=poses[0]['poses']['mecanum_robot'];robot1=poses[-1]['poses']['mecanum_robot'];human_moves={}
    for n,h in poses[0]['poses'].items():
        if n.endswith('_proxy'):human_moves[n]=max(math.hypot(p['poses'][n]['x']-h['x'],p['poses'][n]['y']-h['y']) for p in poses if n in p['poses'])
    sensor_prefix=(len(stamps)>=protocol['minimum_paired_frames'] and len(active)>=protocol['minimum_active_paired_frames'] and effect>=protocol['minimum_effect_frames'] and balanced)
    clean=result['server_shutdown']['quiesce']['pause_acknowledged'] and result['server_shutdown']['quiesce']['simulation_time_stable'] and result['gazebo_clean_exit'] and result['server_shutdown']['status']=='STOPPED' and result['cleanup']['status']=='PASS' and not result['cleanup']['remaining']
    summary=dict(status='INCOMPLETE_NAVIGATION',sensor_prefix_checks='PASS' if sensor_prefix else 'FAIL',
        renderer_shutdown_checks='PASS' if clean else 'FAIL',navigation_accepted=False,training_ready=False,
        simulator_status=result['status'],error=result.get('error'),paired_frames=len(stamps),active_paired_frames=len(active),effect_active_frames=effect,
        callback_pre=int(match[1]) if match else None,callback_post=int(match[2]) if match else None,
        robot_displacement_m=math.hypot(robot1['x']-robot0['x'],robot1['y']-robot0['y']),human_max_displacement_m=human_moves,
        pose_alignment_max_s=max_offset,
        diagnostic_on_depth_proxy=dict(samples=len(on_errors),median_abs_error_m=float(np.median(on_errors)),fraction_within_3cm=float(np.mean(np.array(on_errors)<=.03)),caveat='Planar circle prediction at image row115, nearest measured pose; descriptive, not full dynamic camera equivalence'),
        diagnostic_lidar_proxy=dict(samples=len(scan_errors),median_abs_error_m=float(np.median(scan_errors)),fraction_within_5cm=float(np.mean(np.array(scan_errors)<=.05)),caveat='Native noise retained, possible occlusion and pose delay; not a paired no-noise LiDAR reference'),
        gpu_renderer=result['cleanup']['ogre_renderer_lines'],quiesce=result['server_shutdown']['quiesce'],faults=result['faults'],raw_pair_sha256=raw)
    selected=[active[0],active[len(active)//2],active[-1]];representative={}
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',16)
    for i,stamp in enumerate(selected):
        board=Image.new('RGB',(800,820),'white');draw=ImageDraw.Draw(board)
        draw.text((12,10),f'Actual moving-robot depth pair at {int(stamp)/1e9:.1f}s (native camera height)',font=font,fill='black')
        for j,name in enumerate(['on','off']):
            a=np.load(root/'paired_depth'/name/(stamp+'.npy'));representative[f'{i}_{name}']=a
            grid=observed_bev(a,k,optical_transform(.08));depth,bev=plots.pictures(a,grid);y=50+j*355
            draw.text((12,y),'Proxy '+name.upper()+(' (diagnostic control)' if name=='on' else ' in depth (experiment)'),font=font,fill='black');board.paste(depth,(12,y+30));board.paste(bev,(460,y+30))
        draw.text((12,770),'Measured prefix only: navigation stopped by bounded human avoidance guard.',font=font,fill='black');board.save(out/f'pair_{i}.png')
    np.savez_compressed(out/'representative_depths.npz',**representative)
    for name in ['initial.png','middle.png','final.png','camera_preview_4x.gif','result.json','events.jsonl']:
        if (root/name).exists():shutil.copyfile(root/name,out/name)
    (out/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    (out/'pair_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
    (out/'fault_state.json').write_text(json.dumps(result['faults'],indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ['raw_pair_sha256','faults','quiesce']},indent=2))
    return summary
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--capture',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();analyze(a.capture.resolve(),a.output.resolve())
