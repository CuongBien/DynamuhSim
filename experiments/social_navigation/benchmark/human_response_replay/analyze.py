#!/usr/bin/env python3
"""Measure and plot offline replay data. Every visual is labelled NOT GAZEBO."""
import argparse,hashlib,json,math,csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image,ImageDraw,ImageFont
p=argparse.ArgumentParser();p.add_argument('--report-dir',type=Path,required=True);a=p.parse_args();R=a.report_dir.resolve();sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
layout=json.loads((R/'inputs/layout.json').read_text())
def load(path):return [json.loads(line) for line in path.open()]
def circle_box(x,y,r,b):
    qx,qy=abs(x-b['x'])-b['sx']/2,abs(y-b['y'])-b['sy']/2
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-r

def footprint(ro,h):
    dx,dy=h['x']-ro['x'],h['y']-ro['y'];c,s=math.cos(ro['yaw']),math.sin(ro['yaw'])
    qx,qy=abs(c*dx+s*dy)-.12,abs(-s*dx+c*dy)-.10
    return math.hypot(max(qx,0),max(qy,0))+min(max(qx,qy),0)-.36

def metrics(rows,far):
    values=[];divergence=None;initial=rows[0]['human_before'];ids=[h['id'] for h in initial]
    for row,control in zip(rows,far):
        assert abs(row['state_time']-control['state_time'])<1e-9
        hs=row['humans'];ro=row['robot_next'];before=row['human_before'];inp=row['robot_input']
        assert [h['id'] for h in hs]==ids
        delta_v=max(math.hypot(h['vx']-j['vx'],h['vy']-j['vy']) for h,j in zip(hs,control['humans']))
        delta_p=max(math.hypot(h['x']-j['x'],h['y']-j['y']) for h,j in zip(hs,control['humans']))
        center=(sum(h['x'] for h in hs)/3,sum(h['y'] for h in hs)/3)
        pairs=[math.hypot(h['x']-j['x'],h['y']-j['y'])-.72 for i,h in enumerate(hs) for j in hs[i+1:]]
        gaze=[abs(math.atan2(math.sin(h['yaw']-math.atan2(center[1]-h['y'],center[0]-h['x'])),math.cos(h['yaw']-math.atan2(center[1]-h['y'],center[0]-h['x'])))) for h in hs]
        v=dict(time=row['state_time'],circle_gap=min(math.hypot(h['x']-ro['x'],h['y']-ro['y'])-.61 for h in hs),footprint_gap=min(footprint(ro,h) for h in hs),pair_gap=min(pairs),human_machine_gap=min(circle_box(h['x'],h['y'],.36,b) for h in hs for b in layout['obstacles']),delta_velocity_from_far=delta_v,delta_position_from_far=delta_p,group_radius=max(math.hypot(h['x']-center[0],h['y']-center[1]) for h in hs),max_pair_distance=max(pairs)+.72,inward_gaze_mean_rad=sum(gaze)/3)
        values.append(v)
        if divergence is None and delta_v>.001:
            divergence=dict(input_time_s=row['input_time'],state_time_s=row['state_time'],velocity_difference_mps=delta_v,position_difference_m=delta_p,nominal_circle_gap_before_update_m=min(math.hypot(h['x']-inp['x'],h['y']-inp['y'])-.61 for h in before),definition='First actual human velocity difference >0.001m/s against same-policy far trace; proxy pre-overlap metric, not native internal detection time or human intent.')
    result=dict(min_robot_human_circle_m=min(v['circle_gap'] for v in values),min_robot_human_footprint_m=min(v['footprint_gap'] for v in values),min_human_pair_m=min(v['pair_gap'] for v in values),min_human_machine_m=min(v['human_machine_gap'] for v in values),robot_footprint_overlap_beyond_1cm_s=.1*sum(v['footprint_gap']<-.01 for v in values),human_pair_overlap_beyond_1cm_s=.1*sum(v['pair_gap']<-.01 for v in values),max_position_difference_from_far_m=max(v['delta_position_from_far'] for v in values),max_velocity_difference_from_far_mps=max(v['delta_velocity_from_far'] for v in values),first_velocity_divergence=divergence,max_displacement_m=max(math.hypot(h['x']-j['x'],h['y']-j['y']) for row in rows for h,j in zip(row['humans'],initial)),max_group_radius_m=max(v['group_radius'] for v in values),max_pair_distance_m=max(v['max_pair_distance'] for v in values),max_inward_gaze_mean_deg=max(v['inward_gaze_mean_rad'] for v in values)*180/math.pi,completed_legs=[h.get('completed_legs') for h in rows[-1]['humans']])
    return result,values

results=[];all_rows={};manifest=[]
for framework in ['crowdsimplus','socnavgym']:
    files=sorted((R/framework).glob('**/result.json'))
    assert len(files)==(6 if framework=='crowdsimplus' else 18)
    for path in files:
        item=json.loads(path.read_text());directory=path.parent;rows=load(directory/'trajectory.jsonl');far=load(directory.parent/'far/trajectory.jsonl')
        assert item['status']=='PASS' and len(rows)==200 and item['determinism_max_error']<1e-10
        assert item['input_sha256']==sha(R/'inputs'/f"{item['case']}.jsonl")
        key=str(directory.relative_to(R));row=dict(key=key,framework=item['framework'],case=item['case'],policy=item.get('policy',item.get('variant')),mode=item.get('mode','independent walkers'),**metrics(rows,far)[0]);results.append(row);all_rows[key]=rows
        manifest.append(dict(key=key,result_sha256=sha(path),trajectory_sha256=sha(directory/'trajectory.jsonl'),input_sha256=item['input_sha256']))
        (directory/'metrics.json').write_text(json.dumps(row,indent=2,allow_nan=False));(directory/'metric_timeline.json').write_text(json.dumps(metrics(rows,far)[1]))
# Check the geometry-only control before attributing differences to the modified group solver.
checks=[]
for mode in ['walking','conversation']:
    for case in ['far','head_on','crossing']:
        traces={variant:all_rows[f'socnavgym/{mode}/{variant}/{case}'] for variant in ['native','circle_contact','robot_orca']}
        error=max(abs(h[k]-j[k]) for left,right in [(traces['circle_contact'],traces['robot_orca'])] for u,v in zip(left,right) for h,j in zip(u['humans'],v['humans']) for k in ['x','y','vx','vy'])
        if mode=='conversation' or case=='far':assert error<1e-10,(mode,case,error)
        checks.append(dict(mode=mode,case=case,circle_vs_robot_orca_max_state_difference=error,negative_control=(mode=='conversation' or case=='far')))
(R/'summary.json').write_text(json.dumps(dict(results=results,group_controls=checks),indent=2,allow_nan=False))
(R/'manifest.json').write_text(json.dumps(dict(analysis_source_sha256=sha(Path(__file__)),trials=manifest,media_provenance='Matplotlib/PIL visualizations of offline simulator states with scripted robot inputs, NOT Gazebo camera frames, screen recordings, or AI-generated images.'),indent=2))
with (R/'results.csv').open('w',newline='') as stream:
    keys=[k for k,v in results[0].items() if not isinstance(v,(list,dict))];writer=csv.DictWriter(stream,keys,extrasaction='ignore',lineterminator='\n');writer.writeheader();writer.writerows(results)
fig,axes=plt.subplots(2,2,figsize=(12,7),sharex=True)
for col,case in enumerate(['head_on','crossing']):
    for policy,color in [('orca_plus','#d36523'),('sfm','#2475b4')]:
        rows=all_rows[f'crowdsimplus/{policy}/{case}'];far=all_rows[f'crowdsimplus/{policy}/far'];_,values=metrics(rows,far)
        axes[0,col].plot([v['time'] for v in values],[v['footprint_gap'] for v in values],label=policy,color=color)
        axes[1,col].plot([v['time'] for v in values],[v['delta_velocity_from_far'] for v in values],label=policy,color=color)
    axes[0,col].set_title(case);axes[0,col].axhline(0,c='red',ls=':');axes[1,col].set_xlabel('input simulation seconds')
for ax in axes.flat:ax.grid(alpha=.2);ax.legend()
axes[0,0].set_ylabel('robot footprint to human proxy gap [m]');axes[1,0].set_ylabel('velocity divergence from far control [m/s]');fig.suptitle('OFFLINE REPLAY: scripted robot input, NOT Gazebo camera');fig.tight_layout();fig.savefig(R/'crowd_response.png',dpi=150);plt.close(fig)
# Fast diagram animations; no camera or photo editing is involved.
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13);big=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',17)
colors=['#2274af','#c45c25','#248354']
def animation(name,keys):
    traces=[all_rows[key] for key in keys];width=410*len(keys);frames=[]
    def point(x,y,offset):return (offset+30+(x+29.8)/12.8*350,80+(5.12-y)*350/12.8)
    for index in range(0,200,2):
        image=Image.new('RGB',(width,420),'white');draw=ImageDraw.Draw(image)
        draw.text((12,8),'OFFLINE REPLAY | scripted robot | NOT GAZEBO CAMERA',font=big,fill='black')
        for panel,(key,trace) in enumerate(zip(keys,traces)):
            off=panel*410;row=trace[index];draw.text((off+12,38),key.replace('socnavgym/','').replace('crowdsimplus/',''),font=font,fill='black')
            for box in layout['obstacles']:
                x0,y0=point(box['x']-box['sx']/2,box['y']+box['sy']/2,off);x1,y1=point(box['x']+box['sx']/2,box['y']-box['sy']/2,off)
                x0,x1=max(off+30,x0),min(off+380,x1);y0,y1=max(80,y0),min(360,y1)
                if x0<x1 and y0<y1:draw.rectangle((x0,y0,x1,y1),fill='#dedede')
            draw.rectangle((off+30,80,off+380,360),outline='#777777')
            ro=row['robot_next'];x,y=point(ro['x'],ro['y'],off)
            if off+30<=x<=off+380 and 80<=y<=360:
                rr=.25*350/12.8;draw.ellipse((x-rr,y-rr,x+rr,y+rr),fill='#ae3030',outline='black')
            for hi,h in enumerate(row['humans']):
                x,y=point(h['x'],h['y'],off);rr=.36*350/12.8
                if off+30<=x<=off+380 and 80<=y<=360:
                    draw.ellipse((x-rr,y-rr,x+rr,y+rr),fill=colors[hi],outline='black');draw.line((x,y,x+15*math.cos(h['yaw']),y-15*math.sin(h['yaw'])),fill='black',width=2);draw.text((x-4,y-7),str(h['id']),font=font,fill='white')
            draw.text((off+15,372),f"t={row['state_time']:.1f}s | seed42 | 1x sim speed",font=font,fill='black')
            if name.startswith('crowd_far'):draw.text((off+15,394),'Far robot at (26,0), outside this view',font=font,fill='black')
        frames.append(image.quantize(colors=96))
    frames[0].save(R/(name+'.gif'),save_all=True,append_images=frames[1:],duration=200,loop=0);frames[len(frames)//2].convert('RGB').save(R/(name+'.png'))
for case in ['far','head_on','crossing']:animation('crowd_'+case,[f'crowdsimplus/{policy}/{case}' for policy in ['orca_plus','sfm']])
for mode,case in [('walking','head_on'),('walking','crossing'),('conversation','head_on')]:animation('group_'+mode+'_'+case,[f'socnavgym/{mode}/{variant}/{case}' for variant in ['native','circle_contact','robot_orca']])
lines=['# Phản ứng người với cùng đầu vào robot — offline replay','',
'Đã chạy6 probe CrowdSimPlus (ORCA+/SFM × far/head-on/crossing),20sim s mỗi probe,dt0.1s,seed42. Cùng trace robot world-frame có201 mẫu pose/vận tốc được dùng byte-for-byte cho hai policies. Robot input được lập trình trước, không ghi từ Gazebo; không chạy ROS/Nav2/AMCL.','',
'Ảnh/GIF ở đây là diagram vẽ từ trạng thái simulator đã chạy; có nhãn OFFLINE REPLAY, không phải camera Gazebo hoặc ảnh AI. GIF lấy mỗi2 update và phát200ms/frame (1x sim time), có thể gộp frame trùng nhau. Diagram dùng tỷ lệ XY bằng nhau; vòng đỏ là envelope robot0.25m, các vòng màu là proxy người0.36m. Footprint chữ nhật được đo riêng trong bảng.','',
'## CrowdSimPlus: sáu probe','',
'| Policy | Robot input | Khe robot footprint–người min (m) | Overlap robot >1cm (s) | Khe người–người min (m) | Overlap người >1cm (s) | Đổi vận tốc so far lần đầu (s) |',
'|---|---|---:|---:|---:|---:|---:|']
for row in results:
    if row['framework']!='CrowdSimPlus':continue
    event=row['first_velocity_divergence'];when=f"{event['input_time_s']:.1f}" if event else '—'
    lines.append(f"| {row['policy']} | {row['case']} | {row['min_robot_human_footprint_m']:.4f} | {row['robot_footprint_overlap_beyond_1cm_s']:.1f} | {row['min_human_pair_m']:.4f} | {row['human_pair_overlap_beyond_1cm_s']:.1f} | {when} |")
lines+=['','![Độ hở và thay đổi vận tốc](crowd_response.png)','','![Replay head-on](crowd_head_on.gif)','','Cả6 probe được đối chiếu200 update với human integration của full upstream env.step, rồi chạy lại từ đầu cùng seed để kiểm tính xác định. Ngoại lệ endpoint: cùng luật patrol bên ngoài được áp dụng sau bước native cho cả hai phía; không tuyên bố parity của reward, episode loop hoặc robot planning. `metadata.robot_controller` trong kernel tái dùng là mô tả adapter Gazebo, được ghi đè ngữ cảnh bằng execution_context trong result.','',
'Đổi vận tốc là lần đầu norm chênh actual velocity so với cùng-policy far control vượt0.001m/s; không phải thời điểm phát hiện nội bộ hoặc suy luận ý định. Khoảng hở trước update có trong summary để kiểm khác biệt có xuất hiện trước overlap proxy không.','',
'## SocNavGym: một nhóm3 người','',
'Chạy18 probe: walking/conversation × native/circle_contact/robot_orca × cùng3 input robot. Circle_contact giữ đối chứng hình học; robot_orca thêm robot vào solver nhóm, không sửa vendor. Formation/dispersal không thuộc adapter này.','',
'| Mode | Biến thể | Input | Khe robot footprint–người min (m) | Overlap robot >1cm (s) | Khe người–người min (m) | Chênh vị trí lớn nhất so far (m) |',
'|---|---|---|---:|---:|---:|---:|']
for row in results:
    if row['framework']!='SocNavGym':continue
    lines.append(f"| {row['mode']} | {row['policy']} | {row['case']} | {row['min_robot_human_footprint_m']:.4f} | {row['robot_footprint_overlap_beyond_1cm_s']:.1f} | {row['min_human_pair_m']:.4f} | {row['max_position_difference_from_far_m']:.4f} |")
lines+=['','![Walking group](group_walking_head_on.gif)','','![Conversation group](group_conversation_head_on.gif)','','Các IDs/membership được giữ cố định bởi scene; không diễn giải như formation tự phát. Conversation là đứng quanh tâm/hướng vào nhau, không có thoại ngôn ngữ. Ba biến thể conversation không có solver đi nhóm khác nhau; negative control được đối chiếu. Group objectives khác private endpoints của CrowdSimPlus, nên không xếp hạng hai framework bằng clearance ở bảng này.','',
'## Giới hạn và bằng chứng','',
'Robot replay không né người: nếu va chạm, đầu vào vẫn giữ nguyên. Đó là phép đo phản ứng một chiều, không chứng minh Nav2 sẽ va chạm như vậy. Khoảng hở dùng robot footprint0.24×0.20m, người r0.36m; circle robot0.25m lưu riêng. Native contact có hình học khác; không gộp signed proxy gap với event contact nội bộ. Đo trước và sau update; không chứng nhận continuous collision giữa samples.','',
'`protocol.json`, `group_protocol.json`, `fixture_checks.json`, `manifest.json`, `summary.json`, `results.csv` và từng trajectory/result giữ input/source hashes, native parity và mọi overlap. Native human policies, ROS robot source và baseline AMCL giữ nguyên. Chưa dùng các probe nhỏ để chọn model production; ComplexityNav vẫn chưa nối Gazebo.','']
(R/'REPLAY_REPORT.md').write_text('\n'.join(lines))
print(json.dumps(dict(status='PASS',crowd_probes=6,group_probes=18,group_controls=checks,summary=[{k:r[k] for k in ['framework','policy','mode','case','min_robot_human_footprint_m','min_human_pair_m','first_velocity_divergence']} for r in results]),indent=2))
