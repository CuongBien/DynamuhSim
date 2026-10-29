"""Additional diagnostic: virtual group-disc overlap and abrupt ORCA intents.

Not a frozen social acceptance gate or proof of a sole instability cause.
Actual human clearances are evaluated separately from these virtual discs.
"""
import argparse,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();plan=json.loads((HERE/'plan.json').read_text());report=[]
for case in [c for c in plan['cases'] if c['human_motion_profile']=='group_orca_v1']:
 for condition,root,name in [('native_wall',ROOT/plan['reference_root'],case['reference_id']),('sim_v1',ROOT/plan['raw_root'],case['id'])]:
  run=root/name;end=next(json.loads(v)['stamp_ns']/1e9 for v in (run/'episode/events.jsonl').read_text().splitlines() if json.loads(v)['event']=='action_end');rows=[json.loads(v) for v in (run/'runtime/worker_trajectory.jsonl').read_text().splitlines() if json.loads(v).get('active_phase')];rows=[v for v in rows if v['time']<=end];previous={};radii=[];overlap=0;paired=0;minimum_virtual_gap=math.inf;flips={};eligible={};physical_gaps=[]
  for row in rows:
   centers={tuple(g['members']):g['center'] for g in row['groups_before']};targets={tuple(g['members']):g['goal'] for g in row['encounter_goals']};groups={tuple(sorted(g['members'])):g for g in row.get('group_orca',[])}
   for ids,g in groups.items():
    radii.append(g['radius_m']);v=g['orca_velocity'];speed=math.hypot(*v);angle=math.atan2(v[1],v[0]);old=previous.get(ids);goal=targets.get(ids)
    if old and old['goal']==goal and speed>.05 and old['speed']>.05:
     key=','.join(map(str,ids));eligible[key]=eligible.get(key,0)+1;delta=abs(math.atan2(math.sin(angle-old['angle']),math.cos(angle-old['angle'])))
     if delta>math.pi/2:flips[key]=flips.get(key,0)+1
    previous[ids]=dict(goal=goal,angle=angle,speed=speed)
   idslist=sorted(set(groups)&set(centers))
   if len(idslist)==2:
    i,j=idslist;gap=math.dist(centers[i],centers[j])-groups[i]['radius_m']-groups[j]['radius_m'];minimum_virtual_gap=min(minimum_virtual_gap,gap);paired+=1;overlap+=gap<-.01
   hs=row['before']
   physical_gaps.extend(math.hypot(h['x']-k['x'],h['y']-k['y'])-.72 for i,h in enumerate(hs) for k in hs[:i])
  report.append(dict(id=name,clock=condition,group_radius_min_m=min(radii),group_radius_max_m=max(radii),paired_group_samples=paired,virtual_overlap_samples=overlap,virtual_overlap_fraction=overlap/paired if paired else None,minimum_virtual_group_disc_gap_m=minimum_virtual_gap,minimum_sampled_actual_member_disc_gap_m=min(physical_gaps),raw_orca_intent_flips_over90deg_with_same_goal=flips,eligible_same_goal_consecutive_samples=eligible))
receipt=dict(schema='sim_clock_group_diagnostic/1',cases=report,scope='Additional local-motion diagnostic. Group circles are virtual planning envelopes, not actual body collisions. Same-target raw intent flips can motivate further investigation but do not establish a unique cause or naturalness acceptance.')
with a.output.open('x') as f:f.write(json.dumps(receipt,indent=2,allow_nan=False)+'\n')
print(json.dumps(report,indent=2))
