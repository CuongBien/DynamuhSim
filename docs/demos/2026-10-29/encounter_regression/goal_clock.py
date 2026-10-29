"""Expose host-time retargets while initial/native group targets are still distant."""
import argparse,json,math,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];cases=[]
for case in plan['social_cases']:
 run=raw/case['id'];events=[json.loads(s) for s in (run/'episode/events.jsonl').read_text().splitlines()];times={v['event']:v['stamp_ns']/1e9 for v in events};rows=[json.loads(s) for s in (run/'runtime/worker_trajectory.jsonl').read_text().splitlines()];data=[v for v in rows if v.get('active_phase') and v['time']<=times['action_end']];changes=[]
 for previous,current in zip(data,data[1:]):
  old={tuple(g['members']):g['goal'] for g in previous['encounter_goals']};new={tuple(g['members']):g['goal'] for g in current['encounter_goals']};humans={h['id']:h for h in previous['humans']}
  for ids,target in new.items():
   if ids not in old or old[ids]==target:continue
   ages=[previous['encounter_goal_ages_wall_s'][str(hid)] for hid in ids];distances=[math.hypot(humans[hid]['x']-old[ids][0],humans[hid]['y']-old[ids][1]) for hid in ids];changes.append(dict(time_after_goal_s=current['time']-times['goal_accepted'],members=list(ids),old_target=old[ids],new_target=target,previous_goal_ages_wall_s=ages,previous_member_goal_distance_m=distances,near_50s_while_target_far=min(ages)>49 and max(distances)>1.9,scope='Diagnostic consistent with native wall-time expiry, not attribution of all heading changes or all lifecycle events.'))
 cases.append(dict(id=case['id'],changes=changes,near_50s_far_target_count=sum(v['near_50s_while_target_far'] for v in changes)))
source=ROOT/'experiments/social_navigation/vendor/SocNavGym/socnavgym/envs/utils/human.py';receipt=dict(schema='encounter_goal_clock/1',native_timeout_wall_s=50,human_goal_acceptance_radius_m=1.8,radius_basis='Fixture human.width/2 .36 + interaction_goal_radius1.44; source calls group.has_reached_goal() with default offset.',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),cases=cases,scope='Read-only diagnostic. Source accepts geometric arrival OR host-time expiry; future fixed-clock comparisons are required. Raw inputs and clocks remain unchanged.')
with a.output.open('x') as f:f.write(json.dumps(receipt,indent=2)+'\n')
print({v['id']:v['near_50s_far_target_count'] for v in cases})
