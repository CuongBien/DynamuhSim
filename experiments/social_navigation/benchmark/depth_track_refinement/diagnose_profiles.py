#!/usr/bin/env python3
"""Read-only runtime profile inspection and GT-side direct rejection accounting."""
import argparse,collections,hashlib,json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
HERE=Path(__file__).resolve().parent


def profile_evidence():
    out={}
    def parse(p):return ET.fromstring(p.read_text().replace('gz:expressed_in','gz_expressed_in'))
    for key,rel in [('legacy_proxy','dataset_pilot/generated/pilot01'),('mesh_development','portable_episode/generated/episode01/config_snapshot/runtime'),('mesh_repeat','dataset_pilot/generated/avoidance_fixed01')]:
        p=HERE.parent/rel;world=p/'warehouse.sdf';robot=p/'mecanum_camera.sdf';wr=parse(world);rr=parse(robot);proxies=[]
        for m in wr.findall('.//model'):
            if m.get('name','').startswith('common_') and m.get('name','').endswith('_proxy'):
                cyl=m.find('.//visual/geometry/cylinder');proxies.append(dict(name=m.get('name'),radius_m=float(cyl.findtext('radius')),length_m=float(cyl.findtext('length'))))
        plugins=[dict(n.attrib) for n in wr.findall('.//plugin') if 'depth_proxy_filter' in str(n.attrib)]
        cameras=[]
        for s in rr.findall('.//sensor'):
            if s.find('camera') is not None:cameras.append(dict(name=s.get('name'),topic=s.findtext('topic'),visibility_mask=s.findtext('camera/visibility_mask'),type=s.get('type')))
        out[key]=dict(world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),robot_sha256=hashlib.sha256(robot.read_bytes()).hexdigest(),depth_filter_plugins=plugins,human_proxy_visuals=proxies,cameras=cameras)
    out['read_only_xml_note']='For metadata parsing only, replace unbound literal gz:expressed_in token in memory; no file serialization or runtime edits'
    return out


def rejection_effect():
    report={}
    for name,sensor,evaluation in [('development','development01','development_evaluation01'),('legacy_proxy_validation','validation01','validation_evaluation01'),('mesh_repeat','mesh_repeat01','mesh_repeat_evaluation01')]:
        frames=[json.loads(l) for l in (HERE/'generated'/sensor/'baseline.jsonl').read_text().splitlines()];removed=collections.defaultdict(list)
        for line in (HERE/'generated'/sensor/'shape_rejections.jsonl').read_text().splitlines():
            row=json.loads(line);removed[row['depth_index']].append(row)
        directly=[]
        for line in (HERE/'generated'/evaluation/'baseline_pairs_LABEL_ONLY.jsonl').read_text().splitlines():
            pair=json.loads(line)
            if not pair['geometric_pair_valid']:continue
            track=next(t for t in frames[pair['depth_index']]['tracks'] if t['track_id']==pair['track_id'])
            if any(np.linalg.norm(np.array(r['xy_base'])-track['xy_base'])<1e-10 for r in removed[pair['depth_index']]):directly.append(pair)
        report[name]=dict(baseline_gt_paired_rows_directly_rejected_by_shape_prior=len(directly),gt_people_affected=dict(collections.Counter(str(r['gt_human_id']) for r in directly)))
    return report
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    for name,v in [('depth_profile_evidence.json',profile_evidence()),('rejection_effect.json',rejection_effect())]:
        path=a.output/name
        if path.exists():assert json.loads(path.read_text())==v,'Existing evidence differs, use a new output directory'
        else:path.write_text(json.dumps(v,indent=2)+'\n')
    print('Depth profile and direct rejection evidence verified')
