"""Read-only analysis/export of the declared bridge validation captures."""
import argparse,hashlib,importlib.util,json,xml.etree.ElementTree as ET
from pathlib import Path
import cv2,numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent

def load(name,path):
 spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def world(p):
 tree=ET.parse(p).getroot();nodes=tree.findall("world/plugin[@name='response_experiment::Adapter']/socket")
 if len(nodes)!=1:raise ValueError('Expected single adapter socket')
 nodes[0].text='<RUN_LOCAL_SOCKET>';return ET.tostring(tree)

def export(run,out):
 source=run/'episode/camera_preview_4x.gif';target=out/(run.name+'.webm');cv2.setNumThreads(1)
 with Image.open(source) as gif:
  writer=cv2.VideoWriter(str(target),cv2.VideoWriter_fourcc(*'VP80'),4,gif.size);frames=holds=0
  if not writer.isOpened():raise RuntimeError('VP8 unavailable')
  try:
   for i in range(gif.n_frames):
    gif.seek(i);hold=gif.info.get('duration',250);holds+=hold;frame=cv2.cvtColor(np.array(gif.convert('RGB')),cv2.COLOR_RGB2BGR)
    for _ in range(max(1,round(hold/250))):writer.write(frame);frames+=1
  finally:writer.release()
 cap=cv2.VideoCapture(str(target));decoded=0
 while cap.read()[0]:decoded+=1
 cap.release()
 if decoded!=frames or holds!=250*frames:raise ValueError('Export duration/frame mismatch')
 return dict(source=str(source.relative_to(ROOT)),source_sha256=sha(source),webm_sha256=sha(target),decoded_frames=decoded,encoded_fps=4,holds_ms=holds,scope='Recorded native Gazebo RGB; lossy VP8, original frame holds preserved. Encoded4x, default playbackRate.25 restores sim-time1x. No synthetic motion.')

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
 plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];reference=ROOT/plan['reference_root'];human=load('bridge_human',HERE.parent/'group_cohesion_heading/analysis.py');enc=human.enc
 report=dict(schema='bridge_shutdown_result/1',cases=[],errors=[],defaults_promoted=False,training_ready=False);audits={};media={}
 for case in plan['cases']:
  name=case['id'];run=raw/name;old=reference/case['reference_id']
  try:
   initial_audit=json.loads((raw/(name+'_audit.json')).read_text());audit=json.loads((raw/(name+'_source_verified_audit.json')).read_text());result=json.loads((run/'episode/result.json').read_text());before=json.loads((old/'episode/result.json').read_text());audits[name]=audit
   metric,data,_=enc.analyze_social(run,case);cohesion,_=human.native_cohesion(run,data)
   # The reusable historical analyzer reads the first audit filename. Retain it
   # above, then explicitly evaluate physical gates against the source-root audit.
   metric['audit']=audit['status']
   metric['physical_pass']=audit['status']=='PASS' and metric['actual_goal_reached'] and metric['minimum_swept_robot_human_gap_m']>0 and metric['minimum_static_gap_lower_bound_m']>0 and min(metric['human_safety'].values())>0
   metric['encounter_screen_pass']=metric['physical_pass'] and metric['recovery_free'] and metric['proximity']['coverage']=='EXERCISED' and metric['worker']['initial_intent_executed']
   protected={k:v for k,v in before['source_sha256'].items() if '/vendor/SocNavGym/' in k or k.startswith('python/dynamuhsim/simulation/') or k.endswith('/responsive_motion.py') or k.endswith('/projection.py')}
   checks=dict(full_audit=audit['status']=='PASS',ordered_bridge=audit['checks'].get('ordered_ground_truth_bridge_exit') is True,physical_and_coverage=metric['encounter_screen_pass'],same_protected_sources=bool(protected) and all(result['source_sha256'].get(k)==v for k,v in protected.items()),same_initial_humans=before['ready']['humans']==result['ready']['humans'],same_world=world(run/'runtime/warehouse.sdf')==world(old/'runtime/warehouse.sdf'))
   for file in ['assets/mecanum_camera.sdf','assets/mecanum_aligned.urdf','assets/encounter.json','assets/human_goal_clock.json','assets/human_motion.json','runtime/nav2_runtime.yaml']:
    checks['same:'+file]=(run/file).read_bytes()==(old/file).read_bytes()
   lines=(run/'episode/launch.log').read_text().splitlines();receipt=result['server_shutdown']['ground_truth_bridge_shutdown'];bridge_ix=lines.index(receipt['record']) if receipt.get('record') in lines else -1;gz_ix=next((i for i,v in enumerate(lines) if v.startswith('[INFO] [gz-1]: process has finished cleanly')), -1);checks['bridge_exit_before_gazebo_exit']=0<=bridge_ix<gz_ix
   report['cases'].append(dict(id=name,status='PASS' if all(checks.values()) else 'FAIL',checks=checks,metric=metric,native_cohesion=cohesion,bridge=receipt,server_shutdown=result['server_shutdown'],cleanup=result['cleanup'],runtime_commit=result['head'],initial_audit=dict(status=initial_audit['status'],failed_checks=[k for k,v in initial_audit['checks'].items() if not v],missing_sources=initial_audit.get('missing_sources',[]),receipt_sha256=sha(raw/(name+'_audit.json'))),source_verified_audit_sha256=sha(raw/(name+'_source_verified_audit.json')),original_matching_audit=json.loads((reference/(case['reference_id']+'_audit.json')).read_text())['status'],evidence_sha256={f:sha(run/f) for f in ['capture_freeze.json','episode/result.json','episode/launch.log','runtime/quiesce_shutdown.json','runtime/process_cleanup.json']}))
   media[name]=export(run,out)
  except Exception as error:report['errors'].append(dict(id=name,error=type(error).__name__+': '+str(error)))
 report['status']='PASS' if len(report['cases'])==len(plan['cases']) and not report['errors'] and all(v['status']=='PASS' for v in report['cases']) else 'FAIL'
 for file,value in [('results.json',report),('audits.json',audits),('media_evidence.json',media)]:
  (out/file).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
 cards=''
 for item in report['cases']:
  m=item['metric'];cards+=f"<article><h2>{item['id']}: {item['status']}</h2><p>Full audit: {m['audit']}; active simulation: {m['active_sim_s']:.2f}s; recoveries: {m['max_action_recoveries']}; minimum robot/person surface gap: {m['minimum_swept_robot_human_gap_m']*100:.1f}cm. Bridge exit: {item['bridge']['status']}, clean: {item['bridge']['clean']}.</p><video controls preload='metadata' src='{item['id']}.webm'></video></article>"
 html="<!doctype html><meta charset='utf-8'><title>Bridge shutdown validation</title><style>body{background:#101923;color:#e9f0f6;font:17px system-ui;max-width:1100px;margin:36px auto;padding:20px}a{color:#7cdac5}article{border:1px solid #486071;border-radius:12px;padding:20px;margin:20px 0}video{width:100%}</style><h1>Kiểm chứng đóng bridge</h1><p>Hai lượt mới; mô hình người và Nav2 giữ nguyên. Video là RGB ghi trực tiếp từ Gazebo. Mặc định phát 1× thời gian mô phỏng. Đây là kiểm chứng có giới hạn trên seed45; chưa chốt dữ liệu train.</p><p>Lượt crossing cũ vẫn FAIL và được giữ nguyên. Không ghi đè, không bỏ tiêu chí lỗi. Root cause upstream chưa được xác lập. Audit đầu gọi thiếu source-root nên FAIL source_hashes; giữ nguyên bản đó và kiểm tra chỉ đọc lại với đường dẫn nguồn đầy đủ. Không chạy lại mô phỏng.</p>"+cards+"<p><a href='results.json'>Results + shutdown receipts</a> · <a href='audits.json'>Full audits</a> · <a href='media_evidence.json'>Media hashes</a> · <a href='../group_cohesion_heading/index.html'>Kết quả trước và lỗi cũ</a></p><script>document.querySelectorAll('video').forEach(v=>v.addEventListener('loadedmetadata',()=>v.playbackRate=.25));</script>"
 (out/'index.html').write_text(html);print(report['status'],[(v['id'],v['status']) for v in report['cases']],report['errors'])

if __name__=='__main__':main()
