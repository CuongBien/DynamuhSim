"""Export native observer GIFs without changing raw inputs or synthesizing motion."""
import argparse,hashlib,json
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False);plan=json.loads((HERE/'plan.json').read_text());raw=ROOT/plan['raw_root'];metadata={};cv2.setNumThreads(1);thumbs=[]
for case in plan['robot_cases']+plan['human_cases']:
 name=case['id'];source=raw/name/'episode/camera_preview_4x.gif';out=a.output/(name+'.webm');result=json.loads((raw/name/'episode/result.json').read_text())
 with Image.open(source) as gif:
  writer=cv2.VideoWriter(str(out),cv2.VideoWriter_fourcc(*'VP80'),4,gif.size);count=duration_ms=0
  if not writer.isOpened():raise RuntimeError('VP8 unavailable')
  try:
   for i in range(gif.n_frames):
    gif.seek(i);hold=gif.info.get('duration',250);duration_ms+=hold;repeats=max(1,round(hold/250));frame=cv2.cvtColor(np.array(gif.convert('RGB')),cv2.COLOR_RGB2BGR)
    for _ in range(repeats):writer.write(frame);count+=1
   gif.seek(min(4,gif.n_frames-1));thumb=gif.convert('RGB').resize((640,360));ImageDraw.Draw(thumb).text((12,12),name,fill='white',stroke_width=2,stroke_fill='black');thumbs.append(thumb)
  finally:writer.release()
 if abs(count*250-duration_ms)>1:raise ValueError('Hold duration mismatch')
 cap=cv2.VideoCapture(str(out));actual=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release()
 if actual!=count:raise ValueError('Frame count mismatch')
 metadata[name]=dict(source=str(source.relative_to(ROOT)),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),webm_sha256=hashlib.sha256(out.read_bytes()).hexdigest(),frames=actual,fps=4,duration_ms=duration_ms,original_sim_frame_times=result['media']['frame_times'],scope='Lossy VP8 from native live Gazebo RGB GIF; merged identical-frame holds preserved. No generated/interpolated motion; playback4x.')
sheet=Image.new('RGB',(1280,720))
for i,t in enumerate(thumbs):sheet.paste(t,((i%2)*640,(i//2)*360))
sheet.save(a.output/'native_contact_sheet.jpg');(a.output/'media_evidence.json').write_text(json.dumps(metadata,indent=2)+'\n');print({k:v['frames'] for k,v in metadata.items()})
