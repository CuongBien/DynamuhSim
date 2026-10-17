#!/usr/bin/env python3
"""Validate captures, make small animated previews, and audit original source hashes."""
import hashlib
import json
from pathlib import Path
import cv2
from PIL import Image, ImageDraw

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
REPORT=HERE/'reports'
video_stats={}
for scenario in ['original','groups','formation','reactive','blind']:
    path=REPORT/f'socnavgym_{scenario}_s42.mp4'
    cap=cv2.VideoCapture(str(path))
    count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps=float(cap.get(cv2.CAP_PROP_FPS))
    okay,frame=cap.read()
    video_stats[scenario]=dict(frames=count,fps=fps,first_frame_decodes=okay)
    if scenario in ['groups','formation']:
        frames=[]
        for idx in range(0,count,10):
            cap.set(cv2.CAP_PROP_POS_FRAMES,idx)
            okay,frame=cap.read()
            if not okay:
                break
            im=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)).resize((450,450))
            ImageDraw.Draw(im).text((8,430),'Preview 4x speed; full MP4 at real simulation speed',fill='black')
            frames.append(im)
        if frames:
            frames[0].save(REPORT/f'socnavgym_{scenario}_preview.gif',save_all=True,
                append_images=frames[1:],duration=250,loop=0,optimize=True)
    cap.release()
(REPORT/'video_validation.json').write_text(json.dumps(video_stats,indent=2))
checks=[]
for line in (REPORT/'tracked_sha256_before.txt').read_text().splitlines():
    expected,sep,name=line.partition('  ')
    if not sep:
        continue
    f=ROOT/name
    actual=hashlib.sha256(f.read_bytes()).hexdigest() if f.is_file() else None
    checks.append(dict(path=name,unchanged=actual==expected))
changed=[c['path'] for c in checks if not c['unchanged']]
audit=dict(existing_tracked_files_checked=len(checks),changed_by_this_work=changed)
(REPORT/'source_preservation.json').write_text(json.dumps(audit,indent=2))
print(json.dumps(dict(videos=video_stats,source_audit=audit),indent=2))
