#!/usr/bin/env python3
"""Encode recorded native camera frames, preserving all originals."""
import argparse,json
from pathlib import Path
import cv2,numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[4];HERE=Path(__file__).resolve().parent

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--case',choices=['standing_base','standing_h5','walking_base','walking_h5'],required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
 if a.output.exists():raise FileExistsError(a.output)
 plan=json.loads((HERE/'plan.json').read_text());run=ROOT/plan['runtime']/a.case;source=run/'episode/camera_preview_4x.gif';cv2.setNumThreads(1)
 with Image.open(source) as gif:
  w=cv2.VideoWriter(str(a.output),cv2.VideoWriter_fourcc(*'VP80'),4,gif.size)
  if not w.isOpened():raise RuntimeError('VP8 unavailable')
  expected=0;duration_ms=0
  try:
   for i in range(gif.n_frames):
    gif.seek(i);duration=gif.info.get('duration',250);duration_ms+=duration;repeats=max(1,round(duration/250));frame=cv2.cvtColor(np.array(gif.convert('RGB')),cv2.COLOR_RGB2BGR)
    for _ in range(repeats):w.write(frame);expected+=1
  finally:w.release()
  if abs(expected*250-duration_ms)>1:raise ValueError('GIF duration not representable at4fps; preserve original for a higher-fps encoder')
 cap=cv2.VideoCapture(str(a.output));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release()
 if count!=expected:raise RuntimeError('Encoded frame count mismatch')
 print(json.dumps(dict(source=str(source),frames=count,fps=4,duration_ms=duration_ms,encoding='Lossy VP8 from recorded camera GIF; merged identical-frame hold durations preserved, no synthesized motion.')))
if __name__=='__main__':main()
