#!/usr/bin/env python3
"""Re-encode the native recorded camera sequence; never overwrite original media."""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
from PIL import Image
ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm',choices=['B_aligned_080','C_aligned_045'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    plan=json.loads((HERE/'plan.json').read_text())
    run=ROOT/next(a['run'] for a in plan['arms'] if a['id']==args.arm)
    source=run/'episode/camera_preview_4x.gif'
    cv2.setNumThreads(1)
    with Image.open(source) as gif:
        writer=cv2.VideoWriter(str(args.output),cv2.VideoWriter_fourcc(*'VP80'),4,gif.size)
        if not writer.isOpened():raise RuntimeError('Cannot open VP8 encoder')
        try:
            for i in range(gif.n_frames):
                gif.seek(i)
                writer.write(cv2.cvtColor(np.array(gif.convert('RGB')),cv2.COLOR_RGB2BGR))
        finally:writer.release()
        expected=gif.n_frames
    cap=cv2.VideoCapture(str(args.output));actual=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));cap.release()
    if actual!=expected:raise RuntimeError('Encoded frame count differs from source')
    print(json.dumps(dict(source=str(source),output=str(args.output),frames=actual,fps=4,encoding='Lossy VP8, same recorded source frame sequence; no generated replacement frames.')))
if __name__=='__main__':main()
