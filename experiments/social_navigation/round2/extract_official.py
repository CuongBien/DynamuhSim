#!/usr/bin/env python3
"""Convert only the summary pickle produced by our own native demo run."""
from pathlib import Path
import json
import pickle
import shutil
import numpy as np

HERE=Path(__file__).resolve().parent
REPORT=HERE/'reports'
paths=list((REPORT/'official_run').rglob('*.pkl'))
if len(paths)!=1:
    raise RuntimeError(f'Expected one native summary, got {paths}')
with paths[0].open('rb') as stream:
    data=pickle.load(stream)

def convert(value):
    if isinstance(value,dict): return {k:convert(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [convert(v) for v in value]
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    return value

(REPORT/'official_demo.json').write_text(json.dumps(convert(data),indent=2))
source=paths[0].with_suffix('.mp4')
if not source.exists():
    raise RuntimeError('Native demo did not save MP4')
shutil.copyfile(source,REPORT/'official_demo.mp4')
print(json.dumps(convert(data),indent=2))
