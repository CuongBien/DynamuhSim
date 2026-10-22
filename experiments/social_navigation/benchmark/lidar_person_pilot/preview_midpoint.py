"""Fixed midpoint render from measured data; no quality-selected frame."""
import json
from pathlib import Path
import numpy as np
from evaluate import preview,HERE
for name,inf in [('full01','full02'),('null01','null01')]:
 capture=HERE/'generated/runs'/name;inference=HERE/'generated/inference'/inf;evdir=HERE/'generated/evaluation'/name;obj=json.loads((evdir/'evaluation_LABEL_ONLY.json').read_text());index=len(obj['rows'])//2;out=evdir/'midpoint';out.mkdir(exist_ok=False)
 preview(out,np.load(capture/'full.npy')[index],np.load(evdir/'owner_LABEL_ONLY.npy')[index],obj['rows'][index],capture,inference,json.loads((HERE/'calibration_SENSOR_ONLY.json').read_text()),obj['condition'])
 (out/'source.json').write_text(json.dumps(dict(depth_index=index,stamp_ns=obj['rows'][index]['stamp_ns'],selection='Fixed midpoint of five frames, not quality selected'),indent=2)+'\n')
