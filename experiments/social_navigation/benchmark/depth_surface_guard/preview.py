"""Fixed midpoint before/after views, including remaining false positive."""
import importlib.util,json,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;OLD=HERE.parent/'lidar_person_pilot';sys.path.insert(0,str(OLD))
spec=importlib.util.spec_from_file_location('fixed_sensor_preview',OLD/'evaluate.py');module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
cal=json.loads((OLD/'calibration_SENSOR_ONLY.json').read_text())
for name,capture,inf,evaldir in [('old_before',OLD/'generated/runs/full01',OLD/'generated/inference/full02',OLD/'generated/evaluation/full01'),('old_after',OLD/'generated/runs/full01',HERE/'generated/regression_full01',HERE/'generated/regression_full_eval'),('new_before',HERE/'generated/runs/newscene01',HERE/'generated/newscene_prior',HERE/'generated/newscene_prior_eval'),('new_after',HERE/'generated/runs/newscene01',HERE/'generated/newscene_guard',HERE/'generated/newscene_guard_eval')]:
 ev=json.loads((evaldir/'evaluation_LABEL_ONLY.json').read_text());index=len(ev['rows'])//2;out=HERE/'generated/preview'/name;out.mkdir(parents=True,exist_ok=False);module.preview(out,np.load(capture/'full.npy')[index],np.load(evaldir/'owner_LABEL_ONLY.npy')[index],ev['rows'][index],capture,inf,cal,ev['condition']);(out/'source.json').write_text(json.dumps(dict(depth_index=index,stamp_ns=ev['rows'][index]['stamp_ns'],selection='Fixed midpoint, not chosen by quality'),indent=2)+'\n')
