"""Meaningful native smoke checks: soft-cost detour, distance-only shortcut, hard rejection."""
from pathlib import Path
import json,subprocess
import numpy as np
HERE=Path(__file__).resolve().parent
out=HERE/'generated/smoke';out.mkdir(parents=True,exist_ok=True)
exe=HERE/'generated/build/navfn_probe'
def probe(grid,name):
 p=out/(name+'.u8');p.write_bytes(grid.tobytes());v=json.loads(subprocess.check_output([str(exe),str(p),'100','60','5','30','90','30'],text=True));return v
weighted=np.zeros((60,100),dtype='u1');weighted[15:45,40:60]=240
normal=probe(weighted,'soft_cost');flat=probe(np.zeros_like(weighted),'free')
assert normal['found'] and flat['found']
maxdy=lambda v:float(np.abs(np.array(v['path_cells'])[:,1]-30).max())
assert maxdy(normal)>10 and maxdy(flat)<1
blocked=weighted.copy();blocked[30,90]=254
assert not probe(blocked,'blocked_goal')['found']
print(json.dumps(dict(status='PASS',soft_detour_max_cell_offset=maxdy(normal),free_path_max_cell_offset=maxdy(flat),lethal_goal_rejected=True)))
