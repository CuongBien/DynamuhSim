#!/usr/bin/env python3
import argparse,csv,pathlib
p=argparse.ArgumentParser(); p.add_argument('frames_csv'); p.add_argument('--front-threshold',type=float,default=0.70); p.add_argument('--out',default=None); a=p.parse_args()
src=pathlib.Path(a.frames_csv); dst=pathlib.Path(a.out) if a.out else src.with_name('hard_negatives.csv')
with src.open() as f: rows=list(csv.DictReader(f))
h=[]
for r in rows:
    try: front=float(r['front_min']); v=abs(float(r['cmd_v']))
    except: continue
    if front<a.front_threshold and v>0.03: h.append(r)
with dst.open('w',newline='') as f:
    if rows:
        w=csv.DictWriter(f,fieldnames=rows[0].keys()); w.writeheader(); w.writerows(h)
print(f'rows={len(rows)} hard_negatives={len(h)} output={dst}')
