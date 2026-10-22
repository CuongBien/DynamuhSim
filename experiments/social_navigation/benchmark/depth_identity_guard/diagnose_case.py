#!/usr/bin/env python3
"""Review the known regression using labels only; never modifies sensor tracks."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE=Path(__file__).resolve().parent
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    labels=HERE/'generated/regression43_eval05';source={}
    for name in ['parent','combined']:
        rows=[json.loads(l) for l in (labels/(name+'_pairs_LABEL_ONLY.jsonl')).read_text().splitlines()]
        selected=[r for r in rows if 13_000_000_000<=r['stamp_ns']<=14_400_000_000 and r['track_id']==5]
        source[name]=selected
    trace=dict(role='Known development/regression case only, GT geometric pairing not manually validated identity truth',track_id=5,trace=source,training_ready=False)
    (a.output/'regression_case_LABEL_ONLY.json').write_text(json.dumps(trace,indent=2)+'\n')
    fig,ax=plt.subplots(figsize=(9,3))
    for name,color,label in [('parent','#d93d38','previous ID5'),('combined','#158568','guarded ID5')]:
        lookup={r['stamp_ns']:r for r in source[name]};times=np.arange(13_000_000_000,14_400_000_001,100_000_000,dtype=np.int64)
        ids=[lookup[t]['gt_human_id'] if t in lookup and lookup[t]['geometric_pair_valid'] else np.nan for t in times]
        ax.plot(times/1e9,ids,marker='o',markersize=5,color=color,label=label,alpha=.8)
    ax.set(xlim=(12.95,14.45),ylim=(4.7,6.3),yticks=[5,6],yticklabels=['GT person5','GT person6'],xlabel='simulation time (s)',title='Known case: same ID previously changed GT pairing; guarded ID is left unobserved instead')
    ax.grid(alpha=.25);ax.legend(fontsize=8);fig.text(.02,.015,'Gaps mean no valid geometric pairing for this ID. Other IDs still cover measurements. GT is LABEL ONLY; no future backfill.',fontsize=8)
    fig.tight_layout(rect=[0,.06,1,1]);fig.savefig(a.output/'regression_identity_trace.png',dpi=140);plt.close(fig)
    print('Saved actual label-side trace for the known regression')
