"""Label-side geometric pairing only. Never called by sensor tracker."""
import numpy as np
from scipy.optimize import linear_sum_assignment


def associate(candidate_xy,human_xy,gate_m=.60,ambiguity_margin_m=.15):
    candidate_xy=np.asarray(candidate_xy).reshape(-1,2);human_xy=np.asarray(human_xy).reshape(-1,2)
    out=np.full(len(candidate_xy),-1,dtype=np.int32);errors=np.full(len(candidate_xy),np.nan)
    if not len(candidate_xy) or not len(human_xy):return out,errors
    cost=np.linalg.norm(candidate_xy[:,None,:]-human_xy[None,:,:],axis=-1)
    rows,cols=linear_sum_assignment(np.where(cost<=gate_m,cost,1e6))
    for row,col in zip(rows,cols):
        others_in_row=np.delete(cost[row],col);others_in_col=np.delete(cost[:,col],row)
        alternative=min(others_in_row.min(initial=np.inf),others_in_col.min(initial=np.inf))
        if cost[row,col]<=gate_m and alternative-cost[row,col]>=ambiguity_margin_m:
            out[row]=col;errors[row]=cost[row,col]
    return out,errors
