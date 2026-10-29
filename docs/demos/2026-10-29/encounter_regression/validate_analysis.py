"""Analytic cases protect the exposure-time coverage calculation."""
from pathlib import Path
import importlib.util,math
s=importlib.util.spec_from_file_location('encounter_analysis',Path(__file__).with_name('analysis.py'));m=importlib.util.module_from_spec(s);s.loader.exec_module(m)
assert m.threshold_interval((0,0),(0,0),1)==(0.,1.)
assert m.threshold_interval((2,0),(2,0),1) is None
lo,hi=m.threshold_interval((-2,0),(2,0),1);assert abs(lo-.25)<1e-12 and abs(hi-.75)<1e-12
assert abs(m.union_length([(0,.5),(.25,.75),None,(.9,1)])-.85)<1e-12
assert m.union_length([])==0
assert m.threshold_interval((-1,1),(1,1),1)==(.5,.5)
print('PASS: fixed-inside/outside, analytic transit, tangent contact and interval union (no human double-counting).')
