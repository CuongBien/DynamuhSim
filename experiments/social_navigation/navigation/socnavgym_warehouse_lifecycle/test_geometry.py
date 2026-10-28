import math
from geometry import segment_box,admissible_fraction,clearance
box=dict(x=0.,y=0.,sx=2.,sy=2.)
assert segment_box((-2.,0.),(4.,0.),box)==0.
assert abs(segment_box((2.,2.),(1.,1.),box)-math.sqrt(2))<1e-12
assert abs(segment_box((-3.,2.),(6.,0.),box)-1.)<1e-12
far=[dict(x=10.,y=10.,sx=1.,sy=1.)]
f,g=admissible_fraction([(-1.,0.)],[(-.5,0.)],[.2],(0.,0.),(-1.,0.),.2,far)
assert f==1. and g['robot']>.09
assert clearance([(-1.,0.)],[(0.,0.)],[.2],(0.,0.),(-1.,0.),.2,far)['robot']<0.
try:admissible_fraction([(0.,0.)],[(0.,0.)],[.2],(-1.,0.),(2.,0.),.2,far)
except RuntimeError:pass
else:raise AssertionError('Incoming robot cannot be silently shortened')
f,g=admissible_fraction([(-2.,0.)],[(4.,0.)],[.2],(-2.,4.),(0.,0.),.2,[box])
assert f<.25 and g['static']>=.002-1e-8
print('PASS: rectangle tunneling/corners, moving robot with fixed interval, infeasible input rejection')
