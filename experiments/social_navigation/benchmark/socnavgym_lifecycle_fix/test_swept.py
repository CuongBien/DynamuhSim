"""Meaningful tunneling/initial-invalid fixtures for the geometric fallback."""
import math
from continuous_groups import first_contact,safe_fraction
assert abs(first_contact((-2.,0.),(4.,0.),1.)-.25)<1e-12
alpha=safe_fraction([(-1.,0.),(1.,0.)],[(2.,0.),(-2.,0.)],[.2,.2],(0.,4.),.3,(5.,5.))
assert 0<alpha<.4
for i in range(101):
    f=alpha*i/100.;assert abs((-1+2*f)-(1-2*f))>=.402-1e-10
alpha_robot=safe_fraction([(-2.,0.)],[(4.,0.)],[.2],(0.,0.),.3,(5.,5.))
assert abs(alpha_robot-(2-.502)/4)<2e-6
alpha_wall=safe_fraction([(0.,0.)],[(10.,0.)],[.2],(-4.,0.),.3,(4.8,4.8))
assert abs(alpha_wall-.4598)<2e-6
assert safe_fraction([(-2.,-2.)],[(.1,0.)],[.2],(2.,2.),.3,(5.,5.))==1.
try:first_contact((.1,0.),(.1,0.),.4)
except ValueError:pass
else:raise AssertionError('Invalid initial overlap was not rejected')
print('PASS: pair tunneling, robot tunneling, wall crossing, free motion, invalid start')
