import unittest
import numpy as np
import torch
from surface import evidence
class Contract(unittest.TestCase):
 def plane(self):
  y,z=np.meshgrid(np.linspace(-.3,.3,20),np.linspace(.04,1.,20));return np.c_[np.full(y.size,2.),y.ravel(),z.ravel()]
 def test_single_plane(self):self.assertTrue(evidence(self.plane(),'cpu')['planar_surface_evidence'])
 def test_corner(self):
  p=self.plane();other=p.copy();other[:,:2]=other[:,[1,0]];self.assertTrue(evidence(np.r_[p,other],'cpu')['planar_surface_evidence'])
 def test_sparse_never_becomes_object(self):self.assertFalse(evidence(self.plane()[:16],'cpu')['planar_surface_evidence'])
 def test_curved_surface_unknown(self):
  phi,z=np.meshgrid(np.linspace(-1.2,1.2,30),np.linspace(.04,1.2,20));p=np.c_[2+.2*np.cos(phi.ravel()),.2*np.sin(phi.ravel()),z.ravel()];self.assertFalse(evidence(p,'cpu')['planar_surface_evidence'])
 def test_rotation_translation(self):
  p=self.plane();c,s=np.cos(.6),np.sin(.6);p[:,:2]=p[:,:2]@np.array([[c,s],[-s,c]])+[1.,-.5];self.assertTrue(evidence(p,'cpu')['planar_surface_evidence'])
 def test_short_flat_patch_abstains(self):
  p=self.plane();p[:,2]*=.1;self.assertFalse(evidence(p,'cpu')['planar_surface_evidence'])
 def test_cuda_parity(self):
  self.assertTrue(torch.cuda.is_available());p=self.plane();a=evidence(p,'cpu');b=evidence(p,'cuda');self.assertEqual(a['planar_surface_evidence'],b['planar_surface_evidence']);self.assertEqual(a['explained_fraction'],b['explained_fraction'])
if __name__=='__main__':unittest.main()
