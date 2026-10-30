import importlib.util
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]
STAGE=ROOT/'experiments/social_navigation/perception/depth_generalization'
spec=importlib.util.spec_from_file_location('depth_prior_ablation',STAGE/'ablation.py')
ab=importlib.util.module_from_spec(spec);spec.loader.exec_module(ab)


class PriorAblationTests(unittest.TestCase):
    def test_no_prior_uses_base_score_without_mutating_original(self):
        detection=dict(base_score=.99,score=.1485,inference_valid=True,is_person=False)
        kept=ab.choose_without_prior([detection],.45)
        self.assertEqual(len(kept),1)
        self.assertEqual(kept[0]['score'],.99)
        self.assertEqual(detection['score'],.1485)
        self.assertFalse(detection['is_person'])

    def test_invalid_inference_is_never_promoted(self):
        self.assertEqual(ab.choose_without_prior([dict(base_score=1.,inference_valid=False)],.45),[])

    def test_nonfinite_and_out_of_range_scores_rejected(self):
        for value in [float('nan'),float('inf'),-1.,1.1]:
            self.assertEqual(ab.choose_without_prior([dict(base_score=value,inference_valid=True)],.45),[])

    def test_fixed_threshold_control_changes_only_boundary(self):
        d=dict(base_score=.49,score=.49,is_person=False,inference_valid=True)
        self.assertEqual(len(ab.choose_without_prior([d],.45)),1)
        self.assertEqual(ab.choose_without_prior([d],.5),[])
        self.assertEqual(len(ab.choose_without_prior([dict(d,base_score=.5)],.5)),1)

    def test_reference_rejects_stamp_and_count_mismatch(self):
        a=dict(stamp_ns=10,odom_valid=True,tracks=[])
        with self.assertRaises(ValueError):ab.check_reference(a,dict(a,stamp_ns=11))
        with self.assertRaises(ValueError):ab.check_reference(a,dict(a,tracks=[{}]))

    def test_reference_rejects_identity_and_geometry_drift(self):
        t=dict(track_id=1,proposal_index=0,point_count=30,confirmed=True,hits=4,
               association_ambiguous=False,reacquired_after_gap=False,
               xy_base=[1.,0.],xy_odom=[1.,0.],velocity_odom=[0.,0.],extent_m=[.4,.4,1.4],person_score=.6)
        row=dict(stamp_ns=10,odom_valid=True,tracks=[t])
        ab.check_reference(row,row)
        for change in [dict(track_id=2),dict(xy_base=[1.01,0.]),dict(person_score=.61)]:
            with self.assertRaises(ValueError):ab.check_reference(dict(row,tracks=[dict(t,**change)]),row)

    def test_gt_stage_pairing_is_one_to_one_and_support_masked(self):
        protocol=dict(pair_gate_m=.6,pair_ambiguity_margin_m=.15)
        targets=[[1.,0.],[3.,0.]]
        self.assertEqual(ab.matched_supported([[1.,0.],[3.,0.]],targets,[True,False],protocol),{0})
        self.assertEqual(ab.matched_supported([[1.,0.],[1.1,0.]],targets,[True,False],protocol),set())


if __name__=='__main__':unittest.main()
