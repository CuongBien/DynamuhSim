"""Scientific export regressions: timing, masks, transforms, no-leakage, anchor bounds."""
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
from dynamuhsim.datasets.single_episode_export import (causal,time_window,transform,gather,
    validate_no_leakage,disc_occupancy,NS,STEP,qualify_factual_window,make_windows)


class ExportTests(unittest.TestCase):
    def test_alignment_requires_both_header_and_record_availability(self):
        idx,good=causal([10,20,30],[12,25,35],[9,10,12,20,25,34,35],20)
        np.testing.assert_array_equal(idx,[-1,-1,0,0,1,1,2])
        self.assertFalse(good[0]);self.assertTrue(good[-1])

    def test_missing_empty_stale_and_large_gap_stay_masked(self):
        idx,good=causal([],[],[1,2],10);np.testing.assert_array_equal(idx,[-1,-1]);self.assertFalse(good.any())
        idx,good=causal([0,100],[0,100],[0,50,99,100],10)
        np.testing.assert_array_equal(idx,[0,-1,-1,1]);self.assertFalse(good[1])

    def test_late_old_packet_cannot_replace_newer_source(self):
        idx,good=causal([10,20],[30,21],[21,30],100)
        np.testing.assert_array_equal(idx,[1,1])

    def test_duplicate_command_stamp_uses_last_publication(self):
        idx,_=causal([10,10],[10,10],[10],10,[100,101])
        self.assertEqual(idx[0],1)

    def test_anchor_once_in_history_never_in_future(self):
        h,f=time_window(25,100)
        self.assertEqual(len(h),21);self.assertEqual(len(f),30)
        self.assertEqual((h[0],h[-1],f[0],f[-1]),(5,25,26,55))
        h,f=time_window(0,10);self.assertTrue((h[:-1]==-1).all());self.assertEqual(h[-1],0);self.assertTrue((f[9:]==-1).all())

    def test_masked_negative_index_never_reads_last_future_frame(self):
        bank=np.array([[10.],[999.]])
        result=gather(bank,[-1,0],[False,True])
        self.assertTrue(np.isnan(result[0]).all());self.assertEqual(result[1,0],10.)

    def test_rigid_transform_and_inverse_use_declared_direction(self):
        q=[0,0,np.sin(np.pi/4),np.cos(np.pi/4)]
        matrix=transform([2,3,0],q);point=matrix@np.array([1,0,0,1])
        np.testing.assert_allclose(point,[2,4,0,1],atol=1e-10)
        np.testing.assert_allclose(np.linalg.inv(matrix)@point,[1,0,0,1],atol=1e-10)

    def test_future_sensor_and_gt_observed_track_leakage_rejected(self):
        obs=dict(depth_stamp_ns=np.array([10]),depth_record_ns=np.array([21]),observed_tracks=np.empty((1,0,5)),observed_tracks_available=np.array([False]))
        window=dict(anchor_ns=20,history_query_ns=[20],future_query_ns=(20+np.arange(1,31)*STEP).tolist(),
            obs=dict(indices=dict(depth=[0]),availability=dict(depth=[True])),act=dict(past_published_control_ids=[],past_published_control_available=[]))
        with self.assertRaisesRegex(ValueError,'future leakage'):validate_no_leakage([window],obs,[])
        obs['depth_record_ns'][0]=10;validate_no_leakage([window],obs,[])
        obs['human_world']=np.zeros((1,6,3))
        with self.assertRaisesRegex(ValueError,'GT fields'):validate_no_leakage([window],obs,[])

    def test_occupancy_world_xy_maps_to_yx_rows_and_excludes_other_cells(self):
        grid=disc_occupancy([[.25,.75]],[.02],10,10,np.array([0.,0.]),.1)
        self.assertTrue(grid[7,2]);self.assertEqual(int(grid.sum()),1)

    def window(self):
        return dict(anchor_ns=2*NS,valid=True,reasons=[],act={},
            history_query_ns=(np.arange(21)*STEP).tolist(),future_query_ns=(2*NS+np.arange(1,31)*STEP).tolist())

    def commands(self,times):
        return [dict(source_stamp_ns=int(t),record_stamp_ns=int(t),publish_wall_ns=i,value=[.1,.2,.3]) for i,t in enumerate(times)]

    def test_sensor_task_does_not_require_actions_tracks_contact_or_candidates(self):
        w=qualify_factual_window(self.window(),[],False)
        self.assertTrue(w['sensor_factual_valid']);self.assertFalse(w['past_action_factual_valid'])
        self.assertFalse(w['future_recorded_action_valid']);self.assertFalse(w['counterfactual_valid'])
        self.assertFalse(w['semantic_observed_tracks_valid'])

    def test_future_recorded_task_is_independent_of_past_actions(self):
        w=self.window();qualified=qualify_factual_window(w,self.commands(w['future_query_ns']),True)
        self.assertFalse(qualified['past_action_factual_valid']);self.assertTrue(qualified['future_recorded_action_valid'])
        self.assertFalse(qualified['counterfactual_valid'])

    def test_past_actions_cannot_stand_in_for_future_records(self):
        w=qualify_factual_window(self.window(),self.commands(self.window()['history_query_ns']),True)
        self.assertTrue(w['past_action_factual_valid']);self.assertFalse(w['future_recorded_action_valid'])
        self.assertEqual(w['act']['future_recorded_requested_ids'][0],-1)

    def test_provenance_and_sensor_failures_do_not_become_action_passes(self):
        commands=self.commands(np.arange(51)*STEP);w=self.window()
        q=qualify_factual_window(w,commands,False)
        self.assertTrue(q['sensor_factual_valid']);self.assertFalse(q['past_action_factual_valid'])
        w.update(valid=False,reasons=['missing_future_gt_human_available'])
        q=qualify_factual_window(w,commands,True)
        self.assertFalse(q['sensor_factual_valid']);self.assertFalse(q['future_recorded_action_valid'])

    def test_sensor_target_requires_only_humans_and_keeps_lidar_source_rate(self):
        grid=np.arange(60)*STEP;refs={k:np.arange(60) for k in ['depth','lidar','published_control']}
        masks={k:np.ones(60,dtype=bool) for k in refs}
        windows,_=make_windows(grid,refs,masks,dict(human_available=np.ones((60,1),dtype=bool)),0,required_gt=('human_available',))
        self.assertTrue(windows[20]['valid'])  # No robot/occupancy/contact/observed GT requirement.
        ids,mask=causal([0,2*STEP,4*STEP],[0,2*STEP,4*STEP],grid[:5],3*STEP)
        np.testing.assert_array_equal(ids,[0,0,1,1,2]);self.assertTrue(mask.all())


if __name__=='__main__':unittest.main()
